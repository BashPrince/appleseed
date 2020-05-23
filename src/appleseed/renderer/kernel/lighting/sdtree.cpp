
//
// This source file is part of appleseed.
// Visit https://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2019 Stephen Agyemang, The appleseedhq Organization
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

// Interface header.
#include "sdtree.h"

// appleseed.renderer headers.
#include "renderer/global/globallogger.h"
#include "renderer/modeling/camera/camera.h"

// appleseed.foundation headers.
#include "foundation/math/sampling/mappings.h"
#include "foundation/math/scalar.h"
#include "foundation/string/string.h"
#include "foundation/utility/job/ijob.h"
#include "foundation/utility/job/jobmanager.h"
#include "foundation/utility/job/jobqueue.h"

// Standard headers.
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace foundation;

namespace renderer
{

//
// SD-Tree implementation for "Practical Path Guiding for Efficient Light-Transport Simulation" [Müller et al. 2017].
//

const size_t SpatialSubdivisionThreshold = 4000; // TODO: make this dependent on the filter types
const float DTreeThreshold = 0.01f;
const size_t DTreeMaxDepth = 20;
const float DTreeGlossyAreaFraction = 0.1f;
const float DTreeGlossyEnergyThreshold = 0.7f;

// Sampling fraction optimization constants.

const float Beta1 = 0.9f;
const float Beta2 = 0.999f;
const float OptimizationEpsilon = 1e-8f;
const float Regularization = 0.01f;

// Helper functions.

void atomic_add(
    std::atomic<float>&                 atomic,
    const float                         value)
{
    float current = atomic.load(std::memory_order_relaxed);
    while (!atomic.compare_exchange_weak(current, current + value))
        ;
}

inline float logistic(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

Vector2f cartesian_to_cylindrical(
    const Vector3f&                     direction)
{
    const float cos_theta = direction.z;
    float phi = std::atan2(direction.y, direction.x);

    if (phi < 0.0f)
        phi += TwoPi<float>();

    // D-tree directions are stored as 2D [cos(theta), phi] coordinates to preserve area.
    // Theta is the angle with z-Axis to ensure compatibility with SD-tree visualizer [Müller et.al. 2017]
    return Vector2f(
        (cos_theta + 1.0f) * 0.5f,
        phi * RcpTwoPi<float>());
}

Vector3f cylindrical_to_cartesian(
    const Vector2f&                     cylindrical_direction)
{
    assert(cylindrical_direction[0] >= 0.0f && cylindrical_direction[0] < 1.0f);
    assert(cylindrical_direction[1] >= 0.0f && cylindrical_direction[1] < 1.0f);

    const float phi = TwoPi<float>() * cylindrical_direction[1];
    const float cos_theta = 2.0f * cylindrical_direction[0] - 1.0f;
    const float sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);

    return Vector3f(
        std::cos(phi) * sin_theta,
        std::sin(phi) * sin_theta,
        cos_theta);
}

template<typename T>
void write(
    std::ofstream&                      outstream,
    const T                             data)
{
    outstream.write(reinterpret_cast<const char*>(&data), sizeof(T));
}

// Node structure compatible with SD tree visualizer [Müller et al. 2017].

struct VisualizerNode
{
    std::array<float, 4>                sums;
    std::array<size_t, 4>               children;
};

// QuadTreeNode implementation.

QuadTreeNode::QuadTreeNode(
    const bool                          create_children,
    const float                         radiance_sum)
    : m_is_leaf(!create_children)
    , m_current_iter_radiance_sum(radiance_sum)
    , m_previous_iter_radiance_sum(radiance_sum)
{   
    if(create_children)
    {
        m_upper_left_node.reset(new QuadTreeNode(false));
        m_upper_right_node.reset(new QuadTreeNode(false));
        m_lower_right_node.reset(new QuadTreeNode(false));
        m_lower_left_node.reset(new QuadTreeNode(false));
    }
}

QuadTreeNode::QuadTreeNode(const QuadTreeNode& other)
    : m_current_iter_radiance_sum(other.m_current_iter_radiance_sum.load(std::memory_order_relaxed))
    , m_previous_iter_radiance_sum(other.m_previous_iter_radiance_sum)
    , m_is_leaf(other.m_is_leaf)
{   
    if(!other.m_is_leaf)
    {
        m_upper_left_node.reset(new QuadTreeNode(*other.m_upper_left_node));
        m_upper_right_node.reset(new QuadTreeNode(*other.m_upper_right_node));
        m_lower_right_node.reset(new QuadTreeNode(*other.m_lower_right_node));
        m_lower_left_node.reset(new QuadTreeNode(*other.m_lower_left_node));
    }
}

void QuadTreeNode::add_radiance(
    Vector2f&                           direction,
    const float                         radiance)
{
    if(m_is_leaf)
        atomic_add(m_current_iter_radiance_sum, radiance);
    else
        choose_node(direction)->add_radiance(direction, radiance);
}

void QuadTreeNode::add_radiance(
    const AABB2f&                       splat_aabb,
    const AABB2f&                       node_aabb,
    const float                         radiance)
{
    const AABB2f intersection_aabb(AABB2f::intersect(splat_aabb, node_aabb));

    if(!intersection_aabb.is_valid())
        return;

    const float intersection_volume = intersection_aabb.volume();

    if(intersection_volume <= 0.0f)
        return;

    if(m_is_leaf)
    {
        atomic_add(m_current_iter_radiance_sum, radiance * intersection_volume);
    }
    else
    {
        // Create each child's AABB and recursively add radiance.
        const Vector2f node_size = node_aabb.extent();
        AABB2f child_aabb(node_aabb.min, node_aabb.min + 0.5f * node_size);
        m_upper_left_node->add_radiance(splat_aabb, child_aabb, radiance);
        
        child_aabb.translate(Vector2f(0.5f * node_size.x, 0.0f));
        m_upper_right_node->add_radiance(splat_aabb, child_aabb, radiance);

        child_aabb.translate(Vector2f(0.0f, 0.5f * node_size.x));
        m_lower_right_node->add_radiance(splat_aabb, child_aabb, radiance);

        child_aabb.translate(Vector2f(-0.5f * node_size.x, 0.0f));
        m_lower_left_node->add_radiance(splat_aabb, child_aabb, radiance);
    }
}

size_t QuadTreeNode::max_depth() const
{
    if(m_is_leaf)
        return 1;
    
    size_t max_child_depth = m_upper_left_node->max_depth();
    max_child_depth = std::max(m_upper_right_node->max_depth(), max_child_depth);
    max_child_depth = std::max(m_lower_right_node->max_depth(), max_child_depth);
    max_child_depth = std::max(m_lower_left_node->max_depth(), max_child_depth);
    return 1 + max_child_depth;
}

size_t QuadTreeNode::node_count() const
{
    if(m_is_leaf)
        return 1;
    
    return 1
        + m_upper_left_node->node_count()
        + m_upper_right_node->node_count()
        + m_lower_right_node->node_count()
        + m_lower_left_node->node_count();
}

float QuadTreeNode::radiance_sum() const
{
    return m_previous_iter_radiance_sum;
}

float QuadTreeNode::build_radiance_sums()
{
    if(m_is_leaf)
    {
        m_previous_iter_radiance_sum = m_current_iter_radiance_sum.load(std::memory_order_relaxed);
        return m_previous_iter_radiance_sum;
    }

    m_previous_iter_radiance_sum = 0.0f;
    m_previous_iter_radiance_sum += m_upper_left_node->build_radiance_sums();
    m_previous_iter_radiance_sum += m_upper_right_node->build_radiance_sums();
    m_previous_iter_radiance_sum += m_lower_right_node->build_radiance_sums();
    m_previous_iter_radiance_sum += m_lower_left_node->build_radiance_sums();
    return m_previous_iter_radiance_sum;
}

// Implementation of Algorithm 4 in Practical Path Guiding complementary PDF [Müller et.al. 2017]
// https://tom94.net/data/publications/mueller17practical/mueller17practical-supp.pdf

void QuadTreeNode::restructure(
    const float                         total_radiance_sum,
    const float                         subdiv_threshold,
    std::vector<
      std::pair<float, float>>*         sorted_energy_ratios,
    const size_t                        depth)
{   
    const float fraction = m_previous_iter_radiance_sum / total_radiance_sum;

    // Check if this node satisfies subdivision criterion.
    if(fraction > subdiv_threshold && depth < DTreeMaxDepth)
    {
        if(m_is_leaf)
        {
            // Create new children.
            m_is_leaf = false;
            const float quarter_sum = 0.25f * m_previous_iter_radiance_sum;
            m_upper_left_node.reset(new QuadTreeNode(false, quarter_sum));
            m_upper_right_node.reset(new QuadTreeNode(false, quarter_sum));
            m_lower_right_node.reset(new QuadTreeNode(false, quarter_sum));
            m_lower_left_node.reset(new QuadTreeNode(false, quarter_sum));
        }

        // Recursively ensure children satisfy subdivision criterion.
        m_upper_left_node->restructure(total_radiance_sum, subdiv_threshold, sorted_energy_ratios, depth + 1);
        m_upper_right_node->restructure(total_radiance_sum, subdiv_threshold, sorted_energy_ratios, depth + 1);
        m_lower_right_node->restructure(total_radiance_sum, subdiv_threshold, sorted_energy_ratios, depth + 1);
        m_lower_left_node->restructure(total_radiance_sum, subdiv_threshold, sorted_energy_ratios, depth + 1);            
    }
    else if(!m_is_leaf)
    {
        // If this interior node does not satisfy the subdivision criterion
        // revert it into a leaf.
        m_is_leaf = true;
        m_upper_left_node.reset(nullptr);
        m_upper_right_node.reset(nullptr);
        m_lower_right_node.reset(nullptr);
        m_lower_left_node.reset(nullptr);
    }

    if(sorted_energy_ratios != nullptr && !m_is_leaf && m_upper_left_node->m_is_leaf)
    {
        const std::pair<float, float> ratio(
            std::pow(0.25f, static_cast<float>(depth - 1)),
            4.0f * m_upper_left_node->radiance_sum() / total_radiance_sum);

        auto insert_pos = std::lower_bound(sorted_energy_ratios->cbegin(), sorted_energy_ratios->cend(), ratio);
        sorted_energy_ratios->insert(insert_pos, ratio);
    }

    m_current_iter_radiance_sum.store(0.0f, std::memory_order_relaxed);
}

void QuadTreeNode::reset()
{
    m_upper_left_node.reset(new QuadTreeNode(false));
    m_upper_right_node.reset(new QuadTreeNode(false));
    m_lower_right_node.reset(new QuadTreeNode(false));
    m_lower_left_node.reset(new QuadTreeNode(false));

    m_is_leaf = false;
    m_current_iter_radiance_sum.store(0.0f, std::memory_order_relaxed);
    m_previous_iter_radiance_sum = 0.0f;
}

// Implementation of Algorithm 2 in Practical Path Guiding complementary PDF [Müller et.al. 2017]
// https://tom94.net/data/publications/mueller17practical/mueller17practical-supp.pdf

float QuadTreeNode::pdf(
    Vector2f&                           direction) const
{
    return pdf_recursive(direction) / m_previous_iter_radiance_sum;
}

float QuadTreeNode::pdf_recursive(
    Vector2f&                           direction) const
{
    if(m_is_leaf)
        return m_previous_iter_radiance_sum;
    
    const QuadTreeNode* sub_node = choose_node(direction);
    return 4.0f * sub_node->pdf_recursive(direction);
}

const Vector2f QuadTreeNode::sample(
    Vector2f&                           sample,
    float&                              pdf) const
{
    pdf = 1.0f / m_previous_iter_radiance_sum; // initiate to one for recursive sampling routine
    return sample_recursive(sample, pdf);
}

// Implementation of Algorithm 1 in Practical Path Guiding complementary pdf [Müller et.al. 2017]
// https://tom94.net/data/publications/mueller17practical/mueller17practical-supp.pdf

const Vector2f QuadTreeNode::sample_recursive(
    Vector2f&                           sample,
    float&                              pdf) const
{
    assert(sample.x >= 0.0f && sample.x <= 1.0f);
    assert(sample.y >= 0.0f && sample.y <= 1.0f);

    // Ensure each sample dimension is < 1.0 after renormalization in previous recursive step.
    if(sample.x >= 1.0f)
        sample.x = std::nextafter(1.0f, 0.0f);

    if(sample.y >= 1.0f)
        sample.y = std::nextafter(1.0f, 0.0f);

    if(m_is_leaf)
    {
        pdf *= m_previous_iter_radiance_sum;
        return sample;
    }

    const float upper_left = m_upper_left_node->m_previous_iter_radiance_sum;
    const float upper_right = m_upper_right_node->m_previous_iter_radiance_sum;
    const float lower_right = m_lower_right_node->m_previous_iter_radiance_sum;
    const float lower_left = m_lower_left_node->m_previous_iter_radiance_sum;
    const float sum_left_half = upper_left + lower_left;
    const float sum_right_half = upper_right + lower_right;

    float factor = sum_left_half / m_previous_iter_radiance_sum;

    pdf *= 4.0f;
    
    // Sample child nodes with probability proportional to their energy.
    if(sample.x < factor)
    {
        sample.x /= factor;
        factor = upper_left / sum_left_half;

        if(sample.y < factor)
        {
            sample.y /= factor;
            const Vector2f sampled_direction =
                Vector2f(0.0f, 0.0f) + 0.5f * m_upper_left_node->sample_recursive(sample, pdf);

            return sampled_direction;
        }

        sample.y = (sample.y - factor) / (1.0f - factor);
        const Vector2f sampled_direction =
            Vector2f(0.0f, 0.5f) + 0.5f * m_lower_left_node->sample_recursive(sample, pdf);

        return sampled_direction;
    }
    else
    {
        sample.x = (sample.x - factor) / (1.0f - factor);
        factor = upper_right / sum_right_half;

        if (sample.y < factor)
        {
            sample.y /= factor;
            const Vector2f sampled_direction =
                Vector2f(0.5f, 0.0f) + 0.5f * m_upper_right_node->sample_recursive(sample, pdf);

            return sampled_direction;
        }

        sample.y = (sample.y - factor) / (1.0f - factor);
        const Vector2f sampled_direction =
            Vector2f(0.5f, 0.5f) + 0.5f * m_lower_right_node->sample_recursive(sample, pdf);

        return sampled_direction;
    }
}

size_t QuadTreeNode::depth(
    Vector2f&                           direction) const
{
    if(m_is_leaf)
        return 1;
    
    return 1 + choose_node(direction)->depth(direction);
}

QuadTreeNode* QuadTreeNode::choose_node(
    Vector2f&                           direction) const
{
    if(direction.x < 0.5f)
    {
        direction.x *= 2.0f;
        if(direction.y < 0.5f)
        {
            direction.y *= 2.0f;
            return m_upper_left_node.get();
        }
        else
        {
            direction.y = direction.y * 2.0f - 1.0f;
            return m_lower_left_node.get();
        }
    }
    else
    {
        direction.x = direction.x * 2.0f - 1.0f;
        if(direction.y < 0.5f)
        {
            direction.y *= 2.0f;
            return m_upper_right_node.get();
        }
        else
        {
            direction.y = direction.y * 2.0f - 1.0f;
            return m_lower_right_node.get();
        }
    }
}

void QuadTreeNode::flatten(
    std::list<VisualizerNode>&                nodes) const
{
    nodes.push_back({});
    VisualizerNode& node = nodes.back();

    node.sums[0] = m_upper_left_node->m_previous_iter_radiance_sum;
    if(m_upper_left_node->m_is_leaf)
    {
        node.children[0] = 0;
    }
    else
    {
        const size_t next_index = nodes.size();
        m_upper_left_node->flatten(nodes);
        node.children[0] = next_index;
    }

    node.sums[1] = m_upper_right_node->m_previous_iter_radiance_sum;
    if(m_upper_right_node->m_is_leaf)
    {
        node.children[1] = 0;
    }
    else
    {
        const size_t next_index = nodes.size();
        m_upper_right_node->flatten(nodes);
        node.children[1] = next_index;
    }

    node.sums[2] = m_lower_left_node->m_previous_iter_radiance_sum;
    if (m_lower_left_node->m_is_leaf)
    {
        node.children[2] = 0;
    }
    else
    {
        const size_t next_index = nodes.size();
        m_lower_left_node->flatten(nodes);
        node.children[2] = next_index;
    }

    node.sums[3] = m_lower_right_node->m_previous_iter_radiance_sum;
    if(m_lower_right_node->m_is_leaf)
    {
        node.children[3] = 0;
    }
    else
    {
        const size_t next_index = nodes.size();
        m_lower_right_node->flatten(nodes);
        node.children[3] = next_index;
    }
}

float QuadTreeNode::radiance(Vector2f& direction) const
{
    if (m_is_leaf)
        return m_previous_iter_radiance_sum;
    
    return 4.0f * choose_node(direction)->radiance(direction);
}

void QuadTreeNode::build_radiance_proxy(
    RadianceProxy&                      radiance_proxy,
    const float                         radiance_factor,
    const size_t                        proxy_width,
    const size_t                        end_level,
    const Vector2u                      origin,
    const size_t                        depth) const
{
    if (depth == end_level || m_is_leaf)
    {
        const size_t level_diff = end_level - depth;
        size_t width = 1;
        Vector2u pixel_origin = origin;

        for (size_t i = 0; i < level_diff; ++i)
        {
            width *= 2;
            pixel_origin *= static_cast<size_t>(2);
        }

        const float radiance = radiance_factor * m_previous_iter_radiance_sum;

        for (size_t y = 0; y < width; ++y)
        {
            for (size_t x = 0; x < width; ++x)
            {
                const Vector2u pixel = pixel_origin + Vector2u(x, y);
                const size_t pixel_index = pixel.y * RadianceProxy::ProxyWidth + pixel.x;

                assert (pixel_index >= 0);
                assert (pixel_index < RadianceProxy::ProxyWidth * RadianceProxy::ProxyWidth);
                radiance_proxy.m_map[pixel_index] = radiance;

                assert(radiance_proxy.m_quadtree_strata != nullptr);
                (*radiance_proxy.m_quadtree_strata)[pixel_index] = !m_is_leaf ? this : nullptr;
            }
        }
    }
    else
    {
        const Vector2u sub_node_origin = static_cast<size_t>(2) * origin;
        m_upper_left_node->build_radiance_proxy(
            radiance_proxy,
            radiance_factor * 4.0f,
            proxy_width,
            end_level,
            sub_node_origin,
            depth + 1);
        m_upper_right_node->build_radiance_proxy(
            radiance_proxy,
            radiance_factor * 4.0f,
            proxy_width,
            end_level,
            sub_node_origin + Vector2u(1, 0),
            depth + 1);
        m_lower_left_node->build_radiance_proxy(
            radiance_proxy,
            radiance_factor * 4.0f,
            proxy_width,
            end_level,
            sub_node_origin + Vector2u(0, 1),
            depth + 1);
        m_lower_right_node->build_radiance_proxy(
            radiance_proxy,
            radiance_factor * 4.0f,
            proxy_width,
            end_level,
            sub_node_origin + Vector2u(1, 1),
            depth + 1);
    }
}

// Helper class for ImageImportanceSampler generation.
template<std::size_t N>
class ImageSampler
{
  public:
    ImageSampler(
        const std::array<float, N * N>&       radiance_map)
        : m_radiance_map(radiance_map)
    {
    }

    void sample(const size_t x, const size_t y, float& payload, float& importance)
    {
        payload = importance = m_radiance_map[y * N + x];
    }

  private:
    const std::array<float, N * N>&   m_radiance_map;
};

// Radiance proxy implementation.

bool RadianceProxy::is_built() const
{
    return m_is_built;
}

void RadianceProxy::build(
    const QuadTreeNode&                 quadtree_root,
    const float                         radiance_scale)
{
    m_quadtree_strata = std::make_shared<std::array<const QuadTreeNode*, ProxyWidth * ProxyWidth>>();

    size_t end_level = 0;
    size_t map_width = ProxyWidth;

    while (map_width > 1)
    {
        ++end_level;
        map_width = map_width >> 1;
    }

    quadtree_root.build_radiance_proxy(
        (*this),
        radiance_scale,
        ProxyWidth,
        end_level);
    
    for (float& pixel_val : m_map)
    {
        if (pixel_val < 0.0f || std::isnan(pixel_val) || std::isinf(pixel_val))
            pixel_val = 0.0f;
    }

    m_is_built = true;
}

void RadianceProxy::build_product(
    BSDFProxy&                          bsdf_proxy,
    const Vector3f&                     outgoing,
    const Vector3f&                     shading_normal)
{
    assert(m_is_built);

    if (m_product_is_built)
        return;
    
    bsdf_proxy.finish_parameterization(outgoing, shading_normal);
    m_product_is_built = true;
    
    const float inv_width = 1.0f / ProxyWidth;
    for (size_t y = 0; y < ProxyWidth; ++y)
    {
        for (size_t x = 0; x < ProxyWidth; ++x)
        {
            const Vector2u pixel(x, y);
            const Vector2f cylindrical_direction(
                (x + 0.5f) * inv_width,
                (y + 0.5f) * inv_width);

            const Vector3f incoming = cylindrical_to_cartesian(cylindrical_direction);

            const size_t index = pixel.y * ProxyWidth + pixel.x;
            m_map[index] *= bsdf_proxy.evaluate(incoming);
        }
    }

    ImageSampler<ProxyWidth> image_sampler(m_map);
    m_image_importance_sampler.rebuild(image_sampler, nullptr);
}

float RadianceProxy::radiance(
    const Vector3f&                 direction) const
{
    return 0.0f;
    // const Vector2f spherical_direction(sphere_to_square(direction) * static_cast<float>(m_resolution));
    // const Vector2u pixel(
    //     std::min(static_cast<size_t>(spherical_direction.x), m_resolution - 1),
    //     std::min(static_cast<size_t>(spherical_direction.y), m_resolution - 1));
    
    // return (*m_high_res_map)[pixel.y * m_resolution + pixel.x];
}

float RadianceProxy::proxy_radiance(
    const Vector3f&                 direction) const
{
    const size_t map_width = ProxyWidth;
    const Vector2f spherical_direction(cartesian_to_cylindrical(direction) * static_cast<float>(map_width));
    const Vector2u pixel(
        std::min(static_cast<size_t>(spherical_direction.x), map_width - 1),
        std::min(static_cast<size_t>(spherical_direction.y), map_width - 1));
    
    return m_map[pixel.y * map_width + pixel.x];
}

RadianceProxy::RadianceProxy()
  : m_image_importance_sampler(ProxyWidth, ProxyWidth)
  , m_product_is_built(false)
  , m_is_built(false)
{}

RadianceProxy::RadianceProxy(
    const RadianceProxy&                    other)
  : m_map(other.m_map)
  , m_quadtree_strata(other.m_quadtree_strata)
  , m_image_importance_sampler(ProxyWidth, ProxyWidth)
  , m_product_is_built(false)
  , m_is_built(other.m_is_built)
{}

float RadianceProxy::sample(
    SamplingContext&                        sampling_context,
    foundation::Vector3f&                   direction) const
{
    assert (m_is_built);
    // Sample the importance map.
    sampling_context.split_in_place(2, 1);
    Vector2f s = sampling_context.next2<Vector2f>();
    Vector2u pixel;
    float payload;
    float pdf;
    m_image_importance_sampler.sample(s, pixel.x, pixel.y, payload, pdf);
    assert(pdf >= 0.0f);

    Vector2f cylindrical_direction(pixel.x, pixel.y);
    sampling_context.split_in_place(2, 1);
    s = sampling_context.next2<Vector2f>();

    assert (m_quadtree_strata != nullptr);
    assert (m_quadtree_strata->size() == ProxyWidth * ProxyWidth);
    assert (pixel.y * ProxyWidth + pixel.x < ProxyWidth * ProxyWidth && pixel.y * ProxyWidth + pixel.x >= 0);
    const QuadTreeNode* sub_tree = (*m_quadtree_strata)[pixel.y * ProxyWidth + pixel.x];

    if (sub_tree)
    {
        float tree_pdf;
        cylindrical_direction += sub_tree->sample(s, tree_pdf);
        pdf *= tree_pdf;
    }
    else
    {
        cylindrical_direction += s;
    }

    pdf *= ProxyWidth * ProxyWidth * RcpFourPi<float>();
    cylindrical_direction *= 1.0f / ProxyWidth;
    // assert(cylindrical_direction.x >= 0.0f && cylindrical_direction.x < 1.0f);
    // assert(cylindrical_direction.y >= 0.0f && cylindrical_direction.y < 1.0f);
    cylindrical_direction.x = std::min(cylindrical_direction.x, 0.99999f);
    cylindrical_direction.y = std::min(cylindrical_direction.y, 0.99999f);
    cylindrical_direction = clamp(cylindrical_direction, 0.0f, 1.0f);
    direction = cylindrical_to_cartesian(cylindrical_direction);

    return pdf;
}

float RadianceProxy::pdf(
    const Vector3f&             direction) const
{
    assert(m_is_built);

    const Vector2f cylindrical_direction = cartesian_to_cylindrical(direction) * static_cast<float>(ProxyWidth);
    Vector2u pixel(
        truncate<size_t>(cylindrical_direction.x),
        truncate<size_t>(cylindrical_direction.y));

    pixel.x = std::min(pixel.x, static_cast<size_t>(15));
    pixel.y = std::min(pixel.y, static_cast<size_t>(15));

    // TODO: More precise mapping between directions and map pixels to avoid discrepancies in sampled
    // and evaluated pdf values. There also seems to be another source causing these discrepancies.

    float pdf = m_image_importance_sampler.get_pdf(pixel.x, pixel.y);

    assert (m_quadtree_strata != nullptr);
    assert (m_quadtree_strata->size() == ProxyWidth * ProxyWidth);
    assert (pixel.y * ProxyWidth + pixel.x >= 0);
    assert (pixel.y * ProxyWidth + pixel.x < ProxyWidth * ProxyWidth);
    const QuadTreeNode* sub_tree = (*m_quadtree_strata)[pixel.y * ProxyWidth + pixel.x];

    if (sub_tree)
    {
        Vector2f sub_direction(cylindrical_direction.x - pixel.x, cylindrical_direction.y - pixel.y);
        pdf *= sub_tree->pdf(sub_direction);
    }

    pdf *= ProxyWidth * ProxyWidth * RcpFourPi<float>();
    return pdf;
}

struct DTreeRecord
{
    Vector3f                    direction;
    float                       radiance;
    float                       wi_pdf;
    float                       bsdf_pdf;
    float                       d_tree_pdf;
    float                       product_pdf;
    float                       sample_weight;
    float                       product;
    bool                        is_delta;
    GuidingMethod               guiding_method;
};

// DTree implementation.

DTree::DTree(
    const GPTParameters&                parameters)
    : m_parameters(parameters)
    , m_root_node(true)
    , m_current_iter_sample_weight(0.0f)
    , m_previous_iter_sample_weight(0.0f)
    , m_optimization_step_count(0)
    , m_first_moment(0.0f)
    , m_second_moment(0.0f)
    , m_theta(0.0f)
    , m_optimization_step_count_product(0)
    , m_first_moment_product(0.0f)
    , m_second_moment_product(0.0f)
    , m_theta_product(0.0f)
    , m_is_built(false)
    , m_scattering_mode(ScatteringMode::Diffuse)
{
    m_atomic_flag.clear(std::memory_order_release);
    m_atomic_flag_product.clear(std::memory_order_release);
}

DTree::DTree(
    const DTree&                        other)
    : m_parameters(other.m_parameters)
    , m_current_iter_sample_weight(other.m_current_iter_sample_weight.load(std::memory_order_relaxed))
    , m_previous_iter_sample_weight(other.m_previous_iter_sample_weight)
    , m_root_node(other.m_root_node)
    , m_optimization_step_count(other.m_optimization_step_count)
    , m_first_moment(other.m_first_moment)
    , m_second_moment(other.m_second_moment)
    , m_theta(other.m_theta)
    , m_optimization_step_count_product(other.m_optimization_step_count_product)
    , m_first_moment_product(other.m_first_moment_product)
    , m_second_moment_product(other.m_second_moment_product)
    , m_theta_product(other.m_theta_product)
    , m_is_built(other.m_is_built)
    , m_scattering_mode(other.m_scattering_mode)
{
    m_atomic_flag.clear(std::memory_order_release);
    m_atomic_flag_product.clear(std::memory_order_release);
}

void DTree::record(
    const DTreeRecord&                  d_tree_record)
{
    if(m_parameters.m_bsdf_sampling_fraction_mode == BSDFSamplingFractionMode::Learn && m_is_built && d_tree_record.product > 0.0f)
    {
        if (d_tree_record.guiding_method == GuidingMethod::PathGuiding)
            optimization_step(d_tree_record);
        else
            optimization_step_product(d_tree_record);
    }
        
    if(d_tree_record.is_delta || d_tree_record.wi_pdf <= 0.0f)
        return;

    atomic_add(m_current_iter_sample_weight, d_tree_record.sample_weight);

    const float radiance = d_tree_record.radiance / d_tree_record.wi_pdf * d_tree_record.sample_weight;
    
    Vector2f direction = cartesian_to_cylindrical(d_tree_record.direction);

    switch (m_parameters.m_directional_filter)
    {
    case DirectionalFilter::Nearest:
        m_root_node.add_radiance(direction, radiance);
        break;

    case DirectionalFilter::Box:
    {
        // Determine the node size at the direction.
        const size_t leaf_depth = depth(direction);
        const Vector2f leaf_size(std::pow(0.25f, static_cast<float>(leaf_depth - 1)));
        const AABB2f node_aabb(Vector2f(0.0f), Vector2f(1.0f));
        const AABB2f splat_aabb(direction - 0.5f * leaf_size, direction + 0.5f * leaf_size);

        if(!splat_aabb.is_valid())
            return;

        m_root_node.add_radiance(splat_aabb, node_aabb, radiance / splat_aabb.volume());
        break;
    }
    default:
        break;
    }
}

void DTree::sample(
    SamplingContext&                    sampling_context,
    DTreeSample&                        d_tree_sample,
    const int                           modes) const
{
    if((modes & m_scattering_mode) == 0)
    {
        d_tree_sample.scattering_mode = ScatteringMode::None;
        d_tree_sample.pdf = 0.0f;
        return;
    }

    sampling_context.split_in_place(2, 1);
    Vector2f s = sampling_context.next2<Vector2f>();

    if (m_previous_iter_sample_weight <= 0.0f || m_root_node.radiance_sum() <= 0.0f)
    {
        d_tree_sample.direction = sample_sphere_uniform(s);
        d_tree_sample.pdf = RcpFourPi<float>();
        d_tree_sample.scattering_mode = ScatteringMode::Diffuse;
    }
    else
    {
        const Vector2f direction = m_root_node.sample(s, d_tree_sample.pdf);
        d_tree_sample.pdf *= RcpFourPi<float>();
        d_tree_sample.direction = cylindrical_to_cartesian(direction);
        d_tree_sample.scattering_mode = m_scattering_mode;
    }
}

float DTree::pdf(
    const Vector3f&                     direction,
    const int                           modes) const
{
    if ((modes & m_scattering_mode) == 0)
        return 0.0f;

    if(m_previous_iter_sample_weight <= 0.0f || m_root_node.radiance_sum() <= 0.0f)
        return RcpFourPi<float>();

    Vector2f dir = cartesian_to_cylindrical(direction);
    return m_root_node.pdf(dir) * RcpFourPi<float>();
}

void DTree::halve_sample_weight()
{
    m_current_iter_sample_weight = 0.5f * m_current_iter_sample_weight.load(std::memory_order_relaxed);
}

size_t DTree::node_count() const
{
    return m_root_node.node_count();
}

size_t DTree::max_depth() const
{
    return m_root_node.max_depth();
}

size_t DTree::depth(
    const Vector2f&                     direction) const
{
    Vector2f local_direction = direction;

    return m_root_node.depth(local_direction);
}

ScatteringMode::Mode DTree::get_scattering_mode() const
{
    return m_scattering_mode;
}

void DTree::build()
{
    m_previous_iter_sample_weight = m_current_iter_sample_weight.load(std::memory_order_relaxed);
    m_root_node.build_radiance_sums();
}

void DTree::restructure(
    const float                         subdiv_threshold)
{
    m_is_built = true;
    m_current_iter_sample_weight.store(0.0f, std::memory_order_relaxed);
    const float radiance_sum = m_root_node.radiance_sum();
    m_radiance_proxy.m_is_built = false;

    // Reset D-Trees that did not collect radiance.
    if(radiance_sum <= 0.0f)
    {
        m_root_node.reset();
        m_scattering_mode = ScatteringMode::Diffuse;
        m_optimization_step_count = 0;
        m_first_moment = 0.0f;
        m_second_moment = 0.0f;
        m_theta = 0.0f;
        m_optimization_step_count_product = 0;
        m_first_moment_product = Vector2f(0.0f);
        m_second_moment_product = Vector2f(0.0f);
        m_theta_product = Vector2f(0.0f);
        m_mip_maps.clear();
        m_mip_maps.push_back(std::vector<float>{0.0f});
        return;
    }

    std::vector<std::pair<float, float>> sorted_energy_ratios;
    m_root_node.restructure(
        radiance_sum, subdiv_threshold,
        m_parameters.m_guided_bounce_mode == GuidedBounceMode::Learn ?
            &sorted_energy_ratios : nullptr);

    // Determine what ScatteringMode should be assigned to directions sampled from this D-tree.
    if(m_parameters.m_guided_bounce_mode == GuidedBounceMode::Learn)
    {
        float area_fraction_sum = 0.0f;
        float energy_fraction_sum = 0.0f;
        bool is_glossy = false;
        auto itr = sorted_energy_ratios.cbegin();

        while(itr != sorted_energy_ratios.cend() && area_fraction_sum + itr->first < DTreeGlossyAreaFraction)
        {
            area_fraction_sum += itr->first;
            energy_fraction_sum += itr->second;

            // If a significant part of the energy is stored in a small subset of directions
            // treat bounces as Glossy, otherwise treat them as Diffuse.
            if(energy_fraction_sum > DTreeGlossyEnergyThreshold)
            {
                is_glossy = true;
                break;
            }

            ++itr;
        }

        m_scattering_mode = is_glossy ? ScatteringMode::Glossy : ScatteringMode::Diffuse;
    }

    m_radiance_proxy.build(
        m_root_node,
        RcpFourPi<float>() / m_previous_iter_sample_weight);
}

float DTree::sample_weight() const
{
    return m_current_iter_sample_weight.load(std::memory_order_relaxed);
}

float DTree::mean() const
{
    if (m_previous_iter_sample_weight <= 0.0f)
        return 0.0f;

    return m_root_node.radiance_sum() * (1.0f / m_previous_iter_sample_weight) * RcpFourPi<float>();
}

float DTree::radiance(
    const foundation::Vector3f&         direction) const
{
    if (m_root_node.radiance_sum() <= 0.0f || m_previous_iter_sample_weight <= 0.0)
        return 0.0f;
    
    Vector2f cylindrical_direction = cartesian_to_cylindrical(direction);
    return m_root_node.radiance(cylindrical_direction) / (foundation::FourPi<float>() * m_previous_iter_sample_weight);
}

float DTree::bsdf_sampling_fraction() const
{
    if(m_parameters.m_bsdf_sampling_fraction_mode == BSDFSamplingFractionMode::Learn)
        return logistic(m_theta);
    else
        return m_parameters.m_fixed_bsdf_sampling_fraction;
}

Vector2f DTree::bsdf_sampling_fraction_product() const
{
    if(m_parameters.m_bsdf_sampling_fraction_mode == BSDFSamplingFractionMode::Learn)
        return Vector2f(
            logistic(m_theta_product.x), logistic(m_theta_product.y));
    else
        return Vector2f(0.33333f, 0.5f); // TODO: Meaningful parameters
}

void DTree::acquire_optimization_spin_lock()
{
    while(m_atomic_flag.test_and_set(std::memory_order_acquire))
        ;
}

void DTree::release_optimization_spin_lock()
{
    m_atomic_flag.clear(std::memory_order_release);
}

void DTree::acquire_optimization_spin_lock_product()
{
    while(m_atomic_flag_product.test_and_set(std::memory_order_acquire))
        ;
}

void DTree::release_optimization_spin_lock_product()
{
    m_atomic_flag_product.clear(std::memory_order_release);
}

// BSDF sampling fraction optimization procedure.
// Implementation of Algorithm 3 in chapter "Practical Path Guiding in Production" [Müller 2019]
// released in "Path Guiding in Production" Siggraph Course 2019, [Vorba et. al. 2019]
// Implements the stochastic-gradient-based Adam optimizer [Kingma and Ba 2014]

void DTree::optimization_step(
    const DTreeRecord&                  d_tree_record)
{
    acquire_optimization_spin_lock();

    const float sampling_fraction = bsdf_sampling_fraction();
    const float combined_pdf = sampling_fraction * d_tree_record.bsdf_pdf +
                               (1.0f - sampling_fraction) * d_tree_record.d_tree_pdf;

    const float d_sampling_fraction = -d_tree_record.product *
                                      (d_tree_record.bsdf_pdf - d_tree_record.d_tree_pdf) /
                                      (d_tree_record.wi_pdf * combined_pdf);

    const float d_theta = d_sampling_fraction * sampling_fraction * (1.0f - sampling_fraction);
    const float reg_gradient = m_theta * Regularization;
    const float gradient = (d_theta + reg_gradient) * d_tree_record.sample_weight;

    adam_step(gradient);

    release_optimization_spin_lock();
}

void DTree::adam_step(
    const float                         gradient)
{
    ++m_optimization_step_count;
    const float debiased_learning_rate = m_parameters.m_learning_rate *
                                         std::sqrt(1.0f - std::pow(Beta2, static_cast<float>(m_optimization_step_count))) /
                                         (1.0f - std::pow(Beta1, static_cast<float>(m_optimization_step_count)));

    m_first_moment = Beta1 * m_first_moment + (1.0f - Beta1) * gradient;
    m_second_moment = Beta2 * m_second_moment + (1.0f - Beta2) * gradient * gradient;
    m_theta -= debiased_learning_rate * m_first_moment / (std::sqrt(m_second_moment) + OptimizationEpsilon);

    m_theta = clamp(m_theta, -20.0f, 20.0f);
}

void DTree::optimization_step_product(
    const DTreeRecord&                  d_tree_record)
{
    acquire_optimization_spin_lock_product();

    const Vector2f sampling_fraction = bsdf_sampling_fraction_product();
    const float combined_pdf = sampling_fraction.x * d_tree_record.bsdf_pdf +
                               (1.0f - sampling_fraction.x) * (sampling_fraction.y * d_tree_record.d_tree_pdf +
                               (1.0f - sampling_fraction.y) * d_tree_record.product_pdf);

    Vector2f d_sampling_fraction(-d_tree_record.product / (d_tree_record.wi_pdf * combined_pdf)); // common factor

    d_sampling_fraction.x *= d_tree_record.bsdf_pdf - (sampling_fraction.y * d_tree_record.d_tree_pdf +
                               (1.0f - sampling_fraction.y) * d_tree_record.product_pdf);
    d_sampling_fraction.y *= (1.0f - sampling_fraction.x) * (d_tree_record.product_pdf - d_tree_record.d_tree_pdf);

    Vector2f d_theta = d_sampling_fraction * sampling_fraction * (Vector2f(1.0f) - sampling_fraction);

    const Vector2f reg_gradient = m_theta_product * Regularization;
    const Vector2f gradient = (d_theta + reg_gradient) * d_tree_record.sample_weight;

    adam_step_product(gradient);

    release_optimization_spin_lock_product();
}

void DTree::adam_step_product(
    const Vector2f                      gradient)
{
    ++m_optimization_step_count_product;
    const float debiased_learning_rate = m_parameters.m_learning_rate *
                                         std::sqrt(1.0f - std::pow(Beta2, static_cast<float>(m_optimization_step_count_product))) /
                                         (1.0f - std::pow(Beta1, static_cast<float>(m_optimization_step_count_product)));

    m_first_moment_product = Beta1 * m_first_moment_product + (1.0f - Beta1) * gradient;
    m_second_moment_product = Beta2 * m_second_moment_product + (1.0f - Beta2) * gradient * gradient;
    const Vector2f sqrt_second_moment(std::sqrt(m_second_moment_product.x), std::sqrt(m_second_moment_product.y));
    m_theta_product -= debiased_learning_rate * m_first_moment_product / (sqrt_second_moment + Vector2f(OptimizationEpsilon));

    m_theta_product = clamp(m_theta_product, -20.0f, 20.0f);
}

void DTree::write_to_disk(
    std::ofstream&                      os) const
{
    std::list<VisualizerNode> nodes;
    m_root_node.flatten(nodes);

    write(os, mean());
    write(os, static_cast<uint64_t>(sample_weight()));
    write(os, static_cast<uint64_t>(nodes.size()));

    for(const auto& n : nodes)
        for(int i = 0; i < 4; ++i)
        {
            write(os, n.sums[i]);
            write(os, static_cast<uint16_t>(n.children[i]));
        }
}

const RadianceProxy& DTree::get_radiance_proxy() const
{
    return m_radiance_proxy;
}

// Struct used to gather SD-tree statistics.

struct DTreeStatistics
{
    DTreeStatistics()
      : num_d_trees(0)
      , min_d_tree_depth(std::numeric_limits<size_t>::max())
      , max_d_tree_depth(0)
      , average_d_tree_depth(0.0f)
      , min_d_tree_nodes(std::numeric_limits<size_t>::max())
      , max_d_tree_nodes(0)
      , average_d_tree_nodes(0.0f)
      , min_sample_weight(std::numeric_limits<float>::max())
      , max_sample_weight(0.0f)
      , average_sample_weight(0.0f)
      , min_sampling_fraction(std::numeric_limits<float>::max())
      , max_sampling_fraction(0.0f)
      , average_sampling_fraction(0.0f)
      , min_mean_radiance(std::numeric_limits<float>::max())
      , max_mean_radiance(0.0f)
      , average_mean_radiance(0.0f)
      , glossy_d_tree_fraction(0.0f)
      , num_s_tree_nodes(0)
      , min_s_tree_depth(std::numeric_limits<size_t>::max())
      , max_s_tree_depth(0)
      , average_s_tree_depth(0.0f)
    {}

    size_t                              num_d_trees;
    size_t                              min_d_tree_depth;
    size_t                              max_d_tree_depth;
    float                               average_d_tree_depth;
    size_t                              min_d_tree_nodes;
    size_t                              max_d_tree_nodes;
    float                               average_d_tree_nodes;
    float                               min_sample_weight;
    float                               max_sample_weight;
    float                               average_sample_weight;
    float                               min_sampling_fraction;
    float                               max_sampling_fraction;
    float                               average_sampling_fraction;
    float                               min_mean_radiance;
    float                               max_mean_radiance;
    float                               average_mean_radiance;
    float                               glossy_d_tree_fraction;

    size_t                              num_s_tree_nodes;
    size_t                              min_s_tree_depth;
    size_t                              max_s_tree_depth;
    float                               average_s_tree_depth;

    void build()
    {
        if(num_d_trees <= 0)
            return;

        average_d_tree_depth /= num_d_trees;
        average_s_tree_depth /= num_d_trees;
        average_d_tree_nodes /= num_d_trees;
        average_mean_radiance /= num_d_trees;
        average_sample_weight /= num_d_trees;
        glossy_d_tree_fraction /= num_d_trees;
        average_sampling_fraction /= num_d_trees;
    }
};

// STreeNode implementation.

STreeNode::STreeNode(
    const GPTParameters&                parameters)
    : m_axis(0)
    , m_d_tree(new DTree(parameters))
{}

STreeNode::STreeNode(
    const unsigned int                  parent_axis,
    const DTree*                        parent_d_tree)
    : m_axis((parent_axis + 1) % 3)
    , m_d_tree(new DTree(*parent_d_tree))
{
    m_d_tree->halve_sample_weight();
}

DTree* STreeNode::get_d_tree(
    Vector3f&                           point,
    Vector3f&                           size)
{
    if(is_leaf())
        return m_d_tree.get();
    else
    {
        size[m_axis] *= 0.5f;
        return choose_node(point)->get_d_tree(point, size);
    }
}

// Implementation of Algorithm 3 in Practical Path Guiding complementary pdf [Müller et.al. 2017]
// https://tom94.net/data/publications/mueller17practical/mueller17practical-supp.pdf

void STreeNode::subdivide(
    const size_t                        required_samples)
{
    if(is_leaf())
    {
        if (m_d_tree->sample_weight() > required_samples)
            subdivide();
        else
            return;
    }
    
    m_first_node->subdivide(required_samples);
    m_second_node->subdivide(required_samples);
}

void STreeNode::subdivide()
{
    if(is_leaf())
    {
        m_first_node.reset(new STreeNode(m_axis, m_d_tree.get()));
        m_second_node.reset(new STreeNode(m_axis, m_d_tree.get()));
        m_d_tree.reset(nullptr);
    }
}

void STreeNode::record(
    const AABB3f&                       splat_aabb,
    const AABB3f&                       node_aabb,
    const DTreeRecord&                  d_tree_record)
{
    const AABB3f intersection_aabb(AABB3f::intersect(splat_aabb, node_aabb));

    if(!intersection_aabb.is_valid())
        return;

    const float intersection_volume = intersection_aabb.volume();

    if(intersection_volume <= 0.0f)
        return;

    if(is_leaf())
    {
        m_d_tree->record(
            DTreeRecord{
                d_tree_record.direction,
                d_tree_record.radiance,
                d_tree_record.wi_pdf,
                d_tree_record.bsdf_pdf,
                d_tree_record.d_tree_pdf,
                d_tree_record.product_pdf,
                d_tree_record.sample_weight * intersection_volume,
                d_tree_record.product,
                d_tree_record.is_delta,
                d_tree_record.guiding_method});
    }
    else
    {
        const Vector3f node_size = node_aabb.extent();
        Vector3f offset(0.0f);
        offset[m_axis] = node_size[m_axis] * 0.5f;

        m_first_node->record(splat_aabb, AABB3f(node_aabb.min, node_aabb.max - offset), d_tree_record);
        m_second_node->record(splat_aabb, AABB3f(node_aabb.min + offset, node_aabb.max), d_tree_record);
    }
}

struct RestructureJob : public IJob
{
    RestructureJob(
        DTree*                          d_tree,
        const float                     subdiv_threshold)
      : m_d_tree(d_tree)
      , m_subdiv_threshold(subdiv_threshold)
    {}

    void execute(const size_t thread_index) override
    {
        m_d_tree->restructure(m_subdiv_threshold);
    }

  private:
    DTree*      m_d_tree;
    const float m_subdiv_threshold;
};

void STreeNode::restructure(
    const float                         subdiv_threshold,
    JobQueue&                           jobqueue)
{
    if(is_leaf())
    {
        jobqueue.schedule(new RestructureJob(m_d_tree.get(), subdiv_threshold));
    }
    else
    {
        m_first_node->restructure(subdiv_threshold, jobqueue);
        m_second_node->restructure(subdiv_threshold, jobqueue);
    }
}

void STreeNode::build()
{
    if(is_leaf())
    {
        m_d_tree->build();
    }
    else
    {
        m_first_node->build();
        m_second_node->build();
    }
}

void STreeNode::gather_statistics(
    DTreeStatistics&                    statistics,
    const size_t                        depth) const
{
    statistics.num_s_tree_nodes++;
    if(is_leaf())
    {
        ++statistics.num_d_trees;
        const size_t d_tree_depth = m_d_tree->max_depth();
        statistics.max_d_tree_depth = std::max(statistics.max_d_tree_depth, d_tree_depth);
        statistics.min_d_tree_depth = std::min(statistics.min_d_tree_depth, d_tree_depth);
        statistics.average_d_tree_depth += d_tree_depth;

        const float mean_radiance = m_d_tree->mean();
        statistics.max_mean_radiance = std::max(statistics.max_mean_radiance, mean_radiance);
        statistics.min_mean_radiance = std::min(statistics.min_mean_radiance, mean_radiance);
        statistics.average_mean_radiance += mean_radiance;

        const size_t node_count = m_d_tree->node_count();
        statistics.max_d_tree_nodes = std::max(statistics.max_d_tree_nodes, node_count);
        statistics.min_d_tree_nodes = std::min(statistics.min_d_tree_nodes, node_count);
        statistics.average_d_tree_nodes += node_count;

        const float sample_weight = m_d_tree->sample_weight();
        statistics.max_sample_weight = std::max(statistics.max_sample_weight, sample_weight);
        statistics.min_sample_weight = std::min(statistics.min_sample_weight, sample_weight);
        statistics.average_sample_weight += sample_weight;

        if(m_d_tree->get_scattering_mode() == ScatteringMode::Glossy)
            statistics.glossy_d_tree_fraction += 1.0f;
        
        const float bsdf_sampling_fraction = m_d_tree->bsdf_sampling_fraction();
        statistics.min_sampling_fraction = std::min(statistics.min_sampling_fraction, bsdf_sampling_fraction);
        statistics.max_sampling_fraction = std::max(statistics.max_sampling_fraction, bsdf_sampling_fraction);
        statistics.average_sampling_fraction += bsdf_sampling_fraction;

        statistics.max_s_tree_depth = std::max(statistics.max_s_tree_depth, depth);
        statistics.min_s_tree_depth = std::min(statistics.min_s_tree_depth, depth);
        statistics.average_s_tree_depth += depth;
    }
    else
    {
        m_first_node->gather_statistics(statistics, depth + 1);
        m_second_node->gather_statistics(statistics, depth + 1);
    }
}

STreeNode* STreeNode::choose_node(
    Vector3f&                           point) const
{
    if(point[m_axis] < 0.5f)
    {
        point[m_axis] *= 2.0f;
        return m_first_node.get();
    }
    else
    {
        point[m_axis] = (point[m_axis] - 0.5f) * 2.0f;
        return m_second_node.get();
    }
}

bool STreeNode::is_leaf() const
{
    return m_d_tree != nullptr;
}

void STreeNode::write_to_disk(
    std::ofstream&                      os,
    const AABB3f&                       aabb) const
{
    if(is_leaf())
    {
        if(m_d_tree->sample_weight() > 0.0)
        {
            const Vector3f extent = aabb.extent();
            write(os, aabb.min.x);
            write(os, aabb.min.y);
            write(os, aabb.min.z);
            write(os, extent.x);
            write(os, extent.y);
            write(os, extent.z);

            m_d_tree->write_to_disk(os);
        }
    }
    else
    {
        AABB3f child_aabb(aabb);
        const float half_extent = 0.5f * aabb.extent()[m_axis];
        child_aabb.max[m_axis] -= half_extent;
        m_first_node->write_to_disk(os, child_aabb);

        child_aabb.min[m_axis] += half_extent;
        child_aabb.max[m_axis] += half_extent;
        m_second_node->write_to_disk(os, child_aabb);
    }
}

// STree implementation.

STree::STree(
    const Scene&                        scene,
    const GPTParameters&                parameters)
    : m_parameters(parameters)
    , m_scene_aabb(scene.compute_bbox())
    , m_is_built(false)
    , m_is_final_iteration(false)
    , m_scene(scene)
{
    m_root_node.reset(new STreeNode(m_parameters));

    // Grow the AABB into a cube for nicer hierarchical subdivisions [Müller et. al. 2017].
    const Vector3f size = m_scene_aabb.extent();
    const float maxSize = max_value(size);
    m_scene_aabb.max = m_scene_aabb.min + Vector3f(maxSize);
}

DTree* STree::get_d_tree(
    const Vector3f&                     point,
    Vector3f&                           d_tree_voxel_size)
{
    d_tree_voxel_size = m_scene_aabb.extent();
    Vector3f transformed_point = point - m_scene_aabb.min;
    transformed_point /= d_tree_voxel_size;

    return m_root_node->get_d_tree(transformed_point, d_tree_voxel_size);
}

DTree* STree::get_d_tree(
    const Vector3f&                     point)
{
    Vector3f d_tree_voxel_size;
    return get_d_tree(point, d_tree_voxel_size);
}

void STree::record(
    DTree*                              d_tree,
    const Vector3f&                     point,
    const Vector3f&                     d_tree_node_size,
    DTreeRecord&                        d_tree_record,
    SamplingContext&                    sampling_context)
{
    assert(std::isfinite(d_tree_record.radiance));
    assert(d_tree_record.radiance >= 0.0f);
    assert(std::isfinite(d_tree_record.product));
    assert(d_tree_record.product >= 0.0f);
    assert(std::isfinite(d_tree_record.sample_weight));
    assert(d_tree_record.sample_weight >= 0.0f);

    switch (m_parameters.m_spatial_filter)
    {
    case SpatialFilter::Nearest:
        d_tree->record(d_tree_record);
        break;

    case SpatialFilter::Stochastic:
    {
        // Jitter the position of the record.
        sampling_context.split_in_place(3, 1);

        Vector3f offset = d_tree_node_size;
        offset *= (sampling_context.next2<Vector3f>() - Vector3f(0.5f));
        Vector3f jittered_point = clip_vector_to_aabb(point + offset);

        DTree* stochastic_d_tree = get_d_tree(jittered_point);
        stochastic_d_tree->record(d_tree_record);
        break;
    }

    case SpatialFilter::Box:
        box_filter_splat(point, d_tree_node_size, d_tree_record);
        break;
    }
}

void STree::build(
    const size_t                        iteration)
{
    // Build D-tree radiance and sample weight sums first.
    m_root_node->build();

    const size_t required_samples = static_cast<size_t>(SpatialSubdivisionThreshold * std::pow(2.0f, iteration * 0.5f));

    // First refine the S-tree then refine the D-tree at each spatial leaf.
    m_root_node->subdivide(required_samples);

    JobQueue jobqueue;
    JobManager jobmanager(global_logger(), jobqueue, 12, JobManager::KeepRunningOnEmptyQueue);
    jobmanager.start();
    m_root_node->restructure(DTreeThreshold, jobqueue);
    jobqueue.wait_until_completion();

    DTreeStatistics statistics;
    m_root_node->gather_statistics(statistics);
    statistics.build();

    RENDERER_LOG_INFO(
        "SD-Tree statistics: [min, max, avg]\n"
        "S-Tree:\n"
        "  Node Count                   = %s\n"
        "  S-Tree depth                 = [%s, %s, %s]\n"
        "D-Tree:\n"
        "  Tree Count                   = %s\n"
        "  Node Count                   = [%s, %s, %s]\n"
        "  D-Tree Depth                 = [%s, %s, %s]\n"
        "  Mean Radiance                = [%s, %s, %s]\n"
        "  Sample Weight                = [%s, %s, %s]\n"
        "  BSDF Sampling Fraction       = [%s, %s, %s]\n"
        "  Glossy D-Tree Fraction       = %s\n",
        pretty_uint(statistics.num_s_tree_nodes).c_str(),
        pretty_uint(statistics.min_s_tree_depth).c_str(),
        pretty_uint(statistics.max_s_tree_depth).c_str(),
        pretty_scalar(statistics.average_s_tree_depth, 2).c_str(),
        pretty_uint(statistics.num_d_trees).c_str(),
        pretty_uint(statistics.min_d_tree_nodes).c_str(),
        pretty_uint(statistics.max_d_tree_nodes).c_str(),
        pretty_scalar(statistics.average_d_tree_nodes, 1).c_str(),
        pretty_uint(statistics.min_d_tree_depth).c_str(),
        pretty_uint(statistics.max_d_tree_depth).c_str(),
        pretty_scalar(statistics.average_d_tree_depth, 2).c_str(),
        pretty_scalar(statistics.min_mean_radiance, 3).c_str(),
        pretty_scalar(statistics.max_mean_radiance, 3).c_str(),
        pretty_scalar(statistics.average_mean_radiance, 3).c_str(),
        pretty_scalar(statistics.min_sample_weight, 3).c_str(),
        pretty_scalar(statistics.max_sample_weight, 3).c_str(),
        pretty_scalar(statistics.average_sample_weight, 3).c_str(),
        pretty_scalar(statistics.min_sampling_fraction, 3).c_str(),
        pretty_scalar(statistics.max_sampling_fraction, 3).c_str(),
        pretty_scalar(statistics.average_sampling_fraction, 3).c_str(),
        pretty_scalar(statistics.glossy_d_tree_fraction, 3).c_str());

    m_is_built = true;
}

bool STree::is_built() const
{
    return m_is_built;
}

void STree::start_final_iteration()
{
    m_is_final_iteration = true;
}

bool STree::is_final_iteration() const
{
    return m_is_final_iteration;
}

void STree::box_filter_splat(
    const Vector3f&                     point,
    const Vector3f&                     d_tree_node_size,
    DTreeRecord&                        d_tree_record)
{
    const AABB3f splat_aabb(point - d_tree_node_size * 0.5f, point + d_tree_node_size * 0.5f);

    assert(splat_aabb.is_valid() && splat_aabb.volume() > 0.0f);

    d_tree_record.sample_weight /= splat_aabb.volume();
    m_root_node->record(AABB3f(point - d_tree_node_size * 0.5f, point + d_tree_node_size * 0.5f), m_scene_aabb, d_tree_record);
}

Vector3f STree::clip_vector_to_aabb(
    const Vector3f&                     point)
{
    Vector3f result = point;
    for (int i = 0; i < Vector3f::Dimension; ++i)
    {
        result[i] = std::min(std::max(result[i], m_scene_aabb.min[i]), m_scene_aabb.max[i]);
    }
    return result;
}

void STree::write_to_disk(
    const size_t                        iteration,
    const bool                          append_iteration) const
{
    std::string file_path = m_parameters.m_save_path;

    if(append_iteration)
    {
        const std::string file_extension_str = ".sdt";
        std::ostringstream suffix;
        suffix << "-" << std::setfill('0') << std::setw(2) << iteration << file_extension_str;

        file_path =
            file_path.substr(
                0,
                file_path.length() - file_extension_str.length()) +
            suffix.str();
    }

    std::ofstream os(
        file_path,
        std::ios::out | std::ios::binary);

    if(!os.is_open())
    {
        RENDERER_LOG_WARNING("Could not open file \"%s\" for writing.", file_path.c_str());
        return;
    }

    const Camera *camera = m_scene.get_render_data().m_active_camera;

    if (camera == nullptr)
    {
        RENDERER_LOG_WARNING("Could not retrieve active camera.");
        return;
    }

    const float shutter_mid_time = camera->get_shutter_middle_time();
    Matrix4f camera_matrix = camera->transform_sequence().evaluate(shutter_mid_time).get_local_to_parent();

    // Rotate 180 degrees around y to conform to the visualizer tool's z-axis convention.
    const Matrix4f rotate_y = Matrix4f::make_rotation_y(Pi<float>());
    camera_matrix = camera_matrix * rotate_y;

    write(os, camera_matrix(0, 0)); write(os, camera_matrix(0, 1)); write(os, camera_matrix(0, 2)); write(os, camera_matrix(0, 3));
    write(os, camera_matrix(1, 0)); write(os, camera_matrix(1, 1)); write(os, camera_matrix(1, 2)); write(os, camera_matrix(1, 3));
    write(os, camera_matrix(2, 0)); write(os, camera_matrix(2, 1)); write(os, camera_matrix(2, 2)); write(os, camera_matrix(2, 3));
    write(os, camera_matrix(3, 0)); write(os, camera_matrix(3, 1)); write(os, camera_matrix(3, 2)); write(os, camera_matrix(3, 3));

    m_root_node->write_to_disk(os, m_scene_aabb);
}

// GPTVertex implementation.

void GPTVertex::add_radiance(
    const renderer::Spectrum&           radiance)
{
    m_radiance += radiance;
}

void GPTVertex::record_to_tree(
    STree&                              sd_tree,
    SamplingContext&                    sampling_context)
{
    Spectrum incoming_radiance;
    Spectrum product;

    for (size_t i = 0; i < Spectrum::size(); ++i)
    {
        // Check if components are valid.
        if (!std::isfinite(m_radiance[i]) || m_radiance[i] < 0.0f ||
            !std::isfinite(m_bsdf_value[i]) || m_bsdf_value[i] < 0.0f)
        {
            return;
        }

        const float rcp_factor = m_throughput[i] == 0.0f ? 0.0f : 1.0f / m_throughput[i];

        incoming_radiance[i] = m_radiance[i] * rcp_factor;
        product[i] = incoming_radiance[i] * m_bsdf_value[i];
    }

    DTreeRecord d_tree_record{
        m_direction,
        average_value(incoming_radiance),
        m_wi_pdf,
        m_bsdf_pdf,
        m_d_tree_pdf,
        m_product_pdf,
        1.0f,
        average_value(product),
        m_is_delta,
        m_guiding_method};

    sd_tree.record(m_d_tree, m_point, m_d_tree_node_size, d_tree_record, sampling_context);
}

// GPTVertexPath implementation.

GPTVertexPath::GPTVertexPath()
  : m_path_index(0)
{
}

void GPTVertexPath::add_vertex(
    const GPTVertex&                    vertex)
{
    if(m_path_index < m_path.size())
        m_path[m_path_index++] = vertex;
}

void GPTVertexPath::add_radiance(
    const renderer::Spectrum&           r)
{
    for(int i = 0; i < m_path_index; ++i)
        m_path[i].add_radiance(r);
}

void GPTVertexPath::add_indirect_radiance(
    const renderer::Spectrum&           r)
{
    for(int i = 0; i < m_path_index - 1; ++i)
        m_path[i].add_radiance(r);
}

bool GPTVertexPath::is_full() const
{
    return m_path_index >= m_path.size();
}

void GPTVertexPath::record_to_tree(
    STree&                              sd_tree,
    SamplingContext&                    sampling_context)
{
    for(int i = 0; i < m_path_index; ++i)
        m_path[i].record_to_tree(
                            sd_tree,
                            sampling_context);
}

}   // namespace renderer
