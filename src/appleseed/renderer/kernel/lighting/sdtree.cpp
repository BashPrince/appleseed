
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
#include "foundation/math/scalar.h"
#include "foundation/math/sampling/mappings.h"
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
    if(m_is_leaf)
        return RcpFourPi<float>();
    
    const QuadTreeNode* sub_node = choose_node(direction);
    const float factor = 4.0f * sub_node->m_previous_iter_radiance_sum / m_previous_iter_radiance_sum;
    return factor * sub_node->pdf(direction);
}

const Vector2f QuadTreeNode::sample(
    Vector2f&                           sample,
    float&                              pdf) const
{
    pdf = 1.0f; // initiate to one for recursive sampling routine
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
        pdf *= RcpFourPi<float>();
        return sample;
    }

    const float upper_left = m_upper_left_node->m_previous_iter_radiance_sum;
    const float upper_right = m_upper_right_node->m_previous_iter_radiance_sum;
    const float lower_right = m_lower_right_node->m_previous_iter_radiance_sum;
    const float lower_left = m_lower_left_node->m_previous_iter_radiance_sum;
    const float sum_left_half = upper_left + lower_left;
    const float sum_right_half = upper_right + lower_right;

    float factor = sum_left_half / m_previous_iter_radiance_sum;
    
    // Sample child nodes with probability proportional to their energy.
    if(sample.x < factor)
    {
        sample.x /= factor;
        factor = upper_left / sum_left_half;

        if(sample.y < factor)
        {
            sample.y /= factor;
            const Vector2f sampled_direction =
                Vector2f(0.0f, 0.0f) + 0.5f * m_upper_left_node->sample(sample, pdf);

            const float probability_factor = 4.0f * upper_left / m_previous_iter_radiance_sum;
            pdf *= probability_factor;
            return sampled_direction;
        }

        sample.y = (sample.y - factor) / (1.0f - factor);
        const Vector2f sampled_direction =
            Vector2f(0.0f, 0.5f) + 0.5f * m_lower_left_node->sample(sample, pdf);

        const float probability_factor = 4.0f * lower_left / m_previous_iter_radiance_sum;
        pdf *= probability_factor;
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
                Vector2f(0.5f, 0.0f) + 0.5f * m_upper_right_node->sample(sample, pdf);

            const float probability_factor = 4.0f * upper_right / m_previous_iter_radiance_sum;
            pdf *= probability_factor;
            return sampled_direction;
        }

        sample.y = (sample.y - factor) / (1.0f - factor);
        const Vector2f sampled_direction =
            Vector2f(0.5f, 0.5f) + 0.5f * m_lower_right_node->sample(sample, pdf);

        const float probability_factor = 4.0f * lower_right / m_previous_iter_radiance_sum;
        pdf *= probability_factor;
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

void QuadTreeNode::build_radiance_map(
    std::vector<float>&                         radiance_map,
    const Vector2u&                             index,
    const size_t                                level,
    const size_t                                max_level,
    const size_t                                map_res,
    const float                                 area_weight_scale) const
{
    if (m_is_leaf)
    {
        const size_t level_diff = max_level - level;
        Vector2u origin(index);
        size_t size = 1;

        for (size_t i = 0; i < level_diff; ++i)
        {
            origin *= static_cast<size_t>(2);
            size *= 2;
        }
        
        const float radiance = std::pow(4.0f, static_cast<float>(level - 1)) * m_previous_iter_radiance_sum * area_weight_scale;

        for(size_t y = 0; y < size; ++y)
            for(size_t x = 0; x < size; ++x)
            {
                const Vector2u pixel_pos = origin + Vector2u(x, y);
                const size_t pixel_index = pixel_pos.y * map_res + pixel_pos.x;

                radiance_map[pixel_index] = radiance;
            }
    }
    else
    {
        const Vector2u origin = index * size_t(2);
        m_upper_left_node->build_radiance_map(
            radiance_map,
            origin,
            level + 1,
            max_level,
            map_res,
            area_weight_scale);
        m_upper_right_node->build_radiance_map(
            radiance_map,
            origin + Vector2u(1, 0),
            level + 1,
            max_level,
            map_res,
            area_weight_scale);
        m_lower_left_node->build_radiance_map(
            radiance_map,
            origin + Vector2u(0, 1),
            level + 1,
            max_level,
            map_res,
            area_weight_scale);
        m_lower_right_node->build_radiance_map(
            radiance_map,
            origin + Vector2u(1, 1),
            level + 1,
            max_level,
            map_res,
            area_weight_scale);
    }
}

float QuadTreeNode::radiance(Vector2f& direction) const
{
    if (m_is_leaf)
        return m_previous_iter_radiance_sum;
    
    return 4.0f * choose_node(direction)->radiance(direction);
}

// Radiance proxy implementation.

RadianceProxy::RadianceProxy(
    const std::vector<std::vector<float>>&      mip_maps)
  : m_map({0.0f})
{
    size_t map_res = 1;
    for (size_t i = 0; i < mip_maps.size() - 1; ++i)
        map_res *= 2;

    m_resolution = (map_res / 12) * 12; // multiple of 12
    m_high_res_map = std::make_shared<std::vector<float>>(m_resolution * m_resolution);
    const float inv_res = 1.0f / m_resolution;

    const size_t block_size = m_resolution / 12;
    const float weight = 1.0f / (block_size * block_size);

    for (size_t y = 0; y < m_resolution; ++y)
        for (size_t x = 0; x < m_resolution; ++x)
        {
            Vector2f point(x + 0.5f, y + 0.5f);
            Vector2f point_dx(point.x + 1.0f, point.y);
            Vector2f point_dy(point.x, point.y + 1.0f);

            // Handle dx seam.
            if (point_dx.x >= m_resolution)
            {
                point_dx.x = m_resolution - 0.5f;
                point_dx.y = m_resolution - point_dx.y;
            }

            // Handle dy seam.
            if (point_dy.y >= m_resolution)
            {
                point_dy.y = m_resolution - 0.5f;
                point_dy.x = m_resolution - point_dy.x;
            }

            point *= inv_res;
            point_dx *= inv_res;
            point_dy *= inv_res;

            const Vector3f point_dir_world = square_to_sphere(point);
            const Vector3f point_dx_dir_world = square_to_sphere(point_dx);
            const Vector3f point_dy_dir_world = square_to_sphere(point_dy);

            const Vector2f point_cylindrical = cartesian_to_cylindrical(point_dir_world);
            const Vector2f point_dx_cylindrical = cartesian_to_cylindrical(point_dx_dir_world);
            const Vector2f point_dy_cylindrical = cartesian_to_cylindrical(point_dy_dir_world);

            // Wrap dx/dy around seams and check for the shortest possible distance in the cylindrical map.
            Vector2f point_dx_phi_wrap = point_dx_cylindrical;
            Vector2f point_dx_theta_wrap = point_dx_cylindrical;
            Vector2f point_dy_phi_wrap = point_dy_cylindrical;
            Vector2f point_dy_theta_wrap = point_dy_cylindrical;

            // Wrap around phi.
            if (point_dx_cylindrical.y < 0.5f)
                point_dx_phi_wrap.y += 1.0f;
            else
                point_dx_phi_wrap.y -= 1.0f;

            if (point_dy_cylindrical.y < 0.5f)
                point_dy_phi_wrap.y += 1.0f;
            else
                point_dy_phi_wrap.y -= 1.0f;
            
            // Wrap around theta.
            if (point_dx_cylindrical.x < 0.5f)
            {
                if (point_dx_cylindrical.y < 0.5f)
                    point_dx_theta_wrap.y += 0.5f;
                else
                    point_dx_theta_wrap.y -= 0.5f;
            }

            if (point_dy_cylindrical.x < 0.5f)
            {
                if (point_dy_cylindrical.y < 0.5f)
                    point_dy_theta_wrap.y += 0.5f;
                else
                    point_dy_theta_wrap.y -= 0.5f;
            }

            // Min distance for dx.
            const float min_dx = std::min(
                std::min(
                    foundation::norm(point_dx_cylindrical - point_cylindrical),
                    foundation::norm(point_dx_phi_wrap - point_cylindrical)),
                    foundation::norm(point_dx_theta_wrap - point_cylindrical));

            // Min distance for dy.
            const float min_dy = std::min(
                std::min(
                    foundation::norm(point_dy_cylindrical - point_cylindrical),
                    foundation::norm(point_dy_phi_wrap - point_cylindrical)),
                    foundation::norm(point_dy_theta_wrap - point_cylindrical));

            // Mip map reconstruction.
            
            const float mip_level = std::log2f(std::max(min_dx, min_dy));

            const size_t lower_mip_index = std::max(std::min(static_cast<int>(mip_level), static_cast<int>(mip_maps.size() - 1)), 0);
            const size_t upper_mip_index = std::min(lower_mip_index + 1, mip_maps.size() - 1);

            const std::vector<float>& lower_mip = mip_maps[lower_mip_index];
            const std::vector<float>& upper_mip = mip_maps[upper_mip_index];

            size_t lower_mip_res = map_res;

            for (size_t i = 0; i < lower_mip_index; ++i)
                lower_mip_res /= 2;

            size_t upper_mip_res = (lower_mip_index == upper_mip_index) ? lower_mip_res : lower_mip_res / 2;

            const float lower_mip_value = bilerp(lower_mip, lower_mip_res, point_cylindrical);
            const float upper_mip_value = bilerp(upper_mip, upper_mip_res, point_cylindrical);
            const float value = foundation::lerp(lower_mip_value, upper_mip_value, mip_level - lower_mip_index);

            (*m_high_res_map)[y * m_resolution + x] = value;
            m_map[(y / block_size) * 12 + x / block_size] += value * weight;
        }
}

RadianceProxy::RadianceProxy(
    const RadianceProxy&                        other)
    : m_high_res_map(other.m_high_res_map)
    , m_map(other.m_map)
    , m_resolution(other.m_resolution)
{
}

// Using source code of Fast Equal-Area Mapping of the (Hemi)Sphere using SIMD [Clarberg, 2008]
// http://fileadmin.cs.lth.se/cs/Personal/Petrik_Clarberg/code/simd_mapping.zip

Vector2f RadianceProxy::sphere_to_square(
    const Vector3f &direction) const
{
    float phi = std::atan2(direction.y, direction.x) * foundation::TwoOverPi<float>(); // phi in [-2,2]
    float u, v;

    // There are 8 different cases we need to test (3 levels of nestled
    // if-statements to compute the (u,v) coordiantes in the square.

    if (direction.z < 0.0f) // southern hemisphere
    {
        float r = std::sqrt(1.f + direction.z);

        if (phi >= 0.f)
        {
            if (phi <= 1.f)
            {
                u = 1.f - r * phi;
                v = 2.f - r - u;
            }
            else
            {
                u = r * (2.f - phi) - 1.f;
                v = 2.f - r + u;
            }
        }
        else
        {
            if (phi >= -1.f)
            {
                u = r * phi + 1.f;
                v = r - 2.f + u;
            }
            else
            {
                u = r * (2.f + phi) - 1.f;
                v = r - 2.f - u;
            }
        }
    }
    else // northern hemisphere
    {
        float r = std::sqrt(1.f - direction.z);

        if (phi >= 0.f)
        {
            if (phi < 1.f)
            {
                v = r * phi;
                u = r - v;
            }
            else
            {
                v = r * (2.f - phi);
                u = v - r;
            }
        }
        else
        {
            if (phi > -1.f)
            {
                v = r * phi;
                u = r + v;
            }
            else
            {
                v = -r * (2.f + phi);
                u = -(r + v);
            }
        }
    }

    // Transform (u,v) from [-1,1] to [0,1]
    u = 0.5f * (u + 1.f);
    v = 0.5f * (v + 1.f);

    return Vector2f(u, v);
}

// Using source code of Fast Equal-Area Mapping of the (Hemi)Sphere using SIMD [Clarberg, 2008]
// http://fileadmin.cs.lth.se/cs/Personal/Petrik_Clarberg/code/simd_mapping.zip

Vector3f RadianceProxy::square_to_sphere(
    const Vector2f&                 direction) const
{
    // Transform point from [0,1] to [-1,1]
    float u = 2.f * direction.x - 1.f;
    float v = 2.f * direction.y - 1.f;

    // Compute lengths (a,b) for rotated square (-45deg) scaled by sqrt(2).
    float a = v + u;
    float b = v - u;

    // Compute (r,phi) differently based on which quadrant we are in.
    // There are 8 different cases (3 levels of nestled if-statements).
    // We set z to -1 or 1 based on which hemisphere we are in.

    float r, phi, z;

    if (v >= 0.f)
    {
        if (u >= 0.f) // quadrant 1
        {
            if (a <= 1.f) // north
            {
                r = a;
                z = 1.f;
                phi = v / r;
            }
            else // south
            {
                r = 2.f - a;
                z = -1.f;
                phi = (1.f - u) / r;
            }
        }
        else // quadrant 2
        {
            if (b <= 1.f) // north
            {
                r = b;
                z = 1.f;
                phi = 1.f - u / r;
            }
            else
            {
                r = 2.f - b;
                z = -1.f;
                phi = 1.f + (1.f - v) / r;
            }
        }
    }
    else
    {
        if (u < 0.0f) // quadrant 3
        {
            if (a >= -1.f) // north
            {
                r = -a;
                z = 1.f;
                phi = 2.f - v / r;
            }
            else // south
            {
                r = 2.f + a;
                z = -1.f;
                phi = 2.f + (1.f + u) / r;
            }
        }
        else // quadrant 4
        {
            if (b >= -1.f) // north
            {
                r = -b;
                z = 1.f;
                phi = 3.f + u / r;
            }
            else // south
            {
                r = 2.f + b;
                z = -1.f;
                phi = 3.f + (1.f + v) / r;
            }
        }
    }

    // Fix division-by-zero problem (r=0).
    if (r == 0.f)
        phi = 0.f;

    // Compute 3D coordinate (x,y,z)
    float r2 = r * r;
    phi *= foundation::HalfPi<float>();
    float sin_t = r * std::sqrt(2.f - r2); // sin(theta)

    float x = sin_t * std::cos(phi);
    float y = sin_t * std::sin(phi);
    z *= 1.f - r2;

    return Vector3f(x, y, z);
}

float RadianceProxy::radiance(
    const Vector3f&                 direction) const
{
    const Vector2f spherical_direction(sphere_to_square(direction) * static_cast<float>(m_resolution));
    const Vector2u pixel(
        std::min(static_cast<size_t>(spherical_direction.x), m_resolution - 1),
        std::min(static_cast<size_t>(spherical_direction.y), m_resolution - 1));
    
    return (*m_high_res_map)[pixel.y * m_resolution + pixel.x];
}

float RadianceProxy::proxy_radiance(
    const Vector3f&                 direction) const
{
    const size_t map_width = 12;
    const Vector2f spherical_direction(sphere_to_square(direction) * static_cast<float>(map_width));
    const Vector2u pixel(
        std::min(static_cast<size_t>(spherical_direction.x), map_width - 1),
        std::min(static_cast<size_t>(spherical_direction.y), map_width - 1));
    
    return m_map[pixel.y * map_width + pixel.x];
}

float RadianceProxy::bilerp(
    const std::vector<float>&   mip,
    const size_t                mip_res,
    const Vector2f&             pos) const
{
    const Vector2f pixel_float(
        pos.x * mip_res,
        pos.y * mip_res);

    Vector2i upper_left_mip_pixel(
        std::floor(pixel_float.x - 0.5f),
        std::floor(pixel_float.y - 0.5f));

    Vector2i upper_right_mip_pixel(
        upper_left_mip_pixel.x + 1.0f,
        upper_left_mip_pixel.y);

    Vector2i lower_left_mip_pixel(
        upper_left_mip_pixel.x,
        upper_left_mip_pixel.y + 1.0f);

    Vector2i lower_right_mip_pixel(
        upper_left_mip_pixel.x + 1.0f,
        upper_left_mip_pixel.y + 1.0f);

    // Wrap out of bounds pixels.
    auto wrap = [&](Vector2i& pixel)
    {
        if (pixel.y < 0)
            pixel.y = mip_res - 1;
        else if (pixel.y > mip_res - 1)
            pixel.y = 0;
        
        if (pixel.x < 0)
        {
            pixel.x = 0;
            pixel.y = (pixel.y + mip_res / 2) % mip_res;
        }
        else if (pixel.x > mip_res - 1)
        {
            pixel.x = mip_res - 1;
            pixel.y = (pixel.y + mip_res / 2) % mip_res;
        }
    };

    wrap(upper_left_mip_pixel);
    wrap(upper_right_mip_pixel);
    wrap(lower_left_mip_pixel);
    wrap(lower_right_mip_pixel);

    const float upper_left_pixel_val = mip[upper_left_mip_pixel.y * mip_res + upper_left_mip_pixel.x];
    const float upper_right_pixel_val = mip[upper_right_mip_pixel.y * mip_res + upper_right_mip_pixel.x];
    const float lower_left_pixel_val = mip[lower_left_mip_pixel.y * mip_res + lower_left_mip_pixel.x];
    const float lower_right_pixel_val = mip[lower_right_mip_pixel.y * mip_res + lower_right_mip_pixel.x];

    const float offset_x = pixel_float.x - 0.5f;
    const float offset_y = pixel_float.y - 0.5f;
    const float x_interpolation = offset_x - std::floor(offset_x);
    const float y_interpolation = offset_y - std::floor(offset_y);

    const float upper_row = foundation::lerp(upper_left_pixel_val, upper_right_pixel_val, x_interpolation);
    const float lower_row = foundation::lerp(lower_left_pixel_val, lower_right_pixel_val, x_interpolation);
    return foundation::lerp(upper_row, lower_row, y_interpolation);
}

void QuadTreeNode::evaluate_proxy(
    const RadianceProxy&        proxy,
    const Vector2f&             direction,
    float&                      error_sum,
    float&                      low_res_error_sum,
    const size_t                depth,
    const float                 area_weight_scale) const
{
    if (m_is_leaf)
    {
        const Vector3f cartesian_direction = cylindrical_to_cartesian(direction);
        const Vector2f spherical_direction = proxy.sphere_to_square(cartesian_direction) * static_cast<float>(proxy.m_resolution);
        const Vector2u pixel(
            std::max(std::min(static_cast<size_t>(spherical_direction.x), proxy.m_resolution - 1), size_t(0)),
            std::max(std::min(static_cast<size_t>(spherical_direction.y), proxy.m_resolution - 1), size_t(0)));
        const Vector2u low_res_pixel(
            std::max(std::min(static_cast<size_t>(spherical_direction.x), size_t(11)), size_t(0)),
            std::max(std::min(static_cast<size_t>(spherical_direction.y), size_t(11)), size_t(0)));

        const size_t high_res_index = pixel.y * proxy.m_resolution + pixel.x;
        const size_t low_res_index = low_res_pixel.y * 11 + low_res_pixel.x;

        const float radiance_scale = std::pow(4.0f, depth - 1);
        const float radiance = radiance_scale * m_previous_iter_radiance_sum * area_weight_scale;

        error_sum += std::abs(radiance - (*proxy.m_high_res_map)[high_res_index]);
        low_res_error_sum += std::abs(radiance - proxy.m_map[low_res_index]);
    }
    else
    {
        const float offset = std::pow(0.5f, depth);
        Vector2f child_direction(direction - Vector2f(offset));
        m_upper_left_node->evaluate_proxy(proxy, child_direction, error_sum, low_res_error_sum, depth + 1, area_weight_scale);
        m_upper_right_node->evaluate_proxy(proxy, child_direction + Vector2f(offset * 2.0f, 0.0f), error_sum, low_res_error_sum, depth + 1, area_weight_scale);
        m_lower_right_node->evaluate_proxy(proxy, child_direction + Vector2f(offset * 2.0f, offset * 2.0f), error_sum, low_res_error_sum, depth + 1, area_weight_scale);
        m_lower_left_node->evaluate_proxy(proxy, child_direction + Vector2f(0.0f, offset * 2.0f), error_sum, low_res_error_sum, depth + 1, area_weight_scale);
    }
}

void QuadTreeNode::evaluate_radiance_map(
    const std::vector<float>&   map,
    const size_t                map_res,
    const Vector2f&             direction,
    float&                      error_sum,
    const size_t                depth,
    const float                 area_weight_scale) const
{
    if (m_is_leaf)
    {
        const Vector2f scaled_direction = direction * static_cast<float>(map_res);
        const Vector2u pixel(
            std::max(std::min(static_cast<size_t>(scaled_direction.x), map_res - 1), size_t(0)),
            std::max(std::min(static_cast<size_t>(scaled_direction.y), map_res - 1), size_t(0)));

        const float radiance_scale = std::pow(4.0f, depth - 1);
        const float radiance = radiance_scale * m_previous_iter_radiance_sum * area_weight_scale;

        error_sum += std::abs(radiance - map[pixel.y * map_res + pixel.x]);
    }
    else
    {
        const float offset = std::pow(0.5f, depth);
        Vector2f child_direction(direction - Vector2f(offset));
        m_upper_left_node->evaluate_radiance_map(map, map_res, child_direction, error_sum, depth + 1, area_weight_scale);
        m_upper_right_node->evaluate_radiance_map(map, map_res, child_direction + Vector2f(offset * 2.0f, 0.0f), error_sum, depth + 1, area_weight_scale);
        m_lower_right_node->evaluate_radiance_map(map, map_res, child_direction + Vector2f(offset * 2.0f, offset * 2.0f), error_sum, depth + 1, area_weight_scale);
        m_lower_left_node->evaluate_radiance_map(map, map_res, child_direction + Vector2f(0.0f, offset * 2.0f), error_sum, depth + 1, area_weight_scale);
    }
}

struct DTreeRecord
{
    Vector3f                    direction;
    float                       radiance;
    float                       wi_pdf;
    float                       bsdf_pdf;
    float                       d_tree_pdf;
    float                       sample_weight;
    float                       product;
    bool                        is_delta;
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
    , m_is_built(false)
    , m_scattering_mode(ScatteringMode::Diffuse)
{
    m_atomic_flag.clear(std::memory_order_release);
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
    , m_is_built(other.m_is_built)
    , m_scattering_mode(other.m_scattering_mode)
{
    m_atomic_flag.clear(std::memory_order_release);
}

void DTree::record(
    const DTreeRecord&                  d_tree_record)
{
    if(m_parameters.m_bsdf_sampling_fraction_mode == BSDFSamplingFractionMode::Learn && m_is_built && d_tree_record.product > 0.0f)
        optimization_step(d_tree_record); // also optimizes delta records
        
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
    return m_root_node.pdf(dir);
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

    // Reset D-Trees that did not collect radiance.
    if(radiance_sum <= 0.0f)
    {
        m_root_node.reset();
        m_scattering_mode = ScatteringMode::Diffuse;
        m_optimization_step_count = 0;
        m_first_moment = 0.0f;
        m_second_moment = 0.0f;
        m_theta = 0.0f;
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

    build_radiance_map();
    build_equal_area_maps();
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

void DTree::build_radiance_map()
{
    m_mip_maps.clear();

    if (m_previous_iter_sample_weight <= 0.0f || m_root_node.radiance_sum() <= 0.0f)
    {
        m_mip_maps.push_back(std::vector<float>{0.0f});
    }
    else
    {
        
        const size_t depth = m_root_node.max_depth();
        size_t map_res = 1;
        for (size_t i = 0; i < depth; ++i)
        {
            m_mip_maps.push_back(std::vector<float>(map_res * map_res));
            map_res *= 2;
        }

        map_res /= 2;
        std::reverse(m_mip_maps.begin(), m_mip_maps.end());

        // Build upper mip map
        const float scale_factor = 1.0f / (FourPi<float>() * m_previous_iter_sample_weight);
        m_root_node.build_radiance_map(m_mip_maps.front(), Vector2u(0, 0), 1, depth, map_res, scale_factor);

        auto prev_mip = m_mip_maps.begin();
        auto curr_mip = std::next(prev_mip);

        while (curr_mip != m_mip_maps.end())
        {
            map_res /= 2;

            for (size_t y = 0; y < map_res; ++y)
                for (size_t x = 0; x < map_res; ++x)
                {
                    (*curr_mip)[y * map_res + x] = 0.25f * (
                        (*prev_mip)[y * 2 * map_res * 2 + 2 * x] +
                        (*prev_mip)[y * 2 * map_res * 2 + 2 * x + 1] +
                        (*prev_mip)[(y * 2 + 1) * map_res * 2 + 2 * x] +
                        (*prev_mip)[(y * 2 + 1) * map_res * 2 + 2 * x + 1]);
                }

            ++curr_mip;
            ++prev_mip;
        }
    }
}

void DTree::build_equal_area_maps()
{
    m_radiance_proxy = RadianceProxy(m_mip_maps);
    m_mip_maps.clear();
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

void DTree::acquire_optimization_spin_lock()
{
    while(m_atomic_flag.test_and_set(std::memory_order_acquire))
        ;
}

void DTree::release_optimization_spin_lock()
{
    m_atomic_flag.clear(std::memory_order_release);
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
                d_tree_record.sample_weight * intersection_volume,
                d_tree_record.product,
                d_tree_record.is_delta});
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
        1.0f,
        average_value(product),
        m_is_delta};

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
