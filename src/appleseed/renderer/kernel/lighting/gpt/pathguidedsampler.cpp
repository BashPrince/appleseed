// Interface header.
#include "pathguidedsampler.h"

// appleseed.renderer headers.
#include "renderer/kernel/lighting/tracer.h"
#include "renderer/kernel/shading/directshadingcomponents.h"
#include "renderer/kernel/shading/shadingcontext.h"
#include "renderer/kernel/shading/shadingpoint.h"
#include "renderer/modeling/bsdf/bsdf.h"
#include "renderer/modeling/bsdf/bsdfsample.h"
#include "renderer/modeling/volume/volume.h"

// appleseed.foundation headers.
#include "foundation/math/scalar.h"

// Standard headers.
#include <cassert>

using namespace foundation;
using namespace std;

namespace renderer
{

PathGuidedSampler::PathGuidedSampler(
    GuidingMode                     guiding_mode,
    const bool                      enable_path_guiding,
    GuidedBounceMode                guided_bounce_mode,
    DTree*                          d_tree,
    const BSDF&                     bsdf,
    const void*                     bsdf_data,
    const int                       bsdf_sampling_modes,
    const ShadingPoint&             shading_point,
    const bool                      sd_tree_is_built)
  : BSDFSampler(
      bsdf,
      bsdf_data,
      bsdf_sampling_modes,
      shading_point)
  , m_guiding_mode(guiding_mode)
  , m_enable_path_guiding(enable_path_guiding)
  , m_d_tree(d_tree)
  , m_sd_tree_is_built(sd_tree_is_built)
  , m_guided_bounce_mode(guided_bounce_mode)
  , m_radiance_proxy(d_tree->get_radiance_proxy())
  , m_use_product_guiding(false)
  , m_bsdf_sampling_fraction(d_tree->bsdf_sampling_fraction())
  , m_product_sampling_fractions(d_tree->bsdf_sampling_fraction_product())
{
    assert(m_d_tree);

    if ((m_guiding_mode == GuidingMode::ProductGuiding || m_guiding_mode == GuidingMode::Combined) &&
        m_radiance_proxy.is_built() && m_bsdf.add_parameters_to_proxy(m_bsdf_proxy, bsdf_data, bsdf_sampling_modes))
    {
        m_use_product_guiding = true;
    }
}

bool PathGuidedSampler::sample(
    SamplingContext&                sampling_context,
    const Dual3d&                   outgoing,
    Dual3f&                         incoming,
    DirectShadingComponents&        value,
    float&                          pdf) const
{
    BSDFSample bsdf_sample;
    float d_tree_pdf, product_pdf;

    sample(
        sampling_context,
        bsdf_sample,
        outgoing,
        pdf,
        d_tree_pdf,
        product_pdf);

    // Filter scattering modes.
    if (!(m_bsdf_sampling_modes & bsdf_sample.get_mode()))
        return false;

    incoming = bsdf_sample.m_incoming;
    value = bsdf_sample.m_value;

    return true;
}

float PathGuidedSampler::evaluate(
    const Vector3f&                 outgoing,
    const Vector3f&                 incoming,
    const int                       light_sampling_modes,
    DirectShadingComponents&        value) const
{
    const float bsdf_pdf = m_bsdf.evaluate(
        m_bsdf_data,
        false, // not adjoint
        true,
        m_local_geometry,
        outgoing,
        incoming,
        light_sampling_modes,
        value);

    float bsdf_sampling_fraction, product_sampling_fraction, product_pdf;
    set_sampling_fractions(bsdf_sampling_fraction, product_sampling_fraction);
    const float d_tree_pdf = m_d_tree->pdf(incoming, enable_modes_before_sampling(m_bsdf_sampling_modes));

    if (m_use_product_guiding)
    {
        m_radiance_proxy.build_product(m_bsdf_proxy, outgoing, m_local_geometry.m_shading_basis.get_normal());
        product_pdf = m_radiance_proxy.pdf(incoming);
    }
    else
    {
        product_pdf = 0.0f;
    }

    assert(bsdf_sampling_fraction >= 0.0f && bsdf_sampling_fraction <= 1.0f);
    assert(product_sampling_fraction >= 0.0f && product_sampling_fraction <= 1.0f);

    return guided_path_extension_pdf(
        incoming,
        outgoing,
        bsdf_pdf,
        d_tree_pdf,
        product_pdf,
        bsdf_sampling_fraction,
        product_sampling_fraction);
}

bool PathGuidedSampler::sample(
    SamplingContext&                sampling_context,
    BSDFSample&                     bsdf_sample,
    const Dual3d&                   outgoing,
    float&                          wi_pdf,
    float&                          d_tree_pdf,
    float&                          product_pdf) const
{
    if (!m_sd_tree_is_built || m_bsdf.is_purely_specular() || !m_enable_path_guiding)
    {
        m_bsdf.sample(
            sampling_context,
            m_bsdf_data,
            false,
            true, // multiply by |cos(incoming, normal)|
            m_local_geometry,
            Dual3f(outgoing),
            m_bsdf_sampling_modes,
            bsdf_sample);

        d_tree_pdf = product_pdf = 0.0f;
            
        wi_pdf = guided_path_extension_pdf(
            bsdf_sample.m_incoming.get_value(),
            Vector3f(outgoing.get_value()),
            bsdf_sample.get_probability(),
            d_tree_pdf,
            product_pdf,
            1.0f,
            0.0f);

        return false;
    }

    float bsdf_sampling_fraction, product_sampling_fraction;
    set_sampling_fractions(bsdf_sampling_fraction, product_sampling_fraction);

    assert(bsdf_sampling_fraction >= 0.0f && bsdf_sampling_fraction <= 1.0f);
    assert(product_sampling_fraction >= 0.0f && product_sampling_fraction <= 1.0f);

    sampling_context.split_in_place(1, 1);
    float s = sampling_context.next2<float>();

    if (s < bsdf_sampling_fraction)
    {
        m_bsdf.sample(
            sampling_context,
            m_bsdf_data,
            false,
            true, // multiply by |cos(incoming, normal)|
            m_local_geometry,
            Dual3f(outgoing),
            m_bsdf_sampling_modes,
            bsdf_sample);

        if (bsdf_sample.get_mode() == ScatteringMode::None)
            return false;

        if (bsdf_sample.get_mode() == ScatteringMode::Specular)
        {
            d_tree_pdf = product_pdf = 0.0f;
            wi_pdf = bsdf_sampling_fraction;
            return false;
        }

        if (m_use_product_guiding)
        {
            m_radiance_proxy.build_product(m_bsdf_proxy, Vector3f(outgoing.get_value()), m_local_geometry.m_shading_basis.get_normal());
            product_pdf = m_radiance_proxy.pdf(bsdf_sample.m_incoming.get_value());
        }
        else
        {
            product_pdf = 0.0f;
        }

        d_tree_pdf = m_d_tree->pdf(bsdf_sample.m_incoming.get_value(), enable_modes_before_sampling(m_bsdf_sampling_modes));

        wi_pdf = guided_path_extension_pdf(
            bsdf_sample.m_incoming.get_value(),
            Vector3f(outgoing.get_value()),
            bsdf_sample.get_probability(),
            d_tree_pdf,
            product_pdf,
            bsdf_sampling_fraction,
            product_sampling_fraction);
        
        return false;
    }
    else
    {
        s = (s - bsdf_sampling_fraction) / (1.0f - bsdf_sampling_fraction);
        s = std::min(s, 0.99999f);
        
        DTreeSample d_tree_sample;

        if (m_use_product_guiding)
            m_radiance_proxy.build_product(m_bsdf_proxy, Vector3f(outgoing.get_value()), m_local_geometry.m_shading_basis.get_normal());

        if (s < product_sampling_fraction) // product guiding
        {
            product_pdf = m_radiance_proxy.sample(sampling_context, d_tree_sample.direction);
            d_tree_sample.pdf = m_d_tree->pdf(d_tree_sample.direction, enable_modes_before_sampling(m_bsdf_sampling_modes));
            d_tree_sample.scattering_mode = ScatteringMode::Diffuse;
        }
        else // path guiding
        {
            m_d_tree->sample(sampling_context, d_tree_sample, enable_modes_before_sampling(m_bsdf_sampling_modes));

            product_pdf = m_use_product_guiding ? m_radiance_proxy.pdf(d_tree_sample.direction) : 0.0f;
        }

        const ScatteringMode::Mode scattering_mode = set_mode_after_sampling(d_tree_sample.scattering_mode);

        if (scattering_mode == ScatteringMode::None)
        {
            // Terminate.
            bsdf_sample.set_to_scattering(scattering_mode, 0.0f);
            return true;
        }

        bsdf_sample.m_incoming = foundation::Dual3f(d_tree_sample.direction);
        d_tree_pdf = d_tree_sample.pdf;

        const float bsdf_pdf = m_bsdf.evaluate(
            m_bsdf_data,
            false, // not adjoint
            true,  // multiply by |cos(incoming, normal)|
            m_local_geometry,
            Vector3f(outgoing.get_value()),
            bsdf_sample.m_incoming.get_value(),
            m_bsdf_sampling_modes,
            bsdf_sample.m_value);

        if (bsdf_pdf == 0.0f)
        {
            // Reject invalid directions.
            bsdf_sample.set_to_scattering(ScatteringMode::None, bsdf_pdf);
            return true;
        }
        else
        {
            bsdf_sample.set_to_scattering(scattering_mode, bsdf_pdf);
        }

        wi_pdf = guided_path_extension_pdf(
            bsdf_sample.m_incoming.get_value(),
            Vector3f(outgoing.get_value()),
            bsdf_pdf,
            d_tree_pdf,
            product_pdf,
            bsdf_sampling_fraction,
            product_sampling_fraction);

        return true;
    }
}

float PathGuidedSampler::guided_path_extension_pdf(
    const foundation::Vector3f&     incoming,
    const foundation::Vector3f&     outgoing,
    const float                     bsdf_pdf,
    const float                     d_tree_pdf,
    const float                     product_pdf,
    const float                     bsdf_sampling_fraction,
    const float                     product_sampling_fraction) const
{
    if (!m_sd_tree_is_built || m_bsdf.is_purely_specular() || !m_enable_path_guiding)
    {
        return bsdf_pdf;
    }

    const float guided_mix_pdf = lerp(d_tree_pdf, product_pdf, product_sampling_fraction);
    return lerp(guided_mix_pdf, bsdf_pdf, bsdf_sampling_fraction);
}

const int PathGuidedSampler::enable_modes_before_sampling(
    const int                       modes) const
{
    if (m_guided_bounce_mode == GuidedBounceMode::Learn)
        return modes;
    else
        return ScatteringMode::Diffuse | ScatteringMode::Glossy;
}

ScatteringMode::Mode PathGuidedSampler::set_mode_after_sampling(
    const ScatteringMode::Mode      sampled_mode) const
{
    switch (m_guided_bounce_mode)
    {
    case GuidedBounceMode::Learn:
        return sampled_mode;
        break;
    
    case GuidedBounceMode::StrictlyDiffuse:
        return ScatteringMode::has_diffuse(m_bsdf_sampling_modes) ?
                ScatteringMode::Diffuse : ScatteringMode::None;
        break;
    
    case GuidedBounceMode::StrictlyGlossy:
        return ScatteringMode::has_glossy(m_bsdf_sampling_modes) ?
                ScatteringMode::Glossy : ScatteringMode::None;
        break;
    
    case GuidedBounceMode::PreferDiffuse:
        return ScatteringMode::has_diffuse(m_bsdf_sampling_modes) ?
                ScatteringMode::Diffuse : ScatteringMode::has_glossy(m_bsdf_sampling_modes) ?
                ScatteringMode::Glossy : ScatteringMode::None;
        break;
    
    case GuidedBounceMode::PreferGlossy:
            return ScatteringMode::has_glossy(m_bsdf_sampling_modes) ?
                    ScatteringMode::Glossy : ScatteringMode::has_diffuse(m_bsdf_sampling_modes) ?
                    ScatteringMode::Diffuse : ScatteringMode::None;
        break;
    
    default:
        return sampled_mode;
        break;
    }
}

void PathGuidedSampler::set_sampling_fractions(
    float&                          bsdf_sampling_fraction,
    float&                          product_sampling_fraction) const
{
    if (m_guiding_mode == GuidingMode::Combined && m_use_product_guiding)
    {
        bsdf_sampling_fraction = m_product_sampling_fractions.x;
        product_sampling_fraction = m_product_sampling_fractions.y;
    }
    else if (m_guiding_mode == GuidingMode::ProductGuiding && m_use_product_guiding)
    {
        bsdf_sampling_fraction = m_bsdf_sampling_fraction;
        product_sampling_fraction = 1.0f;
    }
    else
    {
        bsdf_sampling_fraction = m_bsdf_sampling_fraction;
        product_sampling_fraction = 0.0f;
    }
}

GuidingMode PathGuidedSampler::guiding_mode() const
{
    if (m_guiding_mode == GuidingMode::Combined && m_use_product_guiding)
        return GuidingMode::Combined;
    else
        return GuidingMode::PathGuiding;
}

}    //namespace renderer