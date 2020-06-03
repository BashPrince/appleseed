#pragma once

// appleseed.renderer headers.
#include "renderer/kernel/lighting/bsdfproxy.h"
#include "renderer/kernel/lighting/materialsamplers.h"
#include "renderer/kernel/lighting/sdtree.h"
#include "renderer/modeling/bsdf/bsdfsample.h"


namespace renderer
{

//
// Sampler acting as a wrapper for path guided sampling at shading points for the implementation of
// "Practical Path Guiding for Efficient Light-Transport Simulation" [Müller et al. 2017].
//

class PathGuidedSampler
  : public BSDFSampler
{
  public:
    PathGuidedSampler(
        const GuidingMode               guiding_mode,
        const bool                      allow_path_guiding,
        const GuidedBounceMode          guided_bounce_mode,
        DTree*                          d_tree,
        const BSDF&                     bsdf,
        const void*                     bsdf_data,
        const int                       bsdf_sampling_modes,
        const ShadingPoint&             shading_point,
        const bool                      sd_tree_is_built);

    bool sample(
        SamplingContext&                sampling_context,
        const foundation::Dual3d&       outgoing,
        foundation::Dual3f&             incoming,
        DirectShadingComponents&        value,
        float&                          pdf) const override;

    bool sample(
        SamplingContext&                sampling_context,
        BSDFSample&                     bsdf_sample,
        const foundation::Dual3d&       outgoing,
        float&                          wi_pdf,
        float&                          d_tree_pdf,
        float&                          product_pdf) const;

    float evaluate(
        const foundation::Vector3f&     outgoing,
        const foundation::Vector3f&     incoming,
        const int                       light_sampling_modes,
        DirectShadingComponents&        value) const override;
    
    GuidingMode guiding_mode() const;

  private:
    void simple_bsdf_bounce(
        SamplingContext&                sampling_context,
        BSDFSample&                     bsdf_sample,
        const foundation::Dual3d&       outgoing,
        float&                          wi_pdf,
        float&                          d_tree_pdf,
        float&                          product_pdf) const;

    void guiding_aware_bsdf_bounce(
        SamplingContext&                sampling_context,
        BSDFSample&                     bsdf_sample,
        const foundation::Dual3d&       outgoing,
        float&                          wi_pdf,
        float&                          d_tree_pdf,
        float&                          product_pdf) const;

    void guided_bounce(
        SamplingContext&                sampling_context,
        BSDFSample&                     bsdf_sample,
        const foundation::Dual3d&       outgoing,
        float&                          wi_pdf,
        float&                          d_tree_pdf,
        float&                          product_pdf,
        const float                     s) const;

    float guided_path_extension_pdf(
        const foundation::Vector3f&     incoming,
        const foundation::Vector3f&     outgoing,
        const float                     bsdf_pdf,
        const float                     d_tree_pdf,
        const float                     product_pdf) const;

    const int enable_modes_before_sampling(
        const int                       sample_modes) const;

    ScatteringMode::Mode set_mode_after_sampling(
        const ScatteringMode::Mode      sampled_mode) const;

    DTree*                              m_d_tree;
    mutable RadianceProxy               m_radiance_proxy;
    mutable BSDFProxy                   m_bsdf_proxy;
    const GuidingMode                   m_guiding_mode;
    const bool                          m_sd_tree_is_built;
    const bool                          m_enable_path_guiding;
    bool                                m_enable_product_guiding;
    const GuidedBounceMode              m_guided_bounce_mode;
    float                               m_bsdf_sampling_fraction;
    float                               m_product_sampling_fraction;
};

}   // namespace render