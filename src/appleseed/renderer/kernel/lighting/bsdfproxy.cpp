
//
// This source file is part of appleseed.
// Visit https://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2020 Stephen Agyemang, The appleseedhq Organization
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
#include "bsdfproxy.h"

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

BSDFProxy::BSDFProxy()
  : m_diffuse_weight(0.0f)
  , m_translucency_weight(0.0f)
  , m_reflection_weight(0.0f)
  , m_refraction_weight(0.0f)
  , m_reflection_roughness(0.0f)
  , m_refraction_roughness(0.0f)
{}

void BSDFProxy::add_diffuse_weight(const float          diffuse_weight)
{
    m_diffuse_weight += diffuse_weight;
}

void BSDFProxy::add_translucency_weight(const float     translucency_weight)
{
    m_translucency_weight += translucency_weight;
}

void BSDFProxy::add_reflection_weight(
    const float                                             reflection_weight,
    const float                                             roughness)
{
    const float old_weight = m_reflection_weight;
    m_reflection_weight += reflection_weight;
    const float inv_weight = m_reflection_weight > 0.0f ? 1.0f / m_reflection_weight : 0.0f;
    m_reflection_roughness = old_weight * inv_weight * m_reflection_roughness + reflection_weight * inv_weight * roughness;
}

void BSDFProxy::add_refraction_weight(
    const float                                             refraction_weight,
    const float                                             roughness)
{
    const float old_weight = m_refraction_weight;
    m_refraction_weight += refraction_weight;
    const float inv_weight = m_refraction_weight > 0.0f ? 1.0f / m_refraction_weight : 0.0f;
    m_refraction_roughness = old_weight * inv_weight * m_refraction_roughness + refraction_weight * inv_weight * roughness;
}

void BSDFProxy::finish_parameterization(
    const Vector3f&                                         outgoing,
    const Vector3f&                                         shading_normal)
{
    m_is_diffuse = m_diffuse_weight > 0.0f;
    m_is_translucent = m_translucency_weight > 0.0f;
    m_is_reflective = m_reflection_weight > 0.0f;
    m_is_refractive = m_refraction_weight > 0.0f;

    if (is_zero())
        return;

    // Construct lobes in world space.
    // TO-DO: Hemisphere checks, basic asserts.
    m_normal = shading_normal;
    m_reflection_lobe = foundation::reflect(outgoing, m_normal);
    foundation::refract(outgoing, m_normal, m_IOR, m_refraction_lobe);

    // Roughness correction.
    m_reflection_roughness *= 2.0f;
    const float cos_nt = std::abs(foundation::dot(m_normal, m_reflection_lobe));
    const float cos_no = std::abs(foundation::dot(m_normal, outgoing));
    const float scale_factor_refraction = (cos_nt + m_IOR * cos_no) / cos_nt;
    m_refraction_roughness *= scale_factor_refraction;
}

float BSDFProxy::evaluate(
    const Vector3f&                                         incoming) const
{
    float value = 0.0f;
    const float cos_ni = foundation::dot(m_normal, incoming);
    const float cos_negni = -cos_ni;

    if (m_is_diffuse)
    {
        value += m_diffuse_weight * std::max(cos_ni, 0.0f);
    }
    if (m_is_translucent)
    {
        value += m_translucency_weight * std::max(cos_negni, 0.0f);
    }
    if (m_is_reflective)
    {
        
    }
    if (m_is_refractive)
    {

    }

    return value;
}

bool BSDFProxy::is_zero() const
{
    return !(m_is_diffuse || m_is_translucent || m_is_reflective || m_is_refractive);
}

}   // namespace renderer
