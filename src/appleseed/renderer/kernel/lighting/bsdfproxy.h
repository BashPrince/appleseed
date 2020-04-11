
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

#pragma once

// appleseed.renderer headers.
#include "renderer/global/globaltypes.h"

// appleseed.foundation headers.
#include "foundation/math/basis.h"
#include "foundation/math/vector.h"

// Standard headers.

//
// Radiance Proxy implementation as described in "Fast Product Importance Sampling of Environment Maps"
// [Conty Estevez and Lecocq, 2018].
//

namespace renderer {

class BSDFProxy
{
  public:
    BSDFProxy();
    void add_diffuse_weight(const float         diffuse_weight);
    void add_translucency_weight(const float    diffuse_weight);
    void add_reflection_weight(
        const float                             reflection_weight,
        const float                             roughness);
    void add_refraction_weight(
        const float                             refraction_weight,
        const float                             roughness);
    void set_IOR(const float                    IOR);
    void finish_parameterization(
        const foundation::Vector3f&             outgoing,
        const foundation::Vector3f&             shading_normal);
    float evaluate(
      const foundation::Vector3f&               incoming) const;
    bool is_zero() const;

  // TO-DO:
  // Roughness scaling after parameterization.

  private:
    float m_diffuse_weight, m_reflection_weight, m_refraction_weight, m_translucency_weight;
    float m_reflection_roughness, m_refraction_roughness;
    float m_IOR;
    bool  m_is_diffuse, m_is_translucent, m_is_reflective, m_is_refractive;

    foundation::Vector3f m_normal;
    foundation::Vector3f m_reflection_lobe;
    foundation::Vector3f m_refraction_lobe;
};

}   // namespace renderer