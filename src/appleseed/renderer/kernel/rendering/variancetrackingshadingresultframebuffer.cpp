
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
#include "variancetrackingshadingresultframebuffer.h"

// appleseed.renderer headers.
#include "renderer/kernel/aov/tilestack.h"
#include "renderer/kernel/shading/shadingresult.h"

// appleseed.foundation headers.
#include "foundation/image/color.h"
#include "foundation/image/colorspace.h"
#include "foundation/image/tile.h"
#include "foundation/platform/compiler.h"

// Standard headers.
#include <cassert>

using namespace foundation;
using namespace std;

namespace renderer
{

VarianceTrackingShadingResultFrameBuffer::VarianceTrackingShadingResultFrameBuffer(
    const size_t                    width,
    const size_t                    height,
    const size_t                    aov_count)
  : ShadingResultFrameBuffer(
        width,
        height,
        aov_count + 1)
    , m_aov_count(aov_count)
{
}

VarianceTrackingShadingResultFrameBuffer::VarianceTrackingShadingResultFrameBuffer(
    const size_t                    width,
    const size_t                    height,
    const size_t                    aov_count,
    const AABB2u&                   crop_window)
  : ShadingResultFrameBuffer(
        width,
        height,
        aov_count + 1,
        crop_window)
    , m_aov_count(aov_count)
{
}

void VarianceTrackingShadingResultFrameBuffer::add(
    const Vector2u&                 pi,
    const ShadingResult&            sample)
{
    float* ptr = &m_scratch[0];

    const float main_0 = sample.m_main[0]; 
    const float main_1 = sample.m_main[1]; 
    const float main_2 = sample.m_main[2]; 
    const float main_3 = sample.m_main[3]; 

    *ptr++ = main_0;
    *ptr++ = main_1;
    *ptr++ = main_2;
    *ptr++ = main_3;

    for (size_t i = 0, e = m_aov_count; i < e; ++i)
    {
        const Color4f& aov = sample.m_aovs[i];
        *ptr++ = aov[0];
        *ptr++ = aov[1];
        *ptr++ = aov[2];
        *ptr++ = aov[3];
    }

    *ptr++ = main_0 * main_0;
    *ptr++ = main_1 * main_1;
    *ptr++ = main_2 * main_2;
    *ptr++ = main_3 * main_3;

    AccumulatorTile::add(pi, &m_scratch[0]);
}

void VarianceTrackingShadingResultFrameBuffer::develop_to_tile(
    Tile&                           tile,
    TileStack&                      aov_tiles) const
{
    const float* ptr = pixel(0);

    for (size_t y = 0, h = m_height; y < h; ++y)
    {
        for (size_t x = 0, w = m_width; x < w; ++x)
        {
            const float weight = *ptr++;
            const float rcp_weight = weight == 0.0f ? 0.0f : 1.0f / weight;

            const Color4f color(ptr[0], ptr[1], ptr[2], ptr[3]);
            tile.set_pixel(x, y, color * rcp_weight);
            ptr += 4;

            for (size_t i = 0, e = m_aov_count; i < e; ++i)
            {
                const Color4f aov(ptr[0], ptr[1], ptr[2], ptr[3]);
                aov_tiles.set_pixel(x, y, i, aov * rcp_weight);
                ptr += 4;
            }

            ptr += 4; // Skip sum of squared samples
        }
    }
}

float VarianceTrackingShadingResultFrameBuffer::variance(
    const size_t                    num_samples) const
{
    assert(num_samples > 0);

    float tile_variance = 0.0f;
    const float* ptr = pixel(0);

    for (size_t y = 0, h = m_height; y < h; ++y)
    {
        for (size_t x = 0, w = m_width; x < w; ++x)
        {
            const float weight = *ptr;
            const float rcp_weight = weight == 0.0f ? 0.0f : 1.0f / weight;

            const Color3f color(
                ptr[1] * rcp_weight,
                ptr[2] * rcp_weight,
                ptr[3] * rcp_weight);

            ptr += m_channel_count - 4; // skip to beginning of summed squares

            const Color3f summed_squares(
                ptr[0] * rcp_weight,
                ptr[1] * rcp_weight,
                ptr[2] * rcp_weight);

            const Color3f pixel_variance(summed_squares - (color * color) / num_samples);

            tile_variance += std::min(luminance(pixel_variance), 10000.0f);

            ptr += 4;
        }
    }

    return tile_variance;
}

}   // namespace renderer
