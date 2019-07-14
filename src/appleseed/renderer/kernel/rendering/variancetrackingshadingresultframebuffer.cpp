
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

    // Put squared samples in the last buffer channels.
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

float VarianceTrackingShadingResultFrameBuffer::variance() const
{
    float tile_variance = 0.0f;
    const float* ptr = pixel(0);

    for (size_t y = 0, h = m_height; y < h; ++y)
        for (size_t x = 0, w = m_width; x < w; ++x)
        {
            const float weight = *ptr;

            const Color3f sample_sum(
                ptr[1],
                ptr[2],
                ptr[3]);

            ptr += m_channel_count - 4; // skip to beginning of summed squares

            const Color3f square_sum(
                ptr[0],
                ptr[1],
                ptr[2]);

            // Variance estimator: (1 / n) * Sum_i[(X_i - µ)²] = Sum_i[X_i²] - Sum_i[X_i]² / n .
            const Color3f pixel_variance(square_sum - (sample_sum * sample_sum) / weight);

            // Clamp values to mitigate the effect of fireflies.
            tile_variance += std::min(luminance(pixel_variance), 10000.0f);

            ptr += 4;
        }

    return tile_variance;
}

float VarianceTrackingShadingResultFrameBuffer::variance_to_tile(
    Tile&                           tile) const
{
    float tile_variance = 0.0f;
    const float* ptr = pixel(0);

    for (size_t y = 0, h = m_height; y < h; ++y)
        for (size_t x = 0, w = m_width; x < w; ++x)
        {
            const float weight = *ptr;

            const Color3f sample_sum(
                ptr[1],
                ptr[2],
                ptr[3]);

            ptr += m_channel_count - 4; // skip to beginning of summed squares

            const Color3f square_sum(
                ptr[0],
                ptr[1],
                ptr[2]);

            // Variance estimator: (1 / n) * Sum_i[(X_i - µ)²] = Sum_i[X_i²] - Sum_i[X_i]² / n .
            const Color3f pixel_variance(square_sum - (sample_sum * sample_sum) / weight);

            // Clamp values to mitigate the effect of fireflies.
            const float variance_luminance = std::min(luminance(pixel_variance), 10000.0f);
            tile_variance += variance_luminance;
            tile.set_pixel(x, y, Color3f(variance_luminance));

            ptr += 4;
        }

    return tile_variance;
}

}   // namespace renderer
