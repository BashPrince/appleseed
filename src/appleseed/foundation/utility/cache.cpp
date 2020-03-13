
//
// This source file is part of appleseed.
// Visit https://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2010-2013 Francois Beaune, Jupiter Jazz Limited
// Copyright (c) 2014-2018 Francois Beaune, The appleseedhq Organization
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
#include "cache.h"

// appleseed.foundation headers.
#include "foundation/string/string.h"

namespace foundation
{

namespace cache_impl
{
    CacheStatisticsEntry::CacheStatisticsEntry(
        const std::string&      name,
        const std::uint64_t     hit_count,
        const std::uint64_t     miss_count)
      : Statistics::Entry(name)
    {
        m_hit_count = hit_count;
        m_miss_count = miss_count;
    }

    std::unique_ptr<Statistics::Entry> CacheStatisticsEntry::clone() const
    {
        return std::unique_ptr<Entry>(new CacheStatisticsEntry(*this));
    }

    void CacheStatisticsEntry::merge(const Entry* other)
    {
        const CacheStatisticsEntry* typed_other = cast<CacheStatisticsEntry>(other);

        m_hit_count += typed_other->m_hit_count;
        m_miss_count += typed_other->m_miss_count;
    }

    std::string CacheStatisticsEntry::to_string() const
    {
        const std::uint64_t accesses = m_hit_count + m_miss_count;

        if (accesses == 0)
            return "n/a";

        return
                "efficiency " + pretty_percent(m_hit_count, accesses)
            + "  accesses " + pretty_uint(accesses)
            + "  hits " + pretty_uint(m_hit_count)
            + "  misses " + pretty_uint(m_miss_count);
    }
}

}   // namespace foundation
