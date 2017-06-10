
//
// This source file is part of appleseed.
// Visit http://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2010-2013 Francois Beaune, Jupiter Jazz Limited
// Copyright (c) 2014-2017 Francois Beaune, The appleseedhq Organization
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
#include "openfilelogtarget.h"

// appleseed.foundation headers.
#include "foundation/platform/compiler.h"
#include "foundation/utility/log/filelogtargetbase.h"
#include "foundation/utility/log/logmessage.h"

// Standard headers.
#include <cstddef>

using namespace std;

namespace foundation
{

namespace
{
    //
    // A log target that outputs to an open std::FILE.
    //

    class OpenFileLogTarget
      : public FileLogTargetBase
    {
      public:
        // Constructor.
        explicit OpenFileLogTarget(FILE* file)
          : m_file(file)
        {
        }

        // Delete this instance.
        virtual void release() override
        {
            delete this;
        }

        // Write a message.
        virtual void write(
            const LogMessage::Category  category,
            const char*                 file,
            const size_t                line,
            const char*                 header,
            const char*                 message) override
        {
            write_message(m_file, category, header, message);
        }

      private:
        FILE* m_file;
    };
}


//
// Create an instance of a log target that outputs to an open std::FILE.
//

ILogTarget* create_open_file_log_target(FILE* file)
{
    return new OpenFileLogTarget(file);
}

}   // namespace foundation
