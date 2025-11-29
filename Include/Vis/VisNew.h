/***********************************************************************
 **
 ** Copyright (c) 2012-2024 RVBUST Inc.
 **
 ** Permission is hereby granted, free of charge, to any person obtaining
 ** a copy of this software and associated documentation files (the
 ** "Software"), to deal in the Software without restriction, including
 ** without limitation the rights to use, copy, modify, merge, publish,
 ** distribute, sublicense, and/or sell copies of the Software, and to
 ** permit persons to whom the Software is furnished to do so, subject to
 ** the following conditions:
 **
 ** The above copyright notice and this permission notice shall be
 ** included in all copies or substantial portions of the Software.
 **
 ** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 ** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 ** MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 ** NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 ** LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 ** OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 ** WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ***********************************************************************/

/**
 * @file VisNew.h
 * @brief Main header file for the Vis library (new architecture).
 * 
 * This header includes all public API headers for the Vis visualization library.
 * Simply include this file to access all Vis functionality.
 * 
 * @code
 * #include <Vis/VisNew.h>
 * 
 * int main() {
 *     Vis::View view("My Visualization");
 *     view.Axes({0, 0, 0}, {0, 0, 0, 1}, 1.0f, 3.0f);
 *     view.Box({0, 0, 0}, {0.5, 0.5, 0.5}, {1, 0, 0});
 *     // ...
 * }
 * @endcode
 */

#pragma once

// Export macros and version info
#include "Export.h"

// Basic types (Vec3, Quat, Color, Transform, etc.)
#include "Types.h"

// Handle system
#include "Handle.h"

// Main View class
#include "View.h"

//============================================================================
// Library information
//============================================================================

namespace Vis {

/**
 * @brief Set the global log level.
 * @param levelname One of: "trace", "debug", "info", "warn", "error", "off"
 */
void SetLogLevel(const std::string& levelname);

/**
 * @brief Get the library version string.
 */
const char* GetVersion();

}  // namespace Vis

