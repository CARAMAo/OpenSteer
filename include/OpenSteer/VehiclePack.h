// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2005, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
//
// AbstractVehicle: pure virtual base class for generic steerable vehicles
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 01-30-03 cwr: created 
//
//
// ----------------------------------------------------------------------------

#ifndef OPENSTEER_ABSTRACTVEHICLEPACK_H
#define OPENSTEER_ABSTRACTVEHICLEPACK_H


#include <array>
#include <immintrin.h>

// STL vector containers
#include <vector>


// ----------------------------------------------------------------------------

namespace OpenSteer {

    class Vec3Pack{
        public:
            __m256 x;
            __m256 y;
            __m256 z;
    };
   
    class VehiclePack
    {
    public:
        Vec3Pack position;
        Vec3Pack forward;
    };



} // namespace OpenSteer


// ----------------------------------------------------------------------------
#endif // OPENSTEER_ABSTRACTVEHICLEPACK_H
