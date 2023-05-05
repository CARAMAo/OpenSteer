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
// Vec3Pack: OpenSteer's generic type for 3d vectors
//
// This file defines the class Vec3Pack, which is used throughout OpenSteer to
// manipulate 3d geometric data.  It includes standard vector operations (like
// vector addition, subtraction, scale, dot, cross...) and more idiosyncratic
// utility functions.
//
// When integrating OpenSteer into a preexisting 3d application, it may be
// important to use the 3d vector type of that application.  In that case Vec3Pack
// can be changed to inherit from the preexisting application' vector type and
// to match the interface used by OpenSteer to the interface provided by the
// preexisting 3d vector type.
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 03-26-03 cwr: created to replace for Hiranabe-san's execellent but larger
//               vecmath package (http://objectclub.esm.co.jp/vecmath/)
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_Vec3Pack_H
#define OPENSTEER_Vec3Pack_H


#include "OpenSteer/Utilities.h"  // for interpolate, etc.
#include <array>
#include <immintrin.h>

#define PACKSIZE 8

namespace OpenSteer {

    // ----------------------------------------------------------------------------
    class Vec3Pack
    {
    public:

        // three-dimensional Cartesian coordinates
        std::array<float,PACKSIZE> x,y,z;

        // constructors
        Vec3Pack (void){
            x.fill(0.f);
            y.fill(0.f);
            z.fill(0.f);
        }

        Vec3Pack (float X, float Y, float Z) {
            x.fill(X);
            y.fill(Y);
            z.fill(Z);
        }

        Vec3Pack (std::array<float,PACKSIZE> X, std::array<float,PACKSIZE> Y, std::array<float,PACKSIZE> Z){
            x = X;
            y = Y;
            z = Z;
        }

        Vec3Pack(const Vec3Pack &v1){
            for(int i = 0; i < PACKSIZE;i++){
                x[i] = v1.x[i];
                y[i] = v1.y[i];
                z[i] = v1.z[i];
            }
        }

        // assignment
        Vec3Pack operator= (const Vec3Pack& v) {
            for(int i = 0; i< PACKSIZE; i++)
            {
                x[i]=v.x[i]; 
                y[i]=v.y[i]; 
                z[i]=v.z[i];
            }
            return *this;
        }

        // set XYZ coordinates to given three floats
        Vec3Pack set (const float _x, const float _y, const float _z)
        {x.fill(_x); y.fill(_y); z.fill(_z); return *this;}

        // set XYZ coordinates to given three floats
        Vec3Pack set (std::array<float,PACKSIZE> _x, std::array<float,PACKSIZE> _y, std::array<float,PACKSIZE> _z)
        {
        std::copy(_x.begin(),_x.end(),x.begin());
        std::copy(_y.begin(),_y.end(),y.begin());
        std::copy(_z.begin(),_z.end(),z.begin());
        return *this;
        }

    };
    
} // namespace OpenSteer
    

// ----------------------------------------------------------------------------
#endif // OPENSTEER_Vec3Pack_H
