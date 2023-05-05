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
// LocalSpace: a local coordinate system for 3d space
//
// Provide functionality such as transforming from local space to global
// space and vice versa.  Also regenerates a valid space from a perturbed
// "forward vector" which is the basis of abnstract vehicle turning.
//
// These are comparable to a 4x4 homogeneous transformation matrix where the
// 3x3 (R) portion is constrained to be a pure rotation (no shear or scale).
// The rows of the 3x3 R matrix are the basis vectors of the space.  They are
// all constrained to be mutually perpendicular and of unit length.  The top
// ("x") row is called "side", the middle ("y") row is called "up" and the
// bottom ("z") row is called forward.  The translation vector is called
// "position".  Finally the "homogeneous column" is always [0 0 0 1].
//
//     [ R R R  0 ]      [ Sx Sy Sz  0 ]
//     [ R R R  0 ]      [ Ux Uy Uz  0 ]
//     [ R R R  0 ]  ->  [ Fx Fy Fz  0 ]
//     [          ]      [             ]
//     [ T T T  1 ]      [ Tx Ty Tz  1 ]
//
// This file defines three classes:
//   AbstractLocalSpacePack:  pure virtual interface
//   LocalSpacePackMixin:     mixin to layer LocalSpace functionality on any base
//   LocalSpace:          a concrete object (can be instantiated)
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 06-05-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_LOCALSPACEPACK_H
#define OPENSTEER_LOCALSPACEPACK_H


#include "OpenSteer/Vec3Pack.h"
#include "OpenSteer/Vec3Pack.h"
// ----------------------------------------------------------------------------


namespace OpenSteer {

    class AbstractLocalSpacePack
    {
    public:

        virtual ~AbstractLocalSpacePack() { /* Nothing to do. */ }
        

        // accessors (get and set) for side, up, forward and position
        virtual Vec3Pack side (void) const = 0;
        virtual Vec3Pack setSide (Vec3Pack s) = 0;
        virtual Vec3Pack up (void) const = 0;
        virtual Vec3Pack setUp (Vec3Pack u) = 0;
        virtual Vec3Pack forward (void) const = 0;
        virtual Vec3Pack setForward (Vec3Pack f) = 0;
        virtual Vec3Pack position (void) const = 0;
        virtual Vec3Pack setPosition (Vec3Pack p) = 0;
        virtual Vec3Pack setSide     (std::array<float,PACKSIZE> x, std::array<float,PACKSIZE> y, std::array<float,PACKSIZE> z) = 0;
        virtual Vec3Pack setUp       (std::array<float,PACKSIZE> x, std::array<float,PACKSIZE> y, std::array<float,PACKSIZE> z) = 0;
        virtual Vec3Pack setForward  (std::array<float,PACKSIZE> x, std::array<float,PACKSIZE> y, std::array<float,PACKSIZE> z) = 0;
        virtual Vec3Pack setPosition (std::array<float,PACKSIZE> x, std::array<float,PACKSIZE> y, std::array<float,PACKSIZE> z) = 0;
        virtual void setSide     (Vec3 v,int i) = 0;
        virtual void setUp       (Vec3 v,int i) = 0;
        virtual void setForward  (Vec3 v,int i) = 0;
        virtual void setPosition (Vec3 v,int i) = 0;
        
    };


    // ----------------------------------------------------------------------------
    // LocalSpacePackMixin is a mixin layer, a class template with a paramterized base
    // class.  Allows "LocalSpace-ness" to be layered on any class.


    template <class Super>
    class LocalSpacePackMixin : public Super
    {
        // transformation as three orthonormal unit basis vectors and the
        // origin of the local space.  These correspond to the "rows" of
        // a 3x4 transformation matrix with [0 0 0 1] as the final column

    private:
        Vec3Pack _side;     //    side-pointing unit basis vector
        Vec3Pack _up;       //  upward-pointing unit basis vector
        Vec3Pack _forward;  // forward-pointing unit basis vector
        Vec3Pack _position; // origin of local space
        int _size;

    public:

        // accessors (get and set) for side, up, forward and position
        Vec3Pack side     (void) const {return _side;};
        Vec3Pack up       (void) const {return _up;};
        Vec3Pack forward  (void) const {return _forward;};
        Vec3Pack position (void) const {return _position;};
        Vec3Pack setSide     (Vec3Pack s) {return _side = s;};
        Vec3Pack setUp       (Vec3Pack u) {return _up = u;};
        Vec3Pack setForward  (Vec3Pack f) {return _forward = f;};
        Vec3Pack setPosition (Vec3Pack p) {return _position = p;};
        Vec3Pack setSide     (std::array<float,PACKSIZE> x, std::array<float,PACKSIZE> y, std::array<float,PACKSIZE> z){return _side.set    (x,y,z);};
        Vec3Pack setUp       (std::array<float,PACKSIZE> x, std::array<float,PACKSIZE> y, std::array<float,PACKSIZE> z){return _up.set      (x,y,z);};
        Vec3Pack setForward  (std::array<float,PACKSIZE> x, std::array<float,PACKSIZE> y, std::array<float,PACKSIZE> z){return _forward.set (x,y,z);};
        Vec3Pack setPosition (std::array<float,PACKSIZE> x, std::array<float,PACKSIZE> y, std::array<float,PACKSIZE> z){return _position.set(x,y,z);};
        void setSide     (Vec3 v,int i){ _side.x[i] = v.x; _side.y[i] = v.y; _side.z[i] = v.z; };
        void setUp       (Vec3 v,int i){ _up.x[i] = v.x; _up.y[i] = v.y; _up.z[i] = v.z; };
        void setForward  (Vec3 v,int i){ _forward.x[i] = v.x; _forward.y[i] = v.y; _forward.z[i] = v.z; };
        void setPosition (Vec3 v,int i){ _position.x[i] = v.x; _position.y[i] = v.y; _position.z[i] = v.z; };

        // ------------------------------------------------------------------------
        // Global compile-time switch to control handedness/chirality: should
        // LocalSpace use a left- or right-handed coordinate system?  This can be
        // overloaded in derived types (e.g. vehicles) to change handedness.

        bool rightHanded (void) const {return true;}


        // ------------------------------------------------------------------------
        // constructors


        LocalSpacePackMixin (void)
        {
            _size = 0;
        };

        LocalSpacePackMixin (const Vec3Pack& Side,
                         const Vec3Pack& Up,
                         const Vec3Pack& Forward,
                         const Vec3Pack& Position)
            : _side( Side ), _up( Up ), _forward( Forward ), _position( Position ) { _size = 0; }


        LocalSpacePackMixin (const Vec3Pack& Up,
                         const Vec3Pack& Forward,
                         const Vec3Pack& Position)
            : _side(), _up( Up ), _forward( Forward ), _position( Position )
        {
            _size = 0;
        }

        
        virtual ~LocalSpacePackMixin() { /* Nothing to do. */ }
        

       
    };

} // namespace OpenSteer

// ----------------------------------------------------------------------------
#endif // OPENSTEER_LOCALSPACE_H
