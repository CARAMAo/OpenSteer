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


//         // vector addition
//         Vec3Pack operator+ (const Vec3Pack& v) const {
//             __m256 x1 = _mm256_loadu_ps(&x[0]);
//             __m256 y1 = _mm256_loadu_ps(&y[0]);
//             __m256 z1 = _mm256_loadu_ps(&z[0]);

//             __m256 x2 = _mm256_loadu_ps(&v.x[0]);
//             __m256 y2 = _mm256_loadu_ps(&v.y[0]);
//             __m256 z2 = _mm256_loadu_ps(&v.z[0]);

//             __m256 xres = _mm256_add_ps(x1,x2);
//             __m256 yres = _mm256_add_ps(y1,y2);
//             __m256 zres = _mm256_add_ps(z1,z2);

//             std::array<float,PACKSIZE> X,Y,Z;

//             _mm256_storeu_ps(&X[0],xres);
//             _mm256_storeu_ps(&Y[0],yres);
//             _mm256_storeu_ps(&Z[0],zres);

//             return Vec3Pack (X,Y,Z);
//         }

//         // vector subtraction
//         Vec3Pack operator- (const Vec3Pack& v) const {
//             __m256 x1 = _mm256_loadu_ps(&x[0]);
//             __m256 y1 = _mm256_loadu_ps(&y[0]);
//             __m256 z1 = _mm256_loadu_ps(&z[0]);

//             __m256 x2 = _mm256_loadu_ps(&v.x[0]);
//             __m256 y2 = _mm256_loadu_ps(&v.y[0]);
//             __m256 z2 = _mm256_loadu_ps(&v.z[0]);

//             __m256 xres = _mm256_sub_ps(x1,x2);
//             __m256 yres = _mm256_sub_ps(y1,y2);
//             __m256 zres = _mm256_sub_ps(z1,z2);

//             std::array<float,PACKSIZE> X,Y,Z;

//             _mm256_storeu_ps(&X[0],xres);
//             _mm256_storeu_ps(&Y[0],yres);
//             _mm256_storeu_ps(&Z[0],zres);

//             return Vec3Pack (X,Y,Z);
//         }
// //         // unary minus
// //         Vec3Pack operator- (void) const {return Vec3Pack (0.0,0.,0.);}

// //         // vector times scalar product (scale length of vector times argument)
// //         Vec3Pack operator* (const float s) const {return Vec3Pack (x * s, y * s, z * s);}

//         // vector divided by a scalar (divide length of vector by argument)
//         Vec3Pack operator/ (const std::array<float,PACKSIZE> s) const {
//             __m256 x1 = _mm256_loadu_ps(&x[0]);
//             __m256 y1 = _mm256_loadu_ps(&y[0]);
//             __m256 z1 = _mm256_loadu_ps(&z[0]);

//             __m256 ss = _mm256_loadu_ps(&s[0]);

//             __m256 xres = _mm256_div_ps(x1,s);
//             __m256 yres = _mm256_sub_ps(y1,s);
//             __m256 zres = _mm256_sub_ps(z1,s);

//             std::array<float,PACKSIZE> X,Y,Z;

//             _mm256_storeu_ps(&X[0],xres);
//             _mm256_storeu_ps(&Y[0],yres);
//             _mm256_storeu_ps(&Z[0],zres);

//             return Vec3Pack (X,Y,Z);
//         }

//         // dot product
//         std::array<float,PACKSIZE> dot (const Vec3Pack& v) const {
//             __m256 x1 = _mm256_loadu_ps(&x[0]);
//             __m256 y1 = _mm256_loadu_ps(&y[0]);
//             __m256 z1 = _mm256_loadu_ps(&z[0]);

//             __m256 x2 = _mm256_loadu_ps(&v.x[0]);
//             __m256 y2 = _mm256_loadu_ps(&v.y[0]);
//             __m256 z2 = _mm256_loadu_ps(&v.z[0]);

//             __m256 xres = _mm256_mul_ps(x1,x2);
//             __m256 yres = _mm256_mul_ps(y1,y2);
//             __m256 zres = _mm256_mul_ps(z1,z2);

//             std::array<float,PACKSIZE> X,Y,Z;

//             _mm256_storeu_ps(&X[0],xres);
//             _mm256_storeu_ps(&Y[0],yres);
//             _mm256_storeu_ps(&Z[0],zres);

//             return Vec3Pack (X,Y,Z);
//         }

//         // length
//         float length (void) const {return sqrtXXX (lengthSquared ());}

//         // length squared
//         float lengthSquared (void) const {return this->dot (*this);}

//         // normalize: returns normalized version (parallel to this, length = 1)
//         Vec3Pack normalize (void) const
//         {
//             // skip divide if length is zero
//             const float len = length ();
//             return (len>0) ? (*this)/len : (*this);
//         }

//         // cross product (modify "*this" to be A x B)
//         // [XXX  side effecting -- deprecate this function?  XXX]
//         void cross(const Vec3Pack& a, const Vec3Pack& b)
//         {
//             *this = Vec3Pack ((a.y * b.z) - (a.z * b.y),
//                           (a.z * b.x) - (a.x * b.z),
//                           (a.x * b.y) - (a.y * b.x));
//         }

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

//         // +=
//         Vec3Pack operator+= (const Vec3Pack& v) {return *this = (*this + v);}

//         // -=
//         Vec3Pack operator-= (const Vec3Pack& v) {return *this = (*this - v);}

//         // *=
//         Vec3Pack operator*= (const float& s) {return *this = (*this * s);}

        
//         Vec3Pack operator/=( float d ) { return *this = (*this / d);  }
        
//         // equality/inequality
//         bool operator== (const Vec3Pack& v) const {return x==v.x && y==v.y && z==v.z;}
//         bool operator!= (const Vec3Pack& v) const {return !(*this == v);}

//         // @todo Remove - use @c distance from the Vec3PackUtilitites header instead.
//         // XXX experimental (4-1-03 cwr): is this the right approach?  defining
//         // XXX "Vec3Pack distance (Vec3Pack, Vec3Pack)" collided with STL's distance template.
//         static float distance (const Vec3Pack& a, const Vec3Pack& b){ return(a-b).length();}

//         // --------------------------- utility member functions used in OpenSteer

//         // return component of vector parallel to a unit basis vector
//         // (IMPORTANT NOTE: assumes "basis" has unit magnitude (length==1))

//         inline Vec3Pack parallelComponent (const Vec3Pack& unitBasis) const
//         {
//             const float projection = this->dot (unitBasis);
//             return unitBasis * projection;
//         }

//         // return component of vector perpendicular to a unit basis vector
//         // (IMPORTANT NOTE: assumes "basis" has unit magnitude (length==1))

//         inline Vec3Pack perpendicularComponent (const Vec3Pack& unitBasis) const
//         {
//             return (*this) - parallelComponent (unitBasis);
//         }

//         // clamps the length of a given vector to maxLength.  If the vector is
//         // shorter its value is returned unaltered, if the vector is longer
//         // the value returned has length of maxLength and is paralle to the
//         // original input.

//         Vec3Pack truncateLength (const float maxLength) const
//         {
//             const float maxLengthSquared = maxLength * maxLength;
//             const float vecLengthSquared = this->lengthSquared ();
//             if (vecLengthSquared <= maxLengthSquared)
//                 return *this;
//             else
//                 return (*this) * (maxLength / sqrtXXX (vecLengthSquared));
//         }

//         // forces a 3d position onto the XZ (aka y=0) plane

//         Vec3Pack setYtoZero (void) const {return Vec3Pack (this->x, 0, this->z);}

//         // rotate this vector about the global Y (up) axis by the given angle

//         Vec3Pack rotateAboutGlobalY (float angle) const 
//         {
//             const float s = sinXXX (angle);
//             const float c = cosXXX (angle);
//             return Vec3Pack ((this->x * c) + (this->z * s),
//                          (this->y),
//                          (this->z * c) - (this->x * s));
//         }

//         // version for caching sin/cos computation
//         Vec3Pack rotateAboutGlobalY (float angle, float& sin, float& cos) const 
//         {
//             // is both are zero, they have not be initialized yet
//             if (sin==0 && cos==0)
//             {
//                 sin = sinXXX (angle);
//                 cos = cosXXX (angle);
//             }
//             return Vec3Pack ((this->x * cos) + (this->z * sin),
//                          (this->y),
//                          (this->z * cos) - (this->x * sin));
//         }

//         // if this position is outside sphere, push it back in by one diameter

//         Vec3Pack sphericalWrapAround (const Vec3Pack& center, float radius)
//         {
//             const Vec3Pack offset = *this - center;
//             const float r = offset.length();
//             if (r > radius)
//                 return *this + ((offset/r) * radius * -2);
//             else
//                 return *this;
//         }

//         // names for frequently used vector constants
//         static const Vec3Pack zero;
//         static const Vec3Pack side;
//         static const Vec3Pack up;
//         static const Vec3Pack forward;
//     };


//     // ----------------------------------------------------------------------------
//     // scalar times vector product ("float * Vec3Pack")


//     inline Vec3Pack operator* (float s, const Vec3Pack& v) {return v*s;}


// 	// return cross product a x b
// 	inline Vec3Pack crossProduct(const Vec3Pack& a, const Vec3Pack& b)
// 	{
// 		Vec3Pack result((a.y * b.z) - (a.z * b.y),
// 					(a.z * b.x) - (a.x * b.z),
// 					(a.x * b.y) - (a.y * b.x));
// 		return result;
// 	}


//     // ----------------------------------------------------------------------------
//     // default character stream output method

// #ifndef NOT_OPENSTEERDEMO  // only when building OpenSteerDemo

//     inline std::ostream& operator<< (std::ostream& o, const Vec3Pack& v)
//     {
//         return o << "(" << v.x << "," << v.y << "," << v.z << ")";
//     }

// #endif // NOT_OPENSTEERDEMO

//     // ----------------------------------------------------------------------------
//     // Returns a position randomly distributed inside a sphere of unit radius
//     // centered at the origin.  Orientation will be random and length will range
//     // between 0 and 1


//     Vec3Pack RandomVectorInUnitRadiusSphere (void);


//     // ----------------------------------------------------------------------------
//     // Returns a position randomly distributed on a disk of unit radius
//     // on the XZ (Y=0) plane, centered at the origin.  Orientation will be
//     // random and length will range between 0 and 1


//     Vec3Pack randomVectorOnUnitRadiusXZDisk (void);


//     // ----------------------------------------------------------------------------
//     // Returns a position randomly distributed on the surface of a sphere
//     // of unit radius centered at the origin.  Orientation will be random
//     // and length will be 1


//     inline Vec3Pack RandomUnitVector (void)
//     {
//         return RandomVectorInUnitRadiusSphere().normalize();
//     }


//     // ----------------------------------------------------------------------------
//     // Returns a position randomly distributed on a circle of unit radius
//     // on the XZ (Y=0) plane, centered at the origin.  Orientation will be
//     // random and length will be 1


//     inline Vec3Pack RandomUnitVectorOnXZPlane (void)
//     {
//         return RandomVectorInUnitRadiusSphere().setYtoZero().normalize();
//     }


//     // ----------------------------------------------------------------------------
//     // used by limitMaxDeviationAngle / limitMinDeviationAngle below


//     Vec3Pack vecLimitDeviationAngleUtility (const bool insideOrOutside,
//                                         const Vec3Pack& source,
//                                         const float cosineOfConeAngle,
//                                         const Vec3Pack& basis);


//     // ----------------------------------------------------------------------------
//     // Enforce an upper bound on the angle by which a given arbitrary vector
//     // diviates from a given reference direction (specified by a unit basis
//     // vector).  The effect is to clip the "source" vector to be inside a cone
//     // defined by the basis and an angle.


//     inline Vec3Pack limitMaxDeviationAngle (const Vec3Pack& source,
//                                         const float cosineOfConeAngle,
//                                         const Vec3Pack& basis)
//     {
//         return vecLimitDeviationAngleUtility (true, // force source INSIDE cone
//                                               source,
//                                               cosineOfConeAngle,
//                                               basis);
//     }


//     // ----------------------------------------------------------------------------
//     // Enforce a lower bound on the angle by which a given arbitrary vector
//     // diviates from a given reference direction (specified by a unit basis
//     // vector).  The effect is to clip the "source" vector to be outside a cone
//     // defined by the basis and an angle.


//     inline Vec3Pack limitMinDeviationAngle (const Vec3Pack& source,
//                                         const float cosineOfConeAngle,
//                                         const Vec3Pack& basis)
//     {    
//         return vecLimitDeviationAngleUtility (false, // force source OUTSIDE cone
//                                               source,
//                                               cosineOfConeAngle,
//                                               basis);
//     }


//     // ----------------------------------------------------------------------------
//     // Returns the distance between a point and a line.  The line is defined in
//     // terms of a point on the line ("lineOrigin") and a UNIT vector parallel to
//     // the line ("lineUnitTangent")


//     inline float distanceFromLine (const Vec3Pack& point,
//                                    const Vec3Pack& lineOrigin,
//                                    const Vec3Pack& lineUnitTangent)
//     {
//         const Vec3Pack offset = point - lineOrigin;
//         const Vec3Pack perp = offset.perpendicularComponent (lineUnitTangent);
//         return perp.length();
//     }


//     // ----------------------------------------------------------------------------
//     // given a vector, return a vector perpendicular to it (note that this
//     // arbitrarily selects one of the infinitude of perpendicular vectors)


//     Vec3Pack findPerpendicularIn3d (const Vec3Pack& direction);


//     // ----------------------------------------------------------------------------
//     // candidates for global utility functions
//     //
//     // dot
//     // cross
//     // length
//     // distance
//     // normalized
    };
    
} // namespace OpenSteer
    

// ----------------------------------------------------------------------------
#endif // OPENSTEER_Vec3Pack_H
