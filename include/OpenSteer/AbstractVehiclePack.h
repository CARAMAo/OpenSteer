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


#include "OpenSteer/LocalSpacePack.h"
#include <array>
#include "OpenSteer/Vec3Pack.h"

// STL vector containers
#include <vector>


// ----------------------------------------------------------------------------

namespace OpenSteer {
   
    class AbstractVehiclePack : public AbstractLocalSpacePack,public Pack<AbstractVehicle*>
    {
    public:
        AbstractVehiclePack(){}
         ~AbstractVehiclePack() { /* Nothing to do. */ }
        
        // // mass (defaults to unity so acceleration=force)
        // virtual std::array<float,PACKSIZE> mass (void) const = 0;
        // virtual std::array<float,PACKSIZE> setMass (std::array<float,PACKSIZE>) = 0;

        // // size of bounding sphere, for obstacle avoidance, etc.
        // virtual std::array<float,PACKSIZE> radius (void) const = 0;
        // virtual std::array<float,PACKSIZE> setRadius (std::array<float,PACKSIZE>) = 0;
        void push(AbstractVehicle* v){
            int i = size();
            if( i < PACKSIZE){
                setUp(v->up(),i);
                setSide(v->side(),i);
                setForward(v->forward(),i);
                setPosition(v->position(),i);
                setSize(i+1);
            }
        }
        // // velocity of vehicle
        // virtual Vec3Pack velocity (void) const = 0;

        // // speed of vehicle  (may be faster than taking magnitude of velocity)
        // virtual std::array<float,PACKSIZE> speed (void) const = 0;
        // virtual std::array<float,PACKSIZE> setSpeed (std::array<float,PACKSIZE>) = 0;
        
        // groups of (pointers to) abstract vehicles, and iterators over them
        

        // // predict position of this vehicle at some time in the future
        // // (assumes velocity remains constant)
        // virtual Vec3Pack predictFuturePosition (const float predictionTime) const = 0;

        // // ----------------------------------------------------------------------
        // // XXX this vehicle-model-specific functionality functionality seems out
        // // XXX of place on the abstract base class, but for now it is expedient

        // // the maximum steering force this vehicle can apply
        // virtual std::array<float,PACKSIZE> maxForce (void) const = 0;
        // virtual std::array<float,PACKSIZE> setMaxForce (std::array<float,PACKSIZE>) = 0;

        // // the maximum speed this vehicle is allowed to move
        // virtual std::array<float,PACKSIZE> maxSpeed (void) const = 0;
        // virtual std::array<float,PACKSIZE> setMaxSpeed (std::array<float,PACKSIZE>) = 0;

		// // dp - added to support heterogeneous flocks
		// virtual void update(const float currentTime, const float elapsedTime) = 0;
        
    };


    // more convenient short names for AbstractVehiclePack group and iterator
    
    typedef LocalSpacePackMixin<AbstractVehiclePack> AVPack;
    typedef std::vector<AVPack*> AVPackGroup;
    typedef AVPackGroup::const_iterator AVPackIterator;
} // namespace OpenSteer


// ----------------------------------------------------------------------------
#endif // OPENSTEER_ABSTRACTVEHICLEPACK_H
