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
// OpenSteer Boids
// 
// 09-26-02 cwr: created 
//
//
// ----------------------------------------------------------------------------

#include <chrono>
#include <sstream>
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Proximity.h"
#include "OpenSteer/Color.h"
#include "OpenSteer/UnusedParameter.h"

#ifdef WIN32
// Windows defines these as macros :(
#undef min
#undef max
#endif

#ifndef NO_LQ_BIN_STATS
#include <iomanip> // for setprecision
#include <limits> // for numeric_limits::max()
#endif // NO_LQ_BIN_STATS

namespace {

    // Include names declared in the OpenSteer namespace into the
    // namespaces to search to find names.
    using namespace OpenSteer;
    using std::chrono::high_resolution_clock;

    // ----------------------------------------------------------------------------


    typedef OpenSteer::AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
    typedef OpenSteer::AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;

    // ----------------------------------------------------------------------------


    class Boid : public OpenSteer::SimpleVehicle
    {
    public:

        // type for a flock: an STL vector of Boid pointers
        typedef std::vector<Boid*> groupType;


        // constructor
        Boid (ProximityDatabase& pd)
        {
            // allocate a token for this boid in the proximity database
            proximityToken = NULL;
            newPD (pd);

            // reset all boid state
            reset ();
        }


        // destructor
        ~Boid ()
        {
            // delete this boid's token in the proximity database
            delete proximityToken;
        }


        // reset state
        void reset (void)
        {
            // reset the vehicle
            SimpleVehicle::reset ();

            // steering force is clipped to this magnitude
            setMaxForce (27);

            // velocity is clipped to this magnitude
            setMaxSpeed (9);

            // initial slow speed
            setSpeed (maxSpeed() * 0.3f);

            // randomize initial orientation
            regenerateOrthonormalBasisUF (RandomUnitVector ());

            // randomize initial position
            setPosition (RandomVectorInUnitRadiusSphere () * 20);

            // notify proximity database that our position has changed
            proximityToken->updateForNewPosition (position());
        }


        // draw this boid into the scene
        void draw (void)
        {
            drawBasic3dSphericalVehicle (this, gGray70);
            // drawTrail ();
        }


        // per frame simulation update
        void update (const float currentTime, const float elapsedTime)
        {
            OPENSTEER_UNUSED_PARAMETER(currentTime);

            // steer to flock and avoid obstacles if any
            applySteeringForce (steerToFlock (), elapsedTime);

            // wrap around to contrain boid within the spherical boundary
            sphericalWrapAround ();

            // notify proximity database that our position has changed
            proximityToken->updateForNewPosition (position());
        }


        // basic flocking
        Vec3 steerToFlock (void)
        {
            // avoid obstacles if needed
            // XXX this should probably be moved elsewhere
            const Vec3 avoidance = steerToAvoidObstacles (1.0f, obstacles);
            if (avoidance != Vec3::zero) return avoidance;

            const float separationRadius =  OpenSteerDemo::queryRadius > 0. ? OpenSteerDemo::queryRadius : 5.0;
            const float separationAngle  = -0.707f;
            const float separationWeight =  12.0f;

            const float alignmentRadius = OpenSteerDemo::queryRadius > 0. ? OpenSteerDemo::queryRadius : 7.5f;
            const float alignmentAngle  = 0.7f;
            const float alignmentWeight = 8.0f;

            const float cohesionRadius = OpenSteerDemo::queryRadius > 0. ? OpenSteerDemo::queryRadius : 9.0f;
            const float cohesionAngle  = -0.15f;
            const float cohesionWeight = 8.0f;

            const float maxRadius = maxXXX (separationRadius,
                                            maxXXX (alignmentRadius,
                                                    cohesionRadius));

           
            neighbors.clear();

            proximityToken->findNeighbors(position(), maxRadius, neighbors);


            
    #ifndef NO_LQ_BIN_STATS
            // maintain stats on max/min/ave neighbors per boids
            size_t count = neighbors.size();
            if (maxNeighbors < count) maxNeighbors = count;
            if (minNeighbors > count) minNeighbors = count;
            totalNeighbors += count;
    #endif // NO_LQ_BIN_STATS

            // determine each of the three component behaviors of flocking
            const Vec3 separation = steerForSeparation (separationRadius,
                                                        separationAngle,
                                                        neighbors);
            const Vec3 alignment  = steerForAlignment  (alignmentRadius,
                                                        alignmentAngle,
                                                        neighbors);
            const Vec3 cohesion   = steerForCohesion   (cohesionRadius,
                                                        cohesionAngle,
                                                        neighbors);
    
           

            // apply weights to components (save in variables for annotation)
            const Vec3 separationW = separation * separationWeight;
            const Vec3 alignmentW = alignment * alignmentWeight;
            const Vec3 cohesionW = cohesion * cohesionWeight;

            return separationW + alignmentW + cohesionW;
        }


        // constrain this boid to stay within sphereical boundary.
        void sphericalWrapAround (void)
        {
            // when outside the sphere
            if (position().length() > worldRadius)
            {
                // wrap around (teleport)
                setPosition (position().sphericalWrapAround (Vec3::zero,
                                                             worldRadius));
                if (this == OpenSteerDemo::selectedVehicle)
                {
                    OpenSteerDemo::position3dCamera
                        (*OpenSteerDemo::selectedVehicle); 
                    OpenSteerDemo::camera.doNotSmoothNextMove ();
                }
            }
        }


    // ---------------------------------------------- xxxcwr111704_terrain_following
        // control orientation for this boid
        void regenerateLocalSpace (const Vec3& newVelocity,
                                   const float elapsedTime)
        {
            // 3d flight with banking
            regenerateLocalSpaceForBanking (newVelocity, elapsedTime);

            // // follow terrain surface
            // regenerateLocalSpaceForTerrainFollowing (newVelocity, elapsedTime);
        }


        // XXX experiment:
        // XXX   herd with terrain following
        // XXX   special case terrain: a sphere at the origin, radius 40
        void regenerateLocalSpaceForTerrainFollowing  (const Vec3& newVelocity,
                                                       const float /* elapsedTime */)
        {

            // XXX this is special case code, these should be derived from arguments //
            const Vec3 surfaceNormal = position().normalize();                       //
            const Vec3 surfacePoint = surfaceNormal * 40.0f;                         //
            // XXX this is special case code, these should be derived from arguments //

            const Vec3 newUp = surfaceNormal;
            const Vec3 newPos = surfacePoint;
            const Vec3 newVel = newVelocity.perpendicularComponent(newUp);
            const float newSpeed = newVel.length();
            const Vec3 newFor = newVel / newSpeed;

            setSpeed (newSpeed);
            setPosition (newPos);
            setUp (newUp);
            setForward (newFor);
            setUnitSideFromForwardAndUp ();
        }
    // ---------------------------------------------- xxxcwr111704_terrain_following

        // switch to new proximity database -- just for demo purposes
        void newPD (ProximityDatabase& pd)
        {
            // delete this boid's token in the old proximity database
            delete proximityToken;

            // allocate a token for this boid in the proximity database
            proximityToken = pd.allocateToken (this);
        }


        // group of all obstacles to be avoided by each Boid
        static ObstacleGroup obstacles;

        // a pointer to this boid's interface object for the proximity database
        ProximityToken* proximityToken;

        // allocate one and share amoung instances just to save memory usage
        // (change to per-instance allocation to be more MP-safe)
        static AVGroup neighbors;

        static float worldRadius;

        // xxx perhaps this should be a call to a general purpose annotation for
        // xxx "local xxx axis aligned box in XZ plane" -- same code in in
        // xxx CaptureTheFlag.cpp
        void annotateAvoidObstacle (const float minDistanceToCollision)
        {
            const Vec3 boxSide = side() * radius();
            const Vec3 boxFront = forward() * minDistanceToCollision;
            const Vec3 FR = position() + boxFront - boxSide;
            const Vec3 FL = position() + boxFront + boxSide;
            const Vec3 BR = position()            - boxSide;
            const Vec3 BL = position()            + boxSide;
            const Color white (1,1,1);
            annotationLine (FR, FL, white);
            annotationLine (FL, BL, white);
            annotationLine (BL, BR, white);
            annotationLine (BR, FR, white);
        }

    #ifndef NO_LQ_BIN_STATS
            static size_t minNeighbors, maxNeighbors, totalNeighbors;
    #endif // NO_LQ_BIN_STATS
    };

    AVGroup Boid::neighbors;
    float Boid::worldRadius;
    ObstacleGroup Boid::obstacles;
    #ifndef NO_LQ_BIN_STATS
    size_t Boid::minNeighbors, Boid::maxNeighbors, Boid::totalNeighbors;
    #endif // NO_LQ_BIN_STATS

    // ----------------------------------------------------------------------------
    // PlugIn for OpenSteerDemo


    class BoidsBaselinePlugIn : public PlugIn
    {
    public:
        
        const char* name (void) {return "BoidsBaseline";}

        float selectionOrderSortKey (void) {return 0.00000001f;}

        virtual ~BoidsBaselinePlugIn() {} // be more "nice" to avoid a compiler warning

        void open (void)
        {
            
            Boid::worldRadius = OpenSteerDemo::worldRadius > 0. ? OpenSteerDemo::worldRadius : 50.0f;
            
            // make the database used to accelerate proximity queries
            cyclePD = -1;
            nextPD ();
            
          
            // make default-sized flock
            population = 0;
            for (int i = 0; i < OpenSteerDemo::numAgents; i++) addBoidToFlock ();

           
            // initialize camera
            OpenSteerDemo::init3dCamera (*OpenSteerDemo::selectedVehicle);
            OpenSteerDemo::camera.mode = Camera::cmFixed;
            OpenSteerDemo::camera.fixedDistDistance = OpenSteerDemo::cameraTargetDistance;
            OpenSteerDemo::camera.fixedDistVOffset = 0;
            OpenSteerDemo::camera.lookdownDistance = 20;
            OpenSteerDemo::camera.aimLeadTime = 0.5;
            OpenSteerDemo::camera.povOffset.set (0, 0.5, -2);

        }

        void update (const float currentTime, const float elapsedTime)
        {
    #ifndef NO_LQ_BIN_STATS
            Boid::maxNeighbors = Boid::totalNeighbors = 0;
            Boid::minNeighbors = std::numeric_limits<int>::max();
    #endif // NO_LQ_BIN_STATS

            // update flock simulation for each boid
            for (iterator i = flock.begin(); i != flock.end(); i++)
            {
                (**i).update (currentTime, elapsedTime);
            }


           
        }

        void redraw (const float currentTime, const float elapsedTime)
        {
          // selected vehicle (user can mouse click to select another)
            AbstractVehicle* selected = OpenSteerDemo::selectedVehicle;

            // vehicle nearest mouse (to be highlighted)
            AbstractVehicle* nearMouse = OpenSteerDemo::vehicleNearestToMouse ();

            // update camera
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);

            // draw each boid in flock
            for (iterator i = flock.begin(); i != flock.end(); i++) (**i).draw ();

            // highlight vehicle nearest mouse
            OpenSteerDemo::drawCircleHighlightOnVehicle (nearMouse, 1, gGray70);

            // highlight selected vehicle
            OpenSteerDemo::drawCircleHighlightOnVehicle (selected, 1, gGray50);

            
            const float h = drawGetWindowHeight ();
            const Vec3 screenLocation (10, h-50, 0);

            
        }

        void close (void)
        {
            // delete each member of the flock
            while (population > 0) removeBoidFromFlock ();

            // delete the proximity database
            delete pd;
            pd = NULL;
        }

        void reset (void)
        {
            // reset each boid in flock
            for (iterator i = flock.begin(); i != flock.end(); i++) (**i).reset();

            // reset camera position
            OpenSteerDemo::position3dCamera (*OpenSteerDemo::selectedVehicle);

            // make camera jump immediately to new position
            OpenSteerDemo::camera.doNotSmoothNextMove ();

            OpenSteerDemo::clock.setStepCount(0);
            OpenSteerDemo::clock.setPausedState(false);
            OpenSteerDemo::totalStepTime = 0.f;
            OpenSteerDemo::stepTime = 0.f;
        }

        // for purposes of demonstration, allow cycling through various
        // types of proximity databases.  this routine is called when the
        // OpenSteerDemo user pushes a function key.
        void nextPD (void)
        {
            // save pointer to old PD
            ProximityDatabase* oldPD = pd;

            // allocate new PD
            const int totalPD = 2;
            switch (cyclePD = (cyclePD + 1) % totalPD)
            {
            case 0:
                {
                    const Vec3 center;
                    const float div = 10.0f;
                    const Vec3 divisions (div, div, div);
                    const float diameter = Boid::worldRadius * 2.;
                    const Vec3 dimensions (diameter, diameter, diameter);
                    typedef LQProximityDatabase<AbstractVehicle*> LQPDAV;
                    pd = new LQPDAV (center, dimensions, divisions);
                    break;
                }
            case 1:
                {
                    pd = new BruteForceProximityDatabase<AbstractVehicle*> ();
                    break;
                }
            }

            // switch each boid to new PD
            for (iterator i=flock.begin(); i!=flock.end(); i++) (**i).newPD(*pd);

            // delete old PD (if any)
            delete oldPD;
        }

        void handleFunctionKeys (int keyNumber)
        {
            switch (keyNumber)
            {
            case 1:  addBoidToFlock ();         break;
            case 2:  removeBoidFromFlock ();    break;
            case 3:  nextPD ();                 break;
            }
        }

     
        void printMiniHelpForFunctionKeys (void)
        {
            std::ostringstream message;
            message << "Function keys handled by ";
            message << '"' << name() << '"' << ':' << std::ends;
            OpenSteerDemo::printMessage (message);
            OpenSteerDemo::printMessage ("  F1     add a boid to the flock.");
            OpenSteerDemo::printMessage ("  F2     remove a boid from the flock.");
            OpenSteerDemo::printMessage ("  F3     use next proximity database.");
            OpenSteerDemo::printMessage ("  F4     next flock boundary condition.");
            OpenSteerDemo::printMessage ("");
        }

        void addBoidToFlock (void)
        {
            population++;
            Boid* boid = new Boid (*pd);
            flock.push_back (boid);
            if (population == 1) OpenSteerDemo::selectedVehicle = boid;
        }

        void removeBoidFromFlock (void)
        {
            if (population > 0)
            {
                // save a pointer to the last boid, then remove it from the flock
                const Boid* boid = flock.back();
                flock.pop_back();
                population--;

                // if it is OpenSteerDemo's selected vehicle, unselect it
                if (boid == OpenSteerDemo::selectedVehicle)
                    OpenSteerDemo::selectedVehicle = NULL;

                // delete the Boid
                delete boid;
            }
        }

        // return an AVGroup containing each boid of the flock
        const AVGroup& allVehicles (void) {return (const AVGroup&)flock;}

        // flock: a group (STL vector) of pointers to all boids
        Boid::groupType flock;
        typedef Boid::groupType::const_iterator iterator;

        // pointer to database used to accelerate proximity queries
        ProximityDatabase* pd;
        
        // keep track of current flock size
        int population;

        // which of the various proximity databases is currently in use
        int cyclePD;


    };

  
    BoidsBaselinePlugIn gBoidsBaselinePlugIn;



    // ----------------------------------------------------------------------------

} // anonymous namespace
