// Include OPENSTEER_UNUSED_PARAMETER
#include "Opensteer/Vec3.h"
#include "OpenSteer/AbstractVehicle.h"
#include "OpenSteer/AbstractVehiclePack.h"
#include "OpenSteer/SteerLibrary.h"
#include "OpenSteer/Proximity.h"
#include "OpenSteer/SimpleVehicle.h"
#include <iostream>

using OpenSteer::Vec3,OpenSteer::SteerLibraryMixin,OpenSteer::AbstractVehicle;
using OpenSteer::AVProximityDatabase;
using OpenSteer::AVPack;
using OpenSteer::SimpleVehicle;
using OpenSteer::RandomUnitVector;
using OpenSteer::RandomVectorInUnitRadiusSphere;
class Agent : public SimpleVehicle
{
    public:
    Agent(AVProximityDatabase* pd){

        SimpleVehicle::reset();

        proximityToken = pd->allocateToken(this);

        reset();
    }

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
            regenerateOrthonormalBasisUF ( RandomUnitVector() );

            // randomize initial position
            setPosition (RandomVectorInUnitRadiusSphere () * 20);

            // notify proximity database that our position has changed
            proximityToken->updateForNewPosition (position());
        }

    ~Agent(){
        delete proximityToken;
    }

    void update(float ct,float et){

    }

    private:
        AVProximityDatabase::tokenType* proximityToken;
};
    
int main( int argc, char* argv[] ) 
{

    const Vec3 center;
    const float div = 10.0f;
    const Vec3 divisions (div, div, div);
    const float diameter = 100.f * 1.1f * 2;
    const Vec3 dimensions (diameter, diameter, diameter);
    AVProximityDatabase *pd = new AVProximityDatabase(center, dimensions, divisions);
    
    Agent *a = new Agent(pd);
    

    return 0;
}
