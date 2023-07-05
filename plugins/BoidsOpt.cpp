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

#include <chrono>
#include <sstream>
#include <unordered_map>
#include <cstring>
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/Vec3.h"
#include "OpenSteer/VehiclePack.h"
#include <random>
#include <immintrin.h>
#include <algorithm>

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
    

class BoidsSimulation{
    public:
        class SOA_Boid{
            public:

                float* positionX;
                float* positionY;
                float* positionZ;

                float* forwardX;
                float* forwardY;
                float* forwardZ;

                float* speed;

                float* accelerationX;
                float* accelerationY;
                float* accelerationZ;
        };

        SOA_Boid flock;
        int numBoids;
        //simulation parameters
        const float mass = 1.0f;
        const float boidRadius = 0.5f;
        const float maxForce = 27.;
        const float maxSpeed = 9.;

        const float separationRadius =  OpenSteerDemo::queryRadius > 0. ? OpenSteerDemo::queryRadius : 5.0f;
        const float separationAngle  = -0.707f;
        const float separationWeight =  12.0f;

        const float alignmentRadius = OpenSteerDemo::queryRadius > 0. ? OpenSteerDemo::queryRadius : 7.5f;
        const float alignmentAngle  = 0.7f;
        const float alignmentWeight = 8.0f;

        const float cohesionRadius = OpenSteerDemo::queryRadius > 0. ? OpenSteerDemo::queryRadius : 9.0f;
        const float cohesionAngle  = -0.15f;
        const float cohesionWeight = 8.0f;

        //grid data structure
        float size;
        Vec3 origin;
        float cellSize;
        int numCells;

        std::vector<VehiclePack> flockPack;
        int *cellStart;
        int *cellEnd;
        int *gridHash;
        int *boidIndex;

        float* separationBufferX;
        float* separationBufferY;
        float* separationBufferZ;

        float* alignmentBufferX;
        float* alignmentBufferY;
        float* alignmentBufferZ;

        float* cohesionBufferX;
        float* cohesionBufferY;
        float* cohesionBufferZ;

        BoidsSimulation(float gridSize, Vec3 center, int n){
            size = gridSize;
            origin = center - Vec3(size/2.,size/2.,size/2.);
            cellSize = 2.f*std::max(std::max(separationRadius,cohesionRadius),alignmentRadius);
            numCells = ceil(size/cellSize);
            numCells *= numCells*numCells;
            numBoids = n;
            init(numBoids);
        }

        void step(float deltaTime){

            calculateHash();

            sortBoidsAndLoadPacks();

            calculateSteering();

            updatePositions(deltaTime);
        }

        void init(int numBoids){
            

            //allocate grid data structure arrays
            cellStart = (int*) malloc(sizeof(int)*numCells);
            cellEnd = (int*) malloc(sizeof(int)*numCells);

            boidIndex = (int*) malloc(sizeof(int)*numBoids);
            gridHash = (int*) _mm_malloc(sizeof(int)*numBoids,32);

            //allocate SOA Boid structure
            flock.positionX = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            flock.positionY = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            flock.positionZ = (float*) _mm_malloc(sizeof(float)*numBoids,32);

            flock.forwardX = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            flock.forwardY = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            flock.forwardZ = (float*) _mm_malloc(sizeof(float)*numBoids,32);

            flock.accelerationX = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            flock.accelerationY = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            flock.accelerationZ = (float*) _mm_malloc(sizeof(float)*numBoids,32);

            flock.speed = (float*) _mm_malloc(sizeof(float)*numBoids,32);

            separationBufferX = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            separationBufferY = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            separationBufferZ = (float*) _mm_malloc(sizeof(float)*numBoids,32);

            alignmentBufferX = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            alignmentBufferY = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            alignmentBufferZ = (float*) _mm_malloc(sizeof(float)*numBoids,32);

            cohesionBufferX = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            cohesionBufferY = (float*) _mm_malloc(sizeof(float)*numBoids,32);
            cohesionBufferZ = (float*) _mm_malloc(sizeof(float)*numBoids,32);

            //generate random position and forward vectors
            for(int i = 0; i < numBoids; i++){
                
                Vec3 position = RandomVectorInUnitRadiusSphere()*20;
                Vec3 forward = RandomUnitVector();

                flock.positionX[i] = position.x;
                flock.positionY[i] = position.y;
                flock.positionZ[i] = position.z;

                flock.forwardX[i] = forward.x;
                flock.forwardY[i] = forward.y;
                flock.forwardZ[i] = forward.z;

                flock.accelerationX[i] = 0.;
                flock.accelerationY[i] = 0.;
                flock.accelerationZ[i] = 0.;

                flock.speed[i] = 0.3f * maxSpeed;

                boidIndex[i] = i;
            }

        }

        private:

        void calculateHash(){
            __m256 originX = _mm256_set1_ps(origin.x);
            __m256 originY = _mm256_set1_ps(origin.y);
            __m256 originZ = _mm256_set1_ps(origin.z);

            __m256 vCellSize = _mm256_set1_ps(cellSize);
            __m256 vNumCells = _mm256_set1_ps(ceil(size/cellSize));

            for(int i = 0; i < numBoids;i+=8){
                
                __m256 positionX = _mm256_load_ps(&flock.positionX[i]);
                __m256 positionY = _mm256_load_ps(&flock.positionY[i]);
                __m256 positionZ = _mm256_load_ps(&flock.positionZ[i]);

                __m256 discretizedX = _mm256_floor_ps(_mm256_div_ps(_mm256_sub_ps(positionX,originX),vCellSize));
                __m256 discretizedY = _mm256_floor_ps(_mm256_div_ps(_mm256_sub_ps(positionY,originY),vCellSize));
                __m256 discretizedZ = _mm256_floor_ps(_mm256_div_ps(_mm256_sub_ps(positionZ,originZ),vCellSize));

                __m256 hash = _mm256_fmadd_ps(discretizedZ,
                                              _mm256_mul_ps(vNumCells,vNumCells),
                                              _mm256_fmadd_ps(discretizedY,vNumCells,discretizedX));

                _mm256_store_si256((__m256i*)&gridHash[i],_mm256_cvtps_epi32(hash));
            }
        }

        void sortBoidsAndLoadPacks(){
            
            for(int i = 0; i < numBoids; i++){
                boidIndex[i] = i;
            }
           
            //sort agents based on their hash values
            std::sort(boidIndex,boidIndex+numBoids,[&](int a,int b){
                return gridHash[a]<gridHash[b];
            });

            //initialize cell start and cell end arrays
            for(int i = 0; i < numCells; i++){
                cellStart[i] = -1;
                cellEnd[i] = -1;
            }

            flockPack.clear();
            flockPack.reserve(numBoids/8);
           
            for(int i = 0; i < numBoids;i++){
                
                int index = boidIndex[i];
                if(i%8 == 0){
                    flockPack.emplace_back();
                    VehiclePack& currPack = flockPack.back();
                    __m256i indexes = _mm256_setr_epi32(boidIndex[i],boidIndex[i+1],boidIndex[i+2],boidIndex[i+3],
                                                        boidIndex[i+4],boidIndex[i+5],boidIndex[i+6],boidIndex[i+7]);
                    currPack.position.x = _mm256_i32gather_ps(flock.positionX,indexes,4);
                    currPack.position.y = _mm256_i32gather_ps(flock.positionY,indexes,4);
                    currPack.position.z = _mm256_i32gather_ps(flock.positionZ,indexes,4);

                    currPack.forward.x = _mm256_i32gather_ps(flock.forwardX,indexes,4);
                    currPack.forward.y = _mm256_i32gather_ps(flock.forwardY,indexes,4);
                    currPack.forward.z = _mm256_i32gather_ps(flock.forwardZ,indexes,4);

                    currPack.acceleration.x = _mm256_i32gather_ps(flock.accelerationX,indexes,4);
                    currPack.acceleration.y = _mm256_i32gather_ps(flock.accelerationY,indexes,4);
                    currPack.acceleration.z = _mm256_i32gather_ps(flock.accelerationZ,indexes,4);

                    currPack.speed = _mm256_i32gather_ps(flock.speed,indexes,4);
                }

                //write cell start and cell ends
                int currHash = gridHash[index];
                if(i == 0){
                    cellStart[currHash] = 0;
                }else{
                    int prevHash = gridHash[boidIndex[i-1]];
                    if(currHash != prevHash){
                        cellStart[currHash] = i/8;
                        cellEnd[prevHash] = (i-1)/8;
                    }
                    if(i == numBoids - 1){
                        cellEnd[currHash] = i/8;
                    }
                }


            }
            
        }

        inline float _mm256_sumreduce_ps(__m256 a)
        {
            __m128 l = _mm256_castps256_ps128(a);
            __m128 h = _mm256_extractf128_ps(a, 1);
            l = _mm_add_ps(l, h);
            l = _mm_hadd_ps(l, l);
            l = _mm_hadd_ps(l,l);
            return _mm_cvtss_f32(l);
        }
        
        void calculateSteering(){
            int sideCells = ceil(size/cellSize);

           //initialize constant vector values for comparisons (store these as members?)
            __m256 vSeparationDistance = _mm256_set1_ps(separationRadius);
            __m256 vSeparationAngle    = _mm256_set1_ps(separationAngle);
            
            __m256 vCohesionDistance = _mm256_set1_ps(cohesionRadius);
            __m256 vCohesionAngle    = _mm256_set1_ps(cohesionAngle);

            __m256 vAlignmentDistance = _mm256_set1_ps(alignmentRadius);
            __m256 vAlignmentAngle    = _mm256_set1_ps(alignmentAngle);

            for(int i = 0; i < numBoids; i++){
                int index = boidIndex[i];
                __m256 positionX = _mm256_broadcast_ss(&flock.positionX[index]);
                __m256 positionY = _mm256_broadcast_ss(&flock.positionY[index]);
                __m256 positionZ = _mm256_broadcast_ss(&flock.positionZ[index]);

                __m256 forwardX = _mm256_broadcast_ss(&flock.forwardX[index]);
                __m256 forwardY = _mm256_broadcast_ss(&flock.forwardY[index]);
                __m256 forwardZ = _mm256_broadcast_ss(&flock.forwardZ[index]);

                int currX = floor( (flock.positionX[index]-origin.x)/cellSize);
                int currY = floor( (flock.positionY[index]-origin.y)/cellSize);
                int currZ = floor( (flock.positionZ[index]-origin.z)/cellSize);

                __m256 separationX = _mm256_setzero_ps();
                __m256 separationY = _mm256_setzero_ps();
                __m256 separationZ = _mm256_setzero_ps();

                __m256 alignmentX = _mm256_setzero_ps();
                __m256 alignmentY = _mm256_setzero_ps();
                __m256 alignmentZ = _mm256_setzero_ps();

                __m256 cohesionX = _mm256_setzero_ps();
                __m256 cohesionY = _mm256_setzero_ps();
                __m256 cohesionZ = _mm256_setzero_ps();

                int alignmentNeighbors = 0;
                int cohesionNeighbors = 0;

                //iterate over 3x3x3 neighboring cells
                for(int z = -1; z <= 1; z++){
                    for(int y = -1; y <= 1; y++){
                        for(int x = -1; x <= 1; x++){
                            //discretize index
                            int currHash = ((currX+x)%sideCells + sideCells)%sideCells +
                                        (((currY+y)%sideCells + sideCells)%sideCells)*sideCells 
                                        + (((currZ+z)%sideCells + sideCells)%sideCells)*sideCells*sideCells;
                            int start = cellStart[currHash];
                            int end = cellEnd[currHash];
                            //if this cell doesn't contain any agent, skip
                            if(start == -1) continue;
                            //iterate over packets in the cell, compute alignment,cohesion,separation forces for this cell
                            for(int k = start; k <= end; k++){
                                VehiclePack& pack = flockPack[k];
                                __m256 offsetX = _mm256_sub_ps(pack.position.x,positionX);
                                __m256 offsetY = _mm256_sub_ps(pack.position.y,positionY);
                                __m256 offsetZ = _mm256_sub_ps(pack.position.z,positionZ);

                                __m256 vDistanceSquared = _mm256_fmadd_ps(offsetX,offsetX,_mm256_fmadd_ps(offsetY,offsetY,_mm256_mul_ps(offsetZ,offsetZ)));
                                __m256 vDistance = _mm256_sqrt_ps(vDistanceSquared);

                                __m256 normalizedOffsetX = _mm256_div_ps(offsetX,vDistance);
                                __m256 normalizedOffsetY = _mm256_div_ps(offsetY,vDistance);
                                __m256 normalizedOffsetZ = _mm256_div_ps(offsetZ,vDistance);

                                __m256 vForwardness = _mm256_fmadd_ps(forwardX,normalizedOffsetX,_mm256_fmadd_ps(forwardY,normalizedOffsetY,_mm256_mul_ps(forwardZ,normalizedOffsetZ)));
                               
                                __m256 mask = _mm256_cmp_ps(vDistance,_mm256_setzero_ps(),_MM_CMPINT_GT);

                                __m256 separationMask = _mm256_and_ps(_mm256_cmp_ps(vDistance,vSeparationDistance,_MM_CMPINT_LT),_mm256_cmp_ps(vForwardness,vSeparationAngle,_MM_CMPINT_GT));

                                __m256 cohesionMask = _mm256_and_ps(_mm256_cmp_ps(vDistance,vCohesionDistance,_MM_CMPINT_LT),_mm256_cmp_ps(vForwardness,vCohesionAngle,_MM_CMPINT_GT));

                                __m256 alignmentMask = _mm256_and_ps(_mm256_cmp_ps(vDistance,vAlignmentDistance,_MM_CMPINT_LT),_mm256_cmp_ps(vForwardness,vAlignmentAngle,_MM_CMPINT_GT));

                                separationX = _mm256_add_ps(separationX,_mm256_blendv_ps(_mm256_setzero_ps(),_mm256_div_ps(offsetX,vDistanceSquared),_mm256_and_ps(mask,separationMask)));
                                separationY = _mm256_add_ps(separationY,_mm256_blendv_ps(_mm256_setzero_ps(),_mm256_div_ps(offsetY,vDistanceSquared),_mm256_and_ps(mask,separationMask)));
                                separationZ = _mm256_add_ps(separationZ,_mm256_blendv_ps(_mm256_setzero_ps(),_mm256_div_ps(offsetZ,vDistanceSquared),_mm256_and_ps(mask,separationMask)));

                                alignmentX = _mm256_add_ps(alignmentX,_mm256_blendv_ps(_mm256_setzero_ps(),pack.forward.x,_mm256_and_ps(mask,alignmentMask)));
                                alignmentY = _mm256_add_ps(alignmentY,_mm256_blendv_ps(_mm256_setzero_ps(),pack.forward.y,_mm256_and_ps(mask,alignmentMask)));
                                alignmentZ = _mm256_add_ps(alignmentZ,_mm256_blendv_ps(_mm256_setzero_ps(),pack.forward.z,_mm256_and_ps(mask,alignmentMask)));
                                alignmentNeighbors += __builtin_popcount(_mm256_movemask_ps(_mm256_and_ps(mask,alignmentMask)));

                                cohesionX = _mm256_add_ps(cohesionX,_mm256_blendv_ps(_mm256_setzero_ps(),pack.position.x,_mm256_and_ps(mask,cohesionMask)));
                                cohesionY = _mm256_add_ps(cohesionY,_mm256_blendv_ps(_mm256_setzero_ps(),pack.position.y,_mm256_and_ps(mask,cohesionMask)));
                                cohesionZ = _mm256_add_ps(cohesionZ,_mm256_blendv_ps(_mm256_setzero_ps(),pack.position.z,_mm256_and_ps(mask,cohesionMask)));
                                cohesionNeighbors += __builtin_popcount(_mm256_movemask_ps(_mm256_and_ps(mask,cohesionMask)));
                                
                            }
                        }
                    }
                }

                //store the computed forces in a buffer, to be used for position update
                Vec3 separation = Vec3(_mm256_sumreduce_ps(separationX),_mm256_sumreduce_ps(separationY),_mm256_sumreduce_ps(separationZ));
                separation.normalize();

                separationBufferX[i] = -separation.x;
                separationBufferY[i] = -separation.y;
                separationBufferZ[i] = -separation.z;
                
                Vec3 alignment = Vec3(_mm256_sumreduce_ps(alignmentX),_mm256_sumreduce_ps(alignmentY),_mm256_sumreduce_ps(alignmentZ));
                Vec3 currForward = Vec3(flock.forwardX[index],flock.forwardY[index],flock.forwardZ[index]);

                if(alignmentNeighbors > 0) alignment = (alignment/(float)alignmentNeighbors - currForward).normalize();
                alignmentBufferX[i] = alignment.x;
                alignmentBufferY[i] = alignment.y;
                alignmentBufferZ[i] = alignment.z;

                Vec3 cohesion = Vec3(_mm256_sumreduce_ps(cohesionX),_mm256_sumreduce_ps(cohesionY),_mm256_sumreduce_ps(cohesionZ));
                Vec3 currPosition = Vec3(flock.positionX[index],flock.positionY[index],flock.positionZ[index]);

                if(cohesionNeighbors > 0) cohesion = (cohesion/(float)cohesionNeighbors - currPosition).normalize();
                cohesionBufferX[i] = cohesion.x;
                cohesionBufferY[i] = cohesion.y;
                cohesionBufferZ[i] = cohesion.z;
                
            }
        }

        void updatePositions(float deltaTime){
            
            //initialize constant vector values for weighted sum and comparisons (store these as members?)
            __m256 vSeparationWeight = _mm256_set1_ps(separationWeight);
            __m256 vAlignmentWeight = _mm256_set1_ps(alignmentWeight);
            __m256 vCohesionWeight = _mm256_set1_ps(cohesionWeight);

            __m256 vMaxSpeed = _mm256_set1_ps(maxSpeed);
            // __m256 vMaxAdjustedSpeed = _mm256_set1_ps(maxSpeed*0.2f);
            __m256 vMaxForce = _mm256_set1_ps(maxForce);

            float smoothedRate = clip(9*deltaTime,0.15f,0.4f);
            __m256 vSmoothedRate = _mm256_set1_ps(smoothedRate);

            __m256 vMass = _mm256_set1_ps(mass);

            __m256 vDeltaTime = _mm256_set1_ps(deltaTime);

            __m256 vWorldRadius = _mm256_set1_ps(size/2.);

            for(int i = 0; i < flockPack.size(); i++){
                VehiclePack& pack = flockPack[i];
                int index = i*8;

                //compute weighted sum of separation, cohesion and alignment forces
                __m256 separationX = _mm256_load_ps(&separationBufferX[index]);
                __m256 separationY = _mm256_load_ps(&separationBufferY[index]);
                __m256 separationZ = _mm256_load_ps(&separationBufferZ[index]);

                __m256 cohesionX = _mm256_load_ps(&cohesionBufferX[index]);
                __m256 cohesionY = _mm256_load_ps(&cohesionBufferY[index]);
                __m256 cohesionZ = _mm256_load_ps(&cohesionBufferZ[index]);

                __m256 alignmentX = _mm256_load_ps(&alignmentBufferX[index]);
                __m256 alignmentY = _mm256_load_ps(&alignmentBufferY[index]);
                __m256 alignmentZ = _mm256_load_ps(&alignmentBufferZ[index]);

                separationX = _mm256_mul_ps(separationX,vSeparationWeight);
                separationY = _mm256_mul_ps(separationY,vSeparationWeight);
                separationZ = _mm256_mul_ps(separationZ,vSeparationWeight);

                alignmentX = _mm256_mul_ps(alignmentX,vAlignmentWeight);
                alignmentY = _mm256_mul_ps(alignmentY,vAlignmentWeight);
                alignmentZ = _mm256_mul_ps(alignmentZ,vAlignmentWeight);

                cohesionX = _mm256_mul_ps(cohesionX,vCohesionWeight);
                cohesionY = _mm256_mul_ps(cohesionY,vCohesionWeight);
                cohesionZ = _mm256_mul_ps(cohesionZ,vCohesionWeight);

                __m256 forceX = _mm256_add_ps(separationX,_mm256_add_ps(alignmentX,cohesionX));
                __m256 forceY = _mm256_add_ps(separationY,_mm256_add_ps(alignmentY,cohesionY));
                __m256 forceZ = _mm256_add_ps(separationZ,_mm256_add_ps(alignmentZ,cohesionZ));

                //TO-DO adjust raw steering force (damping at low speed)
                // __m256 maskAdjustment = _mm256_cmp_ps(pack.speed,vMaxAdjustedSpeed,_MM_CMPINT_GT);

                //limit magnitude of steering forces (use inverse sqrt for normalization?)
                __m256 vForceMagnitudes = _mm256_fmadd_ps(forceX,forceX,_mm256_fmadd_ps(forceY,forceY,_mm256_mul_ps(forceZ,forceZ)));

                vForceMagnitudes = _mm256_sqrt_ps(vForceMagnitudes);

                __m256 limitMask = _mm256_cmp_ps(vForceMagnitudes,vMaxForce,_MM_CMPINT_GT);

                forceX = _mm256_blendv_ps(forceX,_mm256_mul_ps(_mm256_div_ps(forceX,vForceMagnitudes),vMaxForce),limitMask);
                forceY = _mm256_blendv_ps(forceY,_mm256_mul_ps(_mm256_div_ps(forceY,vForceMagnitudes),vMaxForce),limitMask);
                forceZ = _mm256_blendv_ps(forceZ,_mm256_mul_ps(_mm256_div_ps(forceZ,vForceMagnitudes),vMaxForce),limitMask);

                __m256 accelerationX = _mm256_div_ps(forceX,vMass);
                __m256 accelerationY = _mm256_div_ps(forceY,vMass);
                __m256 accelerationZ = _mm256_div_ps(forceZ,vMass);

                //update acceleration
                pack.acceleration.x = _mm256_fmadd_ps(_mm256_sub_ps(accelerationX,pack.acceleration.x),vSmoothedRate,pack.acceleration.x);
                pack.acceleration.y = _mm256_fmadd_ps(_mm256_sub_ps(accelerationY,pack.acceleration.y),vSmoothedRate,pack.acceleration.y);
                pack.acceleration.z = _mm256_fmadd_ps(_mm256_sub_ps(accelerationZ,pack.acceleration.z),vSmoothedRate,pack.acceleration.z);

                __m256 newVelocityX = _mm256_mul_ps(pack.forward.x,pack.speed);
                __m256 newVelocityY = _mm256_mul_ps(pack.forward.y,pack.speed);
                __m256 newVelocityZ = _mm256_mul_ps(pack.forward.z,pack.speed);

                newVelocityX = _mm256_fmadd_ps(pack.acceleration.x,vDeltaTime,newVelocityX);
                newVelocityY = _mm256_fmadd_ps(pack.acceleration.y,vDeltaTime,newVelocityY);
                newVelocityZ = _mm256_fmadd_ps(pack.acceleration.z,vDeltaTime,newVelocityZ);

                //limity velocity to maxSpeed
                __m256 vVelocityMagnitudes =  _mm256_fmadd_ps(newVelocityX,newVelocityX,_mm256_fmadd_ps(newVelocityY,newVelocityY,_mm256_mul_ps(newVelocityZ,newVelocityZ)));
                vVelocityMagnitudes = _mm256_sqrt_ps(vVelocityMagnitudes);
                limitMask = _mm256_cmp_ps(vVelocityMagnitudes,vMaxSpeed,_MM_CMPINT_GT);

                newVelocityX = _mm256_blendv_ps(newVelocityX,_mm256_mul_ps(_mm256_div_ps(newVelocityX,vVelocityMagnitudes),vMaxSpeed),limitMask);
                newVelocityY = _mm256_blendv_ps(newVelocityY,_mm256_mul_ps(_mm256_div_ps(newVelocityY,vVelocityMagnitudes),vMaxSpeed),limitMask);
                newVelocityZ = _mm256_blendv_ps(newVelocityZ,_mm256_mul_ps(_mm256_div_ps(newVelocityZ,vVelocityMagnitudes),vMaxSpeed),limitMask);

                //update speed
                pack.speed = _mm256_blendv_ps(vVelocityMagnitudes,vMaxSpeed,limitMask);

                //update positions
                pack.position.x = _mm256_fmadd_ps(newVelocityX,vDeltaTime,pack.position.x);
                pack.position.y = _mm256_fmadd_ps(newVelocityY,vDeltaTime,pack.position.y);
                pack.position.z = _mm256_fmadd_ps(newVelocityZ,vDeltaTime,pack.position.z);

                //apply spherical wraparound to position
                __m256 vPositionLength = _mm256_fmadd_ps(pack.position.x,pack.position.x,_mm256_fmadd_ps(pack.position.y,pack.position.y,_mm256_mul_ps(pack.position.z,pack.position.z)));
                vPositionLength = _mm256_sqrt_ps(vPositionLength);
                __m256 wraparoundMask = _mm256_cmp_ps(vPositionLength,vWorldRadius,_MM_CMPINT_GT);
                __m256 normalizedPosX = _mm256_div_ps(pack.position.x,vPositionLength);
                __m256 normalizedPosY = _mm256_div_ps(pack.position.y,vPositionLength);
                __m256 normalizedPosZ = _mm256_div_ps(pack.position.z,vPositionLength);

                __m256 wraparoundPosX = _mm256_fmadd_ps(normalizedPosX,_mm256_mul_ps(vWorldRadius,_mm256_set1_ps(-2.f)),pack.position.x);
                __m256 wraparoundPosY = _mm256_fmadd_ps(normalizedPosY,_mm256_mul_ps(vWorldRadius,_mm256_set1_ps(-2.f)),pack.position.y);
                __m256 wraparoundPosZ = _mm256_fmadd_ps(normalizedPosZ,_mm256_mul_ps(vWorldRadius,_mm256_set1_ps(-2.f)),pack.position.z);

                pack.position.x = _mm256_blendv_ps(pack.position.x,wraparoundPosX,wraparoundMask);
                pack.position.y = _mm256_blendv_ps(pack.position.y,wraparoundPosY,wraparoundMask);
                pack.position.z = _mm256_blendv_ps(pack.position.z,wraparoundPosZ,wraparoundMask);

                //update forwards
                __m256 newForwardX = _mm256_div_ps(newVelocityX,pack.speed);
                __m256 newForwardY = _mm256_div_ps(newVelocityY,pack.speed);
                __m256 newForwardZ = _mm256_div_ps(newVelocityZ,pack.speed);

                pack.forward.x = newForwardX;
                pack.forward.y = newForwardY;
                pack.forward.z = newForwardZ;

                //write updated data to SOA buffer
                _mm256_store_ps(&flock.positionX[index],pack.position.x);
                _mm256_store_ps(&flock.positionY[index],pack.position.y);
                _mm256_store_ps(&flock.positionZ[index],pack.position.z);

                _mm256_store_ps(&flock.forwardX[index],pack.forward.x);
                _mm256_store_ps(&flock.forwardY[index],pack.forward.y);
                _mm256_store_ps(&flock.forwardZ[index],pack.forward.z);

                _mm256_store_ps(&flock.accelerationX[index],pack.acceleration.x);
                _mm256_store_ps(&flock.accelerationY[index],pack.acceleration.y);
                _mm256_store_ps(&flock.accelerationZ[index],pack.acceleration.z);

                 _mm256_store_ps(&flock.speed[index],pack.speed);
            }
        }

};

    // ----------------------------------------------------------------------------
    // PlugIn for OpenSteerDemo

    class BoidsOptPlugIn : public PlugIn
    {
    public:
        
        BoidsSimulation* sim;

        const char* name (void) {return "BoidsOpt";}

        float selectionOrderSortKey (void) {return 0.003f;}

        virtual ~BoidsOptPlugIn() {} // be more "nice" to avoid a compiler warning

        void open (void)
        {
            // make default-sized flock
            sim = new BoidsSimulation(OpenSteerDemo::worldRadius > 0. ? OpenSteerDemo::worldRadius*2.f : 100.f ,Vec3::zero,OpenSteerDemo::numAgents);
            

            // initialize camera
            // OpenSteerDemo::init3dCamera (*OpenSteerDemo::selectedVehicle);
            OpenSteerDemo::camera.mode = Camera::cmFixed;
            OpenSteerDemo::camera.fixedDistDistance = OpenSteerDemo::cameraTargetDistance;
            OpenSteerDemo::camera.fixedDistVOffset = 0;
            OpenSteerDemo::camera.lookdownDistance = 20;
            OpenSteerDemo::camera.aimLeadTime = 0.5;
            OpenSteerDemo::camera.povOffset.set (0, 0.5, -2);

        }

        void update (const float currentTime, const float elapsedTime)
        {
            sim->step(elapsedTime);
        }

        class Boid: OpenSteer::SimpleVehicle{
            public:
                Boid(Vec3 position,Vec3 forward){
                    reset();
                    setPosition(position);
                    setForward(forward);
                    regenerateOrthonormalBasisUF(forward);
                }
                ~Boid(){}
                void update(float a,float b){}
        };


        void redraw (const float currentTime, const float elapsedTime)
        {
          // selected vehicle (user can mouse click to select another)
            AbstractVehicle* selected = OpenSteerDemo::selectedVehicle;

            // vehicle nearest mouse (to be highlighted)
            AbstractVehicle* nearMouse = OpenSteerDemo::vehicleNearestToMouse ();

            // update camera
            OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);

            // draw each boid in flock
            for (int i = 0; i < OpenSteerDemo::numAgents; i++){
                Vec3 position(
                sim->flock.positionX[i],
                sim->flock.positionY[i],
                sim->flock.positionZ[i]);

                Vec3 forward(
                sim->flock.forwardX[i],
                sim->flock.forwardY[i],
                sim->flock.forwardZ[i]);

                Boid *b = new Boid(position,forward);
                
                drawBasic3dSphericalVehicle((AbstractVehicle*)b,gGray70);
               
               

            } 

            // highlight vehicle nearest mouse
            OpenSteerDemo::drawCircleHighlightOnVehicle (nearMouse, 1, gGray70);

            // highlight selected vehicle
            OpenSteerDemo::drawCircleHighlightOnVehicle (selected, 1, gGray50);

            
            const float h = drawGetWindowHeight ();
            const Vec3 screenLocation (10, h-50, 0);

            
        }

        void close (void)
        {
            
        }

        AVGroup mock;
        // return an AVGroup containing each boid of the flock
        const AVGroup& allVehicles (void) {return mock;}

        void reset (void)
        {
            // reset each boid in flock
            sim->init(OpenSteerDemo::numAgents);

            // reset camera position
            OpenSteerDemo::position3dCamera (*OpenSteerDemo::selectedVehicle);

            // make camera jump immediately to new position
            OpenSteerDemo::camera.doNotSmoothNextMove ();

            OpenSteerDemo::clock.setStepCount(0);
            OpenSteerDemo::clock.setPausedState(false);
            OpenSteerDemo::totalStepTime = 0.f;
            OpenSteerDemo::stepTime = 0.f;
        }

    };

    
    BoidsOptPlugIn gBoidsOptPlugIn;



    // ----------------------------------------------------------------------------
} // anonymous namespace
