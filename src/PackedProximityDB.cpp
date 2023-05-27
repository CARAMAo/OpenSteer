#include "OpenSteer/PackedProximityDB.h"
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <cstring>
#include <immintrin.h>

void OpenSteer::PackedProximityDB::update(AVGroup& vehicles){

    population.clear();

    std::sort(  vehicles.begin(),
                vehicles.end(),
                [&](const AbstractVehicle* lhb, const AbstractVehicle* rhb){
                        Vec3 lpos = calcCellPos(lhb->position());
                        Vec3 rpos = calcCellPos(rhb->position());
                        return calcCellHash(lpos.x,lpos.y,lpos.z) < calcCellHash(rpos.x,rpos.y,rpos.z);
                }
            );

    

    alignas(32) float positionX[8];
    memset(positionX,-1,sizeof(float)*8);
    alignas(32) float positionY[8];
    memset(positionY,-1,sizeof(float)*8);
    alignas(32) float positionZ[8];
    memset(positionZ,-1,sizeof(float)*8);

   alignas(32)  float forwardX[8];
    memset(forwardX,-1,sizeof(float)*8);
   alignas(32)  float forwardY[8];
    memset(forwardY,-1,sizeof(float)*8);
   alignas(32)  float forwardZ[8];
    memset(forwardZ,-1,sizeof(float)*8);


    Vec3 cellPos = calcCellPos(vehicles[0]->position());
    int lastCellId = calcCellHash(cellPos.x,cellPos.y,cellPos.z);
    int j = 0;
    int i = 0;

    while(i < vehicles.size()){
        AbstractVehicle* av = vehicles[i];
        Vec3 position = av->position();
        Vec3 forward = av->forward();
        
        Vec3 currCellPos = calcCellPos(position);
        int currCellId = calcCellHash(currCellPos.x,currCellPos.y,currCellPos.z);
        
        if(currCellId == lastCellId && j < 8){
            positionX[j] = position.x;
            positionY[j] = position.y;
            positionZ[j] = position.z;
            forwardX[j] = forward.x;
            forwardY[j] = forward.y;
            forwardZ[j] = forward.z;
            j++;
            i++;
        }
        if(i == vehicles.size() || j == 8 || currCellId != lastCellId){
            j = 0;
            VehiclePack vp;
            vp.position.x = _mm256_load_ps(positionX);
            vp.position.y = _mm256_load_ps(positionY);
            vp.position.z = _mm256_load_ps(positionZ);
            vp.forward.x = _mm256_load_ps(forwardX);
            vp.forward.y = _mm256_load_ps(forwardY);
            vp.forward.z = _mm256_load_ps(forwardZ);
            population[lastCellId].push_back(vp);
            lastCellId = currCellId;

            memset(positionX,-1,sizeof(float)*8);
            memset(positionY,-1,sizeof(float)*8);
            memset(positionZ,-1,sizeof(float)*8);

            memset(forwardX,-1,sizeof(float)*8);
            memset(forwardY,-1,sizeof(float)*8);
            memset(forwardZ,-1,sizeof(float)*8);
        }
    }
}

void OpenSteer::PackedProximityDB::findNeighbors(Vec3 center,float radius,std::vector<VehiclePack>& neighbors){
    Vec3 centerGridPos = calcCellPos(center);
    
    __m256 vMaxDistanceSquared = _mm256_set1_ps(radius*radius);
    __m256 centerX = _mm256_broadcast_ss(&center.x);
    __m256 centerY = _mm256_broadcast_ss(&center.y);
    __m256 centerZ = _mm256_broadcast_ss(&center.z);
    for(int z = -1; z <= 1; z++){
        for(int y = -1; y <= 1; y++){
            for(int x = -1; x <= 1; x++){
                Vec3 currGridPos = centerGridPos + Vec3(x,y,z);
                int currCellId = calcCellHash(currGridPos.x,currGridPos.y,currGridPos.z);
                std::vector<VehiclePack> v = population[currCellId];

                for(VehiclePack& vp : population[currCellId]){
                    __m256 offsetX = _mm256_sub_ps(centerX,vp.position.x);
                    __m256 offsetY = _mm256_sub_ps(centerY,vp.position.y);
                    __m256 offsetZ = _mm256_sub_ps(centerZ,vp.position.z);

                    __m256 vDistanceSquared = _mm256_fmadd_ps(offsetX,offsetX,_mm256_fmadd_ps(offsetY,offsetY,_mm256_mul_ps(offsetZ,offsetZ)));

                    __m256 mask = _mm256_cmp_ps(vDistanceSquared,vMaxDistanceSquared,_MM_CMPINT_LT);
                    if(_mm256_movemask_ps(mask) != 0){
                        neighbors.push_back(vp);
                    }
                }
            }
        }
    }
}