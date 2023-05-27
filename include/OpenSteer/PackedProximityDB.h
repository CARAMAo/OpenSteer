#include "OpenSteer/Vec3.h"
#include "OpenSteer/AbstractVehicle.h"
#include "OpenSteer/VehiclePack.h"
#include <unordered_map>

namespace OpenSteer{
    class PackedProximityDB{
        public:
            PackedProximityDB(Vec3 d,Vec3 o,float size){
                dimensions = d;
                origin = o;
                cellSize = size;
                Vec3 cells = dimensions/cellSize;
                numCellsX = ceil(cells.x);
                numCellsY = ceil(cells.y);
                numCellsZ = ceil(cells.z);
            };

            ~PackedProximityDB(){
                free(cellStartIndexes);
            }
            void update(AVGroup& vehicles);
            void findNeighbors(Vec3 center,float radius,std::vector<VehiclePack>& neighbors);

            inline Vec3 calcCellPos(Vec3 position){
                int cellPosX = floor((position.x - origin.x)/cellSize);
                int cellPosY = floor((position.y - origin.y)/cellSize);
                int cellPosZ = floor((position.z - origin.z)/cellSize);
                return Vec3(cellPosX,cellPosY,cellPosZ);
            }
            inline int calcCellHash(int posX,int posY,int posZ){
                int cellHashX = (numCellsX + (posX%numCellsX))%numCellsX;
                int cellHashY = (numCellsY + (posY%numCellsY))%numCellsY;
                int cellHashZ = (numCellsZ + (posZ%numCellsZ))%numCellsZ;
                return cellHashX + cellHashY*ceil(dimensions.x/cellSize) + cellHashZ*ceil(dimensions.y/cellSize)*ceil(dimensions.x/cellSize);
            }

        private:
            Vec3 dimensions;
            Vec3 origin;
            float cellSize;
            std::unordered_map<int,std::vector<VehiclePack>> population;
            int* cellStartIndexes;
            int numCellsX;
            int numCellsY;
            int numCellsZ;
    };
}