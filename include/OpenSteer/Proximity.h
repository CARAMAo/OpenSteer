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
// Proximity 
//
// Data structures for accelerating proximity/locality/neighborhood queries
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 06-20-01 cwr: created
//
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_PROXIMITY_H
#define OPENSTEER_PROXIMITY_H


#include <algorithm>
#include <vector>
#include "OpenSteer/Vec3.h"
#include "OpenSteer/lq.h"   // XXX temp?


namespace OpenSteer {


    // ----------------------------------------------------------------------------
    // "tokens" are the objects manipulated by the spatial database


    template <class ContentType>
    class AbstractTokenForProximityDatabase
    {
    public:

        virtual ~AbstractTokenForProximityDatabase () {}

        // the client object calls this each time its position changes
        virtual void updateForNewPosition (const Vec3& position) = 0;

        // find all neighbors within the given sphere (as center and radius)
        virtual void findNeighbors (const Vec3& center,
                                    const float radius,
                                    std::vector<ContentType>& results) = 0;
#ifndef NO_LQ_BIN_STATS
        // only meaningful for LQProximityDatabase, provide dummy default
        virtual void getBinPopulationStats (int& min, int& max, float& average)
        {min=max=0; average=0.0;}
#endif // NO_LQ_BIN_STATS
    };


    // ----------------------------------------------------------------------------
    // abstract type for all kinds of proximity databases


    template <class ContentType>
    class AbstractProximityDatabase
    {
    public:

        // type for the "tokens" manipulated by this spatial database
        typedef AbstractTokenForProximityDatabase<ContentType> tokenType;

        
        virtual ~AbstractProximityDatabase() { /* Nothing to do? */ }
        
        // allocate a token to represent a given client object in this database
        virtual tokenType* allocateToken (ContentType parentObject) = 0;

        // insert
        // XXX maybe this should return an iterator?
        // XXX see http://www.sgi.com/tech/stl/set.html
        // virtual void insert (const ContentType& x) = 0;

        // XXX name?
        // returns the number of tokens in the proximity database
        virtual int getPopulation (void) = 0;
    };



    // ----------------------------------------------------------------------------


        
//     class AVProximityDatabase
//     {
//     public:

//         // constructor
//         AVProximityDatabase (const Vec3& center,
//                              const Vec3& dimensions,
//                              const Vec3& divisions)
//         {
//             const Vec3 halfsize (dimensions * 0.5f);
//             origin = Vec3(center - halfsize);
//             width = dimensions.x;
//             height = dimensions.y;
//             depth = dimensions.z;

//             divx = divisions.x;
//             divy = divisions.y;
//             divz = divisions.z;

//             int bincount = divx * divy * divz;
//             bins.reserve(bincount);
//             for(int i = 0; i < bincount ; i++){
//                 bins.push_back(new Bin());
//             }

//             other = new Bin();
//         }

//         // destructor
//         virtual ~AVProximityDatabase ()
//         {
//             // lqDeleteDatabase (lq);
//             // av = NULL;
//         }

//         class Bin{
//         public:
//             Bin(){
//                 size = 0;
//                 x.reserve(100);
//                 y.reserve(100);
//                 z.reserve(100);
//                 objects.reserve(100);
//             }

//             std::vector<float> x;
//             std::vector<float> y;
//             std::vector<float> z;
//             std::vector<AbstractVehicle*> objects;
//             int size;
//         };

//         // "token" to represent objects stored in the database
//         class tokenType
//         {
//         public:

//             // constructor
//             tokenType (AbstractVehicle* parentObject, AVProximityDatabase* lqsd)
//             {
//                 av = lqsd;
//                 Vec3 pos = parentObject->position();
//                 Vec3 index = positionToIndex(pos);
//                 int binID = (index.x * av->divy*av->divz) + (index.y * av->divz) + index.z;
                
//                 if(binID >= av->divx*av->divy*av->divz)
//                     bin = av->other;
//                 else
//                     bin = av->bins[binID];

//                 offset = bin->size;

//                 //insert object in bin
//                 bin->x.emplace_back(pos.x);
//                 bin->y.emplace_back(pos.y);
//                 bin->z.emplace_back(pos.z);
//                 bin->objects.push_back(parentObject);
//                 bin->size+=1;
//             }
            
//             Vec3 positionToIndex(const Vec3& position){
//                 Vec3 pos = position - av->origin;
//                 int x = (pos.x / av->width) * av->divx;
//                 int y = (pos.y / av->height) * av->divy;
//                 int z = (pos.z / av->depth) * av->divz;
                
//                 return Vec3( clamp<int>(x,0,(int)av->divx),
//                                 clamp<int>(y,0,(int)av->divy),
//                                 clamp<int>(z,0,(int)av->divz) );
//             }
//             // destructor
//             virtual ~tokenType (void)
//             {
//                 bin->x.erase( bin->x.begin() + offset);
//                 bin->y.erase( bin->y.begin() + offset);
//                 bin->z.erase( bin->z.begin() + offset);
//                 bin->objects.erase( bin->objects.begin() + offset);
//                 bin->size-=1;
//             }

//             // the client object calls this each time its position changes
//             void updateForNewPosition (const Vec3& p)
//             {
//                 Vec3 index = positionToIndex(p);
//                 int binID = (index.x * av->divy*av->divz) + (index.y * av->divz) + index.z;
//                 Bin* otherBin;
//                 if(binID >= av->divx*av->divy*av->divz)
//                     otherBin = av->other;
//                 else
//                     otherBin = av->bins[binID];
                
//                 if( bin == otherBin){
//                     bin->x[offset] = p.x;
//                     bin->y[offset] = p.y;
//                     bin->z[offset] = p.z;
//                 }
//                 else{
//                     bin->x.erase(bin->x.begin() + offset);
//                     bin->y.erase(bin->y.begin() + offset);
//                     bin->z.erase(bin->z.begin() + offset);
//                     bin->size-=1;
//                     otherBin->objects.push_back( bin->objects[offset] );
//                     bin->objects.erase(bin->objects.begin()+offset);
//                     otherBin->x.push_back(p.x);
//                     otherBin->y.push_back(p.y);
//                     otherBin->z.push_back(p.z);
//                     offset = otherBin->size;
//                     otherBin->size+=1;
//                     bin = otherBin;
//                 }
                
                
//             }

//             // find all neighbors within the given sphere (as center and radius)
//             void findNeighbors (const Vec3& center,
//                                 const float radius,
//                                 std::vector<AVPack*>& results)
//             {
//                 Vec3 binIndex = positionToIndex(center);
//                 int minBinX = binIndex.x - (int) ((radius/av->width)* av->divx);
//                 int minBinY = binIndex.y - (int) ((radius/av->height)* av->divy);
//                 int minBinZ = binIndex.z - (int) ((radius/av->depth)* av->divz);
//                 int maxBinX = binIndex.x + (int) ((radius/av->width)* av->divx);
//                 int maxBinY = binIndex.y + (int) ((radius/av->height)* av->divy);
//                 int maxBinZ = binIndex.z + (int) ((radius/av->depth)* av->divz);
                
//                 int slab = av->divy * av->divz;
//                 int row = av->divz;
//                 int istart = minBinX * slab;
//                 int jstart = minBinY * row;
//                 int kstart = minBinZ;
//                 int iindex,jindex,kindex;
//                 int i,j,k;
//                 Bin* currBin;

//                 __m256 centerX = _mm256_set1_ps(center.x);
//                 __m256 centerY = _mm256_set1_ps(center.y);
//                 __m256 centerZ = _mm256_set1_ps(center.z);

//                 iindex = istart;

//                 results.clear();
//                 results.emplace_back(new AVPack());

//                 for (i = minBinX; i <= maxBinX; i++)
//                 {
//                 /* loop for y bins across diameter of sphere */
//                     jindex = jstart;
//                     for (j = minBinY; j <= maxBinY; j++)
//                     {
//                         /* loop for z bins across diameter of sphere */
//                         kindex = kstart;
//                         for (k = minBinZ; k <= maxBinZ; k++)
//                         {
//                             currBin = av->other;
                            
//                             for(int c = 0; c < bin->size; c++){
//                                 if(bin->objects[c] == bin->objects[offset]) continue;
//                                 float dx = center.x - bin->x[c];
//                                 float dy = center.y - bin->y[c];
//                                 float dz = center.z - bin->z[c];

//                                 float distanceSquared = (dx*dx) + (dy*dy) +(dz*dz);
//                                 if( distanceSquared < radius*radius){
//                                     if(results.back()->size() == PACKSIZE) results.emplace_back(new AVPack());
//                                     results.back()->push(bin->objects[c]);
//                                 }

//                             }   

//                             kindex+=1;
//                         }
//                         jindex += row;
//                     }
//                     iindex += slab;
//                 }

//             }

// #ifndef NO_LQ_BIN_STATS
//             // Get statistics about bin populations: min, max and
//             // average of non-empty bins.
//             void getBinPopulationStats (int& min, int& max, float& average)
//             {
                
//             }
// #endif // NO_LQ_BIN_STATS

//         private:
//             Bin* bin;
//             int offset;
//             AVProximityDatabase* av;
//         };


//         // allocate a token to represent a given client object in this database
//         tokenType* allocateToken (AbstractVehicle* parentObject)
//         {
//             return new tokenType (parentObject, this);
//         }

//         // count the number of tokens currently in the database
//         int getPopulation (void)
//         {
//             int count = 0;
            
//             return count;
//         }
        

    
    
//     Vec3 origin;
//     float width,height,depth;
//     float divx,divy,divz;

//     private:
        
//         std::vector<Bin*> bins;
//         Bin* other;
//     };




    // ----------------------------------------------------------------------------
    // This is the "brute force" O(n^2) approach implemented in terms of the
    // AbstractProximityDatabase protocol so it can be compared directly to other
    // approaches.  (e.g. the Boids plugin allows switching at runtime.)


    template <class ContentType>
    class BruteForceProximityDatabase
        : public AbstractProximityDatabase<ContentType>
    {
    public:

        // constructor
        BruteForceProximityDatabase (void)
        {
        }

        // destructor
        virtual ~BruteForceProximityDatabase ()
        {
        }

        // "token" to represent objects stored in the database
        class tokenType : public AbstractTokenForProximityDatabase<ContentType>
        {
        public:

            // constructor
            tokenType (ContentType parentObject, BruteForceProximityDatabase& pd)
            {
                // store pointer to our associated database and the object this
                // token represents, and store this token on the database's vector
                bfpd = &pd;
                object = parentObject;
                bfpd->group.push_back (this);
            }

            // destructor
            virtual ~tokenType ()
            {
                // remove this token from the database's vector
                bfpd->group.erase (std::find (bfpd->group.begin(),
                                              bfpd->group.end(),
                                              this));
            }

            // the client object calls this each time its position changes
            void updateForNewPosition (const Vec3& newPosition)
            {
                position = newPosition;
            }

            // find all neighbors within the given sphere (as center and radius)
            void findNeighbors (const Vec3& center,
                                const float radius,
                                std::vector<ContentType>& results)
            {
                
                // loop over all tokens
                const float r2 = radius * radius;
                for (tokenIterator i = bfpd->group.begin();
                     i != bfpd->group.end();
                     i++)
                {
                    const Vec3 offset = center - (**i).position;
                    const float d2 = offset.lengthSquared();

                    // push onto result vector when within given radius
                    if (d2 < r2) results.push_back ((**i).object);
                }
            }

        private:
            BruteForceProximityDatabase* bfpd;
            ContentType object;
            Vec3 position;
        };

        typedef std::vector<tokenType*> tokenVector;
        typedef typename tokenVector::const_iterator tokenIterator;    

        // allocate a token to represent a given client object in this database
        tokenType* allocateToken (ContentType parentObject)
        {
            return new tokenType (parentObject, *this);
        }

        // return the number of tokens currently in the database
        int getPopulation (void)
        {
            return (int) group.size();
        }
        
    private:
        // STL vector containing all tokens in database
        tokenVector group;
    };


    // ----------------------------------------------------------------------------
    // A AbstractProximityDatabase-style wrapper for the LQ bin lattice system


    template <class ContentType>
    class LQProximityDatabase : public AbstractProximityDatabase<ContentType>
    {
    public:

        // constructor
        LQProximityDatabase (const Vec3& center,
                             const Vec3& dimensions,
                             const Vec3& divisions)
        {
            const Vec3 halfsize (dimensions * 0.5f);
            const Vec3 origin (center - halfsize);

            lq = lqCreateDatabase (origin.x, origin.y, origin.z, 
                                   dimensions.x, dimensions.y, dimensions.z,  
                                   (int) round (divisions.x),
                                   (int) round (divisions.y),
                                   (int) round (divisions.z));
        }

        // destructor
        virtual ~LQProximityDatabase ()
        {
            lqDeleteDatabase (lq);
            lq = NULL;
        }

        // "token" to represent objects stored in the database
        class tokenType : public AbstractTokenForProximityDatabase<ContentType>
        {
        public:

            // constructor
            tokenType (ContentType parentObject, LQProximityDatabase& lqsd)
            {
                lqInitClientProxy (&proxy, parentObject);
                lq = lqsd.lq;
            }

            // destructor
            virtual ~tokenType (void)
            {
                lqRemoveFromBin (&proxy);
            }

            // the client object calls this each time its position changes
            void updateForNewPosition (const Vec3& p)
            {
                lqUpdateForNewLocation (lq, &proxy, p.x, p.y, p.z);
            }

            // find all neighbors within the given sphere (as center and radius)
            void findNeighbors (const Vec3& center,
                                const float radius,
                                std::vector<ContentType>& results)
            {
                lqMapOverAllObjectsInLocality (lq, 
                                               center.x, center.y, center.z,
                                               radius,
                                               perNeighborCallBackFunction,
                                               (void*)&results);
            }

            // called by LQ for each clientObject in the specified neighborhood:
            // push that clientObject onto the ContentType vector in void*
            // clientQueryState
            // (parameter names commented out to prevent compiler warning from "-W")
            static void perNeighborCallBackFunction  (void* clientObject,
                                                      float /*distanceSquared*/,
                                                      void* clientQueryState)
            {
                typedef std::vector<ContentType> ctv;
                ctv& results = *((ctv*) clientQueryState);
                results.push_back ((ContentType) clientObject);
            }

            // find all neighbors within the given sphere (as center and radius)
            void findNeighborsPack (const Vec3& center,
                                const float radius,
                                std::vector<AVPack*>& results)
            {
                lqMapOverAllObjectsInLocality (lq, 
                                               center.x, center.y, center.z,
                                               radius,
                                               perNeighborPackCallBackFunction,
                                               (void*)&results);
            }

            // called by LQ for each clientObject in the specified neighborhood:
            // push that clientObject onto the ContentType vector in void*
            // clientQueryState
            // (parameter names commented out to prevent compiler warning from "-W")
            static void perNeighborPackCallBackFunction  (void* clientObject,
                                                      float /*distanceSquared*/,
                                                      void* clientQueryState)
            {
                typedef std::vector<AVPack*> ctv;
                ctv& results = *((ctv*) clientQueryState);
                if(results.back()->size() == PACKSIZE) results.push_back(new AVPack());
                results.back()->push((ContentType) clientObject);
            }


#ifndef NO_LQ_BIN_STATS
            // Get statistics about bin populations: min, max and
            // average of non-empty bins.
            void getBinPopulationStats (int& min, int& max, float& average)
            {
                lqGetBinPopulationStats (lq, &min, &max, &average);
            }
#endif // NO_LQ_BIN_STATS

        private:
            lqClientProxy proxy;
            lqDB* lq;
        };


        // allocate a token to represent a given client object in this database
        tokenType* allocateToken (ContentType parentObject)
        {
            return new tokenType (parentObject, *this);
        }

        // count the number of tokens currently in the database
        int getPopulation (void)
        {
            int count = 0;
            lqMapOverAllObjects (lq, counterCallBackFunction, &count);
            return count;
        }
        
        // (parameter names commented out to prevent compiler warning from "-W")
        static void counterCallBackFunction  (void* /*clientObject*/,
                                              float /*distanceSquared*/,
                                              void* clientQueryState)
        {
            int& counter = *(int*)clientQueryState;
            counter++;
        }


    private:
        lqDB* lq;
    };

} // namespace OpenSteer



// ----------------------------------------------------------------------------
#endif // OPENSTEER_PROXIMITY_H
