/*
* This file is part of ArmarX.
*
* ArmarX is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* ArmarX is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @author     Martin Miller (martin dot miller at student dot kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#ifndef math_MarchingCubes
#define math_MarchingCubes

#include "MathForwardDefinitions.h"
#include "Array3D.h"
#include "Triangle.h"
#include <queue>


namespace math
{

 // code from https://nucleardevs.wordpress.com/2011/11/17/marching-cubes-sourcecode/
    class MarchingCubes
    {
    public:
        void Init(int size, Eigen::Vector3f center, Eigen::Vector3f start, int steps, float stepLength, GridCacheFloat3Ptr cache);
        PrimitivePtr Process(float isolevel);
        void Process(float isolevel, PrimitivePtr primitive); //TODO ref
        PrimitivePtr ProcessSingleSurfaceOptimized(float isolevel, Index3 start);
        void ProcessSingleSurfaceOptimized(float isolevel, PrimitivePtr primitive, Index3 start);
        static PrimitivePtr Calculate(Eigen::Vector3f center, Eigen::Vector3f start, int steps, float stepLength, SimpleAbstractFunctionR3R1Ptr modelPtr, float isolevel);

    private:
      struct GridCell{
          public:
              Eigen::Vector3f P[8];
      };

      typedef boost::shared_ptr<Array3D<GridCell>> Array3DGridCellPtr;
      typedef boost::shared_ptr<std::queue<Index3>> FringePtr;

      int _size;
      GridCacheFloat3Ptr Gdata;
      Array3DGridCellPtr _grids;
      Triangle _triangles[16];

      static const int _edgeTable[256];
      static const char _triTable[256][16];


      float GetVal(int x, int y, int z, int i);
      Eigen::Vector3f GetPos(int x, int y, int z, int i);
      void AddAndMark(int x, int y, int z, FringePtr fringe, Array3DBoolPtr marked);
      bool IsValidIndex(int x, int y, int z);

      int Polygonise(int x, int y, int z, float isolevel);

      static Eigen::Vector3f VertexInterp(float isolevel, Eigen::Vector3f p1, Eigen::Vector3f p2, float valp1, float valp2);

      void Build(PrimitivePtr res, int tianglesNum);

    };
}

#endif // math_MarchingCubes
