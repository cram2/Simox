/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2013 H2T,KIT
*             GNU Lesser General Public License
*
*/
#pragma once

#include <vector>

#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>

#include "GraspStudio.h"

namespace GraspStudio
{

    class GRASPSTUDIO_IMPORT_EXPORT MeshConverter
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Create an object. The visualization and collision model is created from the convex hull.
        */
        static VirtualRobot::ManipulationObjectPtr
        CreateManipulationObject(const std::string& name,
                                 VirtualRobot::MathTools::ConvexHull3DPtr hull);
        static VirtualRobot::TriMeshModelPtr
        CreateTriMeshModel(VirtualRobot::MathTools::ConvexHull3DPtr hull);
        static VirtualRobot::ObstaclePtr
        RefineObjectSurface(VirtualRobot::ObstaclePtr object, float maxDist, bool verbose = true);

        //! Returns -1 if obj is not part of vectList, otherwise the index of vectList is returned.
        static int hasVertex(std::vector<Eigen::Vector3f>& vectList, Eigen::Vector3f& obj);
        static void
        checkAndSplitVertex(VirtualRobot::TriMeshModelPtr tm, int faceIdx, float maxDist);

        static float getMaxVertexDistance(VirtualRobot::TriMeshModelPtr tm);
    };
} // namespace GraspStudio
