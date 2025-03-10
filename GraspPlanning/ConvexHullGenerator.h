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
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <vector>

#include "GraspStudio.h"
#include "VirtualRobot/MathTools.h"

namespace GraspStudio
{
    /*!
    * This class can be used as an interface for qhull.
    * A convex hull can be generated out of point arrays.
    * This class is thread safe, which means that multiple threads
    * are allowed to use the static methods of ConvexHullGenerator.
    */
    class GRASPSTUDIO_IMPORT_EXPORT ConvexHullGenerator
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*!
            Creates a convex hull of the points stored in pointsInput.
        */
        static VirtualRobot::MathTools::ConvexHull3DPtr
        CreateConvexHull(const std::vector<Eigen::Vector3f>& pointsInput);
        static VirtualRobot::MathTools::ConvexHull3DPtr
        CreateConvexHull(VirtualRobot::TriMeshModelPtr pointsInput);
        static VirtualRobot::MathTools::ConvexHull6DPtr
        CreateConvexHull(std::vector<VirtualRobot::MathTools::ContactPoint>& pointsInput);

        static void PrintStatistics(VirtualRobot::MathTools::ConvexHull6DPtr convHull);

        /*!
            Convert points to qhull format
        */
        static bool ConvertPoints(const std::vector<Eigen::Vector3f>& points,
                                  double* storePointsQHull);
        static bool ConvertPoints(const std::vector<VirtualRobot::MathTools::ContactPoint>& points,
                                  double* storePointsQHull);

        static void PrintVertices(std::vector<VirtualRobot::MathTools::ContactPoint>& pointsInput);

        static bool checkVerticeOrientation(const Eigen::Vector3f& v1,
                                            const Eigen::Vector3f& v2,
                                            const Eigen::Vector3f& v3,
                                            const Eigen::Vector3f& n);
    };
} // namespace GraspStudio
