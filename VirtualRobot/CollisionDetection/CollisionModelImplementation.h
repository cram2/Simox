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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>

#include "VirtualRobot/VirtualRobot.h"

namespace VirtualRobot
{

    class CollisionChecker;
    class CollisionModel;

    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionModelImplementation
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        friend class CollisionModel;

        /*!Standard Constructor
        If collision checks should be done in parallel, different CollisionCheckers can be specified.
        */
        CollisionModelImplementation(const TriMeshModelPtr& modelData,
                                     const CollisionCheckerPtr& /*pColChecker*/,
                                     int id);

        /*!Standard Destructor
        */
        virtual ~CollisionModelImplementation();


        /*!
        Sets the position of the internal colModel data structure.
        */
        void setGlobalPose(const Eigen::Matrix4f& m);
        const Eigen::Matrix4f& getGlobalPose() const;


        virtual void print();

        const TriMeshModelPtr& getTriMeshModel();


        virtual std::shared_ptr<CollisionModelImplementation>
        clone(bool deepCopy = false) const = 0;

    protected:
        //! delete all data
        virtual void destroyData() = 0;

        TriMeshModelPtr modelData;

        int id;

        Eigen::Matrix4f globalPose;
    };


} // namespace VirtualRobot
