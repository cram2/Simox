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

#include <VirtualRobot/VirtualRobotImportExport.h>

#include <Eigen/Core>
#include <vector>

namespace VirtualRobot
{
    namespace MathTools
    {
        struct Plane;
    }

    class CollisionChecker;
    /*!
        An axis oriented bounding box.
        Todo: Some parts of this class are similar to MathTools::OOBB.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT BoundingBox
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        friend class CollisionChecker;

        BoundingBox();
        BoundingBox(const std::vector< Eigen::Vector3f >& p);

        /*!
            Returns true, if plane "hits" this bounding box.
        */
        bool planeGoesThrough(const VirtualRobot::MathTools::Plane& p);


        /*!
            Returns 8 points that define the bounding box
        */
        std::vector <Eigen::Vector3f> getPoints() const;

        //! Print some info
        void print();

        /*!
            Consider these points for min/max calculation
        */
        void addPoints(const std::vector < Eigen::Vector3f >& p);

        /*!
            Consider these points for min/max calculation
        */
        void addPoints(const BoundingBox& bbox);

        /*!
            Consider this point for min/max calculation
        */
        void addPoint(const Eigen::Vector3f& p);

        //! The axis oriented minimum value
        Eigen::Vector3f getMin() const;

        //! The axis oriented maximum value
        Eigen::Vector3f getMax() const;

        //! set min/max to NAN.
        void clear();

        /*!
            Applies transformation to this bbox. Reorders min and max values according to pose.
        */
        void transform(Eigen::Matrix4f& pose);

        void scale(const Eigen::Vector3f& scaleFactor);

        std::string toXML(int tabs = 2, bool skipMatrixTag = false);

        /*!
         * \brief isInside Checks whether the given point lies within the bounding box. The given vector can contain
         * entries that are none, if this is the case the respective dimension is ignored and the point is only tested
         * for the remaining non-nan dimensions.
         * \param point the point to test
         * \return True if the point lies within the bounding box. False if not.
         */
        bool isInside(Eigen::Vector3f point) const;

    protected:
        Eigen::Vector3f min;
        Eigen::Vector3f max;
    };

} // namespace VirtualRobot

