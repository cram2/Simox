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

#include "../VirtualRobot.h"

#include <string>
#include <vector>

#include <Eigen/Core>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT GraspSet
    {
    public:

        /*!
        */
        GraspSet(const std::string& name, const std::string& robotType, const std::string& eef, const std::vector< GraspPtr >& grasps = std::vector< GraspPtr >());

        /*!
        */
        virtual ~GraspSet();

        std::string getName() const;
        std::string getRobotType() const;
        std::string getEndEffector() const;


        void addGrasp(GraspPtr grasp);
        bool hasGrasp(GraspPtr grasp) const;

        bool hasGrasp(const std::string& name) const;
        bool removeGrasp(GraspPtr grasp);
        bool removeGrasp(unsigned int i);
        void removeAllGrasps();
        bool isCompatibleGrasp(GraspPtr grasp) const;
        void clear();
        void includeGraspSet(GraspSetPtr grasps);

        /*!
            Return number of grasps stored in this set.
        */
        std::size_t getSize() const;

        /*!
            Return grasp number n. If n is out of bounds an empty GraspPtr is returned.
        */
        GraspPtr getGrasp(std::size_t n) const;
        GraspPtr getGrasp(const std::string& name) const;

        void print() const;

        std::string getXMLString(int tabs = 1) const;

        GraspSetPtr clone() const;

        std::vector< GraspPtr > getGrasps() const;

        //! Sets preshape string of all grasps
        void setPreshape(const std::string& preshape);
        
        auto begin() const
        {
            return grasps.begin();
        }
        auto end() const
        {
            return grasps.end();
        }

        bool operator==(const GraspSet& rhs) const
        {
            return (name == rhs.name) and (robotType == rhs.robotType) 
                and (eef == rhs.eef) and (grasps.size() == rhs.grasps.size());
        }
        
    protected:
        std::vector< GraspPtr > grasps;
        std::string name;
        std::string robotType;
        std::string eef;

    };

} // namespace
