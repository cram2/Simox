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

#include <string>

#include <Eigen/Core>

#include "Obstacle.h"
#include "VirtualRobot.h"

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT ManipulationObject : public Obstacle
    {
    public:
        ManipulationObject(const std::string& name,
                           VisualizationNodePtr visualization = VisualizationNodePtr(),
                           CollisionModelPtr collisionModel = CollisionModelPtr(),
                           const SceneObject::Physics& p = SceneObject::Physics(),
                           CollisionCheckerPtr colChecker = CollisionCheckerPtr());
        ManipulationObject(const std::string& name,
                           const TriMeshModelPtr& trimesh,
                           const std::string& filename = "");
        ManipulationObject(const TriMeshModelPtr& trimesh);
        /*!
        */
        ~ManipulationObject() override;

        void print(bool printChildren = false, bool printDecoration = true) const override;

        /*!
            Creates an XML representation of this object.
            \param basePath If set, all visualization and collision model files are made relative to this path.
            \param tabs Create indention at the beginning of each line.
            \param storeLinkToFile If set, the data (e.g. grasps) are not explicitly listed, but an xml tag directing to the XML file,
                    from which this instance was loaded, is set. If not set a deep description is created.
        */
        virtual std::string toXML(const std::string& basePath = std::string(),
                                  int tabs = 0,
                                  bool storeLinkToFile = false,
                                  const std::string& modelPathRelative = "",
                                  bool storeSensors = true);

        /*!
            Clones this object. If no col checker is given, the one of the original object is used.
        */
        ManipulationObjectPtr clone(const std::string& name,
                                    CollisionCheckerPtr colChecker = CollisionCheckerPtr(),
                                    bool deepVisuCopy = true) const;
        ManipulationObjectPtr clone(CollisionCheckerPtr colChecker = CollisionCheckerPtr(),
                                    bool deepVisuCopy = true) const;

        /*!
        Create a standard obstacle from a mesh.
        \param mesh The mesh.
        \param visualizationType Here the type of visualization can be specified (e.g. "Inventor"). If empty, the first registered visualization type (which is usually the only one) is used.
        \param colChecker Only needed if you plan to use the collision checker in parallel. If not given, the object is registered with the global singleton collision checker.
        */
        static ManipulationObjectPtr
        createFromMesh(TriMeshModelPtr mesh,
                       std::string visualizationType = "",
                       CollisionCheckerPtr colChecker = CollisionCheckerPtr());
    };

} // namespace VirtualRobot
