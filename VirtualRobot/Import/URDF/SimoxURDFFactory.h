/*
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
* @copyright  2014 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#pragma once

#include <VirtualRobot/Import/RobotImporterFactory.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotImportExport.h>

#include <urdf_model/link.h>
#include <urdf_model/model.h>

namespace VirtualRobot
{
    class Robot;

    class VIRTUAL_ROBOT_IMPORT_EXPORT SimoxURDFFactory : public RobotImporterFactory
    {
    public:
        SimoxURDFFactory();
        virtual ~SimoxURDFFactory();

        /*!
            Load an urdf model and convert it to a simox robot.
            The 3D model files are assumed to be accessible through the VirtualRobot::RuntimeEnvironment paths; be sure to add any 3d model paths there.
            \param filename The urdf model file
            \param loadMode Currently only full loading is supported (ignored)
        */
        virtual RobotPtr loadFromFile(const std::string& filename,
                                      RobotIO::RobotDescription loadMode = RobotIO::eFull);

        /*!
            \param useColModelsIfNoVisuModel If set (standard), a missing visualization is compensated by using the collision model (e.g. when the visu loading failed)
        */
        void set3DModelMode(bool useColModelsIfNoVisuModel);


        /*!
            Convert an urdf model.
            \param urdfModel The model
            \param useColModelsIfNoVisuModel If set, a missing visualization is compensated by using the collision model (e.g. when the visu loading failed)
        */
        RobotPtr createRobot(std::shared_ptr<urdf::ModelInterface> urdfModel,
                             const std::string& basePath,
                             bool useColModelsIfNoVisuModel = true);

        // AbstractFactoryMethod
    public:
        static std::string getName();
        static std::shared_ptr<RobotImporterFactory> createInstance(void*);

    private:
        static SubClassRegistry registry;


        // RobotImporterFactory interface
    public:
        virtual std::string getFileExtension();
        virtual std::string getFileFilter();


    protected:
        RobotNodePtr createBodyNode(const std::string& name,
                                    RobotPtr robot,
                                    std::shared_ptr<urdf::Link> urdfBody,
                                    const std::string& basePath,
                                    bool useColModelsIfNoVisuModel = true);
        RobotNodePtr createJointNode(RobotPtr robot, std::shared_ptr<urdf::Joint> urdfJoint);
        Eigen::Matrix4f convertPose(const urdf::Pose& p) const;
        VirtualRobot::VisualizationNodePtr convertVisu(const std::shared_ptr<urdf::Geometry>& g,
                                                       const urdf::Pose& pose,
                                                       const std::string& basePath);
        VirtualRobot::VisualizationNodePtr
        convertVisuArray(std::vector<std::shared_ptr<urdf::Visual>> visu_array,
                         const std::string& basePath);
        VirtualRobot::VisualizationNodePtr
        convertVisuArray(std::vector<std::shared_ptr<urdf::Collision>> visu_array,
                         const std::string& basePath);
        std::string getFilename(const std::string& f, const std::string& basePath);

        Primitive::PrimitivePtr convertPrimitive(const urdf::Geometry& g,
                                                 const urdf::Pose& p) const;
        Primitive::PrimitivePtr convertPrimitive(const urdf::Visual& col) const;
        Primitive::PrimitivePtr convertPrimitive(const urdf::Collision& col) const;

        std::vector<Primitive::PrimitivePtr>
        convertToPrimitives(const std::vector<std::shared_ptr<urdf::Collision>>& col_array) const;
        std::vector<Primitive::PrimitivePtr>
        convertToPrimitives(const std::vector<std::shared_ptr<urdf::Visual>>& col_array) const;

        bool useColModelsIfNoVisuModel;
    };

} // namespace VirtualRobot
