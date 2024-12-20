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
* @author     Andre Meixner
* @copyright  2020 Andre Meixner
*             GNU Lesser General Public License
*
*/

#pragma once

#include <SimoxUtility/math/convert.h>

#include "Grasp.h"
#include "VirtualRobot/VirtualRobot.h"

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT ChainedGrasp : public Grasp
    {
    public:
        struct VirtualJoint
        {
            friend class ChainedGrasp;

            bool setValue(float value);

            void resetLimits();

            bool setLimitsValue(float min, float max);

            bool setLimits(float min, float max);

            bool setLimitless();

            bool isLimitless();

            Eigen::Vector3f getAxis() const;

            float getMin() const;

            float getMax() const;

            float isUsed() const;

            std::string toXML() const;

        private:
            VirtualJoint(const std::string& name, bool isRevolute, int axis);

            void addLimits(std::vector<float>& usedLimits) const;

            std::string name;
            float value;
            float min;
            float max;
            bool isRevolute;
            int axis;
        };

        ChainedGrasp(const std::string& name,
                     const std::string& robotType,
                     const std::string& eef,
                     const Eigen::Matrix4f& poseInTCPCoordSystem,
                     const std::string& creation = "",
                     float quality = 0.0f,
                     const std::string& eefPreshape = "");

        virtual ~ChainedGrasp();

        virtual void print(bool printDecoration = true) const override;

        RobotNodePtr attachChain(RobotPtr robot,
                                 GraspableSensorizedObjectPtr object,
                                 bool addObjectVisualization = false);

        void updateChain(RobotPtr robot);

        void detachChain(RobotPtr robot);

        void sampleGraspsUniform(std::vector<Eigen::Matrix4f>& grasps,
                                 RobotPtr robot,
                                 unsigned int grid);
        std::vector<RobotPtr> sampleHandsUniform(RobotPtr robot, unsigned int grid);

        VirtualJoint x;
        VirtualJoint y;
        VirtualJoint z;
        VirtualJoint roll;
        VirtualJoint pitch;
        VirtualJoint yaw;

        VirtualJoint* getVirtualJoint(const std::string& name);

        std::vector<float> getUsedVirtualLimits() const;

        Eigen::Vector3f getPositionXYZ() const;

        Eigen::Vector3f getOrientationRPY() const;

        virtual Eigen::Matrix4f getTransformation() const override;

        /*! Returns hand pose in object coordinate system */
        Eigen::Matrix4f getLocalPoseGrasp();

        void setObjectTransformation(const Eigen::Matrix4f& graspableObjectCoordSysTransformation);

        std::vector<std::string> getNames(bool onlyAdaptable = true);
        RobotNodeSetPtr createRobotNodeSet(RobotPtr robot, RobotNodeSetPtr rns);

        bool visualizeRotationCoordSystem(RobotPtr robot, bool visible = true);

        virtual GraspPtr clone() const override;

        std::string ROBOT_NODE_NAME(const std::string& virtualJointName);

        RobotNodePtr getObjectNode(RobotPtr robot);
        RobotNodePtr getVirtualNode(RobotPtr robot, const std::string& virtualName);

        EndEffectorPtr getEndEffector(RobotPtr robot);
        RobotNodePtr getTCP(RobotPtr robot);

    protected:
        virtual std::string getTransformationXML(const std::string& tabs) const override;

    private:
        Eigen::Matrix4f graspableObjectCoordSysTransformation;

        RobotNodePtr attachChain(RobotNodePtr robotNode,
                                 GraspableSensorizedObjectPtr object,
                                 bool addObjectVisualization = false);
        RobotNodePtr attach(VirtualJoint joint, RobotNodePtr robotNode);
        bool update(VirtualJoint joint, RobotPtr robot);
        void sampleGraspsUniform(std::vector<Eigen::Matrix4f>& grasps,
                                 const std::string& rootName,
                                 RobotNodePtr robotNode,
                                 unsigned int grid);

        Eigen::Matrix4f getLocalPose() const;
    };

} // namespace VirtualRobot
