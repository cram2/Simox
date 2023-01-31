#include "RobotNodeHemisphere.h"
#include "Nodes/Sensor.h"
#include "Robot.h"
#include "VirtualRobotException.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include <SimoxUtility/meta/enum/EnumNames.hpp>
#include <SimoxUtility/math/pose/pose.h>


namespace VirtualRobot
{

    static const float limit = 1.0;

    extern const simox::meta::EnumNames<RobotNodeHemisphere::Role> RoleNames =
    {
        { RobotNodeHemisphere::Role::FIRST, "first" },
        { RobotNodeHemisphere::Role::SECOND, "second" },
    };

    VirtualRobot::RobotNodeHemisphere::RobotNodeHemisphere()
    {
    }

    RobotNodeHemisphere::Role RobotNodeHemisphere::RoleFromString(const std::string& string)
    {
        return RoleNames.from_name(string);
    }

    RobotNodeHemisphere::RobotNodeHemisphere(
            RobotWeakPtr rob,
            const std::string& name,
            float jointLimitLo,
            float jointLimitHi,
            const Eigen::Matrix4f& preJointTransform,
            const Eigen::Vector3f& axis,
            VisualizationNodePtr visualization,
            CollisionModelPtr collisionModel,
            float jointValueOffset,
            const SceneObject::Physics& physics,
            CollisionCheckerPtr colChecker,
            RobotNodeType type
            ) :
        RobotNode(rob, name, -limit, limit, visualization, collisionModel,
                  jointValueOffset, physics, colChecker, type)
    {
        (void) axis;
        (void) jointLimitLo, (void) jointLimitHi;

        initialized = false;
        optionalDHParameter.isSet = false;
        localTransformation = preJointTransform;
        checkValidRobotNodeType();
    }


    RobotNodeHemisphere::RobotNodeHemisphere(
            RobotWeakPtr rob,
            const std::string& name,
            float jointLimitLo,
            float jointLimitHi,
            float a, float d, float alpha, float theta,
            VisualizationNodePtr visualization,
            CollisionModelPtr collisionModel,
            float jointValueOffset,
            const SceneObject::Physics& physics,
            CollisionCheckerPtr colChecker,
            RobotNodeType type
            ) :
        RobotNode(rob, name, jointLimitLo, jointLimitHi, visualization, collisionModel,
                  jointValueOffset, physics, colChecker, type)
    {
        initialized = false;
        optionalDHParameter.isSet = true;
        optionalDHParameter.setAInMM(a);
        optionalDHParameter.setDInMM(d);
        optionalDHParameter.setAlphaRadian(alpha, true);
        optionalDHParameter.setThetaRadian(theta, true);

        // compute DH transformation matrices
        Eigen::Matrix4f RotTheta = Eigen::Matrix4f::Identity();
        RotTheta(0, 0) = cos(theta);
        RotTheta(0, 1) = -sin(theta);
        RotTheta(1, 0) = sin(theta);
        RotTheta(1, 1) = cos(theta);
        Eigen::Matrix4f TransD = Eigen::Matrix4f::Identity();
        TransD(2, 3) = d;
        Eigen::Matrix4f TransA = Eigen::Matrix4f::Identity();
        TransA(0, 3) = a;
        Eigen::Matrix4f RotAlpha = Eigen::Matrix4f::Identity();
        RotAlpha(1, 1) = cos(alpha);
        RotAlpha(1, 2) = -sin(alpha);
        RotAlpha(2, 1) = sin(alpha);
        RotAlpha(2, 2) = cos(alpha);

        localTransformation = RotTheta * TransD * TransA * RotAlpha;
        checkValidRobotNodeType();
    }


    RobotNodeHemisphere::~RobotNodeHemisphere()
    = default;


    void RobotNodeHemisphere::setXmlInfo(const XmlInfo& info)
    {
        VR_ASSERT(second.has_value());
        switch (info.role)
        {
        case Role::FIRST:
            first.emplace(First{});
            first->math.joint.setConstants(info.lever, info.theta0);
            break;

        case Role::SECOND:
            second.emplace(Second{});
            break;
        }
    }


    bool RobotNodeHemisphere::initialize(
            SceneObjectPtr parent,
            const std::vector<SceneObjectPtr>& children)
    {
        VR_ASSERT_MESSAGE(first.has_value() xor second.has_value(), std::stringstream() << first.has_value() << " / " << second.has_value());

        // The second node needs to store a reference to the first node.
        if (second)
        {
            VR_ASSERT_MESSAGE(not second->first, "Second must not be initialized yet.");

            RobotNodeHemisphere* firstNode = dynamic_cast<RobotNodeHemisphere*>(parent.get());
            RobotNodeHemisphere* secondNode = this;

            if (not (firstNode and firstNode->first))
            {
                std::stringstream ss;
                ss << "The parent of a hemisphere joint with role '" << RoleNames.to_name(Role::SECOND) << "' "
                   << "must be a hemisphere joint with role '" << RoleNames.to_name(Role::FIRST) << "' ";
                THROW_VR_EXCEPTION(ss.str());
            }

            // Save pointer to firstNode
            second->first = firstNode;

            // Set up robot node parameters.
            {
                const hemisphere::Joint& joint = second->math().joint;

                firstNode->jointLimitLo = joint.limitLo;
                secondNode->jointLimitLo = joint.limitLo;

                firstNode->jointLimitHi = joint.limitHi;
                secondNode->jointLimitHi = joint.limitHi;
            }
        }

        return RobotNode::initialize(parent, children);
    }


    void RobotNodeHemisphere::JointMath::update(const Eigen::Vector2f& actuators)
    {
        if (actuators != this->actuators)
        {
            joint.computeFkOfAngle(actuators.cast<double>());
        }
    }


    void RobotNodeHemisphere::updateTransformationMatrices(
            const Eigen::Matrix4f& parentPose)
    {
        VR_ASSERT_MESSAGE(first.has_value() xor second.has_value(), std::stringstream() << first.has_value() << " / " << second.has_value());

        if (first)
        {
            globalPose = parentPose * localTransformation;
        }
        else if (second)
        {
            VR_ASSERT_MESSAGE(second->first, "First node must be known to second node.");

            JointMath& math = second->math();
            Eigen::Vector2f actuators(second->first->getJointValue(), this->getJointValue());

            math.update(actuators);

            Eigen::Vector3d translation = math.joint.getEndEffectorTranslation();
            const Eigen::Matrix3d rotation = math.joint.getEndEffectorRotation();
            const Eigen::Matrix4d transform = simox::math::pose(translation, rotation);

            // Update Second
            {
                this->globalPose = parentPose * localTransformation * transform.cast<float>();

                Eigen::IOFormat iof(5, 0, " ", "\n", "    [", "]");
                std::cout << __FUNCTION__ << "() of second actuator with "
                          << "\n  lever = " << math.joint.lever
                          << "\n  theta0 = " << math.joint.theta0
                          << "\n  radius = " << math.joint.radius
                          << "\n  joint value = " << jointValue
                          << "\n  actuator (angle) = \n" << actuators.transpose().format(iof)
                          << "\n  actuator (pos) =  \n" << math.joint.angleToPosition(actuators.cast<double>()).transpose().format(iof)
                          << "\n  local transform = \n" << localTransformation.format(iof)
                          << "\n  joint transform = \n" << transform.format(iof)
                          << std::endl;
            }
        }
    }


    void RobotNodeHemisphere::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        VR_ASSERT_MESSAGE(first.has_value() xor second.has_value(), std::stringstream() << first.has_value() << " / " << second.has_value());

        if (printDecoration)
        {
            std::cout << "******** RobotNodeHemisphere ********" << std::endl;
        }

        RobotNode::print(false, false);

        if (first)
        {
            std::cout << "* Hemisphere joint first node";
        }
        else if (second)
        {
            std::cout << "* Hemisphere joint second node";
            std::cout << "* Transform: \n" << second->math().joint.getEndEffectorTransform() << std::endl;
        }

        if (printDecoration)
        {
            std::cout << "******** End RobotNodeHemisphere ********" << std::endl;
        }

        if (printChildren)
        {
            for (const SceneObjectPtr& child : this->getChildren())
            {
                child->print(true, true);
            }
        }
    }


    RobotNodePtr RobotNodeHemisphere::_clone(
            const RobotPtr newRobot,
            const VisualizationNodePtr visualizationModel,
            const CollisionModelPtr collisionModel,
            CollisionCheckerPtr colChecker,
            float scaling)
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        Physics physics = this->physics.scale(scaling);

        RobotNodeHemispherePtr cloned;
        if (optionalDHParameter.isSet)
        {
            cloned.reset(new RobotNodeHemisphere(
                             newRobot, name, jointLimitLo, jointLimitHi,
                             optionalDHParameter.aMM() * scaling,
                             optionalDHParameter.dMM() * scaling,
                             optionalDHParameter.alphaRadian(),
                             optionalDHParameter.thetaRadian(),
                             visualizationModel, collisionModel,
                             jointValueOffset,
                             physics, colChecker, nodeType));
        }
        else
        {
            Eigen::Matrix4f localTransform = getLocalTransformation();
            simox::math::position(localTransform) *= scaling;
            cloned.reset(new RobotNodeHemisphere(
                             newRobot, name,
                             jointLimitLo, jointLimitHi,
                             localTransform, Eigen::Vector3f::Zero(),
                             visualizationModel, collisionModel,
                             jointValueOffset, physics, colChecker, nodeType));
        }

        if (this->first)
        {
            // We can just copy the math object.
            cloned->first = first;
        }
        else if (this->second)
        {
            cloned->second.emplace(Second{});
            // initialize() takes care of hooking up the second to the first.
        }

        return cloned;
    }


    bool RobotNodeHemisphere::isHemisphereJoint() const
    {
        return true;
    }


    void RobotNodeHemisphere::checkValidRobotNodeType()
    {
        RobotNode::checkValidRobotNodeType();
        THROW_VR_EXCEPTION_IF(nodeType == Body || nodeType == Transform, "RobotNodeHemisphere must be a JointNode or a GenericNode");
    }


    std::string RobotNodeHemisphere::_toXML(const std::string& /*modelPath*/)
    {
        VR_ASSERT_MESSAGE(first.has_value() xor second.has_value(), std::stringstream() << first.has_value() << " / " << second.has_value());

        if (first)
        {
            // TODO
            return "";
        }
        else
        {
            JointMath& math = second->math();

            std::stringstream ss;
            ss << "\t\t<Joint type='Hemisphere'>" << std::endl;
            ss << "\t\t\t<hemisphere lever='" << math.joint.lever << "' theta0='" << math.joint.theta0 << "' />" << std::endl;
            ss << "\t\t\t<limits lo='" << jointLimitLo << "' hi='" << jointLimitHi << "' units='radian'/>" << std::endl;
            ss << "\t\t\t<MaxAcceleration value='" << maxAcceleration << "'/>" << std::endl;
            ss << "\t\t\t<MaxVelocity value='" << maxVelocity << "'/>" << std::endl;
            ss << "\t\t\t<MaxTorque value='" << maxTorque << "'/>" << std::endl;
            std::map< std::string, float >::iterator propIt = propagatedJointValues.begin();

            while (propIt != propagatedJointValues.end())
            {
                ss << "\t\t\t<PropagateJointValue name='" << propIt->first << "' factor='" << propIt->second << "'/>" << std::endl;
                propIt++;
            }

            ss << "\t\t</Joint>" << std::endl;
            return ss.str();
        }
    }

}
