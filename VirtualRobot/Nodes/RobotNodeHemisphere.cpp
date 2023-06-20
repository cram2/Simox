#include "RobotNodeHemisphere.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include <SimoxUtility/math/convert/rad_to_deg.h>
#include <SimoxUtility/math/pose/pose.h>
#include <SimoxUtility/meta/enum/EnumNames.hpp>

#include "Nodes/Sensor.h"
#include "Robot.h"
#include "VirtualRobotException.h"

namespace VirtualRobot
{

    static const float initialLimit = 1.0;

    extern const simox::meta::EnumNames<RobotNodeHemisphere::Role> RoleNames = {
        {RobotNodeHemisphere::Role::FIRST, "first"},
        {RobotNodeHemisphere::Role::SECOND, "second"},
    };

    VirtualRobot::RobotNodeHemisphere::RobotNodeHemisphere()
    {
    }

    RobotNodeHemisphere::Role
    RobotNodeHemisphere::RoleFromString(const std::string& string)
    {
        return RoleNames.from_name(string);
    }

    RobotNodeHemisphere::RobotNodeHemisphere(RobotWeakPtr rob,
                                             const std::string& name,
                                             const Eigen::Matrix4f& preJointTransform,
                                             VisualizationNodePtr visualization,
                                             CollisionModelPtr collisionModel,
                                             float jointValueOffset,
                                             const SceneObject::Physics& physics,
                                             CollisionCheckerPtr colChecker,
                                             RobotNodeType type) :
        RobotNode(rob,
                  name,
                  -initialLimit,
                  initialLimit,
                  visualization,
                  collisionModel,
                  jointValueOffset,
                  physics,
                  colChecker,
                  type)
    {
        initialized = false;
        optionalDHParameter.isSet = false;
        localTransformation = preJointTransform;
        checkValidRobotNodeType();
    }

    RobotNodeHemisphere::RobotNodeHemisphere(RobotWeakPtr rob,
                                             const std::string& name,
                                             float a,
                                             float d,
                                             float alpha,
                                             float theta,
                                             VisualizationNodePtr visualization,
                                             CollisionModelPtr collisionModel,
                                             float jointValueOffset,
                                             const SceneObject::Physics& physics,
                                             CollisionCheckerPtr colChecker,
                                             RobotNodeType type) :
        RobotNode(rob,
                  name,
                  -initialLimit,
                  initialLimit,
                  visualization,
                  collisionModel,
                  jointValueOffset,
                  physics,
                  colChecker,
                  type)
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

    RobotNodeHemisphere::~RobotNodeHemisphere() = default;

    void
    RobotNodeHemisphere::setXmlInfo(const XmlInfo& info)
    {
        VR_ASSERT(secondData.has_value());
        switch (info.role)
        {
            case Role::FIRST:
                firstData.emplace(FirstData{});
                firstData->maths.maths.setConstants(info.lever, info.theta0Rad);
                break;

            case Role::SECOND:
                secondData.emplace(SecondData{});
                break;
        }
    }

    bool
    RobotNodeHemisphere::initialize(SceneObjectPtr parent,
                                    const std::vector<SceneObjectPtr>& children)
    {
        VR_ASSERT_MESSAGE(firstData.has_value() xor secondData.has_value(),
                          std::stringstream()
                              << firstData.has_value() << " / " << secondData.has_value());

        // The second node needs to store a reference to the first node.
        if (secondData)
        {
            VR_ASSERT_MESSAGE(not secondData->firstNode, "Second must not be initialized yet.");

            RobotNodeHemisphere* firstNode = dynamic_cast<RobotNodeHemisphere*>(parent.get());
            RobotNodeHemisphere* secondNode = this;

            if (not(firstNode and firstNode->firstData))
            {
                std::stringstream ss;
                ss << "The parent of a hemisphere joint with role '"
                   << RoleNames.to_name(Role::SECOND) << "' "
                   << "must be a hemisphere joint with role '" << RoleNames.to_name(Role::FIRST)
                   << "' ";
                THROW_VR_EXCEPTION(ss.str());
            }

            // Save pointer to firstNode
            secondData->firstNode = firstNode;
            secondData->secondNode = secondNode;

            // Set up robot node parameters.
            {
                const hemisphere::Maths& maths = secondData->maths().maths;

                firstNode->jointLimitLo = maths.limitLo;
                secondNode->jointLimitLo = maths.limitLo;

                firstNode->jointLimitHi = maths.limitHi;
                secondNode->jointLimitHi = maths.limitHi;
            }
        }

        return RobotNode::initialize(parent, children);
    }

    void
    RobotNodeHemisphere::updateTransformationMatrices(const Eigen::Matrix4f& parentPose)
    {
        VR_ASSERT_MESSAGE(firstData.has_value() xor secondData.has_value(),
                          std::stringstream()
                              << firstData.has_value() << " / " << secondData.has_value());

        if (firstData)
        {
            globalPose = parentPose * localTransformation;
        }
        else if (secondData)
        {
            VR_ASSERT_MESSAGE(secondData->firstNode, "First node must be known to second node.");

            hemisphere::CachedMaths& maths = secondData->maths();
            Eigen::Vector2f actuators(secondData->firstNode->getJointValue(),
                                      this->getJointValue());

            maths.update(actuators);

            Eigen::Vector3d translation = maths.maths.getEndEffectorTranslation();
            const Eigen::Matrix3d rotation = maths.maths.getEndEffectorRotation();
            const Eigen::Matrix4d transform = simox::math::pose(translation, rotation);

            // Update Second
            this->globalPose = parentPose * localTransformation * transform.cast<float>();

            const bool verbose = false;
            if (verbose)
            {
                Eigen::IOFormat iof(5, 0, " ", "\n", "    [", "]");
                std::cout
                    << __FUNCTION__ << "() of second actuator with "
                    << "\n  lever = " << maths.maths.lever
                    << "\n  theta0 = " << maths.maths.theta0Rad
                    << "\n  radius = " << maths.maths.radius << "\n  joint value = " << jointValue
                    << "\n  actuator (angle) = \n"
                    << actuators.transpose().format(iof) << "\n  actuator (pos) =  \n"
                    << maths.maths.angleToPosition(actuators.cast<double>()).transpose().format(iof)
                    << "\n  local transform = \n"
                    << localTransformation.format(iof) << "\n  joint transform = \n"
                    << transform.format(iof) << std::endl;
            }
        }
    }

    void
    RobotNodeHemisphere::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        VR_ASSERT_MESSAGE(firstData.has_value() xor secondData.has_value(),
                          std::stringstream()
                              << firstData.has_value() << " / " << secondData.has_value());

        if (printDecoration)
        {
            std::cout << "******** RobotNodeHemisphere ********" << std::endl;
        }

        RobotNode::print(false, false);

        if (firstData)
        {
            std::cout << "* Hemisphere joint first node";
        }
        else if (secondData)
        {
            std::cout << "* Hemisphere joint second node";
            std::cout << "* Transform: \n"
                      << secondData->maths().maths.getEndEffectorTransform() << std::endl;
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

    RobotNodePtr
    RobotNodeHemisphere::_clone(const RobotPtr newRobot,
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
            cloned.reset(new RobotNodeHemisphere(newRobot,
                                                 name,
                                                 optionalDHParameter.aMM() * scaling,
                                                 optionalDHParameter.dMM() * scaling,
                                                 optionalDHParameter.alphaRadian(),
                                                 optionalDHParameter.thetaRadian(),
                                                 visualizationModel,
                                                 collisionModel,
                                                 jointValueOffset,
                                                 physics,
                                                 colChecker,
                                                 nodeType));
        }
        else
        {
            Eigen::Matrix4f localTransform = getLocalTransformation();
            simox::math::position(localTransform) *= scaling;
            cloned.reset(new RobotNodeHemisphere(newRobot,
                                                 name,
                                                 localTransform,
                                                 visualizationModel,
                                                 collisionModel,
                                                 jointValueOffset,
                                                 physics,
                                                 colChecker,
                                                 nodeType));
        }

        if (this->firstData)
        {
            // We can just copy the math object.
            cloned->firstData = this->firstData;
        }
        else if (this->secondData)
        {
            cloned->secondData.emplace(SecondData{});
            // initialize() takes care of hooking up the second node to the first node.
        }

        return cloned;
    }

    bool
    RobotNodeHemisphere::isHemisphereJoint() const
    {
        return true;
    }

    bool
    RobotNodeHemisphere::isFirstHemisphereJointNode() const
    {
        return firstData.has_value();
    }

    bool
    RobotNodeHemisphere::isSecondHemisphereJointNode() const
    {
        return secondData.has_value();
    }

    const RobotNodeHemisphere::SecondData&
    RobotNodeHemisphere::getSecondData() const
    {
        VR_ASSERT(secondData.has_value());
        return secondData.value();
    }

    RobotNodeHemisphere::SecondData&
    RobotNodeHemisphere::getSecondData()
    {
        VR_ASSERT(secondData.has_value());
        return secondData.value();
    }

    void
    RobotNodeHemisphere::checkValidRobotNodeType()
    {
        RobotNode::checkValidRobotNodeType();
        THROW_VR_EXCEPTION_IF(nodeType == Body || nodeType == Transform,
                              "RobotNodeHemisphere must be a JointNode or a GenericNode");
    }

    std::string
    RobotNodeHemisphere::_toXML(const std::string& /*modelPath*/)
    {
        VR_ASSERT_MESSAGE(firstData.has_value() xor secondData.has_value(),
                          std::stringstream()
                              << firstData.has_value() << " / " << secondData.has_value());

        std::stringstream ss;
        ss << "\t\t<Joint type='Hemisphere'>" << std::endl;
        if (firstData)
        {
            // Constants are defined in first.

            hemisphere::Maths& maths = firstData->maths.maths;
            ss << "\t\t\t<hemisphere role='first'"
               << " lever='" << maths.lever << "'"
               << " theta0='" << simox::math::rad_to_deg(maths.theta0Rad) << "'"
               << " />" << std::endl;
        }
        else
        {
            ss << "\t\t\t<hemisphere role='second' />" << std::endl;
        }

        ss << "\t\t\t<MaxAcceleration value='" << maxAcceleration << "'/>" << std::endl;
        ss << "\t\t\t<MaxVelocity value='" << maxVelocity << "'/>" << std::endl;
        ss << "\t\t\t<MaxTorque value='" << maxTorque << "'/>" << std::endl;

        for (auto propIt = propagatedJointValues.begin(); propIt != propagatedJointValues.end();
             ++propIt)
        {
            ss << "\t\t\t<PropagateJointValue name='" << propIt->first << "' factor='"
               << propIt->second << "'/>" << std::endl;
        }

        ss << "\t\t</Joint>" << std::endl;
        return ss.str();
    }

    hemisphere::CachedMaths&
    RobotNodeHemisphere::SecondData::maths()
    {
        return firstNode->firstData->maths;
    }

    const hemisphere::CachedMaths&
    RobotNodeHemisphere::SecondData::maths() const
    {
        return firstNode->firstData->maths;
    }

    hemisphere::Maths::Jacobian
    RobotNodeHemisphere::SecondData::getJacobian()
    {
        VR_ASSERT(firstNode);
        VR_ASSERT(secondNode);

        hemisphere::CachedMaths& maths = this->maths();

        Eigen::Vector2f actuators(firstNode->getJointValue(), secondNode->getJointValue());
        maths.update(actuators);

        hemisphere::Maths::Jacobian jacobian = maths.maths.getJacobian();
        return jacobian;
    }

} // namespace VirtualRobot
