#include "RobotNodeFourBar.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>
#include <Eigen/src/Geometry/AngleAxis.h>

#include <SimoxUtility/math/pose/pose.h>
#include <SimoxUtility/meta/enum/EnumNames.hpp>

#include "Nodes/FourBar/Joint.h"
#include "Nodes/Sensor.h"
#include "Robot.h"
#include "VirtualRobotException.h"


namespace VirtualRobot
{
    namespace four_bar
    {
        extern const simox::meta::EnumNames<RobotNodeFourBar::Role> RoleNames = {
            {RobotNodeFourBar::Role::PASSIVE, "passive"},
            {RobotNodeFourBar::Role::ACTIVE, "active"},
        };

    } // namespace four_bar


    VirtualRobot::RobotNodeFourBar::RobotNodeFourBar() = default;

    RobotNodeFourBar::Role
    RobotNodeFourBar::RoleFromString(const std::string& string)
    {
        return four_bar::RoleNames.from_name(string);
    }

    RobotNodeFourBar::RobotNodeFourBar(RobotWeakPtr rob,
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
                                       RobotNodeType type) :
        RobotNode(rob,
                  name,
                  jointLimitLo,
                  jointLimitHi,
                  visualization,
                  collisionModel,
                  jointValueOffset,
                  physics,
                  colChecker,
                  type)
    {
        (void)axis;

        initialized = false;
        optionalDHParameter.isSet = false;
        localTransformation = preJointTransform;
        checkValidRobotNodeType();
    }


    RobotNodeFourBar::RobotNodeFourBar(RobotWeakPtr rob,
                                       const std::string& name,
                                       float jointLimitLo,
                                       float jointLimitHi,
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
                  jointLimitLo,
                  jointLimitHi,
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
        // Eigen::Matrix4f RotTheta = Eigen::Matrix4f::Identity();
        // RotTheta(0, 0) = cos(theta);
        // RotTheta(0, 1) = -sin(theta);
        // RotTheta(1, 0) = sin(theta);
        // RotTheta(1, 1) = cos(theta);
        // Eigen::Matrix4f TransD = Eigen::Matrix4f::Identity();
        // TransD(2, 3) = d;
        // Eigen::Matrix4f TransA = Eigen::Matrix4f::Identity();
        // TransA(0, 3) = a;
        // Eigen::Matrix4f RotAlpha = Eigen::Matrix4f::Identity();
        // RotAlpha(1, 1) = cos(alpha);
        // RotAlpha(1, 2) = -sin(alpha);
        // RotAlpha(2, 1) = sin(alpha);
        // RotAlpha(2, 2) = cos(alpha);

        // localTransformation = RotTheta * TransD * TransA * RotAlpha;
        localTransformation.setIdentity();
        checkValidRobotNodeType();
    }

    void
    RobotNodeFourBar::setJointValueNoUpdate(float q)
    {
        std::cout << "RobotNodeFourBar: setting joint value no update " << q << std::endl;

        if (active)
        {
            // update the passive joint
            const float psi = active->math.joint.psi(q);
            active->passive->setJointValueNoUpdate(-psi); // FIXME make joint axis consistent
            RobotNode::setJointValueNoUpdate(q);
        }
        else
        {
            RobotNode::setJointValueNoUpdate(q);
        }
    }


    void
    RobotNodeFourBar::setJointValue(float q)
    {
        // We must update the preceeding node (the passive node).
        // This usually causes issues as the order to update the kinematic chain is strict.
        std::cout << "RobotNodeFourBar: setting joint value " << getName()  << " " << q << std::endl;

        std::cout << "RobotNodeFourBar: active? " << active.has_value() << std::endl;

        // update this node (without the global / internal pose!)
        {
            RobotPtr r = getRobot();
            VR_ASSERT(r);
            WriteLockPtr lock = r->getWriteLock();
            setJointValueNoUpdate(q);
        }

        if (active)
        {
            std::cout << "RobotNodeFourBar: triggering update of passive joint " << std::endl;

            // update all nodes including this one
            active->passive->updatePose(true);
        }
    }


    RobotNodeFourBar::~RobotNodeFourBar() = default;


    void
    RobotNodeFourBar::setXmlInfo(const XmlInfo& info)
    {
        this->xmlInfo = info;

        VR_ASSERT(second.has_value());
        switch (info.role)
        {
            case Role::PASSIVE:
                std::cout << "Role: passive" << std::endl;
                first.emplace(First{});
                // first->math.joint.setConstants(0, info.theta0);
                break;

            case Role::ACTIVE:
                std::cout << "Role: active" << std::endl;
                active.emplace(
                    Second{.passive = nullptr,
                           .math = JointMath{.joint = four_bar::Joint{jointValueOffset,
                                                                      info.dimensions.value()}}});
                break;
        }
    }


    bool
    RobotNodeFourBar::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        VR_ASSERT_MESSAGE(first.has_value() xor second.has_value(),
                          std::stringstream() << first.has_value() << " / " << second.has_value());

        // The second node needs to store a reference to the first node.
        // Whenever the joint value has changed, the passive joint will be updated.
        if (active)
        {
            std::cout << "Initializing active four bar joint" << std::endl;

            VR_ASSERT_MESSAGE(not second->first, "Second must not be initialized yet.");

            VirtualRobot::SceneObjectPtr currentParent = parent;

            while (currentParent != nullptr)
            {
                auto* firstNode = dynamic_cast<RobotNodeFourBar*>(currentParent.get());

                // TODO traverse all nodes until the passive four bar node is reached.
                // TODO then, keep a list of all nodes that have to be updated if the passive node is updated
                // => all child nodes except this one. It is important to not trigger an update of this node as it would
                // result in infinite recursion.

                // RobotNodeFourBar* secondNode = this;

                // if (not(firstNode and firstNode->first))
                // {
                //     std::stringstream ss;
                //     ss << "The parent of a four_bar joint with role '"
                //        << four_bar::RoleNames.to_name(Role::ACTIVE) << "' "
                //        << "must be a four_bar joint with role '"
                //        << four_bar::RoleNames.to_name(Role::PASSIVE) << "' ";
                //     THROW_VR_EXCEPTION(ss.str());
                // }

                if(firstNode == nullptr)
                {
                    currentParent = currentParent->getParent();
                    std::cout << "Parent does not match (yet).";
                    continue;
                }

                std::cout << "Parent matches.";

                // Save pointer to firstNode
                active->passive = firstNode;

                // Set up robot node parameters.
                {
                    // const four_bar::Joint& joint = active->math.joint;

                    // firstNode->jointLimitLo = joint.limitLo;
                    // secondNode->jointLimitLo = joint.limitLo;

                    // firstNode->jointLimitHi = joint.limitHi;
                    // secondNode->jointLimitHi = joint.limitHi;
                }

                break;
            }
        }

        return RobotNode::initialize(parent, children);
    }


    void
    RobotNodeFourBar::JointMath::update(const float theta)
    {
        // if (actuators != this->actuators)
        // {
        joint.computeFkOfAngle(theta);
        // }
    }


    void
    RobotNodeFourBar::updateTransformationMatrices(const Eigen::Matrix4f& parentPose)
    {
        VR_ASSERT_MESSAGE(first.has_value() xor second.has_value(),
                          std::stringstream() << first.has_value() << " / " << second.has_value());

        std::cout << "Updating RobotNodeFourBar::updateTransformationMatrices" << std::endl;

        Eigen::Isometry3f tmp = Eigen::Isometry3f::Identity();

        const auto jV = this->getJointValue();

        if (active)
        {
            std::cout << "active: joint value " << jV << std::endl;

            active->math.update(jV);
            tmp = active->math.joint.getEndEffectorTransform().cast<float>();
        }
        else // passive
        {
            std::cout << "passive: joint value " << jV << std::endl;

            tmp.linear() =
                Eigen::AngleAxisf(jV + jointValueOffset, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        }


        std::cout << "local transformation: " << getName() << tmp.matrix() << std::endl;
        globalPose = parentPose * localTransformation * tmp.matrix();


        // if (first)
        // {
        //     globalPose = parentPose * localTransformation;
        // }
        // else if (active)
        // {
        //     VR_ASSERT_MESSAGE(second->first, "First node must be known to second node.");

        //     JointMath& math = active->math();
        //     Eigen::Vector2f actuators(active->passive->getJointValue(), this->getJointValue());

        //     math.update(actuators);

        //     Eigen::Vector3d translation = math.joint.getEndEffectorTranslation();
        //     const Eigen::Matrix3d rotation = math.joint.getEndEffectorRotation();
        //     const Eigen::Matrix4d transform = simox::math::pose(translation, rotation);

        //     // Update Second
        //     {
        //         this->globalPose = parentPose * localTransformation * transform.cast<float>();

        //         Eigen::IOFormat iof(5, 0, " ", "\n", "    [", "]");
        //         std::cout
        //             << __FUNCTION__ << "() of second actuator with "
        //             << "\n  lever = " << math.joint.lever << "\n  theta0 = " << math.joint.theta0
        //             << "\n  radius = " << math.joint.radius << "\n  joint value = " << jointValue
        //             << "\n  actuator (angle) = \n"
        //             << actuators.transpose().format(iof) << "\n  actuator (pos) =  \n"
        //             << math.joint.angleToPosition(actuators.cast<double>()).transpose().format(iof)
        //             << "\n  local transform = \n"
        //             << localTransformation.format(iof) << "\n  joint transform = \n"
        //             << transform.format(iof) << std::endl;
        //     }
        // }
    }


    void
    RobotNodeFourBar::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        VR_ASSERT_MESSAGE(first.has_value() xor second.has_value(),
                          std::stringstream() << first.has_value() << " / " << second.has_value());

        if (printDecoration)
        {
            std::cout << "******** RobotNodeFourBar ********" << std::endl;
        }

        RobotNode::print(false, false);

        if (first)
        {
            std::cout << "* four_bar joint first node";
        }
        else if (active)
        {
            std::cout << "* four_bar joint second node";
            std::cout << "* Transform: \n"
                      << active->math.joint.getEndEffectorTransform() << std::endl;
        }

        if (printDecoration)
        {
            std::cout << "******** End RobotNodeFourBar ********" << std::endl;
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
    RobotNodeFourBar::_clone(const RobotPtr newRobot,
                             const VisualizationNodePtr visualizationModel,
                             const CollisionModelPtr collisionModel,
                             CollisionCheckerPtr colChecker,
                             float scaling)
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        Physics physics = this->physics.scale(scaling);

        RobotNodeFourBarPtr result;
        if (optionalDHParameter.isSet)
        {
            result.reset(new RobotNodeFourBar(newRobot,
                                              name,
                                              jointLimitLo,
                                              jointLimitHi,
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
            result.reset(new RobotNodeFourBar(newRobot,
                                              name,
                                              jointLimitLo,
                                              jointLimitHi,
                                              localTransform,
                                              Eigen::Vector3f::Zero(),
                                              visualizationModel,
                                              collisionModel,
                                              jointValueOffset,
                                              physics,
                                              colChecker,
                                              nodeType));
        }

        if(xmlInfo)
        {
            result->setXmlInfo(xmlInfo.value());
        }

        return result;
    }


    bool
    RobotNodeFourBar::isFourBarJoint() const
    {
        return true;
    }


    void
    RobotNodeFourBar::checkValidRobotNodeType()
    {
        RobotNode::checkValidRobotNodeType();
        THROW_VR_EXCEPTION_IF(nodeType == Body || nodeType == Transform,
                              "RobotNodeFourBar must be a JointNode or a GenericNode");
    }


    std::string
    RobotNodeFourBar::_toXML(const std::string& /*modelPath*/)
    {
        VR_ASSERT_MESSAGE(first.has_value() xor second.has_value(),
                          std::stringstream() << first.has_value() << " / " << second.has_value());

        if (first)
        {
            // TODO
            return "";
        }
        else
        {
            JointMath& math = active->math;

            // FIXME implement

            std::stringstream ss;
            ss << "\t\t<Joint type='four_bar'>" << std::endl;
            ss << "\t\t\t<four_bar theta0='" << math.joint.theta0 << "' />" << std::endl;
            ss << "\t\t\t<limits lo='" << jointLimitLo << "' hi='" << jointLimitHi
               << "' units='radian'/>" << std::endl;
            ss << "\t\t\t<MaxAcceleration value='" << maxAcceleration << "'/>" << std::endl;
            ss << "\t\t\t<MaxVelocity value='" << maxVelocity << "'/>" << std::endl;
            ss << "\t\t\t<MaxTorque value='" << maxTorque << "'/>" << std::endl;
            std::map<std::string, float>::iterator propIt = propagatedJointValues.begin();

            while (propIt != propagatedJointValues.end())
            {
                ss << "\t\t\t<PropagateJointValue name='" << propIt->first << "' factor='"
                   << propIt->second << "'/>" << std::endl;
                propIt++;
            }

            ss << "\t\t</Joint>" << std::endl;
            return ss.str();
        }
    }

} // namespace VirtualRobot
