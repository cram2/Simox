#include "RobotNodeFourBar.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Transform.h>

#include <SimoxUtility/math/pose/pose.h>
#include <SimoxUtility/meta/enum/EnumNames.hpp>

#include "Nodes/FourBar/Joint.h"
#include "Nodes/Sensor.h"
#include "Robot.h"
#include "VirtualRobot.h"
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
        // std::cout << "RobotNodeFourBar: setting joint value no update " << q << std::endl;

        if (active)
        {
            const float theta = jointValueOffset + q;

            // update the passive joint
            const float psi = active->math.joint.psi(theta);
            active->passive->setJointValueNoUpdate(psi);
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
        // std::cout << "RobotNodeFourBar: setting joint value " << getName() << " " << q << std::endl;

        // std::cout << "RobotNodeFourBar: active? " << active.has_value() << std::endl;

        // update this node (without the global / internal pose!)
        {
            RobotPtr r = getRobot();
            VR_ASSERT(r);
            WriteLockPtr lock = r->getWriteLock();
            setJointValueNoUpdate(q);
        }

        if (active)
        {
            // std::cout << "RobotNodeFourBar: triggering update of passive joint " << std::endl;

            // update all nodes including this one
            active->passive->updatePose(true);
        }
    }


    RobotNodeFourBar::~RobotNodeFourBar() = default;


    void
    RobotNodeFourBar::setXmlInfo(const XmlInfo& info)
    {
        this->xmlInfo = info;
        
        switch (info.role)
        {
            case Role::PASSIVE:
                // std::cout << "Role: passive" << std::endl;
                first.emplace(First{});
                // first->math.joint.setConstants(0, info.theta0);
                break;

            case Role::ACTIVE:
                // std::cout << "Role: active" << std::endl;
                active.emplace(
                    Second{.passive = nullptr,
                           .math = JointMath{.joint = four_bar::Joint{info.dimensions.value()}}});
                break;
        }
    }


    bool
    RobotNodeFourBar::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        VR_ASSERT_MESSAGE(first.has_value() xor active.has_value(),
                          std::stringstream() << first.has_value() << " / " << active.has_value());

        // The active node needs to store a reference to the first node.
        // Whenever the joint value has changed, the passive joint will be updated.
        if (active)
        {
            // std::cout << "Initializing active four bar joint" << std::endl;

            VR_ASSERT_MESSAGE(not active->passive, "Second must not be initialized yet.");

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

                if (firstNode == nullptr)
                {
                    currentParent = currentParent->getParent();
                    // std::cout << "Parent does not match (yet).";
                    continue;
                }

                // std::cout << "Parent matches.";

                // Save pointer to firstNode
                active->passive = firstNode;

                const float theta = jointValueOffset + getJointValue();

                // initialize the passive node
                const float psi = active->math.joint.psi(theta);

                // VR_INFO << "psi " << psi;
                active->passive->setJointValueNoUpdate(psi);

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
    RobotNodeFourBar::JointMath::update(const float /*theta*/)
    {
        // if (actuators != this->actuators)
        // {
        // joint.computeFk(theta);
        // }
    }


    void
    RobotNodeFourBar::updateTransformationMatrices(const Eigen::Matrix4f& parentPose)
    {
        VR_ASSERT_MESSAGE(first.has_value() xor active.has_value(),
                          std::stringstream() << first.has_value() << " / " << active.has_value());

        // std::cout << "Updating RobotNodeFourBar::updateTransformationMatrices" << std::endl;

        Eigen::Isometry3f tmp = Eigen::Isometry3f::Identity();


        if (active)
        {
            // std::cout << "active: joint value " << jV << std::endl;
            const float theta = this->getJointValue() + jointValueOffset;

            active->math.update(theta);
            tmp = active->math.joint.computeFk(theta).matrix().cast<float>();
        }
        else // passive
        {
            // std::cout << "passive: joint value " << jV << std::endl;
            const float psi = this->getJointValue();

            tmp.linear() = Eigen::AngleAxisf(psi, -Eigen::Vector3f::UnitZ())
                               .toRotationMatrix();
        }


        // std::cout << "local transformation: " << getName() << tmp.matrix() << std::endl;
        globalPose = parentPose * localTransformation * tmp.matrix();


        // if (first)
        // {
        //     globalPose = parentPose * localTransformation;
        // }
        // else if (active)
        // {
        //     VR_ASSERT_MESSAGE(active->first, "First node must be known to active node.");

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
        //             << __FUNCTION__ << "() of active actuator with "
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
        VR_ASSERT_MESSAGE(first.has_value() xor active.has_value(),
                          std::stringstream() << first.has_value() << " / " << active.has_value());

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
            std::cout << "* four_bar joint active node";
            // std::cout << "* Transform: \n"
            //           << active->math.joint.getEndEffectorTransform() << std::endl;
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

        if (xmlInfo)
        {
            result->setXmlInfo(xmlInfo.value());
        }

        return result;
    }


    bool
    RobotNodeFourBar::isFourBarJoint() const noexcept
    {
        return true;
    }

    Eigen::Vector3f
    RobotNodeFourBar::getJointRotationAxis(const SceneObjectPtr& coordSystem) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        // FIXME(fabian.reister): improve this code. This is just copied from RobotNodeRevolutes

        const Eigen::Vector3f jointRotationAxis = Eigen::Vector3f::UnitZ();

        Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
        result4f.segment(0, 3) = jointRotationAxis;
        result4f = globalPose * result4f;

        if (coordSystem)
        {
            //res = coordSystem->toLocalCoordinateSystem(res);
            result4f = Eigen::Isometry3f{coordSystem->getGlobalPose()}.inverse() * result4f;
        }

        return result4f.block(0, 0, 3, 1);
    }

    Eigen::Matrix4f
    RobotNodeFourBar::baseFrame(const SceneObjectPtr& coordSystem) const
    {
        VR_ASSERT(active.has_value()); // only defined for active joint

        const auto global_T_base_frame = active->passive->getParent()->getGlobalPose();

        if (coordSystem)
        {
            const Eigen::Isometry3f global_T_ref(coordSystem->getGlobalPose());
            const auto ref_T_global = global_T_ref.inverse();

            return ref_T_global * global_T_base_frame;
        }

        return global_T_base_frame;
    }

    four_bar::Joint::Jacobian
    RobotNodeFourBar::getJacobian(const Eigen::Vector3f& global_P_eef) const
    {
        const auto throwIfFalse = [](const bool exprResult){
            if(not exprResult){throw VirtualRobotException("boom");}
        };

        throwIfFalse(active.has_value());
        throwIfFalse(active->passive != nullptr);
        throwIfFalse(active->passive->getParent() != nullptr);

        const Eigen::Isometry3f global_T_base(active->passive->getParent()->getGlobalPose());
        const auto base_P_eef = global_T_base.inverse() * global_P_eef;

        const Eigen::Vector3d base_P_eef_d = base_P_eef.cast<double>();

        const float theta = this->getJointValue() + jointValueOffset;

        const four_bar::Joint::Jacobian J = active->math.joint.getJacobian(theta, base_P_eef_d);

        // position part 
        // J.head<2>() *= 1000; // [mm] to [m]
        return J;
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
        VR_ASSERT_MESSAGE(first.has_value() xor active.has_value(),
                          std::stringstream() << first.has_value() << " / " << active.has_value());

        if (first)
        {
            // TODO
            return "";
        }
        else
        {
            // JointMath& math = active->math;

            // FIXME implement

            std::stringstream ss;
            ss << "\t\t<Joint type='four_bar'>" << std::endl;
            // ss << "\t\t\t<four_bar theta0='" << math.joint.theta0 << "' />" << std::endl;
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

    bool
    RobotNodeFourBar::isActive() const
    {
        return active.has_value();
    }

    RobotNodeFourBar::Second& 
    RobotNodeFourBar::getActiveData()
    {
        VR_ASSERT(active);
        return active.value();
    }

    const RobotNodeFourBar::Second& 
    RobotNodeFourBar::getActiveData() const
    {
        VR_ASSERT(active);
        return active.value();
    }

    const std::optional<RobotNodeFourBar::XmlInfo>& 
    RobotNodeFourBar::getXmlInfo() const
    {
        return xmlInfo;
    }


} // namespace VirtualRobot
