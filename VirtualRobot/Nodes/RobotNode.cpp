#include "RobotNode.h"

#include <cmath>
#include <filesystem>
#include <iomanip>

#include <eigen3/Eigen/Core>

#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Nodes/Sensor.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Visualization/Visualization.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/VisualizationNode.h>
#include <VirtualRobot/XML/BaseIO.h>
#include <VirtualRobot/math/Helpers.h>

#include "Logging.h"

namespace VirtualRobot
{

    RobotNode::RobotNode(RobotWeakPtr rob,
                         const std::string& name,
                         float jointLimitLo,
                         float jointLimitHi,
                         VisualizationNodePtr visualization,
                         CollisionModelPtr collisionModel,
                         float jointValueOffset,
                         const SceneObject::Physics& physics,
                         CollisionCheckerPtr colChecker,
                         RobotNodeType type) :
        GraspableSensorizedObject(name, visualization, collisionModel, physics, colChecker)
    {
        nodeType = type;
        maxVelocity = -1.0f;
        maxAcceleration = -1.0f;
        maxTorque = -1.0f;
        robot = rob;
        this->jointLimitLo = jointLimitLo;
        this->jointLimitHi = jointLimitHi;
        this->jointValueOffset = jointValueOffset;
        localTransformation = Eigen::Matrix4f::Identity();
        //postJointTransformation = Eigen::Matrix4f::Identity();
        optionalDHParameter.isSet = false;
        //globalPosePostJoint = Eigen::Matrix4f::Identity();
        jointValue = 0.0f;
        limitless = false;
        allowJointLimitAvoidance = true;
    }

    RobotNode::~RobotNode()
    {
        // not needed here
        // when robot is destroyed all references to this RobotNode are also destroyed
        //RobotPtr rob = robot.lock();
        //if (rob)
        //  rob->deregisterRobotNode(static_pointer_cast<RobotNodePtr>(shared_from_this()));
    }

    bool
    RobotNode::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        RobotPtr rob = robot.lock();
        THROW_VR_EXCEPTION_IF(!rob, "Could not init RobotNode without robot");

        // robot
        if (!rob->hasRobotNode(std::static_pointer_cast<RobotNode>(shared_from_this())))
        {
            rob->registerRobotNode(std::static_pointer_cast<RobotNode>(shared_from_this()));
        }

        // update visualization of coordinate systems
        if (visualizationModel && visualizationModel->hasAttachedVisualization("CoordinateSystem"))
        {
            VisualizationNodePtr v =
                visualizationModel->getAttachedVisualization("CoordinateSystem");
            // not needed any more!
            // this is a little hack: The globalPose is used to set the "local" position of the attached Visualization:
            // Since the attached visualizations are already positioned at the global pose of the visualizationModel,
            // we just need the local postJointTransform
            //v->setGlobalPose(postJointTransformation);
        }

        checkValidRobotNodeType();

        return GraspableSensorizedObject::initialize(parent, children);
    }

    void
    RobotNode::checkValidRobotNodeType()
    {
        switch (nodeType)
        {
            case Generic:
                return;
                break;

            case Joint:
                THROW_VR_EXCEPTION_IF(visualizationModel,
                                      "No visualization models allowed in JointNodes");
                THROW_VR_EXCEPTION_IF(collisionModel, "No collision models allowed in JointNodes");
                //THROW_VR_EXCEPTION_IF(postJointTransformation != Eigen::Matrix4f::Identity() , "No postJoint transformations allowed in JointNodes");
                break;

            case Body:
                //THROW_VR_EXCEPTION_IF(postJointTransformation != Eigen::Matrix4f::Identity() , "No transformations allowed in BodyNodes");
                THROW_VR_EXCEPTION_IF(localTransformation != Eigen::Matrix4f::Identity(),
                                      "No transformations allowed in BodyNodes");

                break;

            case Transform:
                THROW_VR_EXCEPTION_IF(visualizationModel,
                                      "No visualization models allowed in TransformationNodes");
                THROW_VR_EXCEPTION_IF(collisionModel,
                                      "No collision models allowed in TransformationNodes");
                break;

            default:
                VR_ERROR << "RobotNodeType nyi..." << std::endl;
        }
    }

    bool
    RobotNode::getEnforceJointLimits() const noexcept
    {
        return enforceJointLimits;
    }

    void
    RobotNode::setEnforceJointLimits(bool value)
    {
        enforceJointLimits = value;
    }

    bool
    RobotNode::getAllowJointLimitAvoidance() const noexcept
    {
        return allowJointLimitAvoidance;
    }

    void
    RobotNode::setAllowJointLimitAvoidance(bool value)
    {
        allowJointLimitAvoidance = value;
    }

    void
    RobotNode::removeAllSensors(bool recursive)
    {
        for (auto it = children.begin(); it != children.end();)
        {
            if (auto node = std::dynamic_pointer_cast<RobotNode>(*it))
            {
                if (recursive)
                    node->removeAllSensors(recursive);
            }
            else if (auto node = std::dynamic_pointer_cast<Sensor>(*it))
            {
                //(*it)->detachedFromParent();
                it = children.erase(it);
                continue;
            }
            it++;
        }
        sensors.clear();
    }

    RobotPtr
    RobotNode::getRobot() const
    {
        RobotPtr result(robot);
        return result;
    }

    void
    RobotNode::setJointValue(float q)
    {
        RobotPtr r = getRobot();
        VR_ASSERT(r);
        WriteLockPtr lock = r->getWriteLock();
        setJointValueNoUpdate(q);
        updatePose();
    }

    void
    RobotNode::setJointValueNotInitialized(float q)
    {
        VR_ASSERT_MESSAGE((!std::isnan(q) && !std::isinf(q)), "Not a valid number...");

        if (limitless)
        {
            while (q > jointLimitHi)
            {
                q -= 2.0f * M_PI;
            }
            while (q < jointLimitLo)
            {
                q += 2.0f * M_PI;
            }
        }
        else
        {
            if (q < jointLimitLo)
            {
                q = jointLimitLo;
            }

            if (q > jointLimitHi)
            {
                q = jointLimitHi;
            }
        }

        jointValue = q;
    }

    void
    RobotNode::setJointValueNoUpdate(float q)
    {
        // VR_ASSERT_MESSAGE(initialized, "Not initialized"); // Throws in FourBarJoint::initialize
        VR_ASSERT_MESSAGE((!std::isnan(q) && !std::isinf(q)), "Not a valid number...");

        if (limitless and enforceJointLimits)
        {
            // limitless joint: map q to allowed interval
            if (q > jointLimitHi)
            {
                q = fmod(q, 2.0f * M_PI);
            }
            while (q > jointLimitHi)
            {
                q -= 2.0f * M_PI;
            }
            if (q < jointLimitLo)
            {
                q = -fmod(fabs(q), 2.0f * M_PI);
            }
            while (q < jointLimitLo)
            {
                q += 2.0f * M_PI;
            }
        }
        else
        {
            if (enforceJointLimits) // non-limitless joint: clamp to borders
            {
                if (q < jointLimitLo)
                {
                    q = jointLimitLo;
                }

                if (q > jointLimitHi)
                {
                    q = jointLimitHi;
                }
            }
        }

        jointValue = q;
    }

    void
    RobotNode::updateTransformationMatrices()
    {
        if (this->getParent())
        {
            updateTransformationMatrices(this->getParent()->getGlobalPose());
        }
        else
        {
            // check for root
            RobotPtr r = getRobot();

            if (r && r->getRootNode() == shared_from_this())
            {
                updateTransformationMatrices(r->getGlobalPose());
            }
            else
            {
                updateTransformationMatrices(Eigen::Matrix4f::Identity());
            }
        }
    }

    void
    RobotNode::setLocalTransformation(Eigen::Matrix4f& newLocalTransformation)
    {
        this->localTransformation = newLocalTransformation;
    }

    void
    RobotNode::updateTransformationMatrices(const Eigen::Matrix4f& parentPose)
    {
        this->globalPose.noalias() = parentPose * localTransformation; // getLocalTransformation();
    }

    void
    RobotNode::updatePose(bool updateChildren)
    {
        THROW_VR_EXCEPTION_IF(!initialized, this->getName() + " is not initialized");

        updateTransformationMatrices();

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);

        // apply propagated joint values
        if (propagatedJointValues.size() > 0)
        {
            RobotPtr r = robot.lock();
            std::map<std::string, float>::iterator it = propagatedJointValues.begin();

            while (it != propagatedJointValues.end())
            {
                RobotNodePtr rn = r->getRobotNode(it->first);

                if (!rn)
                {
                    VR_WARNING << "Could not propagate joint value from " << name << " to "
                               << it->first << " because dependent joint does not exist...";
                }
                else
                {
                    rn->setJointValue(jointValue * it->second);
                }

                it++;
            }
        }
    }

    void
    RobotNode::copyPoseFrom(const RobotNodePtr& other)
    {
        jointValue = other->jointValue;
        //the following code was manually inlined from
        //SceneObject::copyPoseFrom(other);
        //the function is not called directly, since this
        //would increase the runtime of the whole function
        //by a factor > 2
        globalPose = other->globalPose;
        if (visualizationModel && updateVisualization)
        {
            visualizationModel->setGlobalParentPose(globalPose);
        }
        if (collisionModel && updateCollisionModel)
        {
            collisionModel->setGlobalParentPose(globalPose);
        }
    }

    void
    RobotNode::copyPoseFrom(const SceneObjectPtr& sceneobj)
    {
        RobotNodePtr other = std::dynamic_pointer_cast<RobotNode>(sceneobj);
        THROW_VR_EXCEPTION_IF(!other, "The given SceneObject is no RobotNode");
        copyPoseFrom(other);
    }

    void
    RobotNode::updatePose(const Eigen::Matrix4f& globalPose, bool updateChildren)
    {
        THROW_VR_EXCEPTION_IF(!initialized, this->getName() + " is not initialized");

        updateTransformationMatrices(globalPose);

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);

        // apply propagated joint values
        if (propagatedJointValues.size() > 0 && getRobot()->getPropagatingJointValuesEnabled())
        {
            RobotPtr r = robot.lock();

            for (auto& [jname, factor] : propagatedJointValues)
            {
                RobotNodePtr rn = r->getRobotNode(jname);

                if (!rn)
                {
                    VR_WARNING << "Could not propagate joint value from " << name << " to " << jname
                               << " because dependent joint does not exist...";
                }
                else
                {
                    rn->setJointValue(jointValue * factor);
                }
            }
        }
    }

    void
    RobotNode::collectAllRobotNodes(std::vector<RobotNodePtr>& storeNodes)
    {
        storeNodes.push_back(std::static_pointer_cast<RobotNode>(shared_from_this()));

        std::vector<SceneObjectPtr> children = this->getChildren();

        for (size_t i = 0; i < children.size(); i++)
        {
            RobotNodePtr n = std::dynamic_pointer_cast<RobotNode>(children[i]);

            if (n)
            {
                n->collectAllRobotNodes(storeNodes);
            }
        }
    }

    float
    RobotNode::getJointValue() const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        return jointValue;
    }

    void
    RobotNode::respectJointLimits(float& jointValue) const
    {
        if (jointValue < jointLimitLo)
        {
            jointValue = jointLimitLo;
        }

        if (jointValue > jointLimitHi)
        {
            jointValue = jointLimitHi;
        }
    }

    bool
    RobotNode::checkJointLimits(float jointValue, bool verbose) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        bool res = true;

        if (jointValue < jointLimitLo)
        {
            res = false;
        }

        if (jointValue > jointLimitHi)
        {
            res = false;
        }

        if (!res && verbose)
        {
            VR_INFO << "Joint: " << getName() << ": joint value (" << jointValue
                    << ") is out of joint boundaries (lo:" << jointLimitLo
                    << ", hi: " << jointLimitHi << ")" << std::endl;
        }

        return res;
    }

    void
    RobotNode::setGlobalPose(const Eigen::Matrix4f& /*pose*/)
    {
        THROW_VR_EXCEPTION("Use setJointValues to control the position of a RobotNode");
    }

    void
    RobotNode::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        if (printDecoration)
        {
            std::cout << "******** RobotNode ********" << std::endl;
        }

        std::cout << "* Name: " << name << std::endl;
        std::cout << "* Parent: ";
        SceneObjectPtr p = this->getParent();

        if (p)
        {
            std::cout << p->getName() << std::endl;
        }
        else
        {
            std::cout << " -- " << std::endl;
        }

        std::cout << "* Children: ";

        if (this->getChildren().size() == 0)
        {
            std::cout << " -- " << std::endl;
        }

        for (unsigned int i = 0; i < this->getChildren().size(); i++)
        {
            std::cout << this->getChildren()[i]->getName() << ", ";
        }

        std::cout << std::endl;

        physics.print();

        std::cout << "* Limits: Lo: " << jointLimitLo << ", Hi: " << jointLimitHi << std::endl;
        std::cout << "* Limitless: " << (limitless ? "true" : "false") << std::endl;
        std::cout << "* Allow joint limit avoidance: "
                  << (allowJointLimitAvoidance ? "true" : "false") << std::endl;
        std::cout << "* max velocity " << maxVelocity << " [m/s]" << std::endl;
        std::cout << "* max acceleration " << maxAcceleration << " [m/s^2]" << std::endl;
        std::cout << "* max torque " << maxTorque << " [Nm]" << std::endl;
        std::cout << "* jointValue: " << this->getJointValue()
                  << ", jointValueOffset: " << jointValueOffset << std::endl;

        if (optionalDHParameter.isSet)
        {
            std::cout << "* DH parameters: ";
            std::cout << " a:" << optionalDHParameter.aMM() << ", d:" << optionalDHParameter.dMM()
                      << ", alpha:" << optionalDHParameter.alphaRadian()
                      << ", theta:" << optionalDHParameter.thetaRadian() << std::endl;
        }
        else
        {
            std::cout << "* DH parameters: not specified." << std::endl;
        }

        std::cout << "* visualization model: " << std::endl;

        if (visualizationModel)
        {
            visualizationModel->print();
        }
        else
        {
            std::cout << "  No visualization model" << std::endl;
        }

        std::cout << "* collision model: " << std::endl;

        if (collisionModel)
        {
            collisionModel->print();
        }
        else
        {
            std::cout << "  No collision model" << std::endl;
        }

        if (initialized)
        {
            std::cout << "* initialized: true" << std::endl;
        }
        else
        {
            std::cout << "* initialized: false" << std::endl;
        }

        {
            // scope1
            std::ostringstream sos;
            sos << std::setiosflags(std::ios::fixed);
            sos << "* localTransformation:\n" << localTransformation << "\n";
            sos << "* globalPose:\n" << getGlobalPose() << "\n";
            std::cout << sos.str();
        } // scope1

        if (printDecoration)
        {
            std::cout << "******** End RobotNode ********" << std::endl;
        }

        if (printChildren)
        {
            std::vector<SceneObjectPtr> children = this->getChildren();

            for (unsigned int i = 0; i < children.size(); i++)
            {
                children[i]->print(true, true);
            }
        }
    }

    RobotNodePtr
    RobotNode::clone(RobotPtr newRobot,
                     bool cloneChildren,
                     RobotNodePtr initializeWithParent,
                     CollisionCheckerPtr colChecker,
                     std::optional<float> scaling,
                     bool preventCloningMeshesIfScalingIs1)
    {
        const float actualScaling = scaling.value_or(this->scaling);
        const float scalingFactor = scaling.has_value() ? scaling.value() / this->scaling : 1.0f;

        ReadLockPtr lock = getRobot()->getReadLock();

        if (!newRobot)
        {
            VR_ERROR << "Attempting to clone RobotNode for invalid robot";
            return RobotNodePtr();
        }

        std::vector<std::string> clonedChildrenNames;

        VisualizationNodePtr clonedVisualizationNode;

        const bool deepMeshClone =
            !preventCloningMeshesIfScalingIs1 || std::abs(scalingFactor - 1) <= 1e-12;
        if (visualizationModel)
        {
            clonedVisualizationNode = visualizationModel->clone(deepMeshClone, scalingFactor);
        }

        CollisionModelPtr clonedCollisionModel;

        if (collisionModel)
        {
            clonedCollisionModel = collisionModel->clone(colChecker, scalingFactor, deepMeshClone);
        }

        RobotNodePtr result = _clone(
            newRobot, clonedVisualizationNode, clonedCollisionModel, colChecker, scalingFactor);

        if (!result)
        {
            VR_ERROR << "Cloning failed.." << std::endl;
            return result;
        }

        if (!visualizationModelXML.empty())
        {
            result->visualizationModelXML = visualizationModelXML;
        }

        if (!collisionModelXML.empty())
        {
            result->collisionModelXML = collisionModelXML;
        }

        if (cloneChildren)
        {
            std::vector<SceneObjectPtr> children = this->getChildren();

            for (size_t i = 0; i < children.size(); i++)
            {
                RobotNodePtr n = std::dynamic_pointer_cast<RobotNode>(children[i]);

                if (n)
                {
                    RobotNodePtr c = n->clone(newRobot,
                                              true,
                                              RobotNodePtr(),
                                              colChecker,
                                              scaling,
                                              preventCloningMeshesIfScalingIs1);

                    if (c)
                    {
                        result->attachChild(c);
                    }
                }
            }
            appendSensorsTo(result);
            appendGraspSetsTo(result);
        }

        result->setMaxVelocity(maxVelocity);
        result->setMaxAcceleration(maxAcceleration);
        result->setMaxTorque(maxTorque);
        result->setLimitless(limitless);
        result->setAllowJointLimitAvoidance(allowJointLimitAvoidance);

        std::map<std::string, float>::iterator it = propagatedJointValues.begin();

        while (it != propagatedJointValues.end())
        {
            result->propagateJointValue(it->first, it->second);
            it++;
        }


        newRobot->registerRobotNode(result);
        result->primitiveApproximation = primitiveApproximation.clone();

        if (initializeWithParent)
        {
            result->initialize(initializeWithParent);
        }
        result->basePath = basePath;
        result->setScaling(actualScaling);

        return result;
    }

    float
    RobotNode::getJointLimitLo()
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        return jointLimitLo;
    }

    float
    RobotNode::getJointLimitHi()
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        return jointLimitHi;
    }

    bool
    RobotNode::isTranslationalJoint() const noexcept
    {
        return false;
    }

    bool
    RobotNode::isRotationalJoint() const noexcept
    {
        return false;
    }

    bool
    RobotNode::isHemisphereJoint() const noexcept
    {
        return false;
    }

    bool
    RobotNode::isFourBarJoint() const noexcept
    {
        return false;
    }

    void
    RobotNode::setLimitless(bool limitless)
    {
        this->limitless = limitless;
    }

    bool
    RobotNode::isLimitless() const noexcept
    {
        return limitless;
    }

    float
    RobotNode::getDelta(float target) const
    {
        float delta = 0.0f;

        /*if (nodeType != Joint)
        {
            return delta;
        }*/

        // we check if the given target value violates our joint limits
        if (!limitless)
        {
            if (target < jointLimitLo || target > jointLimitHi)
            {
                return delta;
            }
        }

        delta = target - jointValue;

        // eventually take the other way around if it is shorter and if this joint is limitless.
        if (limitless && (std::abs(delta) > static_cast<float>(M_PI)))
        {
            delta = (-1) * ((delta > 0) - (delta < 0)) *
                    ((2 * static_cast<float>(M_PI)) - std::abs(delta));
        }

        return delta;
    }

    void
    RobotNode::showCoordinateSystem(bool enable,
                                    float scaling,
                                    std::string* text,
                                    const std::string& visualizationType)
    {
        if (!enable && !visualizationModel)
        {
            return; // nothing to do
        }

        if (!ensureVisualization(visualizationType))
        {
            return;
        }

        std::string coordName = name;

        if (text)
        {
            coordName = *text;
        }

        if (visualizationModel->hasAttachedVisualization("CoordinateSystem"))
        {
            visualizationModel->detachVisualization("CoordinateSystem");
        }

        if (enable)
        {
            VisualizationFactoryPtr visualizationFactory;

            if (visualizationType.empty())
            {
                visualizationFactory = VisualizationFactory::first(nullptr);
            }
            else
            {
                visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
            }

            if (!visualizationFactory)
            {
                VR_WARNING << "No visualization factory for name " << visualizationType
                           << std::endl;
                return;
            }

            // create coord visu
            VisualizationNodePtr visualizationNode =
                visualizationFactory->createCoordSystem(scaling, &coordName);

            // not needed any more
            // this is a little hack: The globalPose is used to set the "local" position of the attached Visualization:
            // Since the attached visualizations are already positioned at the global pose of the visualizationModel,
            // we just need the local postJointTransform
            if (visualizationNode)
            {
                //visualizationNode->setGlobalPose(postJointTransformation);
                visualizationModel->attachVisualization("CoordinateSystem", visualizationNode);
            }
        }
    }

    void
    RobotNode::showStructure(bool enable, const std::string& visualizationType)
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        if (!enable && !visualizationModel)
        {
            return; // nothing to do
        }

        if (!ensureVisualization(visualizationType))
        {
            return;
        }

        std::stringstream ss;
        ss << getName() << "_RobotNodeStructurePre";
        std::string attachName1 = ss.str();
        std::string attachName2("RobotNodeStructureJoint");
        std::string attachName3("RobotNodeStructurePost");
        SceneObjectPtr par = getParent();
        RobotNodePtr parRN = std::dynamic_pointer_cast<RobotNode>(par);

        // need to add "pre" visualization to parent node!
        if (parRN && parRN->getVisualization())
        {
            parRN->getVisualization()->detachVisualization(attachName1);
        }
        else
        {
            visualizationModel->detachVisualization(attachName1);
        }

        visualizationModel->detachVisualization(attachName2);
        visualizationModel->detachVisualization(attachName3);

        if (enable)
        {
            VisualizationFactoryPtr visualizationFactory;

            if (visualizationType.empty())
            {
                visualizationFactory = VisualizationFactory::first(NULL);
            }
            else
            {
                visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
            }

            if (!visualizationFactory)
            {
                VR_WARNING << "No visualization factory for name " << visualizationType
                           << std::endl;
                return;
            }

            // create visu
            if (!localTransformation.isIdentity())
            {
                VisualizationNodePtr visualizationNode1;

                if (parRN && parRN->getVisualization())
                {
                    // add to parent node (pre joint trafo moves with parent!)
                    //visualizationNode1 = visualizationFactory->createLine(parRN->postJointTransformation, parRN->postJointTransformation*localTransformation);
                    const Eigen::Vector3f localTrafoPos =
                        Eigen::Isometry3f{localTransformation}.translation();

                    visualizationNode1 =
                        visualizationFactory->createLine(Eigen::Vector3f::Zero(), localTrafoPos);

                    if (visualizationNode1)
                    {
                        parRN->getVisualization()->attachVisualization(attachName1,
                                                                       visualizationNode1);
                    }
                }
                else
                {
                    const Eigen::Vector3f localTrafoInvPos =
                        Eigen::Isometry3f{localTransformation}.inverse().translation();

                    visualizationNode1 =
                        visualizationFactory->createLine(localTrafoInvPos, Eigen::Vector3f::Zero());

                    if (visualizationNode1)
                    {
                        visualizationModel->attachVisualization(attachName1, visualizationNode1);
                    }
                }
            }
            else if (Eigen::Vector3f parPos = parRN ? parRN->getPositionInFrame(
                                                          getRobot()->getRobotNode(this->getName()))
                                                    : Eigen::Vector3f::Zero();
                     parPos.norm() > 0.01)
            {
                // Some nodes (e.g. the Revolute and FourBar) offset their position by values that are not reflected in the local transformation.
                // Check RobotNodeFourBar::updateTransformationMatrices and RobotNodeRevolute::updateTransformationMatrices.
                // There the global pose is set to parent * local * tmp, where tmp is important but not part of
                // This code catches these cases by checking the actual distance between the current node and its parent.

                VisualizationNodePtr visualizationNode1 =
                    visualizationFactory->createLine(Eigen::Vector3f::Zero(), parPos);

                if (visualizationNode1)
                {
                    visualizationModel->attachVisualization(attachName1, visualizationNode1);
                }
            }

            VisualizationNodePtr visualizationNode2 = visualizationFactory->createSphere(5.0f);

            if (visualizationNode2)
            {
                visualizationModel->attachVisualization(attachName2, visualizationNode2);
            }
        }
    }

    std::vector<RobotNodePtr>
    RobotNode::getAllParents(RobotNodeSetPtr rns)
    {
        std::vector<RobotNodePtr> result;

        std::vector<RobotNodePtr> rn;

        if (rns)
        {
            rn = rns->getAllRobotNodes();
        }
        else
        {
            RobotPtr r = this->getRobot();
            rn = r->getRobotNodes();
        }

        for (unsigned int i = 0; i < rn.size(); i++)
        {
            if (rn[i]->hasChild(std::static_pointer_cast<SceneObject>(shared_from_this()), true))
            {
                result.push_back(rn[i]);
            }
        }

        return result;
    }

    void
    RobotNode::setJointLimits(float lo, float hi)
    {
        jointLimitLo = lo;
        jointLimitHi = hi;
    }

    bool
    RobotNode::isJoint() const noexcept
    {
        return isRotationalJoint() or isTranslationalJoint() or isHemisphereJoint() or
               isFourBarJoint();
    }

    void
    RobotNode::setMaxJerk(float maxTo)
    {
        maxJerk = maxTo;
    }

    void
    RobotNode::setMaxTorque(float maxTo)
    {
        maxTorque = maxTo;
    }

    void
    RobotNode::setMaxAcceleration(float maxAcc)
    {
        maxAcceleration = maxAcc;
    }

    void
    RobotNode::setMaxDeceleration(float maxDec)
    {
        maxDeceleration = maxDec;
    }

    void
    RobotNode::setMaxVelocity(float maxVel)
    {
        maxVelocity = maxVel;
    }

    float
    RobotNode::getMaxVelocity() const noexcept
    {
        return maxVelocity;
    }

    float
    RobotNode::getMaxAcceleration() const noexcept
    {
        return maxAcceleration;
    }

    float
    RobotNode::getMaxTorque() const noexcept
    {
        return maxTorque;
    }

    void
    RobotNode::updateVisualizationPose(const Eigen::Matrix4f& globalPose,
                                       float jointValue,
                                       bool updateChildren)
    {
        updateVisualizationPose(globalPose, updateChildren);
        this->jointValue = jointValue;
    }

    void
    RobotNode::updateVisualizationPose(const Eigen::Matrix4f& globalPose, bool updateChildren)
    {
        // check if we are a root node
        SceneObjectPtr parent = getParent();
        RobotPtr rob = getRobot();

        if (!parent || parent == rob)
        {
            if (rob &&
                rob->getRootNode() == std::static_pointer_cast<RobotNode>(shared_from_this()))
            {
                Eigen::Matrix4f gpPre = globalPose * getLocalTransformation().inverse();
                rob->setGlobalPose(gpPre, false);
            }
            else
            {
                VR_WARNING << "INTERNAL ERROR: getParent==robot but getRoot!=this ?! " << std::endl;
            }
        }

        this->globalPose = globalPose;

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);
    }

    Eigen::Matrix4f
    RobotNode::getGlobalPose() const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        return globalPose;
    }

    Eigen::Matrix4f
    RobotNode::getPoseInRootFrame() const
    {
        RobotPtr r = getRobot();
        ReadLockPtr lock = r->getReadLock();
        return r->getRootNode()->toLocalCoordinateSystem(globalPose);
    }

    Eigen::Matrix4f
    RobotNode::getPoseInFrame(const RobotNodePtr& frame) const
    {
        if (!frame)
        {
            THROW_VR_EXCEPTION("Frame is null");
        }
        ReadLockPtr lock = getRobot()->getReadLock();
        const Eigen::Matrix4f pinroot = getPoseInRootFrame();
        const Eigen::Matrix4f finroot = frame->getPoseInRootFrame();
        return finroot.inverse() * pinroot;
    }

    Eigen::Vector3f
    RobotNode::getPositionInFrame(const RobotNodePtr& frame) const
    {
        return getPoseInFrame(frame).topRightCorner<3, 1>();
    }

    Eigen::Vector3f
    RobotNode::getPositionInRootFrame() const
    {
        RobotPtr r = getRobot();
        ReadLockPtr lock = r->getReadLock();
        return r->getRootNode()->toLocalCoordinateSystemVec(globalPose.block(0, 3, 3, 1));
    }

    Eigen::Matrix3f
    RobotNode::getOrientationInRootFrame() const
    {
        return getPoseInRootFrame().block<3, 3>(0, 0);
    }

    Eigen::Matrix4f
    RobotNode::getPoseInRootFrame(const Eigen::Matrix4f& localPose) const
    {
        return getPoseInRootFrame() * localPose;
    }

    Eigen::Vector3f
    RobotNode::getPositionInRootFrame(const Eigen::Vector3f& localPosition) const
    {
        return ::math::Helpers::TransformPosition(getPoseInRootFrame(), localPosition);
    }

    Eigen::Vector3f
    RobotNode::getDirectionInRootFrame(const Eigen::Vector3f& localPosition) const
    {
        return ::math::Helpers::TransformDirection(getPoseInRootFrame(), localPosition);
    }

    Eigen::Matrix3f
    RobotNode::getOrientationInRootFrame(const Eigen::Matrix3f& localOrientation) const
    {
        return ::math::Helpers::TransformOrientation(getPoseInRootFrame(), localOrientation);
    }

    RobotNode::RobotNodeType
    RobotNode::getType() const noexcept
    {
        return nodeType;
    }

    void
    RobotNode::propagateJointValue(const std::string& jointName, float factor /*= 1.0f*/)
    {
        if (factor == 0.0f)
        {
            propagatedJointValues.erase(jointName);
        }
        else
        {
            propagatedJointValues[jointName] = factor;
        }
    }

    std::string
    RobotNode::toXML(const std::string& basePath,
                     const std::string& modelPathRelative,
                     bool storeSensors,
                     bool storeModelFiles)
    {
        std::stringstream ss;
        ss << "\t<RobotNode name='" << name << "'>" << std::endl;

        if (!localTransformation.isIdentity())
        {
            ss << "\t\t<Transform>" << std::endl;
            ss << BaseIO::toXML(localTransformation, "\t\t\t");
            ss << "\t\t</Transform>" << std::endl;
        }

        ss << _toXML(modelPathRelative);

        if (physics.isSet())
        {
            ss << physics.toXML(2);
        }

        std::filesystem::path pBase(basePath);

        if (visualizationModel && visualizationModel->getTriMeshModel() &&
            visualizationModel->getTriMeshModel()->faces.size() > 0)
        {
            if (storeModelFiles)
            {
                std::string visuFile = getFilenameReplacementVisuModel();

                std::filesystem::path pModel(modelPathRelative);
                std::filesystem::path modelDirComplete = pBase / pModel;
                std::filesystem::path fn(visuFile);
                std::filesystem::path modelFileComplete = modelDirComplete / fn;

                ss << visualizationModel->toXML(pBase.string(), modelFileComplete.string(), 2);
            }
            else
                ss << visualizationModel->toXML(basePath, 2);
        }

        if (collisionModel && collisionModel->getTriMeshModel() &&
            collisionModel->getTriMeshModel()->faces.size() > 0)
        {
            if (storeModelFiles)
            {
                std::string colFile = getFilenameReplacementColModel();
                std::filesystem::path pModel(modelPathRelative);
                std::filesystem::path modelDirComplete = pBase / pModel;
                std::filesystem::path fn(colFile);
                std::filesystem::path modelFileComplete = modelDirComplete / fn;
                ss << collisionModel->toXML(pBase.string(), modelFileComplete.string(), 2);
            }
            else
                ss << collisionModel->toXML(basePath, 2);
        }

        std::vector<SceneObjectPtr> children = this->getChildren();

        for (size_t i = 0; i < children.size(); i++)
        {
            // check if child is a RobotNode
            RobotNodePtr crn = std::dynamic_pointer_cast<RobotNode>(children[i]);

            if (crn)
            {
                ss << "\t\t<Child name='" << children[i]->getName() << "'/>\n";
            }
        }

        ss << getGraspableSensorizedObjectXML(modelPathRelative, storeSensors, 2);

        ss << "\t</RobotNode>\n\n";
        return ss.str();
    }

    SceneObjectPtr
    RobotNode::clone(const std::string& /*name*/,
                     CollisionCheckerPtr /*colChecker*/,
                     float /*scaling*/) const
    {
        THROW_VR_EXCEPTION("Cloning not allowed this way...");
    }

    SceneObject*
    RobotNode::_clone(const std::string& /*name*/,
                      CollisionCheckerPtr /*colChecker*/,
                      float /*scaling*/) const
    {
        THROW_VR_EXCEPTION("Cloning not allowed this way...");
    }
} // namespace VirtualRobot
