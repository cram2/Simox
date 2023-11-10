/**
* @package    VirtualRobot
* @author     Fabian Reister
* @copyright  2023 Fabian Reister
*/

#include "RobotNodeFourBarFactory.h"

#include "../CollisionDetection/CollisionModel.h"
#include "RobotNode.h"
#include "RobotNodeFourBar.h"

namespace VirtualRobot
{

    RobotNodeFourBarFactory::RobotNodeFourBarFactory() = default;


    RobotNodeFourBarFactory::~RobotNodeFourBarFactory() = default;

    RobotNodePtr
    RobotNodeFourBarFactory::createRobotNode(RobotPtr robot,
                                             const std::string& nodeName,
                                             VisualizationNodePtr visualizationModel,
                                             CollisionModelPtr collisionModel,
                                             float limitLow,
                                             float limitHigh,
                                             float jointValueOffset,
                                             const Eigen::Matrix4f& preJointTransform,
                                             const Eigen::Vector3f& axis,
                                             const Eigen::Vector3f& /*translationDirection*/,
                                             const SceneObject::Physics& physics,
                                             RobotNode::RobotNodeType rntype) const
    {
        return std::make_shared<RobotNodeFourBar>(
            robot,
            nodeName,
            limitLow,
            limitHigh,
            preJointTransform,
            axis,
            visualizationModel,
            collisionModel,
            jointValueOffset,
            physics,
            (collisionModel ? collisionModel->getCollisionChecker() : CollisionCheckerPtr()),
            rntype);
    }

    RobotNodePtr
    RobotNodeFourBarFactory::createRobotNodeDH(RobotPtr robot,
                                               const std::string& nodeName,
                                               VisualizationNodePtr visualizationModel,
                                               CollisionModelPtr collisionModel,
                                               float limitLow,
                                               float limitHigh,
                                               float jointValueOffset,
                                               const DHParameter& dhParameters,
                                               const SceneObject::Physics& physics,
                                               RobotNode::RobotNodeType rntype) const
    {
        return std::make_shared<RobotNodeFourBar>(robot,
                                                  nodeName,
                                                  limitLow,
                                                  limitHigh,
                                                  dhParameters.aMM(),
                                                  dhParameters.dMM(),
                                                  dhParameters.alphaRadian(),
                                                  dhParameters.thetaRadian(),
                                                  visualizationModel,
                                                  collisionModel,
                                                  jointValueOffset,
                                                  physics,
                                                  CollisionCheckerPtr(),
                                                  rntype);
    }

    RobotNodeFactory::SubClassRegistry
        RobotNodeFourBarFactory::registry(RobotNodeFourBarFactory::getName(),
                                          &RobotNodeFourBarFactory::createInstance);

    std::string
    RobotNodeFourBarFactory::getName()
    {
        return "four_bar";
    }

    std::shared_ptr<RobotNodeFactory>
    RobotNodeFourBarFactory::createInstance(void*)
    {
        return std::make_shared<RobotNodeFourBarFactory>();
    }

} // namespace VirtualRobot
