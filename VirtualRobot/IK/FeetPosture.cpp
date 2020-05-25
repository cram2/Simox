#include "CollisionDetection/CollisionChecker.h"
#include "FeetPosture.h"
#include <VirtualRobot/Robot.h>

using namespace std;

namespace VirtualRobot
{


    FeetPosture::FeetPosture(RobotNodeSetPtr leftLeg,
                             RobotNodeSetPtr rightLeg,
                             Eigen::Matrix4f& transformationLeftToRightFoot,
                             RobotNodePtr baseNode,
                             RobotNodePtr leftTCP /*= RobotNodePtr()*/,
                             RobotNodePtr rightTCP /*= RobotNodePtr()*/,
                             RobotNodeSetPtr rnsLeft2RightFoot)
        : leftLeg(leftLeg), rightLeg(rightLeg), left2Right(rnsLeft2RightFoot), transformationLeftToRightFoot(transformationLeftToRightFoot), baseNode(baseNode)
    {
        VR_ASSERT(this->leftLeg);
        VR_ASSERT(this->rightLeg);
        VR_ASSERT(this->leftLeg->getRobot() == this->rightLeg->getRobot());
        leftLegCol = leftLeg;
        rightLegCol = rightLeg;
        this->leftTCP = leftTCP;

        if (!leftTCP)
        {
            this->leftTCP = leftLeg->getTCP();
        }

        this->rightTCP = rightTCP;

        if (!rightTCP)
        {
            this->rightTCP = rightLeg->getTCP();
        }

        VR_ASSERT(this->leftTCP);
        VR_ASSERT(this->rightTCP);
        VR_ASSERT(baseNode);
    }

    FeetPosture::~FeetPosture()
    = default;

    RobotNodeSetPtr FeetPosture::getLeftLeg()
    {
        return leftLeg;
    }

    RobotNodeSetPtr FeetPosture::getRightLeg()
    {
        return rightLeg;
    }

    RobotNodePtr FeetPosture::getLeftTCP()
    {
        return leftTCP;
    }

    RobotNodePtr FeetPosture::getRightTCP()
    {
        return rightTCP;
    }

    RobotNodePtr FeetPosture::getBaseNode()
    {
        return baseNode;
    }

    Eigen::Matrix4f FeetPosture::getTransformationLeftToRightFoot()
    {
        return transformationLeftToRightFoot;
    }

    RobotPtr FeetPosture::getRobot()
    {
        return leftLeg->getRobot();
    }

    void FeetPosture::print()
    {
        std::cout << "FeetPosture:" << std::endl;
        std::cout << "* Left leg:" << leftLeg->getName() << std::endl;
        std::cout << "* Left leg collision model:" << leftLegCol->getName() << std::endl;
        std::cout << "* Left Foot/TCP:" << leftTCP->getName() << std::endl;
        std::cout << "* Right leg:" << rightLeg->getName() << std::endl;
        std::cout << "* Right leg collision model:" << rightLegCol->getName() << std::endl;
        std::cout << "* Right Foot/TCP:" << rightTCP->getName() << std::endl;
        std::cout << "* Base Node: " << baseNode->getName() << std::endl;

        if (left2Right)
        {
            std::cout << "* RNS Left->Right foot: " << left2Right->getName() << std::endl;
        }
        else
        {
            std::cout << "* RNS Left->Right foot: <not set>" << std::endl;
        }
    }

    void FeetPosture::setCollisionCheck(RobotNodeSetPtr leftColModel, RobotNodeSetPtr rightColModel)
    {
        VR_ASSERT(leftLegCol && rightLegCol);
        leftLegCol = leftColModel;
        rightLegCol = rightColModel;
        VR_ASSERT(leftLegCol->getCollisionChecker() == rightLegCol->getCollisionChecker());
    }

    bool FeetPosture::icCurrentLegConfigCollisionFree()
    {
        VR_ASSERT(leftLegCol && rightLegCol);
        return !leftLegCol->getCollisionChecker()->checkCollision(leftLegCol, rightLegCol);
    }

    RobotNodeSetPtr FeetPosture::getLeftLegCol()
    {
        return leftLegCol;
    }

    RobotNodeSetPtr FeetPosture::getRightLegCol()
    {
        return rightLegCol;
    }

    RobotNodeSetPtr FeetPosture::getRNSLeft2RightFoot()
    {
        return left2Right;
    }

    void FeetPosture::setRNSLeft2RightFoot(RobotNodeSetPtr rns)
    {
        left2Right = rns;
    }

} // namespace VirtualRobot
