#pragma once


#include <VirtualRobot/VirtualRobot.h>

#include "inventor.h"

namespace Collada
{


    struct ColladaSimoxRobotNode : InventorRobotNode
    {
        ColladaSimoxRobotNode(VirtualRobot::RobotPtr simoxRobot, float scaleFactor);

        ~ColladaSimoxRobotNode();
        void initialize() override;
        VirtualRobot::RobotPtr simoxRobot;
        VirtualRobot::RobotNodePtr simoxRobotNode;
        float scaleFactor;
    };

    class ColladaSimoxRobot : public InventorRobot
    {
    private:
        VirtualRobot::RobotPtr simoxRobot;
        float scaleFactor;

    public:
        ColladaSimoxRobot(float scaleFactor);

        ColladaRobotNodePtr
        robotNodeFactory() override
        {
            return ColladaRobotNodePtr(new ColladaSimoxRobotNode(simoxRobot, scaleFactor));
        }

        void initialize();

        VirtualRobot::RobotPtr
        getSimoxRobot()
        {
            return simoxRobot;
        }

        ~ColladaSimoxRobot();
    };


} // namespace Collada
