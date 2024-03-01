
#pragma once

#include <vector>
#include <Eigen/Geometry>
#include "Primitive.h"
#include "VirtualRobot.h"

namespace VirtualRobot::affordances
{
    struct Affordance
    {
        using Type = std::string;
        Type type;

        // maybe only primitive?
        VisualizationNodePtr representation;
        std::vector<Primitive::PrimitivePtr> primitiveRepresentation;
    };

    struct Location
    {
        struct FramedPose
        {
            std::string frame;
            Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
        };

        FramedPose pose;

        std::vector<Affordance> affordances;
    };

} // namespace VirtualRobot::affordances
