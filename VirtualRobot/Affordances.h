
#pragma once

#include "Primitive.h"
#include "VirtualRobot.h"

#include <Eigen/Geometry>

#include <vector>

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
