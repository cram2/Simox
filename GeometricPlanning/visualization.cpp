#include "visualization.h"

#include <cmath>

#include <Eigen/Geometry>

#include <GeometricPlanning/ParametricPath.h>
#include <GeometricPlanning/assert/assert.h>
#include <GeometricPlanning/path_primitives/CircleSegment.h>
#include <GeometricPlanning/path_primitives/Line.h>
#include <SimoxUtility/color/Color.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>

#include <Inventor/SbMatrix.h>
#include <Inventor/SbRotation.h>
#include <Inventor/SbVec3f.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>

namespace simox::geometric_planning
{
    constexpr float defaultWidth = 5.f;
    constexpr std::size_t numberOfCircleParts = 20;
    inline const simox::Color DefaultColor = simox::Color::blue();

    namespace detail
    {
        SoNode*
        visualizePathLine(const geometric_planning::ParametricPath& path)
        {
            const auto startPos = path.getPosition(path.parameterRange().min);
            const auto endPos = path.getPosition(path.parameterRange().max);

            return VirtualRobot::CoinVisualizationFactory::createCoinLine(
                startPos, endPos, defaultWidth, DefaultColor);
        }

        SoNode*
        visualizeCircleSegment(const geometric_planning::ParametricPath& path)
        {
            const auto* circleSegment =
                dynamic_cast<geometric_planning::CircleSegment*>(path.path.get());
            REQUIRE(circleSegment != nullptr);

            const float circleCompletion =
                (circleSegment->parameterRange().max - circleSegment->parameterRange().min) /
                (2 * M_PIf32);

            // circle centered at origin
            auto* circleNode = VirtualRobot::CoinVisualizationFactory::createCoinPartCircle(
                circleSegment->getRadius(),
                circleCompletion,
                defaultWidth,
                DefaultColor,
                numberOfCircleParts);


            const Pose global_T_path_origin(path.frame->getGlobalPose());

            const SbVec3f translation(global_T_path_origin.translation().x(),
                                      global_T_path_origin.translation().y(),
                                      global_T_path_origin.translation().z());

            const SbVec3f scale(1., 1., 1.);

            const Eigen::AngleAxisf rot(global_T_path_origin.linear());
            const SbRotation rotation(SbVec3f(rot.axis().x(), rot.axis().y(), rot.axis().z()),
                                      rot.angle());

            SbMatrix transf;
            transf.setTransform(translation, rotation, scale);

            auto* t = new SoMatrixTransform();
            t->matrix.setValue(transf);

            auto* separator = new SoSeparator();
            separator->addChild(t);
            separator->addChild(circleNode);
            return separator;
        }
    } // namespace detail

    SoNode*
    visualize(const geometric_planning::ParametricPath& path)
    {
        if (dynamic_cast<geometric_planning::Line*>(path.path.get()) != nullptr)
        {
            return detail::visualizePathLine(path);
        }

        if (dynamic_cast<geometric_planning::CircleSegment*>(path.path.get()) != nullptr)
        {
            return detail::visualizeCircleSegment(path);
        }

        REQUIRE_MESSAGE(false, "Unknown parametric path");
        return nullptr;
    }

} // namespace simox::geometric_planning
