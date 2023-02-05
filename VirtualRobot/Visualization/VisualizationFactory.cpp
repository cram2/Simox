/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2022 Rainer Kartmann
*/

#include "VisualizationFactory.h"

#include <VirtualRobot/Visualization/VisualizationNode.h>


namespace VirtualRobot
{

    void
    VisualizationFactory::applyDisplacement(VisualizationNodePtr visu,
                                            const Eigen::Matrix4f& displacement)
    {
        if (visu)
        {
            visu->setLocalPose(displacement);
        }
    }


    VisualizationNodePtr
    VisualizationFactory::createBox(float /*width*/,
                                    float /*height*/,
                                    float /*depth*/,
                                    const Color& /*color*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createLine(const Eigen::Vector3f& /*from*/,
                                     const Eigen::Vector3f& /*to*/,
                                     float /*width*/,
                                     const Color& /*color*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createSphere(float /*radius*/, const Color& /*color*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createCylinder(float /*radius*/, float /*height*/, const Color& /*color*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createCircle(float /*radius*/,
                                       float /*circleCompletion*/,
                                       float /*width*/,
                                       const Color& /*color*/,
                                       size_t /*numberOfCircleParts*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createVertexVisualization(const Eigen::Vector3f& /*position*/,
                                                    float /*radius*/,
                                                    const Color& /*color*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createPlane(const Eigen::Vector3f& /*position*/,
                                      const Eigen::Vector3f& /*normal*/,
                                      float /*extend*/,
                                      const Color& /*color*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createCircleArrow(float /*radius*/,
                                            float /*tubeRadius*/,
                                            float /*completion*/,
                                            const Color& /*color*/,
                                            int /*sides*/,
                                            int /*rings*/)
    {
        return nullptr;
    }


    VisualizationNodePtr
    VisualizationFactory::createTorus(float /*radius*/,
                                      float /*tubeRadius*/,
                                      float /*completion*/,
                                      const Color& /*color*/,
                                      int /*sides*/,
                                      int /*rings*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createTrajectory(TrajectoryPtr /*t*/,
                                           const Color& /*colorNode*/,
                                           const Color& /*colorLine*/,
                                           float /*nodeSize*/,
                                           float /*lineSize*/)
    {
        return nullptr;
    }
    VisualizationNodePtr
    VisualizationFactory::createText(const std::string& /*text*/,
                                     bool /*billboard*/,
                                     float /*scaling*/,
                                     const Color& /*c*/,
                                     float /*offsetX*/,
                                     float /*offsetY*/,
                                     float /*offsetZ*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createTriMeshModelVisualization(const TriMeshModelPtr& /*model*/,
                                                          const Eigen::Matrix4f& /*pose*/,
                                                          float /*scaleX*/,
                                                          float /*scaleY*/,
                                                          float /*scaleZ*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createTriMeshModelVisualization(const TriMeshModelPtr& /*model*/,
                                                          bool /*showNormals*/,
                                                          const Eigen::Matrix4f& /*pose*/,
                                                          bool /*showLines*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createBoundingBox(const BoundingBox& /*bbox*/, bool /*wireFrame*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createCoordSystem(float /*scaling*/,
                                            std::string* /*text*/,
                                            float /*axisLength*/,
                                            float /*axisSize*/,
                                            int /*nrOfBlocks*/)
    {
        return nullptr;
    }

    VisualizationNodePtr
    VisualizationFactory::createArrow(const Eigen::Vector3f&  /*n*/,
                                      float  /*length*/,
                                      float  /*width*/,
                                      const Color&  /*color*/)
    {
        return nullptr;
    }
} // namespace VirtualRobot
