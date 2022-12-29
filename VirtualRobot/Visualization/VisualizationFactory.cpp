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


} // namespace VirtualRobot
