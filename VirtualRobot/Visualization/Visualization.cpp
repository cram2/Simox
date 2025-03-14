/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/


#include "Visualization.h"

#include <iostream>

#include "Logging.h"
#include "VisualizationNode.h"

namespace VirtualRobot
{

    Visualization::Visualization(const VisualizationNodePtr visualizationNode)
    {
        //globalPose.setIdentity();
        this->visualizationNodes.push_back(visualizationNode);
    }

    Visualization::Visualization(const std::vector<VisualizationNodePtr>& visualizationNodes)
    {
        //globalPose.setIdentity();
        this->visualizationNodes = visualizationNodes;
    }

    Visualization::~Visualization() = default;

    VirtualRobot::VisualizationPtr
    Visualization::clone()
    {
        VR_ERROR << " clone nyi\n";
        return VisualizationPtr();
    }

    bool
    Visualization::isVisualizationNodeRegistered(VisualizationNodePtr visualizationNode)
    {
        std::vector<VisualizationNodePtr>::iterator i =
            find(visualizationNodes.begin(), visualizationNodes.end(), visualizationNode);
        return (i != visualizationNodes.end());
    }

    bool
    Visualization::highlight(VisualizationNodePtr /*visualizationNode*/, bool /*enable*/)
    {
        return false;
    }

    bool
    Visualization::highlight(unsigned int /*which*/, bool /*enable*/)
    {
        return false;
    }

    void
    Visualization::colorize(VisualizationFactory::Color /*c*/)
    {
    }

    void
    Visualization::setTransparency(float /*transparency*/)
    {
    }

    int
    Visualization::getNumFaces()
    {
        int res = 0;
        std::vector<VisualizationNodePtr>::iterator i = visualizationNodes.begin();

        while (i != visualizationNodes.end())
        {
            res += (*i)->getNumFaces();
            i++;
        }

        return res;
    }

    /*void Visualization::setGlobalPose( const Eigen::Matrix4f &p )
    {
        globalPose = p;
    }*/

} // namespace VirtualRobot
