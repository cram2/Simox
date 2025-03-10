//----------------------------------------------------------------------
/*!
 * \ingroup Saba
 *
 *
 * \author  Nikolaus Vahrenkamp
 * \date    2011-04-04
 *
 * \class CoinRrtWorkspaceVisualization
 *
 * A Coin3D related visualization of an RRT search tree.
 *
 * Copyright Nikolaus Vahrenkamp
 * Karlsruhe Institute of Technology (KIT) - HIS

 */
//----------------------------------------------------------------------


#pragma once


#include "../../Saba.h"
#include "../RrtWorkspaceVisualization.h"


class SoNode;
class SoSeparator;
class SoCallbackAction;
class SoPrimitiveVertex;

namespace Saba
{

    class SABA_IMPORT_EXPORT CoinRrtWorkspaceVisualization :
        virtual public RrtWorkspaceVisualization
    {
    public:
        /*!
            Constructor
            Robot must have a node with name TCPName.
            The visualizations are build by determining the TCP's position in workspace according to the configurations of a path or tree .
        */
        CoinRrtWorkspaceVisualization(VirtualRobot::RobotPtr robot,
                                      CSpacePtr cspace,
                                      const std::string& TCPName);
        CoinRrtWorkspaceVisualization(VirtualRobot::RobotPtr robot,
                                      VirtualRobot::RobotNodeSetPtr robotNodeSet,
                                      const std::string& TCPName);

        ~CoinRrtWorkspaceVisualization() override;


        /*!
            Add visualization of a path in cspace.
        */
        bool addCSpacePath(CSpacePathPtr path,
                           RrtWorkspaceVisualization::ColorSet colorSet = eBlue) override;
        //void setPathStyle(float lineSize = 4.0f, float nodeSize= 15.0f, float renderComplexity = 1.0f);

        /*!
            Add visualization of a tree (e.g an RRT) in cspace.
        */
        bool addTree(CSpaceTreePtr tree,
                     RrtWorkspaceVisualization::ColorSet colorSet = eRed) override;
        //void setTreeStyle(float lineSize = 1.0f, float nodeSize= 15.0f, float renderComplexity = 0.1f);

        /*!
            Add visualization of a configuration in cspace.
        */
        bool addConfiguration(const Eigen::VectorXf& c,
                              RrtWorkspaceVisualization::ColorSet colorSet = eGreen,
                              float nodeSizeFactor = 1.0f) override;

        /*!
            Set the custom line and node color. Does not affect already added trees or paths.
        */
        //void setCustomColor(float nodeR, float nodeG, float nodeB, float lineR = 0.5f, float lineG = 0.5f, float lineB = 0.5f);

        /*!
            Clears all visualizations.
        */
        void reset() override;


        SoSeparator* getCoinVisualization();

    protected:
        void coinInit();

        SoSeparator* visualization;
    };

    typedef std::shared_ptr<CoinRrtWorkspaceVisualization> CoinRrtWorkspaceVisualizationPtr;


} // namespace Saba
