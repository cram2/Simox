
#pragma once

#include <string.h>

#include <vector>

#include <QtCore/QtCore>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>

#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/XML/SceneIO.h>

#include "ui_reachabilityScene.h"
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/sensors/SoTimerSensor.h>

class reachabilityWindow : public QMainWindow
{
    Q_OBJECT
public:
    reachabilityWindow(std::string& sRobotFile, std::string& reachFile, Eigen::Vector3f& axisTCP);
    ~reachabilityWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;
    void resetSceneryAll();
    void selectRobot();
    void createReach();
    void saveReach();
    void loadReach();
    void fillHoles();
    void binarize();

    void computeVolume();
    void collisionModel();
    void reachVisu();
    void updateCutAngleSlider();
    void selectRNS(int nr);
    void selectJoint(int nr);
    void jointValueChanged(int pos);
    void extendReach();

    //void showRobot();
    /*
    void showCoordSystem();
    void robotStructure();
    void robotCoordSystems();
    void robotFullModel();
    void closeHand();
    void openHand();
    void selectEEF(int nr);*/


    SoQtExaminerViewer*
    getExaminerViewer()
    {
        return m_pExViewer;
    };

protected:
    void loadRobot();

    void setupUI();
    QString formatString(const char* s, float f);
    void buildVisu();
    void updateRNSBox();
    void updateJointBox();
    void loadReachFile(std::string filename);

    void updateQualityInfo();
    /*
    void updateEEFBox();
    void displayTriangles();*/
    Ui::MainWindowReachability UI;
    SoQtExaminerViewer*
        m_pExViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotVisuSep;
    SoSeparator* reachabilityVisuSep;

    VirtualRobot::RobotPtr robot;
    std::string robotFile;
    std::string reachFile;
    Eigen::Vector3f axisTCP;
    VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
    std::vector<VirtualRobot::RobotNodePtr> allRobotNodes;
    std::vector<VirtualRobot::RobotNodePtr> currentRobotNodes;
    std::vector<VirtualRobot::RobotNodeSetPtr> robotNodeSets;

    VirtualRobot::WorkspaceRepresentationPtr reachSpace;
    VirtualRobot::RobotNodePtr currentRobotNode;
    /*

    std::vector < VirtualRobot::EndEffectorPtr > eefs;
    VirtualRobot::EndEffectorPtr currentEEF;
    ;*/


    bool useColModel;
    //bool structureEnabled;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualization;
};
