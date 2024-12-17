
#pragma once

#include <string.h>

#include <vector>

#include <QtCore/QtCore>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>

#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "ui_RobotViewer.h"
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/sensors/SoTimerSensor.h>

class showRobotWindow : public QMainWindow
{
    Q_OBJECT
public:
    showRobotWindow(std::string& sRobotFilename);
    ~showRobotWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();
    void rebuildVisualization();
    void showRobot();
    void loadRobot();
    void selectPrimitiveModel(int nr);
    void selectJoint(int nr);
    void selectRNS(int nr);
    void jointValueChanged(int pos);
    void showCoordSystem();
    void robotStructure();
    void robotCoordSystems();
    void robotFullModel();
    void showSensors();
    void closeHand();
    void openHand();
    void selectEEF(int nr);
    void selectPreshape(int nr);
    void selectRobot();
    void reloadRobot();
    void displayPhysics();
    void exportVRML();
    void exportXML();
    void updatePointDistanceVisu();
    void openDiffIK();
    void selectConfiguration(int nr);
    void setConfiguration();

    SoQtExaminerViewer*
    getExaminerViewer()
    {
        return viewer;
    };

protected:
    void setupUI();
    QString formatString(const char* s, float f);
    void updateJointBox();
    void updateRNSBox();
    void updateEEFBox();
    void displayTriangles();
    void updatRobotInfo();
    Ui::MainWindowShowRobot UI;
    SoQtExaminerViewer*
        viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* extraSep;

    VirtualRobot::RobotPtr robot;
    std::string m_sRobotFilename;
    std::vector<VirtualRobot::RobotNodePtr> allRobotNodes;
    std::vector<VirtualRobot::RobotNodePtr> currentRobotNodes;
    std::vector<VirtualRobot::RobotNodeSetPtr> robotNodeSets;
    std::vector<VirtualRobot::EndEffectorPtr> eefs;
    VirtualRobot::EndEffectorPtr currentEEF;
    VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
    VirtualRobot::RobotNodePtr currentRobotNode;

    std::vector<std::string> currentConfigurations;
    std::string currentConfiguration;

    bool useColModel;
    bool structureEnabled;
    bool physicsCoMEnabled;
    bool physicsInertiaEnabled;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualization;

    void testPerformance(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr rns);

    struct PtDistanceVisu
    {
        SoSeparator* sep;
    };

    PtDistanceVisu ptDistance;
};
