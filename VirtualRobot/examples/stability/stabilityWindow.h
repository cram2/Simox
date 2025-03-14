
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
#include <VirtualRobot/XML/SceneIO.h>

#include "ui_stabilityScene.h"
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/sensors/SoTimerSensor.h>

class stabilityWindow : public QMainWindow
{
    Q_OBJECT
public:
    stabilityWindow(std::string& sRobotFile);
    ~stabilityWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();
    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;
    void resetSceneryAll();
    void selectRobot();
    void collisionModel();
    void selectJoint(int nr);
    void jointValueChanged(int pos);
    void selectRNS(int nr);
    void showCoM();
    void showSupportPolygon();
    void performCoMIK();
    void comTargetMovedX(int value);
    void comTargetMovedY(int value);


protected:
    void loadRobot();
    void buildVisu();

    void setupUI();
    QString formatString(const char* s, float f);

    void updateJointBox();
    void updateRNSBox();
    void updateCoM();
    void updateSupportVisu();

    Ui::MainWindowStability UI;
    SoQtExaminerViewer*
        m_pExViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    Eigen::Vector2f m_CoMTarget;

    SoSeparator* sceneSep;
    SoSeparator* robotVisuSep;
    SoSeparator* comVisu;
    SoSeparator* comProjectionVisu;
    SoSeparator* comTargetVisu;
    SoSeparator* supportVisu;

    VirtualRobot::RobotPtr robot;
    std::string robotFile;

    VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
    std::vector<VirtualRobot::RobotNodePtr> allRobotNodes;
    std::vector<VirtualRobot::RobotNodePtr> currentRobotNodes;
    std::vector<VirtualRobot::RobotNodeSetPtr> robotNodeSets;

    VirtualRobot::RobotNodePtr currentRobotNode;


    bool useColModel;


    std::shared_ptr<VirtualRobot::CoinVisualization> visualization;
};
