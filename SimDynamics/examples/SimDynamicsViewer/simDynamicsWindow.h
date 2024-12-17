
#pragma once

#include <string.h>

#include <atomic>
#include <vector>

#include <QtCore/QtCore>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>

#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "SimDynamics/DynamicsEngine/BulletEngine/BulletCoinQtViewer.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "ui_simDynamicsViewer.h"
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/sensors/SoTimerSensor.h>

class SimDynamicsWindow : public QMainWindow
{
    Q_OBJECT
public:
    SimDynamicsWindow(std::string& sRobotFilename);
    ~SimDynamicsWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();
    void buildVisualization();
    void actuation();

    void loadButton();

    void selectRobotNode(int n);
    void jointValueChanged(int n);
    void fixedTimeStepChanged(int n);
    void updateTimerChanged(int n);
    void updateAntiAliasing(int n);
    void comVisu();
    void updateJointInfo();
    void updateRobotInfo();

    void startStopEngine();
    void stepEngine();

    void checkBoxFixedTimeStep();

    void addObject();

    void reloadRobot();

    void resetPose();
    void setPose();

protected:
    bool loadRobot(std::string robotFilename);
    void setupUI();
    void updateJoints();

    void stopCB();

    void updateContactVisu();
    void updateComVisu();

    SimDynamics::DynamicsWorldPtr dynamicsWorld;
    SimDynamics::DynamicsRobotPtr dynamicsRobot;
    SimDynamics::DynamicsObjectPtr dynamicsObject;
    SimDynamics::DynamicsObjectPtr dynamicsObject2;
    std::vector<SimDynamics::DynamicsObjectPtr> dynamicsObjects;

    Ui::MainWindowBulletViewer UI;

    SoSeparator* sceneSep;
    SoSeparator* comSep;
    SoSeparator* contactsSep;
    SoSeparator* forceSep;

    SimDynamics::BulletCoinQtViewerPtr viewer;

    VirtualRobot::RobotPtr robot;

    // beside the viewer cb we need also a callback to update joint info
    static void timerCB(void* data, SoSensor* sensor);
    static void stepCB(void* data, btScalar timeStep);

    SoTimerSensor* timerSensor;

    std::vector<VirtualRobot::RobotNodePtr> robotNodes;

    std::map<VirtualRobot::RobotNodePtr, SoSeparator*> comVisuMap;

    bool useColModel;
    std::string robotFilename;

    std::atomic_uint_fast64_t simuStepCount{0};
};
