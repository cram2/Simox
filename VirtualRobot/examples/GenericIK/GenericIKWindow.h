
#pragma once

#include <string.h>

#include <vector>

#include <QtCore/QtCore>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>

#include <VirtualRobot/IK/ConstrainedIK.h>
#include <VirtualRobot/IK/GazeIK.h>
#include <VirtualRobot/IK/GenericIKSolver.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "ui_GenericIK.h"
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/sensors/SoTimerSensor.h>

class GenericIKWindow : public QMainWindow
{
    Q_OBJECT
public:
    GenericIKWindow(std::string& sRobotFilename);
    ~GenericIKWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();
    void collisionModel();
    void loadRobot();
    void selectKC(int nr);
    void selectIK(int nr);
    void sliderReleased();
    void sliderPressed();

    void box2TCP();
    void solve();


protected:
    void setupUI();
    QString formatString(const char* s, float f);
    void updateKCBox();

    void updatBoxPos(float x, float y, float z, float a, float b, float g);

    static void updateCB(void* data, SoSensor* sensor);


    Ui::MainWindowGenericIKDemo UI;
    SoQtExaminerViewer*
        exViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* boxSep;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;
    VirtualRobot::RobotNodePtr tcp;
    VirtualRobot::RobotNodeSetPtr kc;
    std::vector<VirtualRobot::RobotNodeSetPtr> kinChains;

    VirtualRobot::GenericIKSolverPtr ikSolver;
    VirtualRobot::GazeIKPtr ikGazeSolver;
    VirtualRobot::ConstrainedIKPtr ikConstrainedSolver;
    VirtualRobot::ObstaclePtr box;

    bool useColModel;
};
