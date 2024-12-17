
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

#include "ui_SceneViewer.h"
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/sensors/SoTimerSensor.h>

class showSceneWindow : public QMainWindow
{
    Q_OBJECT
public:
    showSceneWindow(std::string& sSceneFile);
    ~showSceneWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();
    void loadScene();
    void selectScene();

    void selectRobot(int nr);
    void selectObject(int nr);
    void selectGrasp(int nr);
    void selectEEF(int nr);
    void selectRobotConfig(int nr);
    void selectTrajectory(int nr);
    void sliderMoved(int pos);

    void closeHand();
    void openHand();
    void colModel();
    void showRoot();

    SoQtExaminerViewer*
    getExaminerViewer()
    {
        return viewer;
    };

protected:
    void updateGui();
    void updateGrasps();
    void updateGraspVisu();
    void setupUI();
    QString formatString(const char* s, float f);
    void buildVisu();

    Ui::MainWindowShowScene UI;
    SoQtExaminerViewer*
        viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* sceneVisuSep;
    SoSeparator* graspVisu;
    SoSeparator* coordVisu;
    VirtualRobot::GraspPtr currentGrasp;
    VirtualRobot::GraspSetPtr currentGraspSet;
    VirtualRobot::SceneObjectPtr currentObject;
    VirtualRobot::RobotPtr currentRobot;
    VirtualRobot::TrajectoryPtr currentTrajectory;
    VirtualRobot::EndEffectorPtr currentEEF;

    VirtualRobot::ScenePtr scene;
    std::string sceneFile;


    std::shared_ptr<VirtualRobot::CoinVisualization> visualization;
};
