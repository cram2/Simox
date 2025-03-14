
#pragma once

#include <string.h>

#include <vector>

#include <QtCore/QtCore>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>

#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/XML/SceneIO.h>

#include "MotionPlanning/CSpace/CSpacePath.h"
#include "MotionPlanning/Saba.h"
#include "ui_RrtGui.h"
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/sensors/SoTimerSensor.h>

class RrtGuiWindow : public QMainWindow
{
    Q_OBJECT
public:
    RrtGuiWindow(const std::string& sceneFile,
                 const std::string& sConf,
                 const std::string& gConf,
                 const std::string& rns,
                 const std::string& colModelRob1,
                 const std::string& colModelRob2,
                 const std::string& colModelEnv);
    ~RrtGuiWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    void redraw();
public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void loadSceneWindow();

    void selectStart(int nr);
    void selectGoal(int nr);
    void selectRNS(int nr);

    void selectColModelRobA(int nr);
    void selectColModelRobB(int nr);
    void selectColModelEnv(int nr);

    void colModel();
    void solutionSelected();

    void sliderSolution(int pos);

    void buildVisu();

    void plan();

protected:
    void loadScene();

    void setupUI();
    QString formatString(const char* s, float f);
    void buildRRTVisu();
    void selectStart(const std::string& conf);
    void selectGoal(const std::string& conf);
    void selectRNS(const std::string& rns);

    static void timerCB(void* data, SoSensor* sensor);
    void buildRrtVisu();
    void selectColModelRobA(const std::string& colModel);
    void selectColModelRobB(const std::string& colModel);
    void selectColModelEnv(const std::string& colModel);
    Ui::MainWindowRRTDemo UI;
    SoQtExaminerViewer*
        viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* allSep;
    SoSeparator* sceneFileSep;
    SoSeparator* startGoalSep;
    SoSeparator* rrtSep;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotPtr robotStart;
    VirtualRobot::RobotPtr robotGoal;

    Saba::CSpaceSampledPtr cspace;
    Eigen::VectorXf startConfig;
    Eigen::VectorXf goalConfig;

    VirtualRobot::RobotNodeSetPtr rns;
    VirtualRobot::SceneObjectSetPtr colModelRobA;
    VirtualRobot::SceneObjectSetPtr colModelRobB;
    VirtualRobot::SceneObjectSetPtr colModelEnv;

    std::vector<VirtualRobot::RobotConfigPtr> configs;

    std::string sceneFile;
    VirtualRobot::ScenePtr scene;

    Saba::CSpacePathPtr solution;
    Saba::CSpacePathPtr solutionOptimized;
    Saba::CSpaceTreePtr tree;
    Saba::CSpaceTreePtr tree2;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualization;
};
