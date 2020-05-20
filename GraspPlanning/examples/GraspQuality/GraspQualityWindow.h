
#pragma once

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Obstacle.h>

#include "GraspPlanning/GraspStudio.h"
#include "GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>


#include <vector>

#include "ui_GraspQuality.h"

class GraspQualityWindow : public QMainWindow
{
    Q_OBJECT
public:
    GraspQualityWindow(std::string& robotFile, std::string& objectFile);
    ~GraspQualityWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();


    void closeEEF();
    void openEEF();
    void colModel();
    void frictionConeVisu();

    void sliderReleased_ObjectX();
    void sliderReleased_ObjectY();
    void sliderReleased_ObjectZ();
    void sliderReleased_ObjectA();
    void sliderReleased_ObjectB();
    void sliderReleased_ObjectG();

    void buildVisu();

    void selectEEF(int nr);
    void selectGrasp(int nr);
    void objectToTCP();
    void objectToGrasp();

    void graspQuality();
    void showGWS();
    void showOWS();

    void selectObject();

    void evalRobustness();

    void evalRobustnessAll();

protected:

    void loadRobot();
    void loadObject();

    void setupUI();

    void updateObject(float x[6]);

    static void timerCB(void* data, SoSensor* sensor);
    void buildRrtVisu();
    void setEEFComboBox();
    void setGraspComboBox();

    Ui::MainWindowGraspQuality UI;
    SoQtExaminerViewer* m_pExViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* objectSep;
    SoSeparator* frictionConeSep;
    SoSeparator* gws1Sep;
    SoSeparator* gws2Sep;
    SoSeparator* ows1Sep;
    SoSeparator* ows2Sep;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::ManipulationObjectPtr object;

    VirtualRobot::EndEffectorPtr eef;
    std::vector< VirtualRobot::EndEffectorPtr > eefs;
    VirtualRobot::GraspPtr grasp;
    VirtualRobot::GraspSetPtr grasps;

    std::vector<Eigen::Matrix4f> evalPoses;

    VirtualRobot::EndEffector::ContactInfoVector contacts;


    std::string robotFile;
    std::string objectFile;
    std::string eefName;

    GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;

    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot;
    std::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;
};

