
#pragma once

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeFixed.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>
#include <string.h>
#include <QtGui>
#include <QtWidgets/QtWidgets>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>

#include <filesystem>
#include <vector>

// #include "ui_GraspEditor.h"

namespace Ui
{
    class MainWindowGraspEditor;
}


namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT GraspEditorWindow : public QMainWindow
    {
        Q_OBJECT
    public:
        GraspEditorWindow(std::string& objFile, std::string& robotFile, bool embeddedGraspEditor = false);
        ~GraspEditorWindow() override;

        /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
        int main();

    public slots:
        /*! Closes the window and exits SoQt runloop. */
        void quit();

        /*!< Overriding the close event, so we know when the window was closed by the user. */
        void closeEvent(QCloseEvent* event) override;

        void resetSceneryAll();
        void loadObject();
        void loadRobot();

        void selectRobot();
        void selectObject(std::string file = "");
        void saveObject();
        void selectRobotObject(int n);
        void selectEEF(int n);
        void selectGrasp(int n);

        void closeEEF();
        void openEEF();

        void addGrasp();
        void renameGrasp();

        void sliderReleased_ObjectX();
        void sliderReleased_ObjectY();
        void sliderReleased_ObjectZ();
        void sliderReleased_ObjectA();
        void sliderReleased_ObjectB();
        void sliderReleased_ObjectG();

        void sampleGrasps();

        void virtualJointValueChanged();

        void buildVisu();

        void showCoordSystem();

    protected:

        void setupUI();
        QString formatString(const char* s, float f);

        void updateEEFBox();
        void updateGraspBox();

        void buildGraspSetVisu();

        void updateEEF(float x[6]);

        static void timerCB(void* data, SoSensor* sensor);
        void setCurrentGrasp(Eigen::Matrix4f& p);

        Ui::MainWindowGraspEditor* UI;

        // Indicates whether this program is started embedded
        bool embeddedGraspEditor;

        SoQtExaminerViewer* m_pExViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

        SoSeparator* sceneSep;
        SoSeparator* robotSep;
        SoSeparator* objectSep;
        SoSeparator* graspsSep;
        SoSeparator* eefVisu;
        SoSeparator* graspSetVisu;

        VirtualRobot::RobotPtr robot;
        VirtualRobot::RobotPtr robotEEF;
        VirtualRobot::GraspableSensorizedObjectPtr object;
        VirtualRobot::RobotPtr robotObject;
        std::vector<VirtualRobot::EndEffectorPtr> eefs;
        VirtualRobot::EndEffectorPtr currentEEF; // the eef of robot
        VirtualRobot::EndEffectorPtr robotEEF_EEF; // the eef of robotEEF

        VirtualRobot::GraspSetPtr currentGraspSet;
        VirtualRobot::ChainedGraspPtr currentGrasp;

        std::string robotFile;
        std::filesystem::path objectFile;

        SoTimerSensor* timer;


        std::shared_ptr<VirtualRobot::CoinVisualization> visualizationAll;

        std::vector<RobotPtr> hands;

        QSettings settings;
    };

}
