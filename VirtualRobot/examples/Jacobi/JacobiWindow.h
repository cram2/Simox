
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
#include <VirtualRobot/XML/RobotIO.h>

#include "ui_Jacobi.h"
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/sensors/SoTimerSensor.h>

class JacobiWindow : public QMainWindow
{
    Q_OBJECT
public:
    JacobiWindow(std::string& sRobotFilename);
    ~JacobiWindow() override;

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    SoQtExaminerViewer*
    getExaminerViewer()
    {
        return exViewer;
    };

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event) override;

    void resetSceneryAll();
    void collisionModel();
    void loadRobot();
    void selectKC(int nr);
    void sliderReleased();
    void sliderPressed();

    void box2TCP();
    void jacobiTest();
    void jacobiTest2();
    void jacobiTestBi();

protected:
    void setupUI();
    QString formatString(const char* s, float f);
    void updateKCBox();

    void updatBoxPos(float x, float y, float z, float a, float b, float g);
    void updatBox2Pos(float x, float y, float z, float a, float b, float g);
    void updatBoxBiPos(float x, float y, float z, float a, float b, float g);

    static void updateCB(void* data, SoSensor* sensor);


    Ui::MainWindowJacobiDemo UI;
    SoQtExaminerViewer*
        exViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* boxSep;
    SoSeparator* box2Sep;
    SoSeparator* box3Sep;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;
    VirtualRobot::RobotNodePtr tcp;
    VirtualRobot::RobotNodePtr tcp2;
    VirtualRobot::RobotNodePtr elbow;
    VirtualRobot::RobotNodeSetPtr kc;
    std::vector<VirtualRobot::RobotNodeSetPtr> kinChains;

    VirtualRobot::ObstaclePtr box;
    VirtualRobot::ObstaclePtr box2;
    VirtualRobot::ObstaclePtr box3;

    bool useColModel;
};
