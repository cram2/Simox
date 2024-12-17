#include "CoinViewer.h"

#include <chrono>
#include <cmath>

#include <Eigen/Core>

#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>

#include <Inventor/actions/SoLineHighlightRenderAction.h>

CoinViewerExample::CoinViewerExample() : QMainWindow(nullptr)
{
    //setup scene
    {
        sceneSep = new SoSeparator;
        sceneSep->ref();

        auto addSphere = [&](int x, int y, int z, int r)
        {
            Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
            m(0, 3) = x;
            m(1, 3) = y;
            m(2, 3) = z;

            auto s = VirtualRobot::Obstacle::createSphere(r);
            s->setGlobalPose(m);
            sceneSep->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(
                s, VirtualRobot::SceneObject::Full));
        };

        addSphere(0, 1500, 1500, 400);
        addSphere(700, 900, 1500, 300);
        addSphere(0, 2000, 2000, 200);
        addSphere(500, 1500, 2000, 200);
    }

    //setup ui
    {
        UI.setupUi(this);
        viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);
        viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));

        viewer->setAntialiasing(true, 4);

        viewer->setGLRenderAction(new SoLineHighlightRenderAction);
        viewer->setTransparencyType(SoGLRenderAction::BLEND);
        viewer->setAccumulationBuffer(false);
        viewer->setFeedbackVisibility(true);
        viewer->setSceneGraph(sceneSep);

        viewer->viewAll();
    }
    startTimer(10);
}

CoinViewerExample::~CoinViewerExample()
{
    sceneSep->unref();
}

void
CoinViewerExample::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

int
CoinViewerExample::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

void
CoinViewerExample::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void
CoinViewerExample::timerEvent(QTimerEvent*)
{
    static const auto first = std::chrono::high_resolution_clock::now();
    const auto now = std::chrono::high_resolution_clock::now();
    const float x = (now - first).count() / 1e9;
    const float r = 0.5f + std::sin(x);
    const float g = 0.5f + std::cos(x);
    viewer->setBackgroundColor(SbColor(r, g, 1.0f));
    viewer->scheduleRedraw();
}
