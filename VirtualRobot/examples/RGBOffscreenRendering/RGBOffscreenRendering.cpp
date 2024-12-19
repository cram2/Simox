#include "RGBOffscreenRendering.h"
#include <iostream>

#include <Eigen/Core>

#include <SimoxUtility/math/convert/pos_rpy_to_mat4f.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>

#include <Inventor/Qt/SoQt.h>

RGBOffscreenRenderingExample::RGBOffscreenRenderingExample() : QMainWindow(nullptr)
{
    camRenderer = nullptr;
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
        connect(UI.doubleSpinBoxCamYaw,
                SIGNAL(valueChanged(double)),
                this,
                SLOT(camYawUpdated(double)));
    }
    camYawUpdated(UI.doubleSpinBoxCamYaw->value());
}

RGBOffscreenRenderingExample::~RGBOffscreenRenderingExample()
{
    sceneSep->unref();
}

void
RGBOffscreenRenderingExample::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

int
RGBOffscreenRenderingExample::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

void
RGBOffscreenRenderingExample::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void
RGBOffscreenRenderingExample::camYawUpdated(double yaw)
{
    const short width = 640;
    const short height = 480;

    //cam + buffer setup
    if (!camRenderer)
    {
        const auto pixelCount = width * height;
        camRGBBuffer.resize(pixelCount * 3);
        camRenderer =
            VirtualRobot::CoinVisualizationFactory::createOffscreenRenderer(width, height);
    }

    const float zNear = 10;
    const float zFar = 100000;
    const float fov = M_PI / 4;

    const Eigen::Matrix4f pose = simox::math::pos_rpy_to_mat4f(0, 0, 1500, 0, 0, yaw) *
                                 simox::math::pos_rpy_to_mat4f(0, 0, 0, 0, -M_PI / 2, 0);

    std::vector<float> depthImage;
    std::vector<Eigen::Vector3f> pointCloud;
    VirtualRobot::CoinVisualizationFactory::renderOffscreenRgbDepthPointcloud(camRenderer,
                                                                              pose,
                                                                              sceneSep,
                                                                              width,
                                                                              height,
                                                                              true,
                                                                              camRGBBuffer,
                                                                              false,
                                                                              depthImage,
                                                                              false,
                                                                              pointCloud,
                                                                              zNear,
                                                                              zFar,
                                                                              fov);

    QImage img(camRGBBuffer.data(), width, height, QImage::Format_RGB888);
    UI.labelImg->setPixmap(QPixmap::fromImage(img.mirrored(false, true)));
}
