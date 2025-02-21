#include "DepthOffscreenRendering.h"

#include <iostream>

#include <eigen3/Eigen/Core>

#include <SimoxUtility/math/convert/pos_rpy_to_mat4f.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>

#include <Inventor/Qt/SoQt.h>

DepthOffscreenRenderingExample::DepthOffscreenRenderingExample() : QMainWindow(nullptr)
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

DepthOffscreenRenderingExample::~DepthOffscreenRenderingExample()
{
    sceneSep->unref();
}

void
DepthOffscreenRenderingExample::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

int
DepthOffscreenRenderingExample::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

void
DepthOffscreenRenderingExample::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void
DepthOffscreenRenderingExample::camYawUpdated(double yaw)
{
    const short width = 640;
    const short height = 480;

    //cam + buffer setup
    if (!camRenderer)
    {
        const auto pixelCount = width * height;
        camDepthBuffer.resize(pixelCount);
        camRenderer =
            VirtualRobot::CoinVisualizationFactory::createOffscreenRenderer(width, height);
    }

    const float zNear = 10;
    const float zFar = 100000;
    const float fov = M_PI / 4;

    const Eigen::Matrix4f pose = simox::math::pos_rpy_to_mat4f(0, 0, 1500, 0, 0, yaw) *
                                 simox::math::pos_rpy_to_mat4f(0, 0, 0, 0, -M_PI / 2, 0);

    std::vector<unsigned char> rgbImage;
    std::vector<Eigen::Vector3f> pointCloud;
    VirtualRobot::CoinVisualizationFactory::renderOffscreenRgbDepthPointcloud(camRenderer,
                                                                              pose,
                                                                              sceneSep,
                                                                              width,
                                                                              height,
                                                                              false,
                                                                              rgbImage,
                                                                              true,
                                                                              camDepthBuffer,
                                                                              false,
                                                                              pointCloud,
                                                                              zNear,
                                                                              zFar,
                                                                              fov);

    QImage img(width, height, QImage::Format_Grayscale8);
    for (std::size_t i = 0; i < camDepthBuffer.size(); ++i)
    {
        static constexpr float maxdepth = 5000;
        const float c = std::min(camDepthBuffer.at(i), maxdepth) / maxdepth * 255;
        img.bits()[i] = static_cast<uchar>(c);
    }
    UI.labelImg->setPixmap(QPixmap::fromImage(img.mirrored(false, true)));
}
