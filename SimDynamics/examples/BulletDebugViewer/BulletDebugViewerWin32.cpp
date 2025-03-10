
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>

#include <SimDynamics/DynamicsEngine/BulletEngine/BulletEngineFactory.h>
#include <SimDynamics/DynamicsEngine/BulletEngine/BulletOpenGLViewer.h>
#include <SimDynamics/DynamicsWorld.h>

using namespace std;
using namespace VirtualRobot;
using namespace SimDynamics;

/*
#ifdef WIN32

#include "SimDynamics/DynamicsEngine/BulletEngine/BulletOpenGL/DemoApplication.h"
#include <GL/gl.h>
#include <GL/glut.h>

//glut is C code, this global gDemoApplication links glut to the C++ demo
static DemoApplication* gDemoApplication = 0;


#include "SimDynamics/DynamicsEngine/BulletEngine/BulletOpenGL/GlutStuff.h"

static  void glutKeyboardCallback(unsigned char key, int x, int y)
{
    gDemoApplication->keyboardCallback(key, x, y);
}

static  void glutKeyboardUpCallback(unsigned char key, int x, int y)
{
    gDemoApplication->keyboardUpCallback(key, x, y);
}

static void glutSpecialKeyboardCallback(int key, int x, int y)
{
    gDemoApplication->specialKeyboard(key, x, y);
}

static void glutSpecialKeyboardUpCallback(int key, int x, int y)
{
    gDemoApplication->specialKeyboardUp(key, x, y);
}


static void glutReshapeCallback(int w, int h)
{
    gDemoApplication->reshape(w, h);
}

static void glutMoveAndDisplayCallback()
{
    gDemoApplication->moveAndDisplay();
}

static void glutMouseFuncCallback(int button, int state, int x, int y)
{
    gDemoApplication->mouseFunc(button, state, x, y);
}


static void glutMotionFuncCallback(int x, int y)
{
    gDemoApplication->mouseMotionFunc(x, y);
}


static void glutDisplayCallback(void)
{
    gDemoApplication->displayCallback();
}


int glutmain(int argc, char **argv, int width, int height, const char* title, DemoApplication* demoApp) {

    gDemoApplication = demoApp;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
    glutInitWindowPosition(width / 2, height / 2);
    glutInitWindowSize(width, height);
    glutCreateWindow(title);
#ifdef BT_USE_FREEGLUT
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif

    gDemoApplication->myinit();

    glutKeyboardFunc(glutKeyboardCallback);
    glutKeyboardUpFunc(glutKeyboardUpCallback);
    glutSpecialFunc(glutSpecialKeyboardCallback);
    glutSpecialUpFunc(glutSpecialKeyboardUpCallback);

    glutReshapeFunc(glutReshapeCallback);
    //createMenu();
    glutIdleFunc(glutMoveAndDisplayCallback);
    glutMouseFunc(glutMouseFuncCallback);
    glutPassiveMotionFunc(glutMotionFuncCallback);
    glutMotionFunc(glutMotionFuncCallback);
    glutDisplayFunc(glutDisplayCallback);

    glutMoveAndDisplayCallback();

    //enable vsync to avoid tearing on Apple (todo: for Windows)

#if defined(__APPLE__) && !defined (VMDMESA)
    int swap_interval = 1;
    CGLContextObj cgl_context = CGLGetCurrentContext();
    CGLSetParameter(cgl_context, kCGLCPSwapInterval, &swap_interval);
#endif



    glutMainLoop();
    return 0;
}
#endif

*/


DemoApplication*
createDemo()
{
    /*VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc,argv);
    VirtualRobot::RuntimeEnvironment::print();*/

    cout << " --- START --- " << endl;


    //std::string robFile("robots/examples/SimpleRobot/Joint5.xml");
    //std::string robFile("robots/iCub/iCub.xml");
    std::string robFile("robots/ArmarIII/ArmarIII.xml");
    //std::string robFile("robots/ArmarIII/ArmarIII-RightArm.xml");
    //std::string robFile("robots/iCub/iCub_RightHand.xml");
    //std::string robFile("robots/iCub/iCub_testFinger.xml");

    /*if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile2 = VirtualRobot::RuntimeEnvironment::getValue("robot");
        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile2))
        {
            robFile = robFile2;
        }
    }*/
    cout << "Using robot at " << robFile << endl;


    SimDynamics::DynamicsWorldPtr world = SimDynamics::DynamicsWorld::Init();
    SIMDYNAMICS_ASSERT(world);

    world->createFloorPlane();

    VirtualRobot::ObstaclePtr o = VirtualRobot::Obstacle::createBox(100.0f, 100.0f, 100.0f);
    o->setMass(1.0f); // 1kg

    SimDynamics::DynamicsObjectPtr dynObj = world->CreateDynamicsObject(o);
    dynObj->setPosition(Eigen::Vector3f(3000, 3000, 1000.0f));
    world->addObject(dynObj);

    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile);
    VirtualRobot::RobotPtr robot = VirtualRobot::RobotIO::loadRobot(robFile);

    if (robot)
    {
        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        //gp(2,3) = 35.0f;
        gp(2, 3) = 800.0f;
        robot->setGlobalPose(gp);
        DynamicsRobotPtr dynRob = world->CreateDynamicsRobot(robot);
        dynRob->disableActuation();
        world->addRobot(dynRob);
    }

    BulletOpenGLViewer* viewer = new BulletOpenGLViewer(world);
    //viewer.enableContraintsDebugDrawing();

    return viewer;
}
