
#include "simDynamicsWindow.h"

#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>

#include <QFileDialog>

#include <Eigen/Geometry>

#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/XML/ObjectIO.h>

#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoUnits.h>
#include <SimDynamics/DynamicsEngine/BulletEngine/BulletEngine.h>
using namespace std;
using namespace VirtualRobot;
using namespace SimDynamics;

SimDynamicsWindow::SimDynamicsWindow(std::string& sRobotFilename) : QMainWindow(nullptr)
{
    VR_INFO << " start " << std::endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    //resize(1100, 768);

    useColModel = false;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
    std::string robotFilename = sRobotFilename;
    sceneSep = new SoSeparator;
    sceneSep->ref();

    comSep = new SoSeparator;
    sceneSep->addChild(comSep);

    contactsSep = new SoSeparator;
    sceneSep->addChild(contactsSep);

    forceSep = new SoSeparator;
    sceneSep->addChild(forceSep);

    // optional visualizations (not considered by dynamics)
    SoSeparator* cc = CoinVisualizationFactory::CreateCoordSystemVisualization(10.0f);
    sceneSep->addChild(cc);
    BulletEngineConfigPtr config(new BulletEngineConfig());
    config->bulletSolverIterations = 2000;
    config->bulletObjectRestitution = btScalar(0.1);
    config->bulletObjectFriction = btScalar(1.0);
    config->bulletSolverGlobalContactForceMixing = btScalar(0.00001);
    config->bulletObjectDampingLinear = btScalar(0.3);
    config->bulletObjectDampingAngular = btScalar(0.3);
    dynamicsWorld = SimDynamics::DynamicsWorld::Init(config);
    SIMDYNAMICS_ASSERT(dynamicsWorld);

    dynamicsWorld->createFloorPlane();

    VirtualRobot::ObstaclePtr o = VirtualRobot::Obstacle::createBox(
        1000.0f, 1000.0f, 1000.0f, VirtualRobot::VisualizationFactory::Color::Blue());
    o->setMass(1.0f); // 1kg

    dynamicsObject = dynamicsWorld->CreateDynamicsObject(o);
    dynamicsObject->setPosition(Eigen::Vector3f(1000, 2000, 1000.0f));
    dynamicsWorld->addObject(dynamicsObject);

    addObject();

    setupUI();
    loadRobot(robotFilename);

    // build visualization
    buildVisualization();

    viewer->viewAll();

    // register callback
    float TIMER_MS = 30.0f;
    SoSensorManager* sensor_mgr = SoDB::getSensorManager();
    timerSensor = new SoTimerSensor(timerCB, static_cast<void*>(this));
    timerSensor->setInterval(SbTime(TIMER_MS / 1000.0f));
    sensor_mgr->insertTimerSensor(timerSensor);

    viewer->addStepCallback(stepCB, static_cast<void*>(this));
}

SimDynamicsWindow::~SimDynamicsWindow()
{
    stopCB();
    dynamicsWorld.reset();
    SimDynamics::DynamicsWorld::Close();
    robot.reset();
    dynamicsRobot.reset();
    sceneSep->unref();
}

void
SimDynamicsWindow::timerCB(void* data, SoSensor* /*sensor*/)
{
    SimDynamicsWindow* window = static_cast<SimDynamicsWindow*>(data);
    VR_ASSERT(window);

    if (window->UI.checkBoxAutoMoveRobot->isChecked())
    {
        window->setPose();
    }
    // now its safe to update physical information and set the models to the according poses
    window->updateJointInfo();
    window->updateRobotInfo();

    window->updateContactVisu();
    window->updateComVisu();

    window->UI.label_simuStepCount->setText(QString::number(window->simuStepCount));
}

void
SimDynamicsWindow::stepCB(void* data, btScalar /*timeStep*/)
{
    SimDynamicsWindow* window = static_cast<SimDynamicsWindow*>(data);
    VR_ASSERT(window);

    window->simuStepCount++;
}

void
SimDynamicsWindow::setupUI()
{
    UI.setupUi(this);

    viewer.reset(new SimDynamics::BulletCoinQtViewer(dynamicsWorld));
    viewer->initSceneGraph(UI.frameViewer, sceneSep);
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(buildVisualization()));
    connect(UI.checkBoxActuation, SIGNAL(clicked()), this, SLOT(actuation()));
    connect(UI.checkBoxCom, SIGNAL(clicked()), this, SLOT(comVisu()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadButton()));
    connect(UI.pushButtonStartStop, SIGNAL(clicked()), this, SLOT(startStopEngine()));
    connect(UI.pushButtonStep, SIGNAL(clicked()), this, SLOT(stepEngine()));
    connect(UI.comboBoxRobotNode, SIGNAL(activated(int)), this, SLOT(selectRobotNode(int)));
    connect(
        UI.horizontalSliderTarget, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));
    connect(UI.checkBoxFixedTimeStep, SIGNAL(clicked()), this, SLOT(checkBoxFixedTimeStep()));
    UI.horizontalSliderFixedTimeStep->setValue(viewer->getBulletSimTimeStepMsec());
    UI.horizontalSliderUpdateTimer->setValue(viewer->getUpdateTimerInterval());
    connect(UI.horizontalSliderFixedTimeStep,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(fixedTimeStepChanged(int)));
    connect(UI.horizontalSliderUpdateTimer,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(updateTimerChanged(int)));
    connect(UI.spinBoxAntiAliasing, SIGNAL(valueChanged(int)), this, SLOT(updateAntiAliasing(int)));

    connect(UI.pushButtonAddObject, SIGNAL(clicked()), this, SLOT(addObject()));
    connect(UI.pushButton_reloadRobot, SIGNAL(clicked()), this, SLOT(reloadRobot()));

    connect(UI.button_reset, SIGNAL(clicked()), this, SLOT(resetPose()));
    connect(UI.button_set, SIGNAL(clicked()), this, SLOT(setPose()));

    /*connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeHand()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openHand()));
    connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));



    connect(UI.checkBoxStructure, SIGNAL(clicked()), this, SLOT(robotStructure()));
    UI.checkBoxFullModel->setChecked(true);
    connect(UI.checkBoxFullModel, SIGNAL(clicked()), this, SLOT(robotFullModel()));
    connect(UI.checkBoxRobotCoordSystems, SIGNAL(clicked()), this, SLOT(robotCoordSystems()));
    connect(UI.checkBoxShowCoordSystem, SIGNAL(clicked()), this, SLOT(showCoordSystem()));
    connect(UI.comboBoxRobotNodeSet, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));*/
}

void
SimDynamicsWindow::resetSceneryAll()
{
    if (robot)
    {
        robot->applyJointValues();
    }
}

void
SimDynamicsWindow::actuation()
{
    if (!dynamicsRobot)
    {
        return;
    }

    bool actuate = UI.checkBoxActuation->checkState() == Qt::Checked;

    if (actuate)
    {
        ActuationMode actuation;
        actuation.modes.position = 1;
        dynamicsRobot->enableActuation(actuation);
    }
    else
    {
        dynamicsRobot->disableActuation();
    }
}

void
SimDynamicsWindow::buildVisualization()
{
    if (!robot)
    {
        return;
    }

    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel =
        useColModel ? SceneObject::Collision : SceneObject::Full;
    viewer->addVisualization(dynamicsRobot, colModel);
    viewer->addVisualization(dynamicsObject, colModel);

    for (const auto& dynamicsObject : dynamicsObjects)
    {
        viewer->addVisualization(dynamicsObject, colModel);
    }

    if (dynamicsObject2)
    {
        viewer->addVisualization(dynamicsObject2, colModel);
    }
}

void
SimDynamicsWindow::comVisu()
{
    if (!robot)
    {
        return;
    }

    comSep->removeAllChildren();
    comVisuMap.clear();
    bool visuCom = UI.checkBoxCom->checkState() == Qt::Checked;

    if (visuCom)
    {
        std::vector<RobotNodePtr> n = robot->getRobotNodes();

        for (const auto& i : n)
        {
            SoSeparator* sep = new SoSeparator;
            comSep->addChild(sep);
            Eigen::Matrix4f cp = dynamicsRobot->getComGlobal(i);
            sep->addChild(CoinVisualizationFactory::getMatrixTransformScaleMM2M(cp));
            sep->addChild(CoinVisualizationFactory::CreateCoordSystemVisualization(5.0f));
            comVisuMap[i] = sep;
        }
    }
}

void
SimDynamicsWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

int
SimDynamicsWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

void
SimDynamicsWindow::quit()
{
    std::cout << "SimDynamicsWindow: Closing" << std::endl;
    stopCB();
    this->close();
    SoQt::exitMainLoop();
}

/*void simDynamicsWindow::selectRobot()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
    m_sRobotFilename = std::string(fi.toAscii());
    loadRobot();
}*/

void
SimDynamicsWindow::updateJoints()
{
    if (!robot)
    {
        std::cout << " ERROR while creating list of nodes" << std::endl;
        return;
    }

    robotNodes.clear();
    UI.comboBoxRobotNode->clear();
    std::vector<RobotNodePtr> nodes = robot->getRobotNodes();

    for (auto& node : nodes)
    {
        if (node->isRotationalJoint() || node->isTranslationalJoint())
        {
            robotNodes.push_back(node);
            QString qstr(node->getName().c_str());
            UI.comboBoxRobotNode->addItem(qstr);
        }
    }

    if (robotNodes.size() > 0)
    {
        selectRobotNode(0);
    }
    else
    {
        selectRobotNode(-1);
    }
}

bool
SimDynamicsWindow::loadRobot(std::string robotFilename)
{
    std::cout << "Loading Robot from " << robotFilename << std::endl;

    try
    {
        robot = RobotIO::loadRobot(robotFilename, RobotIO::eFull);
    }
    catch (VirtualRobotException& e)
    {
        std::cout << " ERROR while creating robot" << std::endl;
        std::cout << e.what();
        return false;
    }

    if (!robot)
    {
        std::cout << " ERROR while creating robot" << std::endl;
        return false;
    }

    try
    {
        VirtualRobot::BoundingBox bbox = robot->getBoundingBox();

        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        //gp(2,3) = 5.0f;
        gp(2, 3) = -bbox.getMin()(2) + 4.0f;
        robot->setGlobalPose(gp);
        if (UI.checkBox_kinematicJoints->isChecked())
        {
            for (auto n : robot->getRobotNodes())
            {
                n->setSimulationType(SceneObject::Physics::eKinematic);
            }
        }
        dynamicsRobot = dynamicsWorld->CreateDynamicsRobot(robot);
        if (!UI.checkBox_selfCol->isChecked())
        {
            //we don't want to call this function with true (we would enable all collisions)
            dynamicsRobot->enableSelfCollisions(false);
        }
        dynamicsWorld->addRobot(dynamicsRobot);
    }
    catch (VirtualRobotException& e)
    {
        std::cout << " ERROR while building dynamic robot" << std::endl;
        std::cout << e.what();
        return false;
    }

    updateJoints();
    this->robotFilename = robotFilename;
    return true;
}

void
SimDynamicsWindow::selectRobotNode(int n)
{
    UI.comboBoxRobotNode->setCurrentIndex(n);
    RobotNodePtr rn;

    if (n >= 0 && n < (int)robotNodes.size())
    {
        rn = robotNodes[n];
    }

    if (rn)
    {
        int pos = 0;

        float l = rn->getJointLimitHi() - rn->getJointLimitLo();
        float start = rn->getJointValue() - rn->getJointLimitLo();

        if (fabs(l) > 1e-6)
        {
            pos = int(start / l * 200.0f + 0.5f) - 100;
        }

        UI.horizontalSliderTarget->setValue(pos);
        UI.horizontalSliderTarget->setEnabled(true);
    }
    else
    {
        UI.horizontalSliderTarget->setValue(0);
        UI.horizontalSliderTarget->setEnabled(false);
    }
}

void
SimDynamicsWindow::updateJointInfo()
{
    //std::stringstream info;
    std::string info;
    int n = UI.comboBoxRobotNode->currentIndex();
    QString qMin("0");
    QString qMax("0");
    QString qName("Name: <not set>");
    QString qJV("Joint value: 0");
    QString qTarget("Joint target: 0");
    QString qVel("Joint velocity: 0");
    QString qVelTarget("Joint velocity target: 0");
    QString qGP("GlobalPosition (simox): 0/0/0");
    QString qGPRPY("GlobalRPY (simox): 0/0/0");
    QString qVisu("VISU (simox): 0/0/0");
    QString qCom("COM (bullet): 0/0/0");
    QString tmp;
    RobotNodePtr rn;

    if (n >= 0 && n < (int)robotNodes.size())
    {
        rn = robotNodes[n];
    }
    else
    {
        return;
    }

    SimDynamics::DynamicsObjectPtr dynRN = dynamicsRobot->getDynamicsRobotNode(rn);
    SimDynamics::BulletObjectPtr bulletRN =
        std::dynamic_pointer_cast<SimDynamics::BulletObject>(dynRN);

    if (bulletRN)
    {
        //      std::cout << "FORCE: " << bulletRN->getRigidBody()->getTotalForce()[0] << ", " << bulletRN->getRigidBody()->getTotalForce()[1] << ", " << bulletRN->getRigidBody()->getTotalForce()[2] << std::endl;
        //      std::cout << "TORQUE: " << bulletRN->getRigidBody()->getTotalTorque()[0] << ", " << bulletRN->getRigidBody()->getTotalTorque()[1] << ", " << bulletRN->getRigidBody()->getTotalTorque()[2] << std::endl;
        //      std::cout << "getLinearVelocity: " << bulletRN->getRigidBody()->getLinearVelocity()[0] << ", " << bulletRN->getRigidBody()->getLinearVelocity()[1] << ", " << bulletRN->getRigidBody()->getLinearVelocity()[2] << std::endl;
        //      std::cout << "getAngularVelocity: " << bulletRN->getRigidBody()->getAngularVelocity()[0] << ", " << bulletRN->getRigidBody()->getAngularVelocity()[1] << ", " << bulletRN->getRigidBody()->getAngularVelocity()[2] << std::endl;
    }

    BulletRobotPtr bulletRobot = std::dynamic_pointer_cast<SimDynamics::BulletRobot>(dynamicsRobot);

    if (rn && bulletRobot && bulletRobot->hasLink(rn))
    {
        BulletRobot::LinkInfo linkInfo = bulletRobot->getLink(rn);

        linkInfo.nodeA->showCoordinateSystem(true);
        linkInfo.nodeB->showCoordinateSystem(true);

        bulletRobot->enableForceTorqueFeedback(linkInfo);
        Eigen::VectorXf ftA = bulletRobot->getForceTorqueFeedbackA(linkInfo);
        Eigen::VectorXf ftB = bulletRobot->getForceTorqueFeedbackB(linkInfo);

        std::stringstream streamFTA;

        streamFTA << "ForceTorqueA: " << std::fixed;
        streamFTA.precision(1);

        for (int i = 0; i < 6; ++i)
        {
            streamFTA << ftA[i] << ",";
        }

        UI.label_ForceTorqueA->setText(QString::fromStdString(streamFTA.str()));
        std::stringstream streamFTB;
        streamFTB << "ForceTorqueB: " << std::fixed;
        streamFTB.precision(1);

        for (int i = 0; i < 6; ++i)
        {
            streamFTB << ftB[i] << ",";
        }

        UI.label_ForceTorqueB->setText(QString::fromStdString(streamFTB.str()));

        //      std::cout << "ForceTorqueA:" << std::endl;
        //      MathTools::print(ftA);
        //      std::cout << "ForceTorqueB:" << std::endl;
        //      MathTools::print(ftB);

        forceSep->removeAllChildren();

        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        forceSep->addChild(u);

        Eigen::Vector3f n = ftA.head(3);
        //n = linkInfo.nodeA->toGlobalCoordinateSystemVec(n);
        float l = ftA.head(3).norm();
        float w = 5.0f;
        SoSeparator* forceA = new SoSeparator;
        SoSeparator* arrowForceA =
            CoinVisualizationFactory::CreateArrow(n, l, w, VisualizationFactory::Color::Red());

        /*
        std::cout << "FORCE_A: " << linkInfo.dynNode1->getRigidBody()->getTotalForce()[0] << "," << linkInfo.dynNode1->getRigidBody()->getTotalForce()[1] << "," << linkInfo.dynNode1->getRigidBody()->getTotalForce()[2] << std::endl;
        std::cout << "TORQUEA: " << linkInfo.dynNode1->getRigidBody()->getTotalTorque()[0] << "," << linkInfo.dynNode1->getRigidBody()->getTotalTorque()[1] << "," << linkInfo.dynNode1->getRigidBody()->getTotalTorque()[2] << std::endl;
        std::cout << "FORCE_B: " << linkInfo.dynNode2->getRigidBody()->getTotalForce()[0] << "," << linkInfo.dynNode2->getRigidBody()->getTotalForce()[1] << "," << linkInfo.dynNode2->getRigidBody()->getTotalForce()[2] << std::endl;
        std::cout << "TORQUEB: " << linkInfo.dynNode2->getRigidBody()->getTotalTorque()[0] << "," << linkInfo.dynNode2->getRigidBody()->getTotalTorque()[1] << "," << linkInfo.dynNode2->getRigidBody()->getTotalTorque()[2] << std::endl;
        */

        // show as nodeA local coords system
        /*
        Eigen::Matrix4f comCoord = linkInfo.nodeA->getGlobalPose();
        Eigen::Matrix4f comLocal = Eigen::Matrix4f::Identity();
        comLocal.block(0,3,3,1) = linkInfo.nodeA->getCoMLocal();
        comCoord = comCoord * comLocal;
        */
        // show as global coords
        Eigen::Matrix4f comGlobal = Eigen::Matrix4f::Identity();
        comGlobal.block(0, 3, 3, 1) = linkInfo.nodeA->getCoMGlobal();
        SoMatrixTransform* m = CoinVisualizationFactory::getMatrixTransform(comGlobal);
        forceA->addChild(m);
        forceA->addChild(arrowForceA);

        forceSep->addChild(forceA);

        //Eigen::Vector3f jointGlobal = linkInfo.nodeJoint->getGlobalPose().block(0, 3, 3, 1);
        //Eigen::Vector3f comBGlobal = linkInfo.nodeB->getCoMGlobal();

        // force that is applied on objectA by objectB
        //Eigen::Vector3f FBGlobal =  ftA.head(3);
        //Eigen::Vector3f TBGlobal =  ftB.tail(3) ;

        Eigen::VectorXf torqueJointGlobal = bulletRobot->getJointForceTorqueGlobal(
            linkInfo); //= TBGlobal  - (comBGlobal-jointGlobal).cross(FBGlobal) * 0.001;
        //        std::cout << "torqueJointGlobal: " << torqueJointGlobal << std::endl;
        std::stringstream streamFTJoint;
        streamFTJoint.precision(1);
        streamFTJoint << "ForceTorqueJoint: " << std::fixed;

        for (int i = 0; i < 6; ++i)
        {
            streamFTJoint << torqueJointGlobal[i] << ",";
        }

        UI.label_ForceTorqueJoint->setText(QString::fromStdString(streamFTJoint.str()));


        n = ftB.head(3);
        //n = linkInfo.nodeB->toGlobalCoordinateSystemVec(n);
        l = ftB.head(3).norm();
        w = 5.0f;
        SoSeparator* forceB = new SoSeparator;
        SoSeparator* arrowForceB =
            CoinVisualizationFactory::CreateArrow(n, l, w, VisualizationFactory::Color::Red());

        comGlobal = Eigen::Matrix4f::Identity();
        comGlobal.block(0, 3, 3, 1) = linkInfo.nodeB->getCoMGlobal();
        m = CoinVisualizationFactory::getMatrixTransform(comGlobal);
        forceB->addChild(m);
        forceB->addChild(arrowForceB);

        forceSep->addChild(forceB);


        /*
                if (!linkInfo.joint->needsFeedback())
                {
                    linkInfo.joint->enableFeedback(true);
                    btJointFeedback* feedback = new btJointFeedback;
                    feedback->m_appliedForceBodyA = btVector3(0, 0, 0);
                    feedback->m_appliedForceBodyB = btVector3(0, 0, 0);
                    feedback->m_appliedTorqueBodyA = btVector3(0, 0, 0);
                    feedback->m_appliedTorqueBodyB = btVector3(0, 0, 0);
                    linkInfo.joint->setJointFeedback(feedback);
                }
                else
                {
                    btJointFeedback* feedback = linkInfo.joint->getJointFeedback();
                    std::cout << "feedback->m_appliedForceBodyA: " << feedback->m_appliedForceBodyA[0] << "," << feedback->m_appliedForceBodyA[1] << "," << feedback->m_appliedForceBodyA[2] << std::endl;
                    std::cout << "feedback->m_appliedForceBodyB: " << feedback->m_appliedForceBodyB[0] << "," << feedback->m_appliedForceBodyB[1] << "," << feedback->m_appliedForceBodyB[2] << std::endl;
                    std::cout << "feedback->m_appliedTorqueBodyA: " << feedback->m_appliedTorqueBodyA[0] << "," << feedback->m_appliedTorqueBodyA[1] << "," << feedback->m_appliedTorqueBodyA[2] << std::endl;
                    std::cout << "feedback->m_appliedTorqueBodyB: " << feedback->m_appliedTorqueBodyB[0] << "," << feedback->m_appliedTorqueBodyB[1] << "," << feedback->m_appliedTorqueBodyB[2] << std::endl;

                }
                */
    }

    if (rn)
    {
        qMin = QString::number(rn->getJointLimitLo(), 'f', 4);
        qMax = QString::number(rn->getJointLimitHi(), 'f', 4);
        qName = QString("Name: ");
        qName += QString(rn->getName().c_str());
        qJV = QString("Joint value: ");
        tmp = QString::number(rn->getJointValue(), 'f', 3);
        qJV += tmp;
        info += "jv rn:";
        std::string a1 = tmp.toStdString();
        info += a1;

        qJV += QString(" / ");

        if (dynamicsRobot->isNodeActuated(rn))
        {
            tmp = QString::number(dynamicsRobot->getJointAngle(rn), 'f', 3);
        }
        else
        {
            tmp = QString("-");
        }

        qJV += tmp;
        info += ",\tjv bul:";
        a1 = tmp.toStdString();
        info += a1;

        qTarget = QString("Joint target: ");

        if (dynamicsRobot->isNodeActuated(rn))
        {
            tmp = QString::number(dynamicsRobot->getNodeTarget(rn), 'f', 3);
        }
        else
        {
            tmp = QString("-");
        }

        qTarget += tmp;
        info += std::string(",targ:");
        a1 = tmp.toStdString();
        info += a1;

        qVel = QString("Joint velocity: ");

        if (dynamicsRobot->isNodeActuated(rn))
        {
            tmp = QString::number(dynamicsRobot->getJointSpeed(rn), 'f', 3);
        }
        else
        {
            tmp = QString("-");
        }

        qVel += tmp;
        info += ",vel:";
        a1 = tmp.toStdString();
        info += a1;

        qVelTarget = QString("Joint velocity target: ");

        if (dynamicsRobot->isNodeActuated(rn))
        {
            tmp = QString::number(dynamicsRobot->getJointTargetSpeed(rn), 'f', 3);
        }
        else
        {
            tmp = QString("-");
        }

        qVelTarget += tmp;
        info += ",velTarget:";
        a1 = tmp.toStdString();
        info += a1;

        Eigen::Matrix4f gp = rn->getGlobalPose();

        tmp = QString::number(gp(0, 3), 'f', 2);
        info += ",gp:";
        info += tmp.toStdString();

        tmp = QString::number(gp(1, 3), 'f', 2);
        info += "/";
        info += tmp.toStdString();

        tmp = QString::number(gp(2, 3), 'f', 2);
        info += "/";
        info += tmp.toStdString();

        Eigen::Vector3f pos = rn->getGlobalPose().block<3, 1>(0, 3);
        qGP = QString("GlobalPosition (simox): ") + QString::number(pos(0), 'f', 2) + " / " +
              QString::number(pos(1), 'f', 2) + " / " + QString::number(pos(2), 'f', 2);

        auto rpy = VirtualRobot::MathTools::eigen4f2rpy(gp);
        qGPRPY = "GlobalRPY (simox): " + QString::number(rpy(0), 'f', 2) + " / " +
                 QString::number(rpy(1), 'f', 2) + " / " + QString::number(rpy(2), 'f', 2);

        gp = rn->getGlobalPose();
        qVisu = QString("VISU (simox):");
        qVisu += QString::number(gp(0, 3), 'f', 2);
        qVisu += QString("/");
        qVisu += QString::number(gp(1, 3), 'f', 2);
        qVisu += QString("/");
        qVisu += QString::number(gp(2, 3), 'f', 2);

        if (dynamicsRobot->hasDynamicsRobotNode(rn))
        {
            gp = dynamicsRobot->getComGlobal(rn);
        }
        else
        {
            gp = Eigen::Matrix4f::Identity();
        }

        qCom = QString("COM (bullet):");
        qCom += QString::number(gp(0, 3), 'f', 2);
        qCom += QString("/");
        qCom += QString::number(gp(1, 3), 'f', 2);
        qCom += QString("/");
        qCom += QString::number(gp(2, 3), 'f', 2);
    }

    UI.label_TargetMin->setText(qMin);
    UI.label_TargetMax->setText(qMax);
    UI.label_RNName->setText(qName);
    UI.label_RNValue->setText(qJV);
    UI.label_RNTarget->setText(qTarget);
    UI.label_RNVelocity->setText(qVel);
    UI.label_RNVelocityTarget->setText(qVelTarget);
    UI.label_RNPosGP->setText(qGP);
    UI.label_RNRPYGP->setText(qGPRPY);
    UI.label_RNPosVisu->setText(qVisu);
    UI.label_RNPosCom->setText(qCom);
}

void
SimDynamicsWindow::updateRobotInfo()
{
    Eigen::Vector3f com = robot->getCoMGlobal();
    Eigen::Vector3f pos = robot->getGlobalPose().block<3, 1>(0, 3);
    Eigen::Vector3f rpy = VirtualRobot::MathTools::eigen4f2rpy(robot->getGlobalPose());

    UI.label_RobotPos->setText(QString::number(pos(0), 'f', 2) + " / " +
                               QString::number(pos(1), 'f', 2) + " / " +
                               QString::number(pos(2), 'f', 2));
    UI.label_RobotRPY->setText(QString::number(rpy(0), 'f', 2) + " / " +
                               QString::number(rpy(1), 'f', 2) + " / " +
                               QString::number(rpy(2), 'f', 2));
    UI.label_RobotCom->setText(QString::number(com(0), 'f', 2) + " / " +
                               QString::number(com(1), 'f', 2) + " / " +
                               QString::number(com(2), 'f', 2));


    Eigen::Vector3f rpos = robot->getRootNode()->getGlobalPose().block<3, 1>(0, 3);
    Eigen::Vector3f rrpy =
        VirtualRobot::MathTools::eigen4f2rpy(robot->getRootNode()->getGlobalPose());
    UI.label_RootNodePos->setText(QString::number(rpos(0), 'f', 2) + " / " +
                                  QString::number(rpos(1), 'f', 2) + " / " +
                                  QString::number(rpos(2), 'f', 2));
    UI.label_RootNodeRPY->setText(QString::number(rrpy(0), 'f', 2) + " / " +
                                  QString::number(rrpy(1), 'f', 2) + " / " +
                                  QString::number(rrpy(2), 'f', 2));

    Eigen::Vector3f rltpos = robot->getRootNode()->getLocalTransformation().block<3, 1>(0, 3);
    Eigen::Vector3f rltrpy =
        VirtualRobot::MathTools::eigen4f2rpy(robot->getRootNode()->getLocalTransformation());
    UI.label_RootLocalTransfPos->setText(QString::number(rltpos(0), 'f', 2) + " / " +
                                         QString::number(rltpos(1), 'f', 2) + " / " +
                                         QString::number(rltpos(2), 'f', 2));
    UI.label_RootLocalTransfRPY->setText(QString::number(rltrpy(0), 'f', 2) + " / " +
                                         QString::number(rltrpy(1), 'f', 2) + " / " +
                                         QString::number(rltrpy(2), 'f', 2));
}

void
SimDynamicsWindow::jointValueChanged(int n)
{
    /*
    #if 1
        // test: move robot
        float pos1= float(n+100)/201.0f;
        RobotNodePtr rn1 = robot->getRobotNode("Platform");
        DynamicsObjectPtr drn1 = dynamicsRobot->getDynamicsRobotNode(rn1);
        BulletObjectPtr brn1 = boost::dynamic_pointer_cast<BulletObject>(drn1);
        if (brn1)
        {
            Eigen::Vector3f vel;
            vel << pos1*10.0f,0,0;
            brn1->setLinearVelocity(vel);
        }
    #endif
    */
    int j = UI.comboBoxRobotNode->currentIndex();
    RobotNodePtr rn;

    if (j >= 0 && j < (int)robotNodes.size())
    {
        rn = robotNodes[j];
    }

    if (!rn || !dynamicsRobot)
    {
        return;
    }

    float pos = 0;

    float l = rn->getJointLimitHi() - rn->getJointLimitLo();
    pos = float(n + 100) / 201.0f * l + rn->getJointLimitLo();
    dynamicsRobot->actuateNode(rn, pos);
}

void
SimDynamicsWindow::stopCB()
{
    if (timerSensor)
    {
        SoSensorManager* sensor_mgr = SoDB::getSensorManager();
        sensor_mgr->removeTimerSensor(timerSensor);
        delete timerSensor;
        timerSensor = nullptr;
    }

    viewer.reset();
}

void
SimDynamicsWindow::updateContactVisu()
{
    contactsSep->removeAllChildren();
    SoUnits* u = new SoUnits;
    u->units = SoUnits::MILLIMETERS;
    contactsSep->addChild(u);

    if (!UI.checkBoxContacts->isChecked())
    {
        return;
    }

    std::vector<SimDynamics::DynamicsEngine::DynamicsContactInfo> c =
        dynamicsWorld->getEngine()->getContacts();

    for (auto& i : c)
    {
        std::cout << "Contact: " << i.objectAName << " + " << i.objectBName << std::endl;
        SoSeparator* normal = new SoSeparator;
        SoMatrixTransform* m = new SoMatrixTransform;
        SbMatrix ma;
        ma.makeIdentity();
        ma.setTranslate(SbVec3f(i.posGlobalB(0), i.posGlobalB(1), i.posGlobalB(2)));
        m->matrix.setValue(ma);
        normal->addChild(m);
        SoSeparator* n = CoinVisualizationFactory::CreateArrow(i.normalGlobalB, 50.0f);

        if (n)
        {
            normal->addChild(n);
        }

        SoSeparator* normal2 = new SoSeparator;
        SoMatrixTransform* m2 = new SoMatrixTransform;
        ma.makeIdentity();
        ma.setTranslate(SbVec3f(i.posGlobalA(0), i.posGlobalA(1), i.posGlobalA(2)));
        m2->matrix.setValue(ma);
        normal2->addChild(m2);
        SoSeparator* n2 = CoinVisualizationFactory::CreateArrow(-i.normalGlobalB, 50.0f);

        if (n2)
        {
            normal2->addChild(n2);
        }

        contactsSep->addChild(normal);
        contactsSep->addChild(normal2);
    }
}

void
SimDynamicsWindow::updateComVisu()
{
    if (!robot)
    {
        return;
    }

    std::vector<RobotNodePtr> n = robot->getRobotNodes();
    std::map<VirtualRobot::RobotNodePtr, SoSeparator*>::iterator i = comVisuMap.begin();

    while (i != comVisuMap.end())
    {
        SoSeparator* sep = i->second;
        SoMatrixTransform* m = dynamic_cast<SoMatrixTransform*>(sep->getChild(0));

        if (m)
        {
            Eigen::Matrix4f ma = dynamicsRobot->getComGlobal(i->first);
            ma.block(0, 3, 3, 1) *= 0.001f;
            m->matrix.setValue(CoinVisualizationFactory::getSbMatrix(ma));
        }

        i++;
    }
}

void
SimDynamicsWindow::loadButton()
{
    QString fileName = QFileDialog::getOpenFileName(
        this, tr("Select Robot File"), "", tr("Simox Robot File (*.xml)"));
    std::string f = fileName.toStdString();

    if (RuntimeEnvironment::getDataFileAbsolute(f))
    {
        if (dynamicsRobot)
        {
            viewer->removeVisualization(dynamicsRobot);
            dynamicsWorld->removeRobot(dynamicsRobot);
        }

        dynamicsRobot.reset();
        loadRobot(f);
        buildVisualization();
    }
}

void
SimDynamicsWindow::startStopEngine()
{
    if (viewer->engineRunning())
    {
        UI.pushButtonStartStop->setText(QString("Start Engine"));
        UI.pushButtonStep->setEnabled(true);
        UI.horizontalSliderUpdateTimer->setEnabled(false);
        viewer->stopEngine();
    }
    else
    {
        UI.pushButtonStartStop->setText(QString("Stop Engine"));
        UI.pushButtonStep->setEnabled(false);
        UI.horizontalSliderUpdateTimer->setEnabled(true);
        viewer->startEngine();
    }
}

void
SimDynamicsWindow::stepEngine()
{
    viewer->stepPhysics();
}

void
SimDynamicsWindow::checkBoxFixedTimeStep()
{
    if (UI.checkBoxFixedTimeStep->isChecked())
    {
        UI.horizontalSliderFixedTimeStep->setEnabled(true);
        viewer->setSimModeFixedTimeStep();
    }
    else
    {
        UI.horizontalSliderFixedTimeStep->setEnabled(false);
        viewer->setSimModeRealTime();
    }
}

void
SimDynamicsWindow::fixedTimeStepChanged(int n)
{
    viewer->setBulletSimTimeStepMsec(n);
}

void
SimDynamicsWindow::updateTimerChanged(int n)
{
    viewer->setUpdateInterval(n);
}

void
SimDynamicsWindow::updateAntiAliasing(int n)
{
    viewer->setAntiAliasing(n);
}

void
SimDynamicsWindow::addObject()
{
    ManipulationObjectPtr vitalis;
    std::string vitalisPath = "objects/VitalisWithPrimitives.xml";

    if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(vitalisPath))
    {
        vitalis = ObjectIO::loadManipulationObject(vitalisPath);
    }

    if (vitalis)
    {
        vitalis->setMass(0.5f);
        float x, y, z;
        int counter = 0;

        do
        {
            x = float(rand() % 2000 - 1000);
            y = float(rand() % 2000 - 1000);
            z = float(rand() % 2000 + 100);
            Eigen::Matrix4f gp = vitalis->getGlobalPose();
            gp.block(0, 3, 3, 1) = Eigen::Vector3f(x, y, z);
            vitalis->setGlobalPose(gp);
            counter++;

            if (counter > 100)
            {
                std::cout << "Error, could not find valid pose" << std::endl;
                return;
            }
        } while (robot &&
                 CollisionChecker::getGlobalCollisionChecker()->checkCollision(robot, vitalis));

        SimDynamics::DynamicsObjectPtr dynamicsObjectVitalis =
            dynamicsWorld->CreateDynamicsObject(vitalis);
        dynamicsObjectVitalis->setPosition(Eigen::Vector3f(x, y, z));
        dynamicsObjects.push_back(dynamicsObjectVitalis);
        dynamicsWorld->addObject(dynamicsObjectVitalis);
        buildVisualization();
    }
}

void
SimDynamicsWindow::reloadRobot()
{
    if (RuntimeEnvironment::getDataFileAbsolute(robotFilename))
    {
        if (dynamicsRobot)
        {
            viewer->removeVisualization(dynamicsRobot);
            dynamicsWorld->removeRobot(dynamicsRobot);
        }

        dynamicsRobot.reset();
        loadRobot(robotFilename);
        buildVisualization();
    }
}

void
SimDynamicsWindow::resetPose()
{
    Eigen::Vector3f rpos = robot->getRootNode()->getGlobalPose().block<3, 1>(0, 3);
    Eigen::Vector3f rrpy =
        VirtualRobot::MathTools::eigen4f2rpy(robot->getRootNode()->getGlobalPose());

    UI.spinBox_Pos_X->setValue(rpos[0]);
    UI.spinBox_Pos_Y->setValue(rpos[1]);
    UI.spinBox_Pos_Z->setValue(rpos[2]);

    UI.spinBox_Rot_R->setValue(rrpy[0]);
    UI.spinBox_Rot_P->setValue(rrpy[1]);
    UI.spinBox_Rot_Y->setValue(rrpy[2]);
}

void
SimDynamicsWindow::setPose()
{
    Eigen::Matrix4f pose = VirtualRobot::MathTools::rpy2eigen4f(
        UI.spinBox_Rot_R->value(), UI.spinBox_Rot_P->value(), UI.spinBox_Rot_Y->value());
    pose.block<3, 1>(0, 3) = Eigen::Vector3f(
        UI.spinBox_Pos_X->value(), UI.spinBox_Pos_Y->value(), UI.spinBox_Pos_Z->value());

    dynamicsRobot->setGlobalPose(pose);
}
