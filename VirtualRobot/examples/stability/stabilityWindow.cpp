
#include "stabilityWindow.h"

#include "VirtualRobot/BoundingBox.h"
#include "VirtualRobot/CollisionDetection/CollisionChecker.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/IK/CoMIK.h"
#include "VirtualRobot/Logging.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/XML/RobotIO.h"

#ifdef USE_NLOPT
#include "VirtualRobot/IK/ConstrainedOptimizationIK.h"
#include "VirtualRobot/IK/constraints/CoMConstraint.h"
#endif

#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>

#include <QFileDialog>

#include <Eigen/Geometry>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoSphere.h>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

stabilityWindow::stabilityWindow(std::string& sRobotFile) : QMainWindow(nullptr)
{
    VR_INFO << " start " << std::endl;

    robotFile = sRobotFile;
    useColModel = false;
    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotVisuSep = new SoSeparator;
    sceneSep->addChild(robotVisuSep);

    comVisu = new SoSeparator;
    sceneSep->addChild(comVisu);
    comProjectionVisu = new SoSeparator;
    sceneSep->addChild(comProjectionVisu);
    comTargetVisu = new SoSeparator;
    sceneSep->addChild(comTargetVisu);
    supportVisu = new SoSeparator;
    sceneSep->addChild(supportVisu);

    MathTools::Plane p = MathTools::getFloorPlane();
    sceneSep->addChild(
        CoinVisualizationFactory::CreatePlaneVisualization(p.p, p.n, 10000.0f, 0.0f));

    m_CoMTarget = Eigen::Vector2f::Zero();

    setupUI();

    loadRobot();

    m_pExViewer->viewAll();
}

stabilityWindow::~stabilityWindow()
{
    sceneSep->unref();
}

void
stabilityWindow::setupUI()
{
    UI.setupUi(this);
    m_pExViewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    m_pExViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    m_pExViewer->setAccumulationBuffer(false);

    m_pExViewer->setAntialiasing(true, 4);

    m_pExViewer->setGLRenderAction(new SoLineHighlightRenderAction);
    m_pExViewer->setTransparencyType(SoGLRenderAction::BLEND);
    m_pExViewer->setFeedbackVisibility(true);
    m_pExViewer->setSceneGraph(sceneSep);
    m_pExViewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.checkBoxCoM, SIGNAL(clicked()), this, SLOT(showCoM()));
    connect(UI.checkBoxSupportPolygon, SIGNAL(clicked()), this, SLOT(showSupportPolygon()));
    connect(UI.comboBoxRNS, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));

    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));
    connect(UI.horizontalSliderPos, SIGNAL(sliderReleased()), this, SLOT(showSupportPolygon()));

    connect(UI.sliderX, SIGNAL(valueChanged(int)), this, SLOT(comTargetMovedX(int)));
    connect(UI.sliderY, SIGNAL(valueChanged(int)), this, SLOT(comTargetMovedY(int)));
}

QString
stabilityWindow::formatString(const char* s, float f)
{
    QString str1(s);

    if (f >= 0)
    {
        str1 += " ";
    }

    if (fabs(f) < 1000)
    {
        str1 += " ";
    }

    if (fabs(f) < 100)
    {
        str1 += " ";
    }

    if (fabs(f) < 10)
    {
        str1 += " ";
    }

    QString str1n;
    str1n.setNum(f, 'f', 3);
    str1 = str1 + str1n;
    return str1;
}

void
stabilityWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector<RobotNodePtr> nodes;
    robot->getRobotNodes(nodes);
    std::vector<float> jv(nodes.size(), 0.0f);
    robot->setJointValues(nodes, jv);
}

void
stabilityWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    buildVisu();
}

void
stabilityWindow::showSupportPolygon()
{
    if (!robot)
    {
        return;
    }

    updateSupportVisu();
}

void
stabilityWindow::showCoM()
{
    if (!robot)
    {
        return;
    }

    buildVisu();
}

void
stabilityWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

void
stabilityWindow::buildVisu()
{
    if (!robot)
    {
        return;
    }

    robotVisuSep->removeAllChildren();
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel =
        (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
    visualization = robot->getVisualization(colModel);
    SoNode* visualisationNode = nullptr;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        robotVisuSep->addChild(visualisationNode);
    }

    comVisu->removeAllChildren();
    comProjectionVisu->removeAllChildren();

    if (UI.checkBoxCoM->isChecked())
    {
        SoMatrixTransform* m = new SoMatrixTransform();
        comVisu->addChild(m);
        SoMaterial* material = new SoMaterial;
        material->diffuseColor.setValue(1.0f, 0.2f, 0.2f);
        comVisu->addChild(material);
        SoSphere* s = new SoSphere;
        s->radius.setValue(0.0300f);
        SoCube* c = new SoCube;
        c->width.setValue(0.050f);
        c->height.setValue(0.050f);
        c->depth.setValue(0.050f);
        comVisu->addChild(c);
        comVisu->addChild(s);

        m = new SoMatrixTransform();
        comProjectionVisu->addChild(m);
        material = new SoMaterial;
        material->diffuseColor.setValue(0.2f, 0.2f, 1.0f);
        comProjectionVisu->addChild(material);
        s = new SoSphere;
        s->radius.setValue(0.030f);
        c = new SoCube;
        c->width.setValue(0.050f);
        c->height.setValue(0.050f);
        c->depth.setValue(0.050f);
        comProjectionVisu->addChild(c);
        comProjectionVisu->addChild(s);

        m = new SoMatrixTransform();
        comTargetVisu->addChild(m);
        material = new SoMaterial;
        material->diffuseColor.setValue(0.2f, 0.2f, 0.2f);
        comTargetVisu->addChild(material);
        s = new SoSphere;
        s->radius.setValue(0.0300f);
        c = new SoCube;
        c->width.setValue(0.050f);
        c->height.setValue(0.050f);
        c->depth.setValue(0.050f);
        comTargetVisu->addChild(c);
        comTargetVisu->addChild(s);
        updateCoM();
    }

    updateSupportVisu();

    m_pExViewer->scheduleRedraw();
}

void
stabilityWindow::updateCoM()
{
    if (!comVisu || comVisu->getNumChildren() == 0)
    {
        return;
    }

    // Draw CoM
    Eigen::Matrix4f globalPoseCoM;
    globalPoseCoM.setIdentity();

    if (currentRobotNodeSet)
    {
        globalPoseCoM.block<3, 1>(0, 3) = currentRobotNodeSet->getCoM();
    }
    else if (robot)
    {
        globalPoseCoM.block<3, 1>(0, 3) = robot->getCoMGlobal();
    }

    SoMatrixTransform* m = dynamic_cast<SoMatrixTransform*>(comVisu->getChild(0));

    if (m)
    {
        SbMatrix ma(reinterpret_cast<SbMat*>(globalPoseCoM.data()));
        // mm -> m
        ma[3][0] *= 0.001f;
        ma[3][1] *= 0.001f;
        ma[3][2] *= 0.001f;
        m->matrix.setValue(ma);
    }

    // Draw CoM projection
    if (currentRobotNodeSet && comProjectionVisu && comProjectionVisu->getNumChildren() > 0)
    {
        globalPoseCoM(2, 3) = 0;
        m = dynamic_cast<SoMatrixTransform*>(comProjectionVisu->getChild(0));

        if (m)
        {
            SbMatrix ma(reinterpret_cast<SbMat*>(globalPoseCoM.data()));
            // mm -> m
            ma[3][0] *= 0.001f;
            ma[3][1] *= 0.001f;
            ma[3][2] *= 0.001f;
            m->matrix.setValue(ma);
        }
    }
}

void
stabilityWindow::updateSupportVisu()
{
    supportVisu->removeAllChildren();

    if (UI.checkBoxSupportPolygon->isChecked())
    {
        /*SoMaterial *material = new SoMaterial;
        material->diffuseColor.setValue(1.0f,0.2f,0.2f);
        supportVisu->addChild(material);*/

        MathTools::Plane p = MathTools::getFloorPlane();

        //supportVisu->addChild(CoinVisualizationFactory::CreatePlaneVisualization(p.p,p.n,100000.0f,0.5f));

        std::vector<CollisionModelPtr> colModels = robot->getCollisionModels();
        CollisionCheckerPtr colChecker = CollisionChecker::getGlobalCollisionChecker();
        std::vector<CollisionModelPtr>::iterator i = colModels.begin();

        std::vector<MathTools::ContactPoint> points;

        while (i != colModels.end())
        {
            colChecker->getContacts(p, *i, points, 5.0f);
            i++;
        }

        std::vector<Eigen::Vector2f> points2D;

        //MathTools::Plane plane2(Eigen::Vector3f(0,0,0),Eigen::Vector3f(0,1.0f,0));
        for (auto& point : points)
        {

            Eigen::Vector2f pt2d = MathTools::projectPointToPlane2D(point.p, p);
            points2D.push_back(pt2d);
        }

        MathTools::ConvexHull2DPtr cv = MathTools::createConvexHull2D(points2D);
        SoSeparator* visu = CoinVisualizationFactory::CreateConvexHull2DVisualization(
            cv,
            p,
            VisualizationFactory::Color::Blue(),
            VisualizationFactory::Color::Black(),
            6.0f,
            Eigen::Vector3f(0, 0, 2.0f));
        supportVisu->addChild(visu);
    }
}

void
stabilityWindow::updateRNSBox()
{
    UI.comboBoxRNS->clear();
    UI.comboBoxRNS->addItem(QString("<All>"));

    std::vector<VirtualRobot::RobotNodeSetPtr> allRNS;
    robot->getRobotNodeSets(allRNS);
    robotNodeSets.clear();

    for (auto& i : allRNS)
    {
        //if (allRNS[i]->isKinematicChain())
        //{
        //VR_INFO << " RNS <" << allRNS[i]->getName() << "> is a valid kinematic chain" << std::endl;
        robotNodeSets.push_back(i);
        UI.comboBoxRNS->addItem(QString(i->getName().c_str()));
        /*} else
        {
            VR_INFO << " RNS <" << allRNS[i]->getName() << "> is not a valid kinematic chain" << std::endl;
        }*/
    }


    for (auto& allRobotNode : allRobotNodes)
    {
        allRobotNode->showBoundingBox(false);
    }


    updateJointBox();
}

void
stabilityWindow::selectRNS(int nr)
{
    currentRobotNodeSet.reset();
    std::cout << "Selecting RNS nr " << nr << std::endl;

    if (nr <= 0)
    {
        // all joints
        currentRobotNodes = allRobotNodes;
    }
    else
    {
        nr--;

        if (nr >= (int)robotNodeSets.size())
        {
            return;
        }

        currentRobotNodeSet = robotNodeSets[nr];
        currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();

        // Set CoM target to current CoM position
        Eigen::Vector3f com = currentRobotNodeSet->getCoM();
        UI.sliderX->setValue(com(0));
        UI.sliderY->setValue(com(1));
    }

    updateJointBox();
    selectJoint(0);
    updateCoM();
}

int
stabilityWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

void
stabilityWindow::quit()
{
    std::cout << "stabilityWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void
stabilityWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (auto& currentRobotNode : currentRobotNodes)
    {
        UI.comboBoxJoint->addItem(QString(currentRobotNode->getName().c_str()));
    }

    selectJoint(0);
}

void
stabilityWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    float fPos =
        currentRobotNodes[nr]->getJointLimitLo() +
        (float)pos / 1000.0f *
            (currentRobotNodes[nr]->getJointLimitHi() - currentRobotNodes[nr]->getJointLimitLo());
    robot->setJointValue(currentRobotNodes[nr], fPos);
    UI.lcdNumberJointValue->display((double)fPos);

    updateCoM();

    /*RobotNodePtr p = robot->getRobotNode("Foot2 L");
    if (p)
    {
        BoundingBox bbox = p->getCollisionModel()->getBoundingBox(true);
        supportVisu->addChild(CoinVisualizationFactory::CreateBBoxVisualization(bbox));
    }*/
    // show bbox
    if (currentRobotNodeSet)
    {
        for (unsigned int i = 0; i < currentRobotNodeSet->getSize(); i++)
        {
            currentRobotNodeSet->getNode(i)->showBoundingBox(true, true);
        }
    }
}

void
stabilityWindow::selectJoint(int nr)
{
    currentRobotNode.reset();
    std::cout << "Selecting Joint nr " << nr << std::endl;

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    currentRobotNode = currentRobotNodes[nr];
    currentRobotNode->print();
    float mi = currentRobotNode->getJointLimitLo();
    float ma = currentRobotNode->getJointLimitHi();
    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);
    float j = currentRobotNode->getJointValue();
    UI.lcdNumberJointValue->display((double)j);

    if (fabs(ma - mi) > 0 &&
        (currentRobotNode->isTranslationalJoint() || currentRobotNode->isRotationalJoint()))
    {
        UI.horizontalSliderPos->setEnabled(true);
        int pos = (int)((j - mi) / (ma - mi) * 1000.0f);
        UI.horizontalSliderPos->setValue(pos);
    }
    else
    {
        UI.horizontalSliderPos->setValue(500);
        UI.horizontalSliderPos->setEnabled(false);
    }
}

void
stabilityWindow::selectRobot()
{
    QString fi = QFileDialog::getOpenFileName(
        this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
    robotFile = std::string(fi.toLatin1());
    loadRobot();
}

void
stabilityWindow::loadRobot()
{
    robotVisuSep->removeAllChildren();
    std::cout << "Loading Scene from " << robotFile << std::endl;

    try
    {
        robot = RobotIO::loadRobot(robotFile);
    }
    catch (VirtualRobotException& e)
    {
        std::cout << " ERROR while creating robot" << std::endl;
        std::cout << e.what();
        return;
    }

    if (!robot)
    {
        std::cout << " ERROR while creating robot" << std::endl;
        return;
    }

    // get nodes
    robot->getRobotNodes(allRobotNodes);
    updateRNSBox();
    selectRNS(0);

    if (allRobotNodes.size() == 0)
    {
        selectJoint(-1);
    }
    else
    {
        selectJoint(0);
    }


    // build visualization
    buildVisu();
    m_pExViewer->viewAll();
}

void
stabilityWindow::performCoMIK()
{
    if (!currentRobotNodeSet)
    {
        return;
    }

#ifdef USE_NLOPT
    ConstrainedOptimizationIKPtr ik(
        new ConstrainedOptimizationIK(robot, currentRobotNodeSet, 0.01));
    CoMConstraintPtr comConstraint(
        new CoMConstraint(robot, currentRobotNodeSet, currentRobotNodeSet, m_CoMTarget, 5.0f));
    ik->addConstraint(comConstraint);
    ik->initialize();

    if (!ik->solve())
    {
        std::cout << "IK solver did not succeed" << std::endl;
    }

#else
    CoMIK comIK(currentRobotNodeSet, currentRobotNodeSet);
    comIK.setGoal(m_CoMTarget);

    if (!comIK.solveIK(0.3f, 0, 20))
    {
        std::cout << "IK solver did not succeed" << std::endl;
    }
#endif

    updateCoM();
}

void
stabilityWindow::comTargetMovedX(int value)
{
    if (!currentRobotNodeSet)
    {
        return;
    }

    Eigen::Matrix4f T;
    T.setIdentity();

    m_CoMTarget(0) = value;
    T.block(0, 3, 2, 1) = m_CoMTarget;

    if (comTargetVisu && comTargetVisu->getNumChildren() > 0)
    {
        SoMatrixTransform* m = dynamic_cast<SoMatrixTransform*>(comTargetVisu->getChild(0));

        if (m)
        {
            SbMatrix ma(reinterpret_cast<SbMat*>(T.data()));
            // mm -> m
            ma[3][0] *= 0.001f;
            ma[3][1] *= 0.001f;
            ma[3][2] *= 0.001f;
            m->matrix.setValue(ma);
        }
    }

    performCoMIK();
}

void
stabilityWindow::comTargetMovedY(int value)
{
    Eigen::Matrix4f T;
    T.setIdentity();

    m_CoMTarget(1) = value;
    T.block(0, 3, 2, 1) = m_CoMTarget;

    if (comTargetVisu && comTargetVisu->getNumChildren() > 0)
    {
        SoMatrixTransform* m = dynamic_cast<SoMatrixTransform*>(comTargetVisu->getChild(0));

        if (m)
        {
            SbMatrix ma(reinterpret_cast<SbMat*>(T.data()));
            // mm -> m
            ma[3][0] *= 0.001f;
            ma[3][1] *= 0.001f;
            ma[3][2] *= 0.001f;
            m->matrix.setValue(ma);
        }
    }

    performCoMIK();
}
