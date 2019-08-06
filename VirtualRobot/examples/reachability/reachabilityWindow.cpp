
#include "reachabilityWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/Workspace/Manipulability.h"
#include "VirtualRobot/IK/PoseQualityExtendedManipulability.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include <VirtualRobot/RuntimeEnvironment.h>

#include <QFileDialog>
#include <Eigen/Geometry>

#include <ctime>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoSeparator.h>

#include "ui_reachabilityCreate.h"

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;


reachabilityWindow::reachabilityWindow(std::string& sRobotFile, std::string& reachFile, Eigen::Vector3f& axisTCP)
    : QMainWindow(nullptr)
{
    VR_INFO << " start " << endl;

    this->axisTCP = axisTCP;
    robotFile = sRobotFile;
    useColModel = false;
    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotVisuSep = new SoSeparator;
    reachabilityVisuSep = new SoSeparator;

    sceneSep->addChild(robotVisuSep);
    sceneSep->addChild(reachabilityVisuSep);

    setupUI();

    loadRobot();

    if (!reachFile.empty())
    {
        if (RuntimeEnvironment::getDataFileAbsolute(reachFile))
        {
            loadReachFile(reachFile);
        }
    }

    m_pExViewer->viewAll();
}


reachabilityWindow::~reachabilityWindow()
{
    sceneSep->unref();
}


void reachabilityWindow::updateCutAngleSlider()
{
    if (UI.sliderCutMinAngle->value() > UI.sliderCutMaxAngle->value())
    {
        UI.sliderCutMaxAngle->setValue(UI.sliderCutMinAngle->value() + UI.sliderCutMaxAngle->tickInterval());
    }

    reachVisu();
}

void reachabilityWindow::setupUI()
{
    UI.setupUi(this);
    m_pExViewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    m_pExViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    m_pExViewer->setAccumulationBuffer(true);

    m_pExViewer->setAntialiasing(true, 4);

    m_pExViewer->setGLRenderAction(new SoLineHighlightRenderAction);
    m_pExViewer->setTransparencyType(SoGLRenderAction::BLEND);
    m_pExViewer->setFeedbackVisibility(true);
    m_pExViewer->setSceneGraph(sceneSep);
    m_pExViewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));
    connect(UI.pushButtonLoadReachability, SIGNAL(clicked()), this, SLOT(loadReach()));
    connect(UI.pushButtonCreateReachability, SIGNAL(clicked()), this, SLOT(createReach()));
    connect(UI.pushButtonExtendReachability, SIGNAL(clicked()), this, SLOT(extendReach()));
    connect(UI.pushButtonSaveReachability, SIGNAL(clicked()), this, SLOT(saveReach()));
    connect(UI.pushButtonFillHoles, SIGNAL(clicked()), this, SLOT(fillHoles()));
    connect(UI.pushButtonBinarize, SIGNAL(clicked()), this, SLOT(binarize()));
    connect(UI.pushButtonVolume, SIGNAL(clicked()), this, SLOT(computeVolume()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.checkBoxReachabilityVisu, SIGNAL(clicked()), this, SLOT(reachVisu()));
    connect(UI.checkBoxReachabilityCut, SIGNAL(clicked()), this, SLOT(reachVisu()));
    connect(UI.comboBoxRNS, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));

    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));
    connect(UI.sliderCutReach, SIGNAL(sliderReleased()), this, SLOT(reachVisu()));
    connect(UI.sliderCutMinAngle, SIGNAL(sliderReleased()), this, SLOT(updateCutAngleSlider()));
    connect(UI.sliderCutMaxAngle, SIGNAL(sliderReleased()), this, SLOT(updateCutAngleSlider()));


    UI.sliderCutReach->setValue(50);
    UI.sliderCutReach->setEnabled(false);
    UI.sliderCutMinAngle->setValue(0);
    UI.sliderCutMinAngle->setEnabled(false);
    UI.sliderCutMaxAngle->setValue(100);
    UI.sliderCutMaxAngle->setEnabled(false);
    UI.checkBoxReachabilityCut->setEnabled(false);

    m_pExViewer->setAccumulationBuffer(true);
    m_pExViewer->setAntialiasing(true, 4);

}

QString reachabilityWindow::formatString(const char* s, float f)
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


void reachabilityWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector< RobotNodePtr > nodes;
    robot->getRobotNodes(nodes);
    std::vector<float> jv(nodes.size(), 0.0f);
    robot->setJointValues(nodes, jv);
}



void reachabilityWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    buildVisu();
}

void reachabilityWindow::reachVisu()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    reachabilityVisuSep->removeAllChildren();
    SoSeparator* visualisationNode = new SoSeparator;

    if (UI.checkBoxReachabilityVisu->checkState() == Qt::Checked)
    {

        UI.sliderCutMinAngle->setEnabled(true);
        int minAngleDist =  UI.sliderCutMinAngle->maximum() - UI.sliderCutMinAngle->minimum();
        float minAngle = (UI.sliderCutMinAngle->value() - UI.sliderCutMinAngle->minimum()) / static_cast<float>(minAngleDist);
        minAngle = (2.0f * minAngle - 1) * static_cast<float>(M_PI);

        UI.sliderCutMaxAngle->setEnabled(true);
        int maxAngleDist =  UI.sliderCutMaxAngle->maximum() - UI.sliderCutMaxAngle->minimum();
        float maxAngle = (UI.sliderCutMaxAngle->value() - UI.sliderCutMaxAngle->minimum()) /  static_cast<float>(maxAngleDist);
        maxAngle = (2.0f * maxAngle - 1) * static_cast<float>(M_PI);

        UI.checkBoxReachabilityCut->setEnabled(true);
        float heightPercent = 1.0f;
        if (UI.checkBoxReachabilityCut->checkState() == Qt::Checked)
        {
            UI.sliderCutReach->setEnabled(true);
            int dist = UI.sliderCutReach->maximum() - UI.sliderCutReach->minimum();
            float pos = (float)(UI.sliderCutReach->value() - UI.sliderCutReach->minimum()) / (float)dist;
            heightPercent = pos;


            WorkspaceRepresentation::WorkspaceCut2DPtr cutData = reachSpace->createCut(pos,reachSpace->getDiscretizeParameterTranslation(), true);
            VR_INFO << "Slider pos: " << pos  << ", maxEntry:" << reachSpace->getMaxSummedAngleReachablity() << ", cut maxCoeff:" << cutData->entries.maxCoeff() << endl;
            SoNode *reachvisu2 = CoinVisualizationFactory::getCoinVisualization(cutData, VirtualRobot::ColorMap(VirtualRobot::ColorMap::eHot), Eigen::Vector3f::UnitZ(), reachSpace->getMaxSummedAngleReachablity(), minAngle, maxAngle);
            visualisationNode->addChild(reachvisu2);

        }
        else
        {
            UI.sliderCutReach->setEnabled(false);
        }

        Eigen::Vector3f minBB, maxBB;
        reachSpace->getWorkspaceExtends(minBB, maxBB);
        float zDist = maxBB(2) - minBB(2);
        float maxZ =  minBB(2) + heightPercent*zDist - reachSpace->getDiscretizeParameterTranslation();
        SoNode *reachvisu =  CoinVisualizationFactory::getCoinVisualization(reachSpace, VirtualRobot::ColorMap(VirtualRobot::ColorMap::eHot), true, maxZ, minAngle, maxAngle);
        visualisationNode->addChild(reachvisu);

    } else
    {
        UI.sliderCutReach->setEnabled(false);
        UI.sliderCutMinAngle->setEnabled(false);
        UI.sliderCutMaxAngle->setEnabled(false);
        UI.checkBoxReachabilityCut->setEnabled(false);
    }
    if (visualisationNode)
    {
        reachabilityVisuSep->addChild(visualisationNode);
    }
}

void reachabilityWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void reachabilityWindow::buildVisu()
{
    if (!robot)
    {
        return;
    }

    robotVisuSep->removeAllChildren();
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    visualization = robot->getVisualization<CoinVisualization>(colModel);
    SoNode* visualisationNode = nullptr;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        robotVisuSep->addChild(visualisationNode);
    }
}

int reachabilityWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void reachabilityWindow::quit()
{
    std::cout << "reachabilityWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void reachabilityWindow::updateRNSBox()
{
    UI.comboBoxRNS->clear();

    //UI.comboBoxRNS->addItem(QString("<All>"));
    for (auto & robotNodeSet : robotNodeSets)
    {
        UI.comboBoxRNS->addItem(QString(robotNodeSet->getName().c_str()));
    }

    selectRNS(0);
}

void reachabilityWindow::selectRNS(int nr)
{
    currentRobotNodeSet.reset();
    cout << "Selecting RNS nr " << nr << endl;
    std::string tcp = "<not set>";

    if (nr < 0 || nr >= (int)robotNodeSets.size())
    {
        return;
    }

    currentRobotNodeSet = robotNodeSets[nr];
    currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();

    if (currentRobotNodeSet->getTCP())
    {
        tcp = currentRobotNodeSet->getTCP()->getName();
    }

    QString qTCP("TCP: ");
    qTCP += tcp.c_str();
    UI.labelTCP->setText(qTCP);

    updateQualityInfo();

    //updateJointBox();
    //selectJoint(0);
    //displayTriangles();
}

void reachabilityWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (auto & allRobotNode : allRobotNodes)
    {
        UI.comboBoxJoint->addItem(QString(allRobotNode->getName().c_str()));
    }

    selectJoint(0);
}

void reachabilityWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)allRobotNodes.size())
    {
        return;
    }

    float fPos = allRobotNodes[nr]->getJointLimitLo() + (float)pos / 1000.0f * (allRobotNodes[nr]->getJointLimitHi() - allRobotNodes[nr]->getJointLimitLo());
    robot->setJointValue(allRobotNodes[nr], fPos);
    UI.lcdNumberJointValue->display((double)fPos);

    updateQualityInfo();
}


void reachabilityWindow::selectJoint(int nr)
{
    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)allRobotNodes.size())
    {
        return;
    }

    currentRobotNode = allRobotNodes[nr];
    currentRobotNode->print();
    float mi = currentRobotNode->getJointLimitLo();
    float ma = currentRobotNode->getJointLimitHi();
    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);
    float j = currentRobotNode->getJointValue();
    UI.lcdNumberJointValue->display((double)j);

    if (fabs(ma - mi) > 0 && (currentRobotNode->isTranslationalJoint() || currentRobotNode->isRotationalJoint()))
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
    updateQualityInfo();
}

/*
void reachabilityWindow::showCoordSystem()
{
    float size = 0.75f;
    int nr = UI.comboBoxJoint->currentIndex();
    if (nr<0 || nr>=(int)currentRobotNodes.size())
        return;

    // first check if robot node has a visualization


    currentRobotNodes[nr]->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);
    // rebuild visualization
    collisionModel();
}

*/

void reachabilityWindow::selectRobot()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
    robotFile = std::string(fi.toLatin1());
    loadRobot();
}

void reachabilityWindow::loadRobot()
{
    robotVisuSep->removeAllChildren();
    cout << "Loading Scene from " << robotFile << endl;

    try
    {
        robot = RobotIO::loadRobot(robotFile);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while creating robot" << endl;
        cout << e.what();
        return;
    }

    if (!robot)
    {
        cout << " ERROR while creating robot" << endl;
        return;
    }

    // get nodes
    robot->getRobotNodes(allRobotNodes);
    std::vector < VirtualRobot::RobotNodeSetPtr > allRNS;
    robot->getRobotNodeSets(allRNS);
    robotNodeSets.clear();

    for (auto & i : allRNS)
    {
        if (i->isKinematicChain())
        {
            VR_INFO << " RNS <" << i->getName() << "> is a valid kinematic chain" << endl;
            robotNodeSets.push_back(i);
        }
        else
        {
            VR_INFO << " RNS <" << i->getName() << "> is not a valid kinematic chain" << endl;
        }
    }

    updateRNSBox();
    updateJointBox();

    updateQualityInfo();

    // build visualization
    buildVisu();
    m_pExViewer->viewAll();
}
void reachabilityWindow::extendReach()
{
    if (!robot)
    {
        return;
    }

    if (!reachSpace)
    {
        cout << " Please load/create reachability data first..." << endl;
        return;
    }

    int steps = UI.spinBoxExtend->value();

#if 0
    ManipulabilityPtr manipSpace = boost::dynamic_pointer_cast<Manipulability>(reachSpace);
    if (manipSpace && manipSpace->getManipulabilityMeasure())
    {
        manipSpace->getManipulabilityMeasure()->setVerbose(true);
    }
#endif
    //reachSpace->addRandomTCPPoses(steps, 1, true);
    reachSpace->addRandomTCPPoses(steps, QThread::idealThreadCount() < 1 ? 1 : QThread::idealThreadCount(), true);

    reachSpace->print();
    UI.checkBoxReachabilityVisu->setChecked(false);
    UI.sliderCutReach->setEnabled(false);
    UI.sliderCutMinAngle->setEnabled(false);
    UI.sliderCutMaxAngle->setEnabled(false);
    UI.checkBoxReachabilityCut->setEnabled(false);
    reachabilityVisuSep->removeAllChildren();
}

void reachabilityWindow::createReach()
{
    if (!robot || !currentRobotNodeSet)
    {
        return;
    }

    // setup window
    Ui::ReachabilityCreate UICreate;
    QDialog diag;
    UICreate.setupUi(&diag);
    RobotNodePtr baseNode = currentRobotNodeSet->getKinematicRoot();
    RobotNodePtr tcpNode = currentRobotNodeSet->getTCP();
    UICreate.labelRNS->setText(QString("RobotNodeSet: ") + QString(currentRobotNodeSet->getName().c_str()));
    UICreate.labelBaseNode->setText(QString("Base: ") + QString(baseNode->getName().c_str()));
    UICreate.labelTCP->setText(QString("TCP: ") + QString(tcpNode->getName().c_str()));
    ReachabilityPtr reachSpaceTest(new Reachability(robot));
    float minB[6];// = {-1000.0f,-1000.0f,-1000.0f,(float)-M_PI,(float)-M_PI,(float)-M_PI};
    float maxB[6];// ={1000.0f,1000.0f,1000.0f,(float)M_PI,(float)M_PI,(float)M_PI};
    reachSpaceTest->checkForParameters(currentRobotNodeSet, 1000, minB, maxB, baseNode, tcpNode);

    //float ex = currentRobotNodeSet->getMaximumExtension();
    UICreate.doubleSpinBoxMinX->setValue(minB[0]);
    UICreate.doubleSpinBoxMaxX->setValue(maxB[0]);
    UICreate.doubleSpinBoxMinY->setValue(minB[1]);
    UICreate.doubleSpinBoxMaxY->setValue(maxB[1]);
    UICreate.doubleSpinBoxMinZ->setValue(minB[2]);
    UICreate.doubleSpinBoxMaxZ->setValue(maxB[2]);


    std::vector < VirtualRobot::RobotNodeSetPtr > allRNS;
    robot->getRobotNodeSets(allRNS);

    for (auto & i : allRNS)
    {
        UICreate.comboBoxColModelDynamic->addItem(QString(i->getName().c_str()));
        UICreate.comboBoxColModelStatic->addItem(QString(i->getName().c_str()));
    }

    UICreate.comboBoxQualityMeasure->addItem(QString("Reachability"));
    UICreate.comboBoxQualityMeasure->addItem(QString("Manipulability"));
    UICreate.comboBoxQualityMeasure->addItem(QString("Ext. Manipulability"));

    if (diag.exec())
    {
        reachSpace.reset(new Reachability(robot));

        minB[0] = UICreate.doubleSpinBoxMinX->value();
        minB[1] = UICreate.doubleSpinBoxMinY->value();
        minB[2] = UICreate.doubleSpinBoxMinZ->value();
        minB[3] = UICreate.doubleSpinBoxMinRo->value();
        minB[4] = UICreate.doubleSpinBoxMinPi->value();
        minB[5] = UICreate.doubleSpinBoxMinYa->value();
        maxB[0] = UICreate.doubleSpinBoxMaxX->value();
        maxB[1] = UICreate.doubleSpinBoxMaxY->value();
        maxB[2] = UICreate.doubleSpinBoxMaxZ->value();
        maxB[3] = UICreate.doubleSpinBoxMaxRo->value();
        maxB[4] = UICreate.doubleSpinBoxMaxPi->value();
        maxB[5] = UICreate.doubleSpinBoxMaxYa->value();

        SceneObjectSetPtr staticModel;
        SceneObjectSetPtr dynamicModel;

        if (UICreate.checkBoxColDetecion->isChecked())
        {
            std::string staticM = std::string(UICreate.comboBoxColModelStatic->currentText().toLatin1());
            std::string dynM = std::string(UICreate.comboBoxColModelDynamic->currentText().toLatin1());
            staticModel = robot->getRobotNodeSet(staticM);
            dynamicModel = robot->getRobotNodeSet(dynM);
        }

        float discrTr = UICreate.doubleSpinBoxDiscrTrans->value();
        float discrRo = UICreate.doubleSpinBoxDiscrRot->value();

        std::string measure = std::string(UICreate.comboBoxQualityMeasure->currentText().toLatin1());

        if (measure != "Reachability")
        {
            reachSpace.reset(new Manipulability(robot));
            ManipulabilityPtr manipSpace = boost::dynamic_pointer_cast<Manipulability>(reachSpace);
            manipSpace->setMaxManipulability(UICreate.doubleSpinBoxMaxManip->value());
        }

        reachSpace->initialize(currentRobotNodeSet, discrTr, discrRo, minB, maxB, staticModel, dynamicModel, baseNode, tcpNode); //200.0f,0.4f,minB,maxB,staticModel,dynamicModel,baseNode);

        if (measure == "Ext. Manipulability")
        {
            ManipulabilityPtr man = boost::dynamic_pointer_cast<Manipulability>(reachSpace);
            PoseQualityExtendedManipulabilityPtr manMeasure(new PoseQualityExtendedManipulability(currentRobotNodeSet));
            man->setManipulabilityMeasure(manMeasure);
            if (UICreate.checkBoxColDetecion->isChecked() && UICreate.checkBoxSelfDistance->isChecked())
            {
                std::string staticM = std::string(UICreate.comboBoxColModelStatic->currentText().toLatin1());
                std::string dynM = std::string(UICreate.comboBoxColModelDynamic->currentText().toLatin1());
                RobotNodeSetPtr m1 = robot->getRobotNodeSet(staticM);
                RobotNodeSetPtr m2 = robot->getRobotNodeSet(dynM);
                man->initSelfDistanceCheck(m1, m2);
                manMeasure->considerObstacles(true, UICreate.doubleSpinBoxSelfDistA->value(), UICreate.doubleSpinBoxSelfDistB->value());
            }
        }

        reachSpace->print();

        reachSpace->addCurrentTCPPose();
        reachSpace->print();
    }
}

void reachabilityWindow::fillHoles()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    cout << "filling holes of reachability space" << endl;
    int res = reachSpace->fillHoles();
    cout << "Filled " << res << " voxels" << endl;
    reachSpace->print();
}

void reachabilityWindow::binarize()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    cout << "Binarizing reachability space" << endl;
    reachSpace->binarize();
    reachSpace->print();
}

void reachabilityWindow::computeVolume()
{
    if (!reachSpace)
        return;

    VirtualRobot::WorkspaceRepresentation::VolumeInfo vi;
    vi = reachSpace->computeVolumeInformation();

    cout << "Reachability Volume Information:" << endl;
    cout << "Nr 3d Voxels:" << vi.voxelCount3D << endl;
    cout << "Nr filled 3d Voxels:" << vi.filledVoxelCount3D << endl;
    cout << "Nr border 3d Voxels:" << vi.borderVoxelCount3D << endl;
    cout << "Volume per 3d Voxel:" << vi.volumeVoxel3D << " m^3" << endl;
    cout << "Volume of all filled 3d Voxels:" << vi.volumeFilledVoxels3D << " m^3" << endl;
    cout << "Volume of filledVoxels - borderVoxels*0.5:" << vi.volume3D << " m^3" << endl;
}

void reachabilityWindow::saveReach()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    QString fi = QFileDialog::getSaveFileName(this, tr("Save Reachability to File"), QString(), tr("bin Files (*.bin);;all Files (*.*)"));

    if (fi.isEmpty())
    {
        return;
    }

    reachFile = std::string(fi.toLatin1());
    reachSpace->save(reachFile);

}
void reachabilityWindow::loadReachFile(std::string filename)
{
    if (!robot)
    {
        return;
    }

    reachFile = filename;
    bool loadOK = true;

    // try manipulability file
    try
    {
        reachSpace.reset(new Manipulability(robot));
        reachSpace->load(reachFile);
    }
    catch (...)
    {
        loadOK = false;
    }

    if (!loadOK)
    {
        // try reachability file

        loadOK = true;

        try
        {
            reachSpace.reset(new Reachability(robot));
            reachSpace->load(reachFile);
        }
        catch (...)
        {
            loadOK = false;
        }
    }

    if (!loadOK)
    {
        VR_ERROR << "Could not load reach/manip file" << endl;
        reachSpace.reset();
        return;
    }

    reachSpace->print();

    if (reachSpace->getNodeSet())
    {
        cout << "Using RNS: " << reachSpace->getNodeSet()->getName() << endl;

        for (size_t i = 0; i < robotNodeSets.size(); i++)
        {
            cout << "checking " << robotNodeSets[i]->getName() << endl;

            if (robotNodeSets[i] == reachSpace->getNodeSet())
            {
                cout << "Found RNS.." << endl;
                UI.comboBoxRNS->setCurrentIndex(i);
                selectRNS(i);
            }
        }
    }
}

void reachabilityWindow::loadReach()
{
    if (!robot)
    {
        return;
    }

    QString fi = QFileDialog::getOpenFileName(this, tr("Open Reachability File"), QString(), tr("bin Files (*.bin);;all Files (*.*)"));

    if (fi.isEmpty())
    {
        return;
    }

    reachFile = std::string(fi.toLatin1());
    loadReachFile(reachFile);
}


void reachabilityWindow::updateQualityInfo()
{
    if (!currentRobotNodeSet)
        return;



    std::stringstream ss;
    std::stringstream ss2;
    std::stringstream ss3;
    std::stringstream ss4;
    PoseQualityManipulabilityPtr manMeasure(new PoseQualityManipulability(currentRobotNodeSet));
    PoseQualityExtendedManipulabilityPtr extManMeasure(new PoseQualityExtendedManipulability(currentRobotNodeSet));
    float manip = manMeasure->getPoseQuality();
    float extManip = extManMeasure->getPoseQuality();
    ss << "Manipulability: " << manip;
    std::string manipString = ss.str();
    UI.labelManip->setText(manipString.c_str());

    ss2 << "Ext. Manipulability: " << extManip;
    manipString = ss2.str();
    UI.labelExtManip->setText(manipString.c_str());

    float reachManip = 1.0f;
    float poseManip = 1.0f;
    if (reachSpace)
    {
        ManipulabilityPtr p = boost::dynamic_pointer_cast<Manipulability>(reachSpace);
        if (p)
        {
            reachManip = p->getManipulabilityAtPose(p->getTCP()->getGlobalPose());
            poseManip = p->measureCurrentPose();
        }
        else
        {
            reachManip = 0.0f;
        }
    }
    ss3 << "Quality in Reach Data: " << reachManip;
    manipString = ss3.str();
    UI.labelWSData->setText(manipString.c_str());

    ss4 << "Quality at pose: " << poseManip;
    manipString = ss4.str();
    UI.labelPose->setText(manipString.c_str());
}
