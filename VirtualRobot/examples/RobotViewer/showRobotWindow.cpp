
#include "showRobotWindow.h"
#include "DiffIKWidget.h"
#include "SimoxUtility/algorithm/get_map_keys_values.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Import/RobotImporterFactory.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <VirtualRobot/RobotFactory.h>

#include <SimoxUtility/algorithm/string/string_tools.h>

#include <QFileDialog>
#include <Eigen/Geometry>

#include <ctime>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoUnits.h>
#include <sstream>
#include <filesystem>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

showRobotWindow::showRobotWindow(std::string& sRobotFilename)
    : QMainWindow(nullptr)
{
    VR_INFO << " start " << std::endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    //resize(1100, 768);

    useColModel = false;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
    m_sRobotFilename = sRobotFilename;
    sceneSep = new SoSeparator;
    sceneSep->ref();
    /*SoUnits *u = new SoUnits;
    u->units = SoUnits::MILLIMETERS;
    sceneSep->addChild(u);*/
    robotSep = new SoSeparator;
    extraSep = new SoSeparator;
    sceneSep->addChild(extraSep);

    /*SoShapeHints * shapeHints = new SoShapeHints;
    shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
    shapeHints->shapeType = SoShapeHints::UNKNOWN_SHAPE_TYPE;
    sceneSep->addChild(shapeHints);*/
    /*SoLightModel * lightModel = new SoLightModel;
    lightModel->model = SoLightModel::BASE_COLOR;
    sceneSep->addChild(lightModel);*/

    sceneSep->addChild(robotSep);
    ptDistance.sep = new SoSeparator;
    sceneSep->addChild(ptDistance.sep);

    setupUI();

    loadRobot();

    viewer->viewAll();
}


showRobotWindow::~showRobotWindow()
{
    robot.reset();
    sceneSep->unref();
    DiffIKWidget::close();
}

/*
void CShowRobotWindow::saveScreenshot()
{
    static int counter = 0;
    SbString framefile;

    framefile.sprintf("MPL_Render_Frame%06d.png", counter);
    counter++;

    viewer->getSceneManager()->render();
    viewer->getSceneManager()->scheduleRedraw();
    QGLWidget* w = (QGLWidget*)viewer->getGLWidget();

    QImage i = w->grabFrameBuffer();
    bool bRes = i.save(framefile.getString(), "PNG");
    if (bRes)
        std::cout << "wrote image " << counter << std::endl;
    else
        std::cout << "failed writing image " << counter << std::endl;

}*/

void showRobotWindow::setupUI()
{
    UI.setupUi(this);
    //centralWidget()->setLayout(UI.gridLayoutViewer);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));

    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    viewer->setAccumulationBuffer(false);
    viewer->setAntialiasing(true, 4);

    UI.groupBoxDistance->setChecked(false);

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));
    connect(UI.pushButtonReLoad, SIGNAL(clicked()), this, SLOT(reloadRobot()));

    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeHand()));
    connect(UI.ExportVRML20, SIGNAL(clicked()), this, SLOT(exportVRML()));
    connect(UI.ExportXML, SIGNAL(clicked()), this, SLOT(exportXML()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openHand()));
    connect(UI.comboBoxEndEffector, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
    connect(UI.comboBoxEndEffectorPS, SIGNAL(activated(int)), this, SLOT(selectPreshape(int)));

    connect(UI.checkBoxPhysicsCoM, SIGNAL(clicked()), this, SLOT(displayPhysics()));
    connect(UI.checkBoxPhysicsInertia, SIGNAL(clicked()), this, SLOT(displayPhysics()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(rebuildVisualization()));
    connect(UI.comboBoxPrimitiveModel, SIGNAL(activated(int)), this, SLOT(selectPrimitiveModel(int)));
    connect(UI.checkBoxRobotSensors, SIGNAL(clicked()), this, SLOT(showSensors()));
    connect(UI.checkBoxStructure, SIGNAL(clicked()), this, SLOT(robotStructure()));
    UI.checkBoxFullModel->setChecked(true);
    connect(UI.checkBoxFullModel, SIGNAL(clicked()), this, SLOT(robotFullModel()));
    connect(UI.checkBoxRobotCoordSystems, SIGNAL(clicked()), this, SLOT(robotCoordSystems()));
    connect(UI.checkBoxShowCoordSystem, SIGNAL(clicked()), this, SLOT(showCoordSystem()));
    connect(UI.comboBoxRobotNodeSet, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.comboBoxConfiguration, SIGNAL(activated(int)), this, SLOT(selectConfiguration(int)));
    connect(UI.buttonSetConfiguration, SIGNAL(clicked()), this, SLOT(setConfiguration()));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));

    connect(UI.checkBoxDistToPtEnabled,  &QCheckBox::clicked,           this, &showRobotWindow::updatePointDistanceVisu);
    connect(UI.doubleSpinBoxDistancePtX, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &showRobotWindow::updatePointDistanceVisu);
    connect(UI.doubleSpinBoxDistancePtY, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &showRobotWindow::updatePointDistanceVisu);
    connect(UI.doubleSpinBoxDistancePtZ, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &showRobotWindow::updatePointDistanceVisu);

    connect(UI.openDiffIKButton, SIGNAL(clicked()), this, SLOT(openDiffIK()));
}

QString showRobotWindow::formatString(const char* s, float f)
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

void showRobotWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector<float> jv(allRobotNodes.size(), 0.0f);
    robot->setJointValues(allRobotNodes, jv);

    selectJoint(UI.comboBoxJoint->currentIndex());
}

void showRobotWindow::displayTriangles()
{
    QString text1, text2, text3;
    int trisAllFull, trisRNSFull, trisJointFull;
    trisAllFull = trisRNSFull = trisJointFull = 0;
    int trisAllCol, trisRNSCol, trisJointCol;
    trisAllCol = trisRNSCol = trisJointCol = 0;

    if (robot)
    {
        trisAllFull = robot->getNumFaces(false);
        trisAllCol = robot->getNumFaces(true);
        trisRNSFull = trisAllFull;
        trisRNSCol = trisAllCol;
    }

    if (currentRobotNodeSet)
    {
        trisRNSFull = currentRobotNodeSet->getNumFaces(false);
        trisRNSCol = currentRobotNodeSet->getNumFaces(true);
    }

    if (currentRobotNode)
    {
        trisJointFull = currentRobotNode->getNumFaces(false);
        trisJointCol = currentRobotNode->getNumFaces(true);
    }

    UI.labelTriVisuTotal->setText(QString::number(trisAllFull));
    UI.labelTriVisuRNS  ->setText(QString::number(trisRNSFull));
    UI.labelTriVisuJoint->setText(QString::number(trisJointFull));

    UI.labelTriColTotal->setText(QString::number(trisAllCol));
    UI.labelTriColRNS  ->setText(QString::number(trisRNSCol));
    UI.labelTriColJoint->setText(QString::number(trisJointCol));
}

void showRobotWindow::robotFullModel()
{
    if (!robot)
    {
        return;
    }

    bool showFullModel = UI.checkBoxFullModel->checkState() == Qt::Checked;

    if (UI.checkBoxColModel->checkState())
        rebuildVisualization();

    robot->setupVisualization(showFullModel, true);

}

void showRobotWindow::rebuildVisualization()
{
    if (!robot)
    {
        return;
    }

    robotSep->removeAllChildren();

    if (UI.checkBoxColModel->checkState() == Qt::Checked)
    {
        visualization = robot->getVisualization<CoinVisualization>(SceneObject::Collision);
        SoNode* visualisationNode = nullptr;

        if (visualization)
        {
            visualisationNode = visualization->getCoinVisualization();
        }

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
        }
    }

    if (UI.checkBoxFullModel->checkState() == Qt::Checked
        || UI.checkBoxStructure->checkState() == Qt::Checked
        || UI.checkBoxRobotSensors->checkState() == Qt::Checked
        || UI.checkBoxRobotCoordSystems->checkState() == Qt::Checked)
    {
        visualization = robot->getVisualization<CoinVisualization>(SceneObject::Full);
        SoNode* visualisationNode = nullptr;

        if (visualization)
        {
            visualisationNode = visualization->getCoinVisualization();
        }

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
        }
    }

    selectJoint(UI.comboBoxJoint->currentIndex());

    UI.checkBoxStructure->setEnabled(!useColModel);
    UI.checkBoxRobotSensors->setEnabled(!useColModel);
    UI.checkBoxRobotCoordSystems->setEnabled(!useColModel);

}

void showRobotWindow::showSensors()
{
    if (!robot)
    {
        return;
    }

    bool showSensors = UI.checkBoxRobotSensors->isChecked();

    std::vector<SensorPtr> sensors = robot->getSensors();

    for (auto& sensor : sensors)
    {
        sensor->setupVisualization(showSensors, showSensors);
        sensor->showCoordinateSystem(showSensors);
    }

    // rebuild visualization
    rebuildVisualization();
}

void showRobotWindow::displayPhysics()
{
    if (!robot)
    {
        return;
    }

    physicsCoMEnabled = UI.checkBoxPhysicsCoM->checkState() == Qt::Checked;
    physicsInertiaEnabled = UI.checkBoxPhysicsInertia->checkState() == Qt::Checked;
    robot->showPhysicsInformation(physicsCoMEnabled, physicsInertiaEnabled);

    // rebuild visualization
    rebuildVisualization();

}

void showRobotWindow::exportVRML()
{
    if (!robot)
    {
        return;
    }

    QString fi = QFileDialog::getSaveFileName(this, tr("VRML 2.0 File"), QString(), tr("VRML Files (*.wrl)"));
    std::string s = std::string(fi.toLatin1());

    if (s.empty())
    {
        return;
    }
    if (!simox::alg::ends_with(s, ".wrl"))
    {
        s += ".wrl";
    }

    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    // Use currently selected node as origin

    robot->setPropagatingJointValuesEnabled(false);

    robot->setGlobalPoseForRobotNode(robot->getRobotNode(UI.comboBoxJoint->currentText().toStdString()), Eigen::Matrix4f::Identity());
    VR_INFO << "Using selected node " << UI.comboBoxJoint->currentText().toStdString() << " as origin for exported model." << std::endl;

    robot->setPropagatingJointValuesEnabled(true);

    visualization = robot->getVisualization<CoinVisualization>(colModel);
    visualization->exportToVRML2(s);
}

void showRobotWindow::exportXML()
{
    if (!robot)
    {
        return;
    }

    // XML
    QString fi = QFileDialog::getSaveFileName(this, tr("xml File"), QString(), tr("xml Files (*.xml)"));
    std::string s = std::string(fi.toLatin1());

    if (!s.empty())
    {

        std::filesystem::path p1(s);
        std::string fn = p1.filename().generic_string();
        std::string fnPath = p1.parent_path().generic_string();
        RobotIO::saveXML(robot, fn, fnPath);
    }

}

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/Visualization/VisualizationNode.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
void showRobotWindow::updatePointDistanceVisu()
{
    ptDistance.sep->removeAllChildren();
    if (!UI.checkBoxDistToPtEnabled->isChecked())
    {
        UI.labelDistancePtDist->setText("");
        return;
    }


    const Eigen::Vector3f pt = Eigen::Vector3d
    {
        UI.doubleSpinBoxDistancePtX->value(),
        UI.doubleSpinBoxDistancePtY->value(),
        UI.doubleSpinBoxDistancePtZ->value()
    }.cast<float>();

    CDManager cd;
    // add models to cd
    {
        cd.addCollisionModel(robot->getRobotNodes());
        auto tri = std::make_shared<VirtualRobot::TriMeshModel>();
        auto i1 = tri->addVertex(pt);
        auto i2 = tri->addVertex(pt);
        auto i3 = tri->addVertex(pt);
        tri->addFace(i1, i2, i3);

        cd.addCollisionModel(
            std::make_shared<VirtualRobot::SceneObject>(
                "sphere",
                std::make_shared<VirtualRobot::VisualizationNode>(tri),
                std::make_shared<VirtualRobot::CollisionModel>(tri)
            ));

    }

    float distance;
    //calc and visu
    {
        Eigen::Vector3f pt1;
        Eigen::Vector3f pt2;
        int tri1;
        int tri2;
        distance = cd.getDistance(pt1, pt2, tri1, tri2);
        const Eigen::Vector3f dir = (pt1 - pt2).normalized();
        const float spherSize = 10;
        using Factory = VirtualRobot::CoinVisualizationFactory;
        using Color = VirtualRobot::CoinVisualizationFactory::Color;
        ptDistance.sep->addChild(Factory::CreateArrow(pt2, dir, distance));
        ptDistance.sep->addChild(Factory::CreateSphere(pt, spherSize, Color::Green()));
        ptDistance.sep->addChild(Factory::CreateSphere(pt1, spherSize, Color::Blue()));
        ptDistance.sep->addChild(Factory::CreateSphere(pt2, spherSize, Color(0,1,1)));
    }

    UI.labelDistancePtDist->setText(QString::number(distance));
}

void showRobotWindow::openDiffIK() {
    DiffIKWidget::open(this, sceneSep);
    DiffIKWidget::update(robot);
}

void showRobotWindow::showRobot()
{
    //m_pGraspScenery->showRobot(m_pShowRobot->state() == QCheckBox::On);
}

void showRobotWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}

int showRobotWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

void showRobotWindow::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void showRobotWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (auto& currentRobotNode : currentRobotNodes)
    {
        UI.comboBoxJoint->addItem(QString(currentRobotNode->getName().c_str()));
    }
}

void showRobotWindow::updateRNSBox()
{
    UI.comboBoxRobotNodeSet->clear();
    UI.comboBoxRobotNodeSet->addItem(QString("<All>"));

    for (auto& robotNodeSet : robotNodeSets)
    {
        UI.comboBoxRobotNodeSet->addItem(QString(robotNodeSet->getName().c_str()));
    }
}

void showRobotWindow::selectRNS(int nr)
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

        if (nr >= static_cast<int>(robotNodeSets.size()))
        {
            return;
        }

        currentRobotNodeSet = robotNodeSets[nr];
        currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();
        std::cout << "COM:" << currentRobotNodeSet->getCoM();
        /*cout << "HIGHLIGHTING rns " << currentRobotNodeSet->getName() << std::endl;
        if (visualization)
        {

            robot->highlight(visualization,false);
            currentRobotNodeSet->highlight(visualization,true);
        }*/

    }

    updateJointBox();
    DiffIKWidget::update(robot);
    selectJoint(0);
    displayTriangles();
}

void showRobotWindow::selectJoint(int nr)
{
    if (currentRobotNode)
    {
        currentRobotNode->showBoundingBox(false);
    }

    currentRobotNode.reset();
    std::cout << "Selecting Joint nr " << nr << std::endl;

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    currentRobotNode = currentRobotNodes[nr];
    currentRobotNode->showBoundingBox(true, true);
    currentRobotNode->print();
    float mi = currentRobotNode->getJointLimitLo();
    float ma = currentRobotNode->getJointLimitHi();
    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);
    float j = currentRobotNode->getJointValue();
    UI.lcdNumberJointValue->display(static_cast<double>(j));

    if (std::fabs(ma - mi) > 0 && (currentRobotNode->isJoint()))
    {
        UI.horizontalSliderPos->setEnabled(true);
        int pos = static_cast<int>((j - mi) / (ma - mi) * 1000.0f);
        UI.horizontalSliderPos->setValue(pos);
    }
    else
    {
        UI.horizontalSliderPos->setValue(500);
        UI.horizontalSliderPos->setEnabled(false);
    }

    if (currentRobotNodes[nr]->showCoordinateSystemState())
    {
        UI.checkBoxShowCoordSystem->setCheckState(Qt::Checked);
    }
    else
    {
        UI.checkBoxShowCoordSystem->setCheckState(Qt::Unchecked);
    }

    std::cout << "HIGHLIGHTING node " << currentRobotNodes[nr]->getName() << std::endl;

    if (visualization)
    {
        robot->highlight(visualization, false);
        currentRobotNode->highlight(visualization, true);
    }

    displayTriangles();
}


void showRobotWindow::selectConfiguration(int nr)
{
    std::cout << "Selecting configuration nr " << nr << std::endl;

    currentConfiguration = currentConfigurations[nr];
}


void showRobotWindow::setConfiguration()
{
    std::cout << "Setting robot to configuration `" << currentConfiguration << "`" << std::endl;

    if(not robot->setToConfiguration(currentConfiguration))
    {
        VR_WARNING << "Failed to set robot to config `" << currentConfiguration << "`!";
    }
}


void showRobotWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= static_cast<int>(currentRobotNodes.size()))
    {
        return;
    }

    float fPos = currentRobotNodes[nr]->getJointLimitLo()
            + static_cast<float>(pos) / 1000.0f
            * (currentRobotNodes[nr]->getJointLimitHi()
               - currentRobotNodes[nr]->getJointLimitLo());
    robot->setJointValue(currentRobotNodes[nr], fPos);
    UI.lcdNumberJointValue->display(static_cast<double>(fPos));

    DiffIKWidget::update(robot);
    updatePointDistanceVisu();
}

void showRobotWindow::showCoordSystem()
{
    float size = 0.75f;
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= static_cast<int>(currentRobotNodes.size()))
    {
        return;
    }

    // first check if robot node has a visualization
    /*VisualizationNodePtr visu = robotNodes[nr]->getVisualization();
    if (!visu)
    {
        // create dummy visu
        SoSeparator *s = new SoSeparator();
        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
        robotNodes[nr]->setVisualization(visualizationNode);
        //visualizationNode->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);

    }*/

    currentRobotNodes[nr]->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);
    // rebuild visualization
    rebuildVisualization();
}



void showRobotWindow::selectRobot()
{
    string supportedExtensions = RobotImporterFactory::getAllExtensions();
    string supported = "Supported Formats, " + supportedExtensions + " (" + supportedExtensions + ")";
    string filter = supported + ";;" + RobotImporterFactory::getAllFileFilters();
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr(filter.c_str()));
    std::string s = m_sRobotFilename = std::string(fi.toLatin1());

    if (!s.empty())
    {
        m_sRobotFilename = s;
        loadRobot();
    }
}
void showRobotWindow::reloadRobot()
{
    if (!m_sRobotFilename.empty())
    {
        loadRobot();
    }
}

void showRobotWindow::testPerformance(RobotPtr robot, RobotNodeSetPtr rns)
{
    int loops = 10000;
    Eigen::VectorXf limitMin(rns->getSize());
    Eigen::VectorXf limitMax(rns->getSize());
    for (size_t i = 0; i < rns->getSize(); i++)
    {
        limitMin[i] = rns->getNode(i)->getJointLimitLo();
        limitMax[i] = rns->getNode(i)->getJointLimitHi();
    }
    Eigen::VectorXf v(rns->getSize());
    //float minV = rn->getJointLimitLo();
    //float maxV = rn->getJointLimitHi();

    clock_t start = clock();
    robot->setupVisualization(true, false);
    robot->setUpdateVisualization(true);
    robot->setUpdateCollisionModel(true);
    robot->setThreadsafe(true);
    for (int i = 0; i < loops; i++)
    {
        for (size_t k = 0; k < rns->getSize(); k++)
        {
            float p = float(rand() % 1000) / 1000.0f;
            v[k] = limitMin[k] + p * (limitMax[k] - limitMin[k]);
        }
        rns->setJointValues(v);
    }
    clock_t end = clock();
    float timeMS = static_cast<float>(end - start) / static_cast<float>(CLOCKS_PER_SEC) * 1000.0f;
    VR_INFO << "Time (visu on, thread on): " << timeMS / static_cast<float>(loops) << std::endl;

    start = clock();
    robot->setupVisualization(false, false);
    robot->setUpdateVisualization(false);
    robot->setUpdateCollisionModel(false);
    robot->setThreadsafe(true);
    for (int i = 0; i < loops; i++)
    {
        /*float v = float(rand() % 1000) / 1000.0f;
        v = minV + v * (maxV - minV);
        rn->setJointValue(v);*/
        for (size_t k = 0; k < rns->getSize(); k++)
        {
            float p = float(rand() % 1000) / 1000.0f;
            v[k] = limitMin[k] + p * (limitMax[k] - limitMin[k]);
        }
        rns->setJointValues(v);
    }
    end = clock();
    timeMS = static_cast<float>(end - start) / static_cast<float>CLOCKS_PER_SEC * 1000.0f;
    VR_INFO << "Time (visu off, thread on): " << timeMS / static_cast<float>(loops) << std::endl;

    start = clock();
    robot->setupVisualization(true, false);
    robot->setUpdateVisualization(true);
    robot->setUpdateCollisionModel(true);
    robot->setThreadsafe(false);
    for (int i = 0; i < loops; i++)
    {
        for (size_t k = 0; k < rns->getSize(); k++)
        {
            float p = float(rand() % 1000) / 1000.0f;
            v[k] = limitMin[k] + p * (limitMax[k] - limitMin[k]);
        }
        rns->setJointValues(v);
    }
    end = clock();
    timeMS = static_cast<float>(end - start) / static_cast<float>CLOCKS_PER_SEC * 1000.0f;
    VR_INFO << "Time (visu on, thread off): " << timeMS / static_cast<float>(loops) << std::endl;


    start = clock();
    robot->setupVisualization(false, false);
    robot->setUpdateVisualization(false);
    robot->setUpdateCollisionModel(false);
    robot->setThreadsafe(false);
    for (int i = 0; i < loops; i++)
    {
        for (size_t k = 0; k < rns->getSize(); k++)
        {
            float p = float(rand() % 1000) / 1000.0f;
            v[k] = limitMin[k] + p * (limitMax[k] - limitMin[k]);
        }
        rns->setJointValues(v);
    }
    end = clock();
    timeMS = static_cast<float>(end - start) / static_cast<float>CLOCKS_PER_SEC * 1000.0f;
    VR_INFO << "Time (visu off, thread off): " << timeMS / static_cast<float>(loops) << std::endl;
}

void showRobotWindow::loadRobot()
{
    UI.checkBoxDistToPtEnabled->setChecked(false);
    robotSep->removeAllChildren();
    std::cout << "Loading Robot from " << m_sRobotFilename << std::endl;
    currentEEF.reset();
    currentRobotNode.reset();
    currentRobotNodes.clear();
    currentRobotNodeSet.reset();
    robot.reset();

    try
    {
        QFileInfo fileInfo(m_sRobotFilename.c_str());
        std::string suffix(fileInfo.suffix().toLatin1());
        RobotImporterFactoryPtr importer = RobotImporterFactory::fromFileExtension(suffix, nullptr);

        if (!importer)
        {
            std::cout << " ERROR while grabbing importer" << std::endl;
            return;
        }

        robot = importer->loadFromFile(m_sRobotFilename, RobotIO::eFull);
    }
    catch (VirtualRobotException& e)
    {
        std::cout << " ERROR while creating robot! Exception:" << std::endl;
        std::cout << e.what() << std::endl;
        return;
    }

    if (!robot)
    {
        std::cout << " ERROR while creating robot! robot is null" << std::endl;
        return;
    }

    updatRobotInfo();
}

void showRobotWindow::updatRobotInfo()
{
    if (!robot)
    {
        return;
    }

    UI.checkBoxColModel->setChecked(false);
    UI.checkBoxFullModel->setChecked(true);
    UI.checkBoxPhysicsCoM->setChecked(false);
    UI.checkBoxPhysicsInertia->setChecked(false);
    UI.checkBoxRobotCoordSystems->setChecked(false);
    UI.checkBoxShowCoordSystem->setChecked(false);
    UI.checkBoxStructure->setChecked(false);

    // get nodes
    robot->getRobotNodes(allRobotNodes);
    robot->getRobotNodeSets(robotNodeSets);
    robot->getEndEffectors(eefs);
    updateEEFBox();
    DiffIKWidget::update(robot);
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

    if (eefs.size() == 0)
    {
        selectEEF(-1);
    }
    else
    {
        selectEEF(0);
    }

    displayTriangles();

    UI.comboBoxPrimitiveModel->clear();
    std::set<std::string> ids;
    robot->getPrimitiveApproximationIDs(ids);
    UI.comboBoxPrimitiveModel->addItem(""); // TODO
    for (const std::string& id : ids)
    {
        UI.comboBoxPrimitiveModel->addItem(QString::fromStdString(id));
    }


    UI.comboBoxConfiguration->clear();
    currentConfigurations = ::simox::alg::get_keys(robot->getConfigurations());

    if(not currentConfigurations.empty())
    {
        currentConfiguration = currentConfigurations.front();
    }
    else
    {
        currentConfiguration.clear();
    }

    // add empty / zero?
    // UI.comboBoxConfiguration->addItem(QString(""));

    for (auto& configurationName : currentConfigurations)
    {
        UI.comboBoxConfiguration->addItem(QString(configurationName.c_str()));
    }


    // build visualization
    rebuildVisualization();
    robotStructure();
    displayPhysics();
    viewer->viewAll();
}

void showRobotWindow::robotStructure()
{
    if (!robot)
    {
        return;
    }

    structureEnabled = UI.checkBoxStructure->checkState() == Qt::Checked;
    robot->showStructure(structureEnabled);
    // rebuild visualization
    rebuildVisualization();
}

void showRobotWindow::robotCoordSystems()
{
    if (!robot)
    {
        return;
    }

    bool robotAllCoordsEnabled = UI.checkBoxRobotCoordSystems->checkState() == Qt::Checked;
    robot->showCoordinateSystems(robotAllCoordsEnabled);
    // rebuild visualization
    rebuildVisualization();
}

void showRobotWindow::closeHand()
{
    if (currentEEF)
    {
        currentEEF->closeActors();
    }
}

void showRobotWindow::openHand()
{
    if (currentEEF)
    {
        currentEEF->openActors();
    }
}


void showRobotWindow::selectPrimitiveModel(int nr)
{
    std::cout << "Selecting primitive model nr " << nr << std::endl;
    std::vector<std::string> ids = { UI.comboBoxPrimitiveModel->itemText(nr).toStdString() };
    robot->setPrimitiveApproximationModel(ids, true);
    rebuildVisualization();
}

void showRobotWindow::selectEEF(int nr)
{
    std::cout << "Selecting EEF nr " << nr << std::endl;

    UI.comboBoxEndEffectorPS->clear();
    currentEEF.reset();

    if (nr < 0 || nr >= static_cast<int>(eefs.size()))
    {
        return;
    }
    currentEEF = eefs[nr];

    std::vector<std::string> ps = currentEEF->getPreshapes();
    UI.comboBoxEndEffectorPS->addItem(QString("none"));
    for (auto& p : ps)
    {
        UI.comboBoxEndEffectorPS->addItem(QString(p.c_str()));
    }
}

void showRobotWindow::selectPreshape(int nr)
{
    std::cout << "Selecting EEF preshape nr " << nr << std::endl;

    if (!currentEEF || nr == 0)
    {
        return;
    }

    nr--; // first entry is "none"

    std::vector<std::string> ps = currentEEF->getPreshapes();
    if (nr < 0 || nr >= (int)ps.size())
    {
        return;
    }

    VirtualRobot::RobotConfigPtr c = currentEEF->getPreshape(ps.at(nr));

    robot->setConfig(c);
}

void showRobotWindow::updateEEFBox()
{
    UI.comboBoxEndEffector->clear();

    for (auto& eef : eefs)
    {
        UI.comboBoxEndEffector->addItem(QString(eef->getName().c_str()));
    }
}
