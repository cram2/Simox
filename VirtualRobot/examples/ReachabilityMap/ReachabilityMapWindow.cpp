
#include "ReachabilityMapWindow.h"
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Workspace/Manipulability.h>
#include <VirtualRobot/Workspace/WorkspaceGrid.h>
#include <QFileDialog>
#include <Eigen/Geometry>
#include <ctime>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include "VirtualRobot/Workspace/NaturalPosture.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>


#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

//#define ENDLESS

ReachabilityMapWindow::ReachabilityMapWindow(std::string& sRobotFile, std::string& reachFile, std::string& objFile, std::string& eef)
    : QMainWindow(nullptr)
{
    VR_INFO << " start " << std::endl;

    robotFile = sRobotFile;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robotFile);
    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotVisuSep = new SoSeparator;
    robotVisuSep->ref();
    reachabilityVisuSep = new SoSeparator;
    reachabilityVisuSep->ref();
    reachabilityMapVisuSep = new SoSeparator;
    reachabilityMapVisuSep->ref();
    allGraspsVisuSep = new SoSeparator;
    allGraspsVisuSep->ref();
    graspVisuSep = new SoSeparator;
    graspVisuSep->ref();
    objectVisuSep = new SoSeparator;
    objectVisuSep->ref();

    sceneSep->addChild(objectVisuSep);
    sceneSep->addChild(graspVisuSep);
    sceneSep->addChild(reachabilityVisuSep);
    sceneSep->addChild(reachabilityMapVisuSep);

    setupUI();

    loadRobot();

    if (!reachFile.empty())
    {
        if (RuntimeEnvironment::getDataFileAbsolute(reachFile))
        {
            loadReachFile(reachFile);
        }
    }

    if (!objFile.empty())
    {
        if (RuntimeEnvironment::getDataFileAbsolute(objFile))
        {
            loadObjectFile(objFile);
        }
    }

    setupEnvironment();
    updateVisu();

    if (!eef.empty())
    {
        selectEEF(eef);
    }

    viewer->viewAll();
}


ReachabilityMapWindow::~ReachabilityMapWindow()
{
    robotVisuSep->unref();
    reachabilityVisuSep->unref();
    reachabilityMapVisuSep->unref();
    allGraspsVisuSep->unref();
    graspVisuSep->unref();
    objectVisuSep->unref();
    sceneSep->unref();
}


void ReachabilityMapWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(false);

    viewer->setAntialiasing(true, 4);

    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonObjectRandom, SIGNAL(clicked()), this, SLOT(setObjectRandom()));

    connect(UI.checkBoxRobot, SIGNAL(clicked()), this, SLOT(updateVisu()));
    connect(UI.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(updateVisu()));
    connect(UI.checkBoxObject, SIGNAL(clicked()), this, SLOT(updateVisu()));
    connect(UI.checkBoxReachabilityVisu, SIGNAL(clicked()), this, SLOT(updateVisu()));
    connect(UI.checkBoxReachabilityMapVisu, SIGNAL(clicked()), this, SLOT(updateVisu()));

    connect(UI.radioButtonAllGrasps, SIGNAL(clicked()), this, SLOT(selectGrasp()));
    connect(UI.radioButtonOneGrasp, SIGNAL(clicked()), this, SLOT(selectGrasp()));
    connect(UI.comboBoxGrasp, SIGNAL(currentIndexChanged(int)), this, SLOT(selectGrasp()));
    connect(UI.comboBoxEEF, SIGNAL(currentIndexChanged(int)), this, SLOT(selectEEF()));

}

QString ReachabilityMapWindow::formatString(const char* s, float f)
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


void ReachabilityMapWindow::resetSceneryAll()
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

void ReachabilityMapWindow::updateVisu()
{
    if (UI.checkBoxRobot->isChecked())
    {
        if (robotVisuSep->getNumChildren() == 0)
        {
            buildRobotVisu();
        }

        if (sceneSep->findChild(robotVisuSep) < 0)
        {
            sceneSep->addChild(robotVisuSep);
        }
    }
    else
    {
        if (sceneSep->findChild(robotVisuSep) >= 0)
        {
            sceneSep->removeChild(robotVisuSep);
        }
    }

    if (UI.checkBoxGrasps->isChecked())
    {
        if (graspVisuSep->getNumChildren() == 0)
        {
            buildGraspVisu();
        }

        if (sceneSep->findChild(graspVisuSep) < 0)
        {
            sceneSep->addChild(graspVisuSep);
        }
    }
    else
    {
        if (sceneSep->findChild(graspVisuSep) >= 0)
        {
            sceneSep->removeChild(graspVisuSep);
        }
    }

    if (UI.checkBoxObject->isChecked())
    {
        if (objectVisuSep->getNumChildren() == 0)
        {
            buildObjectVisu();
        }

        if (sceneSep->findChild(objectVisuSep) < 0)
        {
            sceneSep->addChild(objectVisuSep);
        }
    }
    else
    {
        if (sceneSep->findChild(objectVisuSep) >= 0)
        {
            sceneSep->removeChild(objectVisuSep);
        }
    }

    if (UI.checkBoxReachabilityVisu->isChecked())
    {
        if (reachabilityVisuSep->getNumChildren() == 0)
        {
            buildReachVisu();
        }

        if (sceneSep->findChild(reachabilityVisuSep) < 0)
        {
            sceneSep->addChild(reachabilityVisuSep);
        }
    }
    else
    {
        if (sceneSep->findChild(reachabilityVisuSep) >= 0)
        {
            sceneSep->removeChild(reachabilityVisuSep);
        }
    }

    if (UI.checkBoxReachabilityMapVisu->isChecked())
    {
        if (reachabilityMapVisuSep->getNumChildren() == 0)
        {
            buildReachGridVisu();
        }

        if (sceneSep->findChild(reachabilityMapVisuSep) < 0)
        {
            sceneSep->addChild(reachabilityMapVisuSep);
        }
    }
    else
    {
        if (sceneSep->findChild(reachabilityMapVisuSep) >= 0)
        {
            sceneSep->removeChild(reachabilityMapVisuSep);
        }
    }
}


void ReachabilityMapWindow::buildReachVisu()
{
    if (!robot || !reachSpace || !graspObject || !eef)
    {
        return;
    }


    reachabilityVisuSep->removeAllChildren();

    GraspSetPtr gs = graspObject->getGraspSet(eef);

    if (!gs || gs->getSize() == 0)
    {
        return;
    }

    Eigen::Matrix4f pose = graspObject->getGlobalPose();

    if (UI.radioButtonOneGrasp->isChecked())
    {
        QString qs(UI.comboBoxGrasp->currentText());
        std::string s(qs.toLatin1());
        GraspPtr g = gs->getGrasp(s);

        if (g)
        {
            pose = g->getTcpPoseGlobal(graspObject->getGlobalPose());
        }
    }

    WorkspaceRepresentation::WorkspaceCut2DPtr cutData = reachSpace->createCut(pose,reachSpace->getDiscretizeParameterTranslation(), false);
    int maxCoeff = cutData->entries.maxCoeff();
    VR_INFO << "Max coeff:" << maxCoeff << std::endl;

    SoNode *visualisationNode = CoinVisualizationFactory::getCoinVisualization(cutData, VirtualRobot::ColorMap(VirtualRobot::ColorMap::eHot), Eigen::Vector3f::UnitZ(), maxCoeff);

    if (visualisationNode)
    {
        if (reachSpace->getBaseNode())
        {
            Eigen::Matrix4f gp = reachSpace->getBaseNode()->getGlobalPose();
            reachabilityVisuSep->addChild(CoinVisualizationFactory::getMatrixTransform(gp));
        }
        reachabilityVisuSep->addChild(visualisationNode);
    }

}

void ReachabilityMapWindow::buildRobotVisu()
{
    robotVisuSep->removeAllChildren();

    if (!robot)
    {
        return;
    }

    std::shared_ptr<VirtualRobot::CoinVisualization> visualization = robot->getVisualization<CoinVisualization>();
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

void ReachabilityMapWindow::buildObjectVisu()
{
    objectVisuSep->removeAllChildren();

    if (!graspObject)
    {
        return;
    }

    std::shared_ptr<VirtualRobot::CoinVisualization> visualization = graspObject->getVisualization<CoinVisualization>();
    SoNode* visualisationNode = nullptr;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        objectVisuSep->addChild(visualisationNode);
    }

    if (environment)
    {
        visualization = environment->getVisualization<CoinVisualization>();

        if (visualization)
        {
            visualisationNode = visualization->getCoinVisualization();
        }

        if (visualisationNode)
        {
            objectVisuSep->addChild(visualisationNode);
        }
    }
}

void ReachabilityMapWindow::buildGraspVisu()
{
    graspVisuSep->removeAllChildren();

    if (!robot || !graspObject || !eef)
    {
        return;
    }

    GraspSetPtr gs = graspObject->getGraspSet(eef);

    if (!gs || gs->getSize() == 0)
    {
        return;
    }

    if (UI.radioButtonOneGrasp->isChecked())
    {
        QString qs(UI.comboBoxGrasp->currentText());
        std::string s(qs.toLatin1());
        GraspPtr g = gs->getGrasp(s);

        if (!g)
        {
            return;
        }

        SoSeparator* v = CoinVisualizationFactory::CreateGraspVisualization(g, eef, graspObject->getGlobalPose());

        if (v)
        {
            graspVisuSep->addChild(v);
        }
    }
    else
    {
        SoSeparator* v = CoinVisualizationFactory::CreateGraspSetVisualization(gs, eef, graspObject->getGlobalPose());

        if (v)
        {
            graspVisuSep->addChild(v);
        }
    }
}

void ReachabilityMapWindow::buildReachGridVisu()
{
    if (!robot || !reachGrid)
    {
        return;
    }

    reachabilityMapVisuSep->removeAllChildren();

    SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachGrid, VirtualRobot::ColorMap::eHot, true);

    if (visualisationNode)
    {
        reachabilityMapVisuSep->addChild(visualisationNode);
    }
}

void ReachabilityMapWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


int ReachabilityMapWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void ReachabilityMapWindow::quit()
{
    std::cout << "ReachabilityMapWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void ReachabilityMapWindow::updateEEFBox()
{
    UI.comboBoxEEF->clear();

    if (!robot)
    {
        selectGrasp();
        return;
    }

    std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();

    for (auto & eef : eefs)
    {
        UI.comboBoxEEF->addItem(QString(eef->getName().c_str()));
    }

    selectEEF(0);
}

void ReachabilityMapWindow::selectGrasp()
{
    if (!grasps)
    {
        return;
    }

    graspVisuSep->removeAllChildren();
    reachabilityMapVisuSep->removeAllChildren();
    reachabilityVisuSep->removeAllChildren();

    if (UI.radioButtonAllGrasps->isChecked())
    {
        buildReachMapAll();
    }
    else
    {
        GraspPtr g = grasps->getGrasp(UI.comboBoxGrasp->currentIndex());
        buildReachMap(g);
    }

    updateVisu();
}


void ReachabilityMapWindow::selectEEF()
{
    selectEEF(UI.comboBoxEEF->currentIndex());
}

void ReachabilityMapWindow::selectEEF(int nr)
{
    eef.reset();
    grasps.reset();
    UI.comboBoxGrasp->clear();
    graspVisuSep->removeAllChildren();

    if (!robot)
    {
        selectGrasp();
        return;
    }

    std::cout << "Selecting EEF nr " << nr << std::endl;

    std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();
    std::string tcp = "<not set>";

    if (nr < 0 || nr >= (int)eefs.size())
    {
        return;
    }

    eef = eefs[nr];

    if (graspObject)
    {
        grasps = graspObject->getGraspSet(eef);

        if (grasps)
        {
            for (unsigned int i = 0; i < grasps->getSize(); i++)
            {
                UI.comboBoxGrasp->addItem(QString(grasps->getGrasp(i)->getName().c_str()));
            }
        }
    }

    selectGrasp();
}

void ReachabilityMapWindow::selectEEF(std::string& eef)
{
    if (!robot)
    {
        return;
    }

    std::vector<EndEffectorPtr> eefs = robot->getEndEffectors();

    for (size_t i = 0; i < eefs.size(); i++)
    {
        if (eefs[i]->getName() == eef)
        {
            selectEEF((int)i);
            UI.comboBoxEEF->setCurrentIndex((int)i);
        }
    }
}

void ReachabilityMapWindow::loadRobot()
{
    robotVisuSep->removeAllChildren();
    std::cout << "Loading robot from " << robotFile << std::endl;

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

    //Eigen::Matrix4f gp = MathTools::rpy2eigen4f(0,0,M_PI/2.0);
    Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
    //gp(0,3) = 3000;
    robot->setGlobalPose(gp);

    updateEEFBox();

    // build visualization
    updateVisu();
    viewer->viewAll();
}

void ReachabilityMapWindow::loadReachFile(std::string filename)
{
    if (!robot)
    {
        return;
    }

    reachFile = filename;
    bool success = false;

    // 1st try to load as manipulability file
    try
    {
        reachSpace.reset(new Manipulability(robot));
        reachSpace->load(reachFile);
        success = true;

        VR_INFO << "Map '" << reachFile << "' loaded as Manipulability map";
    }
    catch (...)
    {
    }

    // 2nd try to load as reachability file
    if (!success)
    {
        try
        {
            reachSpace.reset(new Reachability(robot));
            reachSpace->load(reachFile);
            success = true;

            VR_INFO << "Map '" << reachFile << "' loaded as Reachability map";
        }
        catch (...)
        {
            VR_ERROR << "Coulkd not load reachability file..." << std::endl;
        }
    }

    // 3rd try to load as natural posture file
    if (!success)
    {
        try
        {
            reachSpace.reset(new NaturalPosture(robot));
            reachSpace->load(reachFile);
            success = true;

            VR_INFO << "Map '" << reachFile << "' loaded as natural posture map";
        }
        catch (...)
        {
            VR_ERROR << "Coulkd not load reachability file..." << std::endl;
        }
    }


    reachSpace->print();
    /*if (reachSpace->getNodeSet())
    {
        std::cout << "Using RNS: " << reachSpace->getNodeSet()->getName() << std::endl;
        for (size_t i=0;i<robotNodeSets.size();i++)
        {
            std::cout << "checking " << robotNodeSets[i]->getName() << std::endl;
            if (robotNodeSets[i] == reachSpace->getNodeSet())
            {
                std::cout << "Found RNS.." << std::endl;
                //selectRNS(i);
            }
        }
    }*/

}
void ReachabilityMapWindow::setObjectRandom()
{
    if (graspObject)
    {
        Eigen::Matrix4f gp;
        gp.setIdentity();
        gp(0, 3) = 50.0f + (float)(rand() % 1100);
        gp(1, 3) = -50.0f - (float)(rand() % 720);
        gp(2, 3) = 1030.0f;

        graspObject->setGlobalPose(gp);
        selectGrasp();
    }
}
void ReachabilityMapWindow::setupEnvironment()
{
    std::string objectFile("objects/Table.xml");

    if (!RuntimeEnvironment::getDataFileAbsolute(objectFile))
    {
        VR_ERROR << "No path to " << objectFile << std::endl;
        return;
    }

    try
    {
        environment = ObjectIO::loadManipulationObject(objectFile);
    }
    catch(const VirtualRobotException &e)
    {
        VR_ERROR << "Could not load " << objectFile << std::endl;
        return;
    }

    if (!environment)
    {
        return;
    }

    Eigen::Matrix4f gp;
    gp.setIdentity();
    environment->setGlobalPose(gp);
    setObjectRandom();

}

void ReachabilityMapWindow::loadObjectFile(std::string filename)
{
    if (!robot)
    {
        return;
    }

    objectFile = filename;

    try
    {
        graspObject = ObjectIO::loadManipulationObject(filename);
    }
    catch(const VirtualRobotException &e)
    {
        VR_ERROR << "Could not load " << filename << std::endl;
        return;
    }
}

bool ReachabilityMapWindow::buildReachMapAll()
{
    reachabilityMapVisuSep->removeAllChildren();

    if (!grasps)
    {
        return false;
    }

    Eigen::Vector3f minBB, maxBB;
    reachSpace->getWorkspaceExtends(minBB, maxBB);
    reachGrid.reset(new WorkspaceGrid(minBB(0), maxBB(0), minBB(1), maxBB(1), reachSpace->getDiscretizeParameterTranslation()));

    Eigen::Matrix4f gp = graspObject->getGlobalPose();
    reachGrid->setGridPosition(gp(0, 3), gp(1, 3));

    for (int i = 0; i < (int)grasps->getSize(); i++)
    {
        GraspPtr g = grasps->getGrasp(i);
        reachGrid->fillGridData(reachSpace, graspObject, g, robot->getRootNode());
    }

    updateVisu();
    return true;
}

bool ReachabilityMapWindow::buildReachMap(VirtualRobot::GraspPtr g)
{
    reachabilityMapVisuSep->removeAllChildren();
    Eigen::Vector3f minBB, maxBB;
    reachSpace->getWorkspaceExtends(minBB, maxBB);
    reachGrid.reset(new WorkspaceGrid(minBB(0), maxBB(0), minBB(1), maxBB(1), reachSpace->getDiscretizeParameterTranslation()));

    Eigen::Matrix4f gp = graspObject->getGlobalPose();
    reachGrid->setGridPosition(gp(0, 3), gp(1, 3));

    reachGrid->fillGridData(reachSpace, graspObject, g, robot->getRootNode());

    updateVisu();
    return true;

}
