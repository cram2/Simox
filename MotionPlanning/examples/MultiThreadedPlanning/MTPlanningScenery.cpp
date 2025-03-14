#include "MTPlanningScenery.h"

#include <sstream>

#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include "VirtualRobot/Obstacle.h"
#include "VirtualRobot/RobotNodeSet.h"
#include "VirtualRobot/RuntimeEnvironment.h"
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <MotionPlanning/Planner/Rrt.h>
#include <MotionPlanning/PostProcessing/ShortcutProcessor.h>
#include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
using namespace std;
using namespace VirtualRobot;
using namespace Saba;

MTPlanningScenery::MTPlanningScenery()
{

    robotModelVisuColModel = true;

    sceneSep = new SoSeparator();
    sceneSep->ref();
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    sceneSep->addChild(u);
    addBBCube(sceneSep);

    colModel = "colModel";
    kinChainName = "All";
    TCPName = "Visu";

    obstSep = nullptr;

    robotFilename = "robots/examples/MultiThreadedPlanning/CartMover.xml";
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robotFilename);

    plannersStarted = false;
    optimizeStarted = false;
    startEndVisu = nullptr;

    buildScene();
}

MTPlanningScenery::~MTPlanningScenery()
{
    startPositions.clear();
    goalPositions.clear();

    sceneSep->unref();
}

void
MTPlanningScenery::reset()
{
    for (auto& robot : robots)
    {
        robot.reset();
    }

    robots.clear();

    if (plannersStarted)
    {
        stopPlanning();
    }

    if (optimizeStarted)
    {
        stopOptimizing();
    }

    for (auto& planner : planners)
    {
        planner.reset();
    }

    planners.clear();

    for (auto& CSpace : CSpaces)
    {
        CSpace.reset();
    }

    CSpaces.clear();

    for (auto& planningThread : planningThreads)
    {
        planningThread.reset();
    }

    planningThreads.clear();

    for (auto& optimizeThread : optimizeThreads)
    {
        optimizeThread.reset();
    }

    optimizeThreads.clear();

    solutions.clear();
    optiSolutions.clear();

    for (auto& visualisation : visualisations)
    {
        if (visualisation != nullptr)
        {
            sceneSep->removeChild(visualisation);
        }
    }

    visualisations.clear();

    if (startEndVisu != nullptr)
    {
        sceneSep->removeChild(startEndVisu);
    }

    startEndVisu = nullptr;


    /////////////////////////////////////////////////////////////////////////////
    // Sequential Planing
    /*if(robot != NULL)
    {
        delete robot;
        robot = NULL;
    }

    if(CSpace1 != NULL)
    {
        delete CSpace1;
        CSpace1 = NULL;
    }

    if(CSpace2 != NULL)
    {
        delete CSpace2;
        CSpace2 = NULL;
    }

    for(int i=0; i<(int)m_vBiPlannersForSeq.size(); i++)
    {
        delete m_vBiPlannersForSeq[i];
    }
    m_vBiPlannersForSeq.clear();

    for (int i=0; i<(int)visualizationsForSeq.size(); i++)
    {
        if (visualizationsForSeq[i]!=NULL)
            sceneSep->removeChild(visualizationsForSeq[i]);
    }
    visualizationsForSeq.clear();

    solutionsForSeq.clear();
    optiSolutionsForSeq.clear();*/
}

void
MTPlanningScenery::buildScene()
{
    if (obstSep)
    {
        sceneSep->removeChild(obstSep);
        obstSep = nullptr;
    }

    float fCubeSize = 50.0f;
    float fPlayfieldSize = 1000.0f - fCubeSize;
    environment.reset(new VirtualRobot::SceneObjectSet("Environment"));

    obstSep = new SoSeparator();

    sceneSep->addChild(obstSep);

    int ob = 2000;
    std::cout << "Randomly placing " << ob << " obstacles..." << std::endl;

    for (int i = 0; i < ob; i++)
    {
        VirtualRobot::ObstaclePtr o =
            VirtualRobot::Obstacle::createBox(fCubeSize, fCubeSize, fCubeSize);

        Eigen::Vector3f p;
        p(0) = (float)(rand() % (2 * (int)fPlayfieldSize) - (int)fPlayfieldSize);
        p(1) = (float)(rand() % (2 * (int)fPlayfieldSize) - (int)fPlayfieldSize);
        p(2) = (float)(rand() % (2 * (int)fPlayfieldSize) - (int)fPlayfieldSize);
        Eigen::Matrix4f m;
        m.setIdentity();
        m.block(0, 3, 3, 1) = p;
        o->setGlobalPose(m);
        environment->addSceneObject(o);
        std::shared_ptr<CoinVisualization> visualization = o->getVisualization<CoinVisualization>();
        SoNode* visualisationNode = nullptr;

        if (visualization)
        {
            visualisationNode = visualization->getCoinVisualization();
        }

        obstSep->addChild(visualisationNode);
    }

    environmentUnited = environment->createStaticObstacle("MultiThreadedObstacle");
}

void
MTPlanningScenery::getRandomPos(float& x, float& y, float& z)
{
    float fPlayfieldSize = 1000.0f;

    if (rand() % 2 == 0)
    {
        x = -fPlayfieldSize;
    }
    else
    {
        x = fPlayfieldSize;
    }

    y = (float)(rand() % (2 * (int)fPlayfieldSize) - (int)fPlayfieldSize);
    z = (float)(rand() % (2 * (int)fPlayfieldSize) - (int)fPlayfieldSize);
    int i = rand() % 3;

    if (i == 0)
    {
        float tt = x;
        x = y;
        y = tt;
    }
    else if (i == 1)
    {
        float tt = x;
        x = z;
        z = tt;
    }
}

void
MTPlanningScenery::buildPlanningThread(bool bMultiCollisionCheckers, int id)
{
    if (!environmentUnited)
    {
        std::cout << "Build Environment first!..." << std::endl;
        return;
    }

    if (plannersStarted)
    {
        return;
    }

    std::cout << " Build planning thread ";

    if (bMultiCollisionCheckers)
    {
        std::cout << "with own instance of collision checker" << std::endl;
    }
    else
    {
        std::cout << "with collision checker singleton" << std::endl;
    }

    this->loadRobotMTPlanning(bMultiCollisionCheckers);

    if (this->robots.empty())
    {
        std::cout << "Could not load a robot!..." << std::endl;
        return;
    }

    RobotPtr pRobot = robots[robots.size() - 1];
    RobotNodeSetPtr kinChain = pRobot->getRobotNodeSet(kinChainName);

    CDManagerPtr pCcm(new VirtualRobot::CDManager(pRobot->getCollisionChecker()));
    std::cout << "Set CSpace for " << robots.size() << ".th robot." << std::endl;
    pCcm->addCollisionModel(pRobot->getRobotNodeSet(colModel));
    ObstaclePtr pEnv = environmentUnited;

    //SceneObjectSetPtr pEnv = environment;
    if (bMultiCollisionCheckers)
    {
        //clone environment
        pEnv = environmentUnited->clone("Cloned Environment", pRobot->getCollisionChecker());
    }

    pCcm->addCollisionModel(pEnv);
    CSpaceSampledPtr pCSpace(new CSpaceSampled(pRobot, pCcm, kinChain));

    if (!bMultiCollisionCheckers)
    {
        pCSpace->exclusiveRobotAccess(
            true); // only needed, when one collision checker is shared between the threads
    }

    pCSpace->setSamplingSizeDCD(1.0f);
    pCSpace->setSamplingSize(20.0f);
    BiRrtPtr pPlanner(new BiRrt(pCSpace));

    //setup random start and goal
    float x, y, z;
    Eigen::VectorXf start(3);
    Eigen::VectorXf goal(3);
    pRobot->setUpdateVisualization(false);

    do
    {
        getRandomPos(x, y, z);

        start[0] = x;
        start[1] = y;
        start[2] = z;
        std::cout << "START: " << x << "," << y << "," << z << std::endl;
        pRobot->setJointValues(kinChain, start);
    } while (pCcm->isInCollision());

    startPositions.push_back(start);

    do
    {
        getRandomPos(x, y, z);
        goal[0] = x;
        goal[1] = y;
        goal[2] = z;
        std::cout << "GOAL: " << x << "," << y << "," << z << std::endl;
        pRobot->setJointValues(kinChain, goal);
    } while (pCcm->isInCollision());


    goalPositions.push_back(goal);

    pRobot->setUpdateVisualization(true);

    pPlanner->setStart(start);
    pPlanner->setGoal(goal);
    PlanningThreadPtr pThread(new PlanningThread(pPlanner));
    planners.push_back(pPlanner);
    CSpaces.push_back(pCSpace);
    planningThreads.push_back(pThread);
    solutions.push_back(CSpacePathPtr());
    optiSolutions.push_back(CSpacePathPtr());
    optimizeThreads.push_back(PathProcessingThreadPtr());
    visualisations.push_back(nullptr);

    if (startEndVisu == nullptr)
    {
        startEndVisu = new SoSeparator();
        sceneSep->addChild(startEndVisu);
    }

    SoSphere* s = new SoSphere();
    s->radius = 30.0f;
    SoMaterial* mat = new SoMaterial();
    mat->ambientColor.setValue(1.0, 0, 0);
    mat->diffuseColor.setValue(1.0, 0, 0);
    SoMaterial* mat2 = new SoMaterial();
    mat2->ambientColor.setValue(0, 0, 1.0);
    mat2->diffuseColor.setValue(0, 0, 1.0);

    RobotNodePtr rn = pRobot->getRobotNode(TCPName);

    if (!rn)
    {
        return;
    }

    pRobot->setJointValues(kinChain, start);
    Eigen::Matrix4f gp = rn->getGlobalPose();
    SoMatrixTransform* mt = CoinVisualizationFactory::getMatrixTransform(
        gp); //no transformation -> our scene is already in MM units
    pRobot->setJointValues(kinChain, goal);
    gp = rn->getGlobalPose();
    SoMatrixTransform* mt2 = CoinVisualizationFactory::getMatrixTransform(
        gp); //no transformation -> our scene is already in MM units

    SoSeparator* sep1 = new SoSeparator();
    sep1->addChild(mt);
    sep1->addChild(mat);
    sep1->addChild(s);

    SoSeparator* sep1a = new SoSeparator();
    SoBaseColor* bc1 = new SoBaseColor();
    bc1->rgb.setValue(0, 0, 0);
    sep1a->addChild(bc1);
    SoTranslation* tr1 = new SoTranslation();
    tr1->translation.setValue(35, 0, 0);
    sep1a->addChild(tr1);
    SoScale* sc1 = new SoScale();
    sc1->scaleFactor.setValue(10, 10, 10);
    sep1a->addChild(sc1);
    std::stringstream ss;
    ss << "Start_" << id;
    SoSeparator* bb1 = CoinVisualizationFactory::CreateBillboardText(ss.str());
    sep1a->addChild(bb1);
    sep1->addChild(sep1a);
    SoSeparator* sep2 = new SoSeparator();
    sep2->addChild(mt2);
    sep2->addChild(mat2);
    sep2->addChild(s);

    SoSeparator* sep2a = new SoSeparator();
    SoBaseColor* bc = new SoBaseColor();
    bc->rgb.setValue(0, 0, 0);
    sep2a->addChild(bc);
    SoTranslation* tr = new SoTranslation();
    tr->translation.setValue(30, 0, 0);
    sep2a->addChild(tr);
    SoScale* sc2 = new SoScale();
    sc2->scaleFactor.setValue(10, 10, 10);
    sep2a->addChild(sc2);
    std::stringstream ss2;
    ss2 << "Goal_" << id;
    SoSeparator* bb2 = CoinVisualizationFactory::CreateBillboardText(ss2.str());
    sep2a->addChild(bb2);
    sep2->addChild(sep2a);

    startEndVisu->addChild(sep1);
    startEndVisu->addChild(sep2);
}

PathProcessingThreadPtr
MTPlanningScenery::buildOptimizeThread(CSpaceSampledPtr cspace, CSpacePathPtr path)
{

    ShortcutProcessorPtr o(new ShortcutProcessor(path, cspace));
    PathProcessingThreadPtr optiThread(new PathProcessingThread(o));
    return optiThread;
}

void
MTPlanningScenery::stopPlanning()
{
    std::cout << "Stopping " << planningThreads.size() << " planning threads..." << std::endl;

    for (auto& planningThread : planningThreads)
    {
        planningThread->stop();
    }

    for (auto& robot : robots)
    {
        robot->setUpdateVisualization(true);
    }

    std::cout << "... done" << std::endl;
    plannersStarted = false;
}

void
MTPlanningScenery::stopOptimizing()
{
    if (!optimizeStarted)
    {
        std::cout << "Start the optimizing first!..." << std::endl;
        return;
    }

    std::cout << "Stopping " << optimizeThreads.size() << " optimizing threads..." << std::endl;

    for (auto& optimizeThread : optimizeThreads)
    {
        optimizeThread->stop();
    }

    for (auto& robot : robots)
    {
        robot->setUpdateVisualization(true);
    }

    std::cout << "...done" << std::endl;
    optimizeStarted = false;
}

void
MTPlanningScenery::startPlanning()
{
    if (plannersStarted)
    {
        std::cout << "already started!..." << std::endl;
        return;
    }

    std::cout << "Starting " << planningThreads.size() << " planning threads..." << std::endl;

    for (auto& robot : robots)
    {
        robot->setUpdateVisualization(false);
    }

    for (auto& planningThread : planningThreads)
    {
        planningThread->start();
    }

    std::cout << "... done" << std::endl;

    // give thread some time to startup
    //boost::this_thread::sleep(boost::posix_time::milliseconds(750));
    // now we can check if there is a solution (todo: better startup?!)

    plannersStarted = true;
}

void
MTPlanningScenery::startOptimizing()
{
    if (!plannersStarted)
    {
        std::cout << "Plan the solutions first!..." << std::endl;
        return;
    }

    if (CSpaces.empty() || solutions.empty())
    {
        std::cout << "Build planning threads first!..." << std::endl;
        return;
    }

    if (plannersStarted)
    {
        for (auto& planningThread : planningThreads)
        {
            if (planningThread->isRunning())
            {
                std::cout << "Planning is not finish!..." << std::endl;
                return;
            }
        }
    }

    if (optimizeStarted)
    {
        std::cout << "Path processors already started..." << std::endl;
        return;
    }

    for (int i = 0; i < (int)solutions.size(); i++)
    {
        if (solutions[i])
        {
            optimizeThreads[i] = buildOptimizeThread(CSpaces[i], solutions[i]);
            optiSolutions[i].reset();
        }
    }

    int j = 0;

    for (auto& robot : robots)
    {
        robot->setUpdateVisualization(false);
    }

    for (auto& optimizeThread : optimizeThreads)
    {
        if (optimizeThread)
        {
            optimizeThread->start(SHORTEN_LOOP);
            j++;
        }
    }

    std::cout << "... done" << std::endl;
    std::cout << "Starting " << j << " path processing threads..." << std::endl;
    optimizeStarted = true;
}

void
MTPlanningScenery::loadRobotMTPlanning(bool bMultiCollisionCheckers)
{
    CollisionCheckerPtr pColChecker = CollisionChecker::getGlobalCollisionChecker();

    if (bMultiCollisionCheckers)
    {
        pColChecker.reset(new CollisionChecker());
    }

    RobotPtr pRobot;

    if ((int)robots.size() == 0)
    {
        // xml parsing
        pRobot = RobotIO::loadRobot(robotFilename);

        if (!pRobot)
        {
            std::cout << "Error parsing file " << robotFilename << ". Aborting" << std::endl;
            return;
        }

        RobotNodeSetPtr kinChain = pRobot->getRobotNodeSet(kinChainName);

        if (!kinChain)
        {
            std::cout << "No rns " << kinChainName << ". Aborting" << std::endl;
            return;
        }

        // get robot
        std::cout << "Successfully read " << robotFilename << std::endl;
    }
    else
    {
        pRobot = robots[0];
    }

    if (!pRobot)
    {
        std::cout << "error while parsing xml file: no pRobot.." << std::endl;
        return;
    }

    std::string sNewName;
    std::ostringstream oss;
    oss << pRobot->getName() << "_" << robots.size();
    sNewName = oss.str();
    pRobot = pRobot->clone(sNewName, pColChecker);
    robots.push_back(pRobot);

    if ((int)robots.size() == 1)
    {
        std::shared_ptr<CoinVisualization> visualization = robots[0]->getVisualization(
            robotModelVisuColModel ? SceneObject::Full : SceneObject::Collision);
        //SoNode* visualisationNode = NULL;
        robotSep = new SoSeparator();

        if (visualization)
        {
            robotSep->addChild(visualization->getCoinVisualization());
        }

        //sceneSep->addChild(robotSep);
    }

    //colModelRobots.push_back(pRobot->getCollisionModel(colModel));

    int trFull = pRobot->getNumFaces(false);
    int trCol = pRobot->getNumFaces(true);

    std::cout << "Loaded/Cloned robot with " << trFull << "/" << trCol << " number of triangles."
              << std::endl;
    //reset();
    std::cout << "Loaded/Cloned " << (int)robots.size() << " robots..." << std::endl;
}

// constructs a bounding box cube for the rrt and adds it as child to result
void
MTPlanningScenery::addBBCube(SoSeparator* result)
{
    // parameters for cube
    float lineSize = 2.0;
    SoMaterial* materialLine = new SoMaterial();
    materialLine->ambientColor.setValue(0.0, 0.0, 0.0);
    materialLine->diffuseColor.setValue(0.0, 0.0, 0.0);
    SoDrawStyle* lineStyle = new SoDrawStyle();
    lineStyle->lineWidth.setValue(lineSize);
    float x1 = -1000.0f;
    float y1 = -1000.0f;
    float z1 = -1000.0f;
    float x2 = 1000.0f;
    float y2 = 1000.0f;
    float z2 = 1000.0f;

    // construct 4 liensets:
    SoSeparator* s2 = new SoSeparator();
    s2->addChild(lineStyle);
    s2->addChild(materialLine);
    int32_t pointn = 5;
    SbVec3f points[5];
    points[0].setValue(x1, y1, z1);
    points[1].setValue(x2, y1, z1);
    points[2].setValue(x2, y2, z1);
    points[3].setValue(x1, y2, z1);
    points[4].setValue(x1, y1, z1);
    SoCoordinate3* coordinate3 = new SoCoordinate3;
    coordinate3->point.setValues(0, pointn, points);
    s2->addChild(coordinate3);
    SoLineSet* lineSet = new SoLineSet;
    lineSet->numVertices.setValues(0, 1, &pointn);
    s2->addChild(lineSet);
    result->addChild(s2);

    s2 = new SoSeparator();
    s2->addChild(lineStyle);
    s2->addChild(materialLine);
    pointn = 5;
    points[0].setValue(x1, y1, z2);
    points[1].setValue(x2, y1, z2);
    points[2].setValue(x2, y2, z2);
    points[3].setValue(x1, y2, z2);
    points[4].setValue(x1, y1, z2);
    coordinate3 = new SoCoordinate3;
    coordinate3->point.setValues(0, pointn, points);
    s2->addChild(coordinate3);
    lineSet = new SoLineSet;
    lineSet->numVertices.setValues(0, 1, &pointn);
    s2->addChild(lineSet);
    result->addChild(s2);

    s2 = new SoSeparator();
    s2->addChild(lineStyle);
    s2->addChild(materialLine);
    pointn = 5;
    points[0].setValue(x1, y1, z1);
    points[1].setValue(x2, y1, z1);
    points[2].setValue(x2, y1, z2);
    points[3].setValue(x1, y1, z2);
    points[4].setValue(x1, y1, z1);
    coordinate3 = new SoCoordinate3;
    coordinate3->point.setValues(0, pointn, points);
    s2->addChild(coordinate3);
    lineSet = new SoLineSet;
    lineSet->numVertices.setValues(0, 1, &pointn);
    s2->addChild(lineSet);
    result->addChild(s2);

    s2 = new SoSeparator();
    s2->addChild(lineStyle);
    s2->addChild(materialLine);
    pointn = 5;
    points[0].setValue(x1, y2, z1);
    points[1].setValue(x2, y2, z1);
    points[2].setValue(x2, y2, z2);
    points[3].setValue(x1, y2, z2);
    points[4].setValue(x1, y2, z1);
    coordinate3 = new SoCoordinate3;
    coordinate3->point.setValues(0, pointn, points);
    s2->addChild(coordinate3);
    lineSet = new SoLineSet;
    lineSet->numVertices.setValues(0, 1, &pointn);
    s2->addChild(lineSet);
    result->addChild(s2);
}

void
MTPlanningScenery::setRobotModelShape(bool collisionModel)
{
    robotModelVisuColModel = collisionModel;

    if (!sceneSep || !robotSep)
    {
        return;
    }

    // update robotsep
    //sceneSep->removeChild(robotSep);
    if (robots.size() > 0)
    {
        std::shared_ptr<CoinVisualization> visualization = robots[0]->getVisualization(
            robotModelVisuColModel ? SceneObject::Full : SceneObject::Collision);
        //SoNode* visualisationNode = NULL;
        robotSep = new SoSeparator();

        if (visualization)
        {
            robotSep->addChild(visualization->getCoinVisualization());
        }

        //sceneSep->addChild(robotSep);
    }
}

void
MTPlanningScenery::checkPlanningThreads()
{
    if (!plannersStarted)
    {
        return;
    }

    for (unsigned int i = 0; i < planningThreads.size(); i++)
    {
        if (!planningThreads[i]->isRunning())
        {
            if (!solutions[i])
            {
                CSpacePathPtr sol = planners[i]->getSolution();

                if (sol)
                {
                    solutions[i] = sol->clone();
                    std::cout << "fetching solution " << i << std::endl;
                    CoinRrtWorkspaceVisualizationPtr visu(
                        new CoinRrtWorkspaceVisualization(robots[i], CSpaces[i], TCPName));
                    visu->addCSpacePath(solutions[i]);
                    visu->addTree(planners[i]->getTree());
                    visualisations[i] = visu->getCoinVisualization();
                    sceneSep->addChild(visualisations[i]);
                }
                else
                {
                    std::cout << "no solution in thread " << i << std::endl;
                }
            }
        }
    }
}

void
MTPlanningScenery::checkOptimizeThreads()
{
    if (!optimizeStarted)
    {
        return;
    }

    for (int i = 0; i < (int)optimizeThreads.size(); i++)
    {
        if (!optimizeThreads[i]->isRunning())
        {
            CSpacePathPtr pOptiSol = optimizeThreads[i]->getProcessedPath();

            if (pOptiSol)
            {
                if (!optiSolutions[i])
                {
                    std::cout << "fetching solution " << i << std::endl;
                    sceneSep->removeChild(visualisations[i]);
                    optiSolutions[i] = pOptiSol->clone();
                    CoinRrtWorkspaceVisualizationPtr visu(
                        new CoinRrtWorkspaceVisualization(robots[i], CSpaces[i], TCPName));
                    visu->addCSpacePath(optiSolutions[i]);
                    visualisations[i] = visu->getCoinVisualization();
                    sceneSep->addChild(visualisations[i]);
                }
            }
            else
            {
                std::cout << "No optimized solution in thread " << i << std::endl;
                std::cout << "show the original solution" << std::endl;
            }
        }
    }
}

void
MTPlanningScenery::getThreadCount(int& nWorking, int& nIdle)
{
    nWorking = 0;
    nIdle = 0;

    for (auto pThread : planningThreads)
    {
        if (pThread->isRunning())
        {
            nWorking++;
        }
        else
        {
            nIdle++;
        }
    }
}

void
MTPlanningScenery::getOptimizeThreadCount(int& nWorking, int& nIdle)
{
    nWorking = 0;
    nIdle = 0;

    for (auto pOptiThread : optimizeThreads)
    {
        if (pOptiThread && pOptiThread->isRunning())
        {
            nWorking++;
        }
        else
        {
            nIdle++;
        }
    }
}

int
MTPlanningScenery::getThreads()
{
    return (int)(planningThreads.size());
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Sequential Planning Methods
/*void MTPlanningScenery::loadRobotSTPlanning()
{
    if(robot != NULL)
    {
        sceneSep->removeChild(robotSep);
        delete robot;
        robot = NULL;
    }

    // xml parsing
    CXMLRobotParser xmlParser;
    //xmlParser.forceBoundingBox(true);
    if (!xmlParser.parse(robotFilename.c_str()))
    {
        std::cout << "Error parsing file " << robotFilename << ". Aborting" << std::endl;
        return;
    }

    // get robot
    robot = xmlParser.GetRobot();
    if (robot == NULL)
    {
      std::cout << "error while parsing xml file: no pRobot.." << std::endl;
      return;
    }
    std::cout << "Successfully read " << robotFilename << std::endl;

    robotSep = new SoSeparator();
    robot->GetVisualisationGraph(robotSep, robotModelVisuColModel);
    sceneSep->addChild(robotSep);

    m_pColModelRobot = robot->GetCollisionModel(colModel);

    int trFull = robot->GetNumberOfTriangles(false);
    int trCol = robot->GetNumberOfTriangles(true);

    std::cout << "Loaded robot with " << trFull << "/" << trCol << " number of triangles." << std::endl;
    //reset();
}


void MTPlanningScenery::plan(int index)
{
    if(robot == NULL)
    {
        std::cout << "Load a robot first!..." << std::endl;
        return;
    }
    if(environment == NULL)
    {
        std::cout << "Create the Environment first!..." << std::endl;
        return;
    }

    CColCheckManagement *pCcm = new CColCheckManagement();
    pCcm->AddCollisionModel(robot->GetCollisionModel(colModel));
    pCcm->SetEnvironment(environment);
    this->CSpace1 = new CSpaceSampled(robot, pCcm, kinChain);
    CSpace1->SetColCheckSamplingSize(1.0f);
    CSpace1->SetSamplingSize(20.0f);
    this->CSpace2 = new CSpaceSampled(robot, pCcm, kinChain);
    CSpace2->SetColCheckSamplingSize(1.0f);
    CSpace2->SetSamplingSize(20.0f);

    CRrtBiPlanner *pBiPlanner = new CRrtBiPlanner(CSpace1,CSpace2,CRrtBiPlanner::RRT_CONNECT_COMPLETE_PATH,CRrtBiPlanner::RRT_CONNECT_COMPLETE_PATH);
    pBiPlanner->SetStart(startPositions[index]);
    pBiPlanner->SetGoal(goalPositions[index]);
    this->m_vBiPlannersForSeq.push_back(pBiPlanner);

    if (startEndVisu == NULL)
    {
        startEndVisu = new SoSeparator();
        sceneSep->addChild(startEndVisu);
    }
    SoSphere *s = new SoSphere();
    s->radius = 30.0f;
    SoMaterial *mat = new SoMaterial();
    mat->ambientColor.setValue(1.0,0,0);
    mat->diffuseColor.setValue(1.0,0,0);
    SoMaterial *mat2 = new SoMaterial();
    mat2->ambientColor.setValue(0,0,1.0);
    mat2->diffuseColor.setValue(0,0,1.0);
    SoMatrixTransform *mt = new SoMatrixTransform();
    SbMatrix m;
    robot->SetConfiguration(startPositions[index], &kinChain);
    CRobotNode *rn = robot->GetNode(TCPName);
    if (rn == NULL)
        return;
    m = *(rn->GetGlobalPose(false));
    //m.setTranslate(SbVec3f(start[0],start[1],start[2]));
    mt->matrix.setValue(m);
    SoMatrixTransform *mt2 = new SoMatrixTransform();
    SbMatrix m2;
    robot->SetConfiguration(goalPositions[index], &kinChain);
    m2 = *(rn->GetGlobalPose(false));
    //m2.setTranslate(SbVec3f(goal[0],goal[1],goal[2]));
    mt2->matrix.setValue(m2);
    SoSeparator *sep1 = new SoSeparator();
    sep1->addChild(mt);
    sep1->addChild(mat);
    sep1->addChild(s);
    SoSeparator *sep2 = new SoSeparator();
    sep2->addChild(mt2);
    sep2->addChild(mat2);
    sep2->addChild(s);
    startEndVisu->addChild(sep1);
    startEndVisu->addChild(sep2);

    bool res = pBiPlanner->Plan();
    if(res)
    {
        std::cout << "successfully!..." << std::endl;
        solutionsForSeq.push_back(pBiPlanner->GetSolution());
        visualizationsForSeq.push_back(NULL);
        showSolution(solutionsForSeq[g_iSolutionIndex], g_iSolutionIndex);
        optiSolutionsForSeq.push_back(NULL);
    }
    else
    {
        std::cout << "MTPlanningScenery::plan::Plan failed..." << std::endl;
    }
    g_iSolutionIndex++;
}


void MTPlanningScenery::optimizeSolution(int solutionIndex)
{
    if((CSpace1 == NULL) || (solutionsForSeq[solutionIndex] == NULL))
    {
        std::cout << "MTPlanningScenery::optimizeSolution::Plan a solution first!..." << std::endl;
        return;
    }

    int iBeforeSize = solutionsForSeq[solutionIndex]->GetPathSize();
    CShortcutOptimizer rrtOpt(solutionsForSeq[solutionIndex],CSpace1);
    optiSolutionsForSeq[solutionIndex] = new CRrtSolution(rrtOpt.ShortenSolutionRandom(SHORTEN_LOOP));
    if(optiSolutionsForSeq[solutionIndex] != NULL)
    {
        showSolution(optiSolutionsForSeq[solutionIndex], solutionIndex);
        int iEndSize = optiSolutionsForSeq[solutionIndex]->GetPathSize();
        std::cout << "MTPlanningScenery::optimizeSolution::New Size: " << iEndSize << " Nodes." << std::endl;
        std::cout << "MTPlanningScenery::optimizeSolution::Kicked " << (iBeforeSize - iEndSize) << " Nodes." << std::endl;
    }
    else
    {
        std::cout << "MTPlanningScenery::optimizeSolution::Optimize failed..." << std::endl;
    }
}

void MTPlanningScenery::showSolution(CRrtSolution *solToShow, int solutionIndex)
{
    if(solToShow == NULL)
    {
        std::cout << "MTPlanningScenery::showSolution::No solution!..." << std::endl;
        return;
    }

    if(visualizationsForSeq[solutionIndex] != NULL)
    {
        sceneSep->removeChild(visualizationsForSeq[solutionIndex]);
    }
    CRrtWSpaceVisualization visu(robot, TCPName);
    visu.AddTree(kinChain, m_vBiPlannersForSeq[solutionIndex]->GetPlanningTree(), solToShow);
    visu.BuildVisualizations(false, true);
    visualizationsForSeq[solutionIndex] = visu.GetTreeVisualisation();
    sceneSep->addChild(this->visualizationsForSeq[solutionIndex]);
}
*/
