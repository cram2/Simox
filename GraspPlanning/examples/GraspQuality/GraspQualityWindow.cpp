
#include "GraspQualityWindow.h"
#include "GraspPlanning/Visualization/CoinVisualization/CoinConvexHullVisualization.h"
#include "GraspPlanning/GraspQuality/GraspEvaluationPoseUncertainty.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"

#include <VirtualRobot/Visualization/VisualizationNode.h>

#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include <QFileDialog>
#include <Eigen/Geometry>

#include <ctime>
#include <vector>
#include <iostream>
#include <cmath>



#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

GraspQualityWindow::GraspQualityWindow(std::string& robFile, std::string& objFile)
    : QMainWindow(nullptr)
{
    VR_INFO << " start " << std::endl;

    this->robotFile = robFile;
    this->objectFile = objFile;

    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;
    objectSep = new SoSeparator;
    frictionConeSep = new SoSeparator;
    gws1Sep = new SoSeparator;
    gws2Sep = new SoSeparator;
    ows1Sep = new SoSeparator;
    ows2Sep = new SoSeparator;

    sceneSep->addChild(robotSep);
    sceneSep->addChild(objectSep);
    sceneSep->addChild(frictionConeSep);
    sceneSep->addChild(gws1Sep);
    sceneSep->addChild(gws2Sep);
    sceneSep->addChild(ows1Sep);
    sceneSep->addChild(ows2Sep);

    setupUI();


    loadRobot();
    loadObject();


    m_pExViewer->viewAll();

    SoSensorManager* sensor_mgr = SoDB::getSensorManager();
    SoTimerSensor* timer = new SoTimerSensor(timerCB, this);
    timer->setInterval(SbTime(TIMER_MS / 1000.0f));
    sensor_mgr->insertTimerSensor(timer);
}


GraspQualityWindow::~GraspQualityWindow()
{
    sceneSep->unref();
}


void GraspQualityWindow::timerCB(void* data, SoSensor* /*sensor*/)
{
    GraspQualityWindow* ikWindow = static_cast<GraspQualityWindow*>(data);
    float x[6];
    x[0] = (float)ikWindow->UI.horizontalSliderX->value();
    x[1] = (float)ikWindow->UI.horizontalSliderY->value();
    x[2] = (float)ikWindow->UI.horizontalSliderZ->value();
    x[3] = (float)ikWindow->UI.horizontalSliderRo->value();
    x[4] = (float)ikWindow->UI.horizontalSliderPi->value();
    x[5] = (float)ikWindow->UI.horizontalSliderYa->value();
    x[0] /= 10.0f;
    x[1] /= 10.0f;
    x[2] /= 10.0f;
    x[3] /= 300.0f;
    x[4] /= 300.0f;
    x[5] /= 300.0f;

    if (x[0] != 0 || x[1] != 0 || x[2] != 0 || x[3] != 0 || x[4] != 0 || x[5] != 0)
    {
        ikWindow->updateObject(x);
    }
}


void GraspQualityWindow::setupUI()
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
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonToTCP, SIGNAL(clicked()), this, SLOT(objectToTCP()));
    connect(UI.pushButtonQuality, SIGNAL(clicked()), this, SLOT(graspQuality()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectObject()));
    connect(UI.pushButtonRobustness, SIGNAL(clicked()), this, SLOT(evalRobustness()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxCones, SIGNAL(clicked()), this, SLOT(frictionConeVisu()));
    //connect(UI.checkBoxGWS1, SIGNAL(clicked()), this, SLOT(showGWS()));
    //connect(UI.checkBoxGWS2, SIGNAL(clicked()), this, SLOT(showGWS()));
    //connect(UI.checkBoxOWS1, SIGNAL(clicked()), this, SLOT(showOWS()));
    //connect(UI.checkBoxOWS2, SIGNAL(clicked()), this, SLOT(showOWS()));

    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectX()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectY()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectZ()));
    connect(UI.horizontalSliderRo, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectA()));
    connect(UI.horizontalSliderPi, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectB()));
    connect(UI.horizontalSliderYa, SIGNAL(sliderReleased()), this, SLOT(sliderReleased_ObjectG()));

    connect(UI.comboBoxEEF, SIGNAL(activated(int)), this, SLOT(selectEEF(int)));
    connect(UI.comboBoxGrasp, SIGNAL(activated(int)), this, SLOT(selectGrasp(int)));

    connect(UI.pushButtonEvaluateAll, SIGNAL(clicked()), this, SLOT(evalRobustnessAll()));
}


void GraspQualityWindow::resetSceneryAll()
{
    //if (rns)
    //  rns->setJointValues(startConfig);
}


void GraspQualityWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void GraspQualityWindow::buildVisu()
{

    robotSep->removeAllChildren();
    //bool colModel = (UI.checkBoxColModel->isChecked());
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (robot)
    {
        visualizationRobot = robot->getVisualization(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            //visualizationRobot->highlight(true);
        }
    }

    objectSep->removeAllChildren();

    if (object)
    {
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, colModel);

        if (visualisationNode)
        {
            objectSep->addChild(visualisationNode);
        }
    }

    frictionConeSep->removeAllChildren();
    bool fc = (UI.checkBoxCones->isChecked());

    if (fc && contacts.size() > 0)
    {
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(contacts);

        if (visualisationNode)
        {
            frictionConeSep->addChild(visualisationNode);
        }

    }

    m_pExViewer->scheduleRedraw();
}

int GraspQualityWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void GraspQualityWindow::quit()
{
    std::cout << "GraspQualityWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void GraspQualityWindow::evalRobustness()
{
    int numSamples = UI.sbSamples->value();
    float varMM = UI.sbVariationMM->value();
    float varDeg = UI.sbVariationDeg->value();
    GraspStudio::GraspEvaluationPoseUncertainty::PoseUncertaintyConfig c;
    c.init(varMM, varDeg);
    GraspStudio::GraspEvaluationPoseUncertaintyPtr eval(new GraspStudio::GraspEvaluationPoseUncertainty(c));

    openEEF();
    closeEEF();
    if (contacts.size() == 0)
    {
        return;
    }
    evalPoses = eval->generatePoses(object->getGlobalPose(), contacts, numSamples);


    if (evalPoses.size() == 0)
    {
        return;
    }

    /*
    int r = rand() % evalPoses.size();
    Eigen::Matrix4f p = evalPoses.at(r);
    GraspStudio::GraspEvaluationPoseUncertainty::PoseEvalResult re = eval->evaluatePose(eef, object, p, qualityMeasure);
    std::cout << "FC: " << re.forceClosure << std::endl;
    std::cout << "init col: " << re.initialCollision << std::endl;
    std::cout << "QUAL: " << re.quality << std::endl;
    */
    GraspStudio::GraspEvaluationPoseUncertainty::PoseEvalResults re = eval->evaluatePoses(eef, object, evalPoses, qualityMeasure);
    if (eef && grasp)
    {
        VR_INFO << "#### Robustness for eef " << eef->getName() << ", grasp " << grasp->getName() << std::endl;
    }
    re.print();
}

void GraspQualityWindow::evalRobustnessAll()
{
    int numSamples = UI.sbSamples->value();
    float varMM = UI.sbVariationMM->value();
    float varDeg = UI.sbVariationDeg->value();

    std::vector<GraspSetPtr> sets = object->getAllGraspSets();
    for (GraspSetPtr set : sets)
    {
        VirtualRobot::EndEffectorPtr eef = robot->getEndEffector(set->getEndEffector());
        std::vector<GraspPtr> grasps = set->getGrasps();
        for (GraspPtr grasp : grasps)
        {
            GraspStudio::GraspEvaluationPoseUncertainty::PoseUncertaintyConfig c;
            c.init(varMM, varDeg);
            GraspStudio::GraspEvaluationPoseUncertaintyPtr eval(new GraspStudio::GraspEvaluationPoseUncertainty(c));

            VR_INFO << "Setting object pose to grasp " << grasp->getName() << std::endl;
            Eigen::Matrix4f pos =  eef->getTcp()->getGlobalPose();
            pos = grasp->getObjectTargetPoseGlobal(pos);
            object->setGlobalPose(pos);

            VirtualRobot::EndEffector::ContactInfoVector contacts;
            eef->openActors();
            contacts = eef->closeActors(object);

            VR_INFO << contacts.size() << std::endl;
            if (contacts.size() == 0)
            {
                continue;
            }
            std::vector<Eigen::Matrix4f> evalPoses = eval->generatePoses(object->getGlobalPose(), contacts, numSamples);

            VR_INFO << evalPoses.size() << std::endl;
            if (evalPoses.size() == 0)
            {
                continue;
            }

            GraspStudio::GraspEvaluationPoseUncertainty::PoseEvalResults re = eval->evaluatePoses(eef, object, evalPoses, qualityMeasure);
            if (eef && grasp)
            {
                VR_INFO << "#### Robustness for eef " << eef->getName() << ", grasp " << grasp->getName() << std::endl;
            }
            re.print();

            grasp->setQuality(re.avgQuality);
        }
    }

    ObjectIO::saveManipulationObject(object, objectFile + "_new.xml");
}

void GraspQualityWindow::selectObject()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Object File"), QString(), tr("XML Files (*.xml)"));
    objectFile = std::string(fi.toLatin1());
    loadObject();
}

void GraspQualityWindow::loadObject()
{
    openEEF();

    if (!objectFile.empty())
    {
        try
        {
            object = ObjectIO::loadManipulationObject(objectFile);
        }
        catch (...)
        {
            VR_ERROR << "Could not load file " << objectFile << std::endl;
        }
    }

    if (!object)
    {
        VR_INFO << "Building standard box" << std::endl;
        ObstaclePtr o = Obstacle::createBox(50.0f, 50.0f, 10.0f);
        object = ManipulationObject::createFromMesh(o->getVisualization()->getTriMeshModel());
    }

    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();

    selectEEF(0);

    buildVisu();
}

void GraspQualityWindow::loadRobot()
{
    eefs.clear();
    robot.reset();
    robot = RobotIO::loadRobot(robotFile);

    if (!robot)
    {
        VR_ERROR << " no robot at " << robotFile << std::endl;
        return;
    }


    setEEFComboBox();
    selectEEF(0);
    //objectToTCP();

    buildVisu();
}

void GraspQualityWindow::objectToTCP()
{
    if (object && eef && eef->getTcp())
    {
        Eigen::Matrix4f pos =  eef->getTcp()->getGlobalPose();
        object->setGlobalPose(pos);
    }
}


void GraspQualityWindow::objectToGrasp()
{
    if (object && grasp && eef->getTcp())
    {
        VR_INFO << "Setting object pose to grasp " << grasp->getName() << std::endl;
        Eigen::Matrix4f pos =  eef->getTcp()->getGlobalPose();
        pos = grasp->getObjectTargetPoseGlobal(pos);
        object->setGlobalPose(pos);
    }
}

void GraspQualityWindow::graspQuality()
{
    if (qualityMeasure && object && contacts.size() > 0)
    {
        qualityMeasure->setVerbose(true);

        qualityMeasure->setContactPoints(contacts);
        float volume = qualityMeasure->getVolumeGraspMeasure();
        float epsilon = qualityMeasure->getGraspQuality();
        bool fc = qualityMeasure->isGraspForceClosure();
        std::cout << "Grasp Quality (epsilon measure):" << epsilon << std::endl;
        std::cout << "v measure:" << volume << std::endl;
        std::cout << "Force closure:";

        if (fc)
        {
            std::cout << "yes" << std::endl;
        }
        else
        {
            std::cout << "no" << std::endl;
        }

    }
}

void GraspQualityWindow::selectEEF(int nr)
{
    eef.reset();

    if (nr < 0 || nr >= (int)eefs.size())
    {
        setGraspComboBox();
        selectGrasp(-1);
        objectToTCP();
        return;
    }

    eef = eefs[nr];
    setGraspComboBox();
    selectGrasp(0);
}

void GraspQualityWindow::selectGrasp(int nr)
{
    grasp.reset();

    if (!grasps || nr < 0 || nr >= (int)grasps->getSize())
    {
        objectToTCP();
        return;
    }

    grasp = grasps->getGrasp(nr);
    objectToGrasp();
}

void GraspQualityWindow::setGraspComboBox()
{
    UI.comboBoxGrasp->clear();
    grasp.reset();
    grasps.reset();

    if (!eef || !object)
    {
        return;
    }

    grasps = object->getGraspSet(eef);

    if (!grasps || grasps->getSize() == 0)
    {
        VR_INFO << "No grasps found for eef " << eef->getName() << std::endl;
        return;
    }

    VR_INFO << "Found " << grasps->getSize() << " grasps for eef " << eef->getName() << std::endl;

    for (size_t i = 0; i < grasps->getSize(); i++)
    {
        QString nameGrasp(grasps->getGrasp(i)->getName().c_str());
        UI.comboBoxGrasp->addItem(nameGrasp);
    }
}


void GraspQualityWindow::setEEFComboBox()
{
    UI.comboBoxEEF->clear();
    eef.reset();
    eefs.clear();

    if (!robot)
    {
        return;
    }

    robot->getEndEffectors(eefs);

    for (auto& eef : eefs)
    {
        QString nameEEF(eef->getName().c_str());
        UI.comboBoxEEF->addItem(nameEEF);
    }
}

void GraspQualityWindow::closeEEF()
{
    contacts.clear();

    if (eef)
    {
        contacts = eef->closeActors(object);
    }

    buildVisu();
}

void GraspQualityWindow::openEEF()
{
    contacts.clear();

    if (eef)
    {
        eef->openActors();
    }

    buildVisu();
}



void GraspQualityWindow::updateObject(float x[6])
{
    if (object)
    {
        //cout << "getGlobalPose robot:" << endl << robotEEF->getGlobalPose() << std::endl;
        //cout << "getGlobalPose TCP:" << endl <<  robotEEF_EEF->getTcp()->getGlobalPose() << std::endl;
        Eigen::Matrix4f m;
        MathTools::posrpy2eigen4f(x, m);

        m = object->getGlobalPose() * m;
        object->setGlobalPose(m);
        std::cout << "object " << std::endl;
        std::cout << m << std::endl;

    }

    m_pExViewer->scheduleRedraw();
}

void GraspQualityWindow::sliderReleased_ObjectX()
{
    UI.horizontalSliderX->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectY()
{
    UI.horizontalSliderY->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectZ()
{
    UI.horizontalSliderZ->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectA()
{
    UI.horizontalSliderRo->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectB()
{
    UI.horizontalSliderPi->setValue(0);
}

void GraspQualityWindow::sliderReleased_ObjectG()
{
    UI.horizontalSliderYa->setValue(0);
}

void GraspQualityWindow::frictionConeVisu()
{
    buildVisu();
}

void GraspQualityWindow::colModel()
{
    buildVisu();
}

void GraspQualityWindow::showGWS()
{
    gws1Sep->removeAllChildren();
    gws2Sep->removeAllChildren();
    ows1Sep->removeAllChildren();
    ows2Sep->removeAllChildren();

    if (!qualityMeasure)
    {
        return;
    }

    VirtualRobot::MathTools::ConvexHull6DPtr ch = qualityMeasure->getConvexHullGWS();
    VirtualRobot::MathTools::ConvexHull6DPtr chOWS = qualityMeasure->getConvexHullOWS();

    if (!ch)
    {
        return;
    }
    /*
        if (UI.checkBoxGWS1->isChecked())
        {
            GraspStudio::CoinConvexHullVisualizationPtr v(new GraspStudio::CoinConvexHullVisualization(ch, true));
            SoSeparator* s = v->getCoinVisualization();

            if (s)
            {
                gws1Sep->addChild(mt);
                gws1Sep->addChild(s);
            }
        }

        if (UI.checkBoxGWS2->isChecked())
        {
            GraspStudio::CoinConvexHullVisualizationPtr v(new GraspStudio::CoinConvexHullVisualization(chOWS, false));
            SoSeparator* s = v->getCoinVisualization();

            if (s)
            {
                gws2Sep->addChild(mt);
                gws2Sep->addChild(s);
            }

        }

        if (UI.checkBoxOWS1->isChecked())
        {
            GraspStudio::CoinConvexHullVisualizationPtr v(new GraspStudio::CoinConvexHullVisualization(chOWS, true));
            SoSeparator* s = v->getCoinVisualization();

            if (s)
            {
                ows1Sep->addChild(mt);
                ows1Sep->addChild(s);
            }
        }

        if (UI.checkBoxOWS2->isChecked())
        {
            GraspStudio::CoinConvexHullVisualizationPtr v(new GraspStudio::CoinConvexHullVisualization(chOWS, false));
            SoSeparator* s = v->getCoinVisualization();

            if (s)
            {
                ows2Sep->addChild(mt);
                ows2Sep->addChild(s);
            }
        }*/
}

void GraspQualityWindow::showOWS()
{

}
