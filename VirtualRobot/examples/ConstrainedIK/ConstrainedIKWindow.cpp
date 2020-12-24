/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Peter Kaiser
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#include "ConstrainedIKWindow.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/IK/ConstrainedHierarchicalIK.h"
#include "VirtualRobot/IK/ConstrainedStackedIK.h"
#include <VirtualRobot/Random.h>

#ifdef USE_NLOPT
#include "VirtualRobot/IK/ConstrainedOptimizationIK.h"
#endif

#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoSphere.h>

#include <ctime>
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

ConstrainedIKWindow::ConstrainedIKWindow(std::string& sRobotFilename)
    : QMainWindow(nullptr)
{
    robotFilename = sRobotFilename;
    sceneSep = new SoSeparator();
    sceneSep->ref();
    robotSep = new SoSeparator();
    sceneSep->addChild(robotSep);
    setupUI();

    loadRobot();

    boxSep = new SoSeparator;
    sceneSep->addChild(boxSep);

    tsrSep = new SoSeparator;
    sceneSep->addChild(tsrSep);

    poseSep = new SoSeparator;
    sceneSep->addChild(poseSep);

    exViewer->viewAll();
}


ConstrainedIKWindow::~ConstrainedIKWindow()
{
    sceneSep->unref();
}

void ConstrainedIKWindow::setupUI()
{
    UI.setupUi(this);
    exViewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    exViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    exViewer->setAccumulationBuffer(false);
    exViewer->setAntialiasing(true, 4);

    exViewer->setTransparencyType(SoGLRenderAction::BLEND);
    exViewer->setFeedbackVisibility(true);
    exViewer->setSceneGraph(sceneSep);
    exViewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadRobot()));
    connect(UI.comboBoxKC, SIGNAL(activated(int)), this, SLOT(selectKC(int)));
    connect(UI.pushButtonSolve, SIGNAL(clicked()), this, SLOT(solve()));

    connect(UI.tsrLowX, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowY, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowZ, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowPitch, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowRoll, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrLowYaw, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighX, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighY, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighZ, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighPitch, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighRoll, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));
    connect(UI.tsrHighYaw, SIGNAL(valueChanged(double)), this, SLOT(updateTSR(double)));

    connect(UI.tsrRandom, SIGNAL(clicked()), this, SLOT(randomTSR()));
    connect(UI.tsrGroup, SIGNAL(clicked()), this, SLOT(enableTSR()));

    connect(UI.poseX, SIGNAL(valueChanged(double)), this, SLOT(updatePose(double)));
    connect(UI.poseY, SIGNAL(valueChanged(double)), this, SLOT(updatePose(double)));
    connect(UI.poseZ, SIGNAL(valueChanged(double)), this, SLOT(updatePose(double)));
    connect(UI.poseRoll, SIGNAL(valueChanged(double)), this, SLOT(updatePose(double)));
    connect(UI.posePitch, SIGNAL(valueChanged(double)), this, SLOT(updatePose(double)));
    connect(UI.poseYaw, SIGNAL(valueChanged(double)), this, SLOT(updatePose(double)));

    connect(UI.poseRandom, SIGNAL(clicked()), this, SLOT(randomPose()));
    connect(UI.poseGroup, SIGNAL(clicked()), this, SLOT(enablePose()));

    connect(UI.balanceGroup, SIGNAL(clicked()), this, SLOT(enableBalance()));

    connect(UI.evaluationStart, SIGNAL(clicked()), this, SLOT(performanceEvaluation()));

    UI.ikSolver->addItem("Constrained Hierarchical IK");
    UI.ikSolver->addItem("Constrained Stacked IK");
    UI.ikSolver->setCurrentIndex(0);

#ifdef USE_NLOPT
    UI.ikSolver->addItem("Constrained Optimization IK");
    UI.ikSolver->setCurrentIndex(2);
#endif
}

void ConstrainedIKWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector<RobotNodePtr> rn;
    robot->getRobotNodes(rn);
    std::vector<float> jv(rn.size(), 0.0f);
    robot->setJointValues(rn, jv);

    exViewer->render();
}



void ConstrainedIKWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    robotSep->removeAllChildren();

    std::shared_ptr<CoinVisualization> visualization = robot->getVisualization<CoinVisualization>(SceneObject::Full);
    SoNode* visualisationNode = nullptr;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        robotSep->addChild(visualisationNode);
    }

    exViewer->render();
}


void ConstrainedIKWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


int ConstrainedIKWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void ConstrainedIKWindow::quit()
{
    std::cout << "ConstrainedIKWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void ConstrainedIKWindow::updateKCBox()
{
    UI.comboBoxKC->clear();

    if (!robot)
    {
        return;
    }

    std::vector<RobotNodeSetPtr> rns;
    robot->getRobotNodeSets(rns);
    kinChains.clear();

    for (auto & rn : rns)
    {
        if (rn->isKinematicChain())
        {
            UI.comboBoxKC->addItem(QString(rn->getName().c_str()));
            kinChains.push_back(rn);
        }
    }
}

void ConstrainedIKWindow::computePoseError()
{
    // Compute target pose
    Eigen::Vector3f pos, rpy;
    pos << UI.poseX->value(), UI.poseY->value(), UI.poseZ->value();
    rpy << UI.poseRoll->value(), UI.posePitch->value(), UI.poseYaw->value();

    Eigen::Matrix4f targetPose;
    MathTools::posrpy2eigen4f(pos, rpy, targetPose);

    Eigen::Matrix4f currentPose = tcp->getGlobalPose();

    float errorPos = (currentPose.block(0, 3, 3, 1) - targetPose.block(0, 3, 3, 1)).norm();
    MathTools::Quaternion q1 = MathTools::eigen4f2quat(currentPose);
    MathTools::Quaternion q2 = MathTools::eigen4f2quat(targetPose);
    MathTools::Quaternion d = getDelta(q1, q2);
    float errorOri = fabs(180.0f - (d.w + 1.0f) * 90.0f);

    QString qd2 = "Error Pos: : ";
    qd2 += QString::number(errorPos, 'f', 2);
    qd2 += " mm";
    UI.labelPos->setText(qd2);
    QString qd3 = "Error Ori: : ";
    qd3 += QString::number(errorOri, 'f', 2);
    qd3 += " deg";
    UI.labelOri->setText(qd3);
}

void ConstrainedIKWindow::computeTSRError()
{

}

void ConstrainedIKWindow::selectKC(int nr)
{
    std::cout << "Selecting kinematic chain nr " << nr << std::endl;

    if (nr < 0 || nr >= (int)kinChains.size())
    {
        return;
    }

    if (tcp)
    {
        tcp->showCoordinateSystem(false);
    }

    kc = kinChains[nr];
    kc->print();
    QString nameQ = "TCP: ";
    tcp = kc->getTCP();

    if (tcp)
    {
        QString n(tcp->getName().c_str());
        nameQ += n;
        tcp->showCoordinateSystem(true);
    }

    unsigned int d = kc->getSize();
    QString qd = "Nr of joints: ";
    qd += QString::number(d);

    UI.label_TCP->setText(nameQ);
    UI.label_NrJoints->setText(qd);

    // since it may be that a tcp coord system was created in this method we must re-build the visualization in order to show it
    collisionModel();
}

void ConstrainedIKWindow::selectIK(int nr)
{
    if(nr == 0)
    {
        UI.comboBoxKC->setCurrentIndex(0);
    }
}

void ConstrainedIKWindow::solve()
{
    if (!kc || !tcp)
    {
        return;
    }


    ConstrainedIKPtr ik;

    switch(UI.ikSolver->currentIndex())
    {
        case 0:
            VR_INFO << "Using Constrainbed Hierarchical IK" << std::endl;
            ik.reset(new ConstrainedHierarchicalIK(robot, kc));
            break;

        case 1:
            VR_INFO << "Using Constrained Stacked IK" << std::endl;
            ik.reset(new ConstrainedStackedIK(robot, kc));
            break;

#ifdef USE_NLOPT
        case 2:
            VR_INFO << "Using Constrained Optimization IK" << std::endl;
            ik.reset(new ConstrainedOptimizationIK(robot, kc));
            break;
#endif

        default:
            VR_ERROR << "Unknown IK solver selected" << std::endl;
            return;
    }

    if(UI.tsrGroup->isChecked())
    {
        ik->addConstraint(tsrConstraint);
    }

    if(UI.poseGroup->isChecked())
    {
        ik->addConstraint(positionConstraint);
        ik->addConstraint(orientationConstraint);
    }

    if(UI.balanceGroup->isChecked())
    {
        ik->addConstraint(balanceConstraint);
    }

    ik->initialize();

    clock_t startT = clock();
    bool result = ik->solve();
    clock_t endT = clock();

    VR_INFO << "IK " << (result? "Successful" : "Failed") << std::endl;

    float runtime = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
    QString qd = "Time: ";
    qd += QString::number(runtime, 'f', 2);
    qd += " ms";
    UI.labelTime->setText(qd);

    std::vector<RobotNodePtr> nodes = kc->getAllRobotNodes();

    std::cout << "Joint values: " << std::endl;
    for (auto &node : nodes)
    {
        std::cout << node->getName() << ": " << node->getJointValue() << std::endl;
    }

    if(UI.poseGroup->isChecked())
    {
        computePoseError();
    }
    if(UI.poseGroup->isChecked())
    {
        computeTSRError();
    }

    exViewer->render();
}

void ConstrainedIKWindow::updateTSR(double /*value*/)
{
    Eigen::Matrix<float, 6, 2> bounds;
    bounds << -fabs(UI.tsrLowX->value() - UI.tsrHighX->value()) / 2, fabs(UI.tsrLowX->value() - UI.tsrHighX->value()) / 2,
              -fabs(UI.tsrLowY->value() - UI.tsrHighY->value()) / 2, fabs(UI.tsrLowY->value() - UI.tsrHighY->value()) / 2,
              -fabs(UI.tsrLowZ->value() - UI.tsrHighZ->value()) / 2, fabs(UI.tsrLowZ->value() - UI.tsrHighZ->value()) / 2,
              -M_PI, M_PI,
              -M_PI, M_PI,
              -M_PI, M_PI;
              /*UI.tsrLowRoll->value(), UI.tsrHighRoll->value(),
              UI.tsrLowPitch->value(), UI.tsrHighPitch->value(),
              UI.tsrLowYaw->value(), UI.tsrHighYaw->value();*/

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation(0,3) = UI.tsrLowX->value() + fabs(UI.tsrLowX->value() - UI.tsrHighX->value()) / 2;
    transformation(1,3) = UI.tsrLowY->value() + fabs(UI.tsrLowY->value() - UI.tsrHighY->value()) / 2;
    transformation(2,3) = UI.tsrLowZ->value() + fabs(UI.tsrLowZ->value() - UI.tsrHighZ->value()) / 2;

    tsrConstraint.reset(new TSRConstraint(robot, kc, tcp, transformation, Eigen::Matrix4f::Identity(), bounds));

    tsrSep->removeAllChildren();
    VisualizationFactory::Color color(1, 0, 0, 0.5);
    tsrSep->addChild(CoinVisualizationFactory::getCoinVisualization(tsrConstraint, color));
}

void ConstrainedIKWindow::randomTSR(bool quiet)
{
    // Store joint angles
    RobotConfigPtr originalConfig(new RobotConfig(robot, "original config"));
    kc->getJointValues(originalConfig);

    // Apply random joint angles
    for(unsigned int i = 0; i < kc->getSize(); i++)
    {
        RobotNodePtr node = kc->getNode(i);

        float v = node->getJointLimitLo() + (node->getJointLimitHi() - node->getJointLimitLo()) * (rand()%1000 / 1000.0);
        node->setJointValue(v);
    }

    // Obtain TCP pose
    Eigen::Matrix4f tcpPose = kc->getTCP()->getGlobalPose();

    if(!quiet)
    {
        VR_INFO << "Sampled TCP Pose: " << std::endl << tcpPose << std::endl;
    }

    // Relax TCP pose randomly to form a TSR
    float lowX = tcpPose(0,3) - rand()%100;
    float highX = tcpPose(0,3) + rand()%100;
    float lowY = tcpPose(1,3) - rand()%100;
    float highY = tcpPose(1,3) + rand()%100;
    float lowZ = tcpPose(2,3) - rand()%100;
    float highZ = tcpPose(2,3) + rand()%100;

    Eigen::Vector3f rpy;
    MathTools::eigen4f2rpy(tcpPose, rpy);

    float lowRoll = rpy.x() - (M_PI / 4) * (rand()%100 / 100.0);
    float highRoll = rpy.x() + (M_PI / 4) * (rand()%100 / 100.0);
    float lowPitch = rpy.y() - (M_PI / 4) * (rand()%100 / 100.0);
    float highPitch = rpy.y() + (M_PI / 4) * (rand()%100 / 100.0);
    float lowYaw = rpy.z() - (M_PI / 4) * (rand()%100 / 100.0);
    float highYaw = rpy.z() + (M_PI / 4) * (rand()%100 / 100.0);

    if(!quiet)
    {
        VR_INFO << "Random TSR: " << std::endl
                << "    [" << lowX << ", " << highX << "]," << std::endl
                << "    [" << lowY << ", " << highY << "]," << std::endl
                << "    [" << lowZ << ", " << highZ << "]," << std::endl
                << "    [" << lowRoll << ", " << highRoll << "]," << std::endl
                << "    [" << lowPitch << ", " << highPitch << "]," << std::endl
                << "    [" << lowYaw << ", " << highYaw << "]," << std::endl;
    }

    // Apply TSR
    UI.tsrLowX->setValue(lowX);
    UI.tsrHighX->setValue(highX);
    UI.tsrLowY->setValue(lowY);
    UI.tsrHighY->setValue(highY);
    UI.tsrLowZ->setValue(lowZ);
    UI.tsrHighZ->setValue(highZ);
    UI.tsrLowRoll->setValue(lowRoll);
    UI.tsrHighRoll->setValue(highRoll);
    UI.tsrLowPitch->setValue(lowPitch);
    UI.tsrHighPitch->setValue(highPitch);
    UI.tsrLowYaw->setValue(lowYaw);
    UI.tsrHighYaw->setValue(highYaw);

    updateTSR(0);

    // Restore original joint angles
    kc->setJointValues(originalConfig);
}

void ConstrainedIKWindow::enableTSR()
{
    if(UI.tsrGroup->isChecked())
    {
        updateTSR(0);
    }
    else
    {
        tsrSep->removeAllChildren();
    }
}

void ConstrainedIKWindow::updatePose(double /*value*/)
{
    Eigen::Vector3f pos, rpy;
    pos << UI.poseX->value(), UI.poseY->value(), UI.poseZ->value();
    rpy << UI.poseRoll->value(), UI.posePitch->value(), UI.poseYaw->value();

    Eigen::Matrix4f pose;
    MathTools::posrpy2eigen4f(pos, rpy, pose);

    positionConstraint.reset(new PositionConstraint(robot, kc, tcp, pose.block<3,1>(0,3)));

    orientationConstraint.reset(new OrientationConstraint(robot, kc, tcp, pose.block<3,3>(0,0)));
    orientationConstraint->setOptimizationFunctionFactor(1000);

    poseSep->removeAllChildren();
    VisualizationFactory::Color color(1, 0, 0, 0.5);
    poseSep->addChild(CoinVisualizationFactory::getCoinVisualization(positionConstraint, color));
}

void ConstrainedIKWindow::randomPose(bool quiet)
{
    // Store joint angles
    RobotConfigPtr originalConfig(new RobotConfig(robot, "original config"));
    kc->getJointValues(originalConfig);

    // Apply random joint angles
    for(unsigned int i = 0; i < kc->getSize(); i++)
    {
        RobotNodePtr node = kc->getNode(i);

        float v = node->getJointLimitLo() + (node->getJointLimitHi() - node->getJointLimitLo()) * (rand()%1000 / 1000.0);
        node->setJointValue(v);
    }

    // Obtain TCP pose
    Eigen::Matrix4f tcpPose = kc->getTCP()->getGlobalPose();

    if(!quiet)
    {
        VR_INFO << "Sampled TCP Pose: " << std::endl << tcpPose << std::endl;
    }

    Eigen::Vector3f rpy;
    MathTools::eigen4f2rpy(tcpPose, rpy);

    UI.poseX->setValue(tcpPose(0,3));
    UI.poseY->setValue(tcpPose(1,3));
    UI.poseZ->setValue(tcpPose(2,3));
    UI.poseRoll->setValue(rpy.x());
    UI.posePitch->setValue(rpy.y());
    UI.poseYaw->setValue(rpy.z());

    updatePose(0);

    // Restore original joint angles
    kc->setJointValues(originalConfig);
}

void ConstrainedIKWindow::enablePose()
{
    if(UI.poseGroup->isChecked())
    {
        updatePose(0);
    }
    else
    {
        poseSep->removeAllChildren();
    }
}

void ConstrainedIKWindow::enableBalance()
{
    if(UI.balanceGroup->isChecked())
    {
        RobotNodePtr contactNode = robot->getRobotNode(UI.lineEditContactNode->text().toStdString());

        SceneObjectSetPtr contactNodes(new SceneObjectSet);
        contactNodes->addSceneObject(contactNode);

        balanceConstraint.reset(new BalanceConstraint(robot, kc, kc, contactNodes));
    }
}

void ConstrainedIKWindow::performanceEvaluation()
{

    VirtualRobot::PRNG64Bit().seed(UI.evaluationRandomSeed->value());

    ConstrainedIKPtr ik;
    int ikSolver = UI.ikSolver->currentIndex();

    int resultSuccessful = 0;
    float totalTime = 0;
    float totalJointAngles = 0;

    int num = UI.evaluationNumberOfRuns->value();
    for(int i = 0; i < num; i++)
    {
        VR_INFO << "Evaluation run " << (i+1) << std::endl;

        // Reset joint angles to zero
        std::vector<RobotNodePtr> rn;
        robot->getRobotNodes(rn);
        std::vector<float> jv(rn.size(), 0.0f);
        robot->setJointValues(rn, jv);

        switch(ikSolver)
        {
            case 0:
                ik.reset(new ConstrainedHierarchicalIK(robot, kc));
                break;

            case 1:
                ik.reset(new ConstrainedStackedIK(robot, kc));
                break;

#ifdef USE_NLOPT
            case 2:
                ik.reset(new ConstrainedOptimizationIK(robot, kc));
                break;
#endif

            default:
                VR_ERROR << "Unknown IK solver selected" << std::endl;
                return;
        }

        if(UI.tsrGroup->isChecked())
        {
            randomTSR(true);
            ik->addConstraint(tsrConstraint);
        }

        if(UI.poseGroup->isChecked())
        {
            randomPose(true);
            ik->addConstraint(positionConstraint);
            ik->addConstraint(orientationConstraint);
        }

        if(UI.balanceGroup->isChecked())
        {
            ik->addConstraint(balanceConstraint);
        }

        ik->initialize();

        clock_t startT = clock();
        bool result = ik->solve();
        clock_t endT = clock();

        resultSuccessful += (int)result;
        totalTime += (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);

        for(unsigned int i = 0; i < kc->getSize(); i++)
        {
            totalJointAngles += fabs(kc->getNode(i)->getJointValue());
        }
    }

    VR_INFO << std::endl << "Evaluation result:"
            << std::endl << "    Success rate: " << (100 * (resultSuccessful / (float)num)) << "%"
            << std::endl << "    Avg. runtime: " << (totalTime / num) << "ms"
            << std::endl << "    Joint angle sum: " << totalJointAngles << "rad"
            << std::endl;
}

void ConstrainedIKWindow::loadRobot()
{
    std::cout << "ConstrainedIKWindow: Loading robot" << std::endl;
    robotSep->removeAllChildren();
    std::cout << "Loading Robot from " << robotFilename << std::endl;

    try
    {
        robot = RobotIO::loadRobot(robotFilename);
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

    updateKCBox();

    if (kinChains.size() == 0)
    {
        selectKC(-1);
    }
    else
    {
        selectKC(0);
    }

    // build visualization
    collisionModel();
    exViewer->viewAll();
    exViewer->render();
}

