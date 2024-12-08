#include "DiffIKWidget.h"
#include "VirtualRobot/Nodes/RobotNode.h"
#include "VirtualRobot/RobotNodeSet.h"
#include "ui_DiffIKWidget.h"

#include <QDialog>
#include <QVBoxLayout>
#include <QRegExp>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoUnits.h>
#include <SimoxUtility/algorithm/string/string_conversion.h>
#include <SimoxUtility/math/convert.h>
#include <VirtualRobot/Manipulability/SingleChainManipulabilityTracking.h>
#include <VirtualRobot/Manipulability/SingleChainManipulability.h>
#include <VirtualRobot/Manipulability/BimanualManipulability.h>
#include <VirtualRobot/Manipulability/BimanualManipulabilityTracking.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/IK/CompositeDiffIK/SoechtingNullspaceGradient.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <QThread>

DiffIKWidget* DiffIKWidget::diffIKWidget = nullptr;

DiffIKWidget::DiffIKWidget(SoSeparator *sceneSep, QDialog *parent) :
    QWidget(parent),
    dialog(parent),
    ui(new Ui::DiffIKWidget),
    manipSep(new SoSeparator()),
    followManipSep(new SoSeparator()),
    endeffectorSep(new SoSeparator()),
    currentRobotNodeSet(nullptr),
    newRNS(nullptr),
    clonedRobot(nullptr)

{
    ui->setupUi(this);
    sceneSep->addChild(manipSep);
    sceneSep->addChild(followManipSep);
    sceneSep->addChild(endeffectorSep);

    connect(ui->comboBoxRNS, SIGNAL(currentTextChanged(QString)), this, SLOT(setRobotNodeSet(QString)));
    connect(ui->comboBoxRNS2, SIGNAL(currentTextChanged(QString)), this, SLOT(setRobotNodeSet2(QString)));
    connect(ui->printJacobianButton, SIGNAL(clicked()), this, SLOT(printJacobian()));
    connect(ui->printJacobian2Button, SIGNAL(clicked()), this, SLOT(printJacobian2()));

    connect(ui->checkBoxVisManip, SIGNAL(toggled(bool)), this, SLOT(updateCurrentManipulabilityEllipsoidVis()));
    connect(ui->checkBoxBimanual, SIGNAL(toggled(bool)), this, SLOT(updateCurrentManipulabilityEllipsoidVis()));
    connect(ui->comboBoxManip, SIGNAL(currentTextChanged(QString)), this, SLOT(updateCurrentManipulabilityEllipsoidVis()));
    connect(ui->comboBoxManipType, SIGNAL(currentTextChanged(QString)), this, SLOT(updateCurrentManipulabilityEllipsoidVis()));
    connect(ui->elliosoidScaling, SIGNAL(valueChanged(int)), this, SLOT(updateCurrentManipulabilityEllipsoidVis()));
    connect(ui->ellipsoidTransparency, SIGNAL(valueChanged(double)), this, SLOT(updateCurrentManipulabilityEllipsoidVis()));
    connect(this, SIGNAL(currentManipUpdated()), this, SLOT(updateCurrentManipulabilityEllipsoidVis()));

    connect(ui->followBox, SIGNAL(toggled(bool)), this, SLOT(updateFollowManipulabilityEllipsoidVis()));
    connect(ui->comboBoxManip, SIGNAL(currentTextChanged(QString)), this, SLOT(updateFollowManipulabilityEllipsoidVis()));
    connect(ui->elliosoidScaling, SIGNAL(valueChanged(int)), this, SLOT(updateFollowManipulabilityEllipsoidVis()));
    connect(ui->ellipsoidTransparency, SIGNAL(valueChanged(double)), this, SLOT(updateFollowManipulabilityEllipsoidVis()));
    connect(this, SIGNAL(currentManipUpdated()), this, SLOT(updateFollowManipulabilityEllipsoidVis()));
    connect(ui->followManipulability, SIGNAL(textChanged()), this, SLOT(updateFollowManipulabilityEllipsoidVis()));
    connect(ui->checkBoxBimanual, SIGNAL(toggled(bool)), this, SLOT(updateFollowManipulabilityEllipsoidVis()));

    connect(ui->stepButton, SIGNAL(clicked()), this, SLOT(stepFollowManip()));
    connect(ui->solveButton, SIGNAL(clicked()), this, SLOT(followManip()));
    connect(this, SIGNAL(distanceUpdated(double)), this, SLOT(updateDistance(double)));

    connect(ui->xTarget, SIGNAL(valueChanged(double)), this, SLOT(updateEndeffectorPoseVis()));
    connect(ui->yTarget, SIGNAL(valueChanged(double)), this, SLOT(updateEndeffectorPoseVis()));
    connect(ui->zTarget, SIGNAL(valueChanged(double)), this, SLOT(updateEndeffectorPoseVis()));
    connect(ui->rollTarget, SIGNAL(valueChanged(double)), this, SLOT(updateEndeffectorPoseVis()));
    connect(ui->pitchTarget, SIGNAL(valueChanged(double)), this, SLOT(updateEndeffectorPoseVis()));
    connect(ui->yawTarget, SIGNAL(valueChanged(double)), this, SLOT(updateEndeffectorPoseVis()));
    connect(ui->checkBoxVisTarget, SIGNAL(toggled(bool)), this, SLOT(updateEndeffectorPoseVis()));
    connect(ui->checkBoxOriIK, SIGNAL(toggled(bool)), this, SLOT(updateEndeffectorPoseVis()));
    connect(ui->currentPoseButton, SIGNAL(clicked()), this, SLOT(setEndEffectorPose()));
    connect(ui->solveIKButton, SIGNAL(clicked()), this, SLOT(solveIK()));

    connect(ui->resetJointValues, SIGNAL(clicked()), this, SLOT(resetJointValues()));
    connect(ui->setAverageJointValues, SIGNAL(clicked()), this, SLOT(setAverageJointValues()));
    connect(ui->setRandomJointValues, SIGNAL(clicked()), this, SLOT(setRandomJointValues()));

    qRegisterMetaType<VirtualRobot::AbstractManipulabilityTrackingPtr>("VirtualRobot::AbstractManipulabilityTrackingPtr");
    qRegisterMetaType<Eigen::MatrixXd>("Eigen::MatrixXd");
    qRegisterMetaType<Eigen::VectorXf>("Eigen::VectorXf");
    qRegisterMetaType<std::map<std::string, float>>("std::map<std::string, float>");
    qRegisterMetaType<VirtualRobot::CompositeDiffIKPtr>("VirtualRobot::CompositeDiffIKPtr");
    qRegisterMetaType<VirtualRobot::NullspaceManipulabilityPtr>("VirtualRobot::NullspaceManipulabilityPtr");
    qRegisterMetaType<VirtualRobot::RobotNodeSetPtr>("VirtualRobot::RobotNodeSetPtr");

    Worker *worker = new Worker;
    worker->moveToThread(&workerThread);
    connect(&workerThread, &QThread::finished, worker, &QObject::deleteLater);
    connect(this, &DiffIKWidget::followManipAsync, worker, &Worker::followManip);
    connect(this, &DiffIKWidget::solveIKAsync, worker, &Worker::solveIK);
    connect(worker, &Worker::distanceUpdated, this, &DiffIKWidget::updateDistance);
    connect(worker, &Worker::currentManipUpdated, this, &DiffIKWidget::updateCurrentManipulabilityEllipsoidVis);
    connect(worker, &Worker::jointValuesUpdated, this, &DiffIKWidget::updateJointValues);
    connect(worker, &Worker::finished, this, &DiffIKWidget::workerFinished);
    workerThread.start();
}

DiffIKWidget::~DiffIKWidget()
{
    manipSep->unref();
    followManipSep->unref();
    endeffectorSep->unref();
    workerThread.quit();
    workerThread.wait();
    delete ui;
}

QDialog *DiffIKWidget::getDialog() {
    return dialog;
}

void DiffIKWidget::open(QWidget *parent, SoSeparator *sceneSep) {
    if (!diffIKWidget) {
        auto diffIKDialog = new QDialog(parent);
        QVBoxLayout* layout = new QVBoxLayout();
        diffIKWidget = new DiffIKWidget(sceneSep, diffIKDialog);
        layout->addWidget(diffIKWidget);
        diffIKDialog->setLayout(layout);
    }
    diffIKWidget->getDialog()->show();
}

void DiffIKWidget::update(VirtualRobot::RobotPtr robot) {
    if (diffIKWidget && diffIKWidget->getDialog()->isVisible()) {
        if (diffIKWidget->robot != robot) {
            diffIKWidget->robot = robot;
            diffIKWidget->addRobotNodeSets();
        }
        diffIKWidget->updateCurrentManipulabilityEllipsoidVis();
        diffIKWidget->updateFollowManipulabilityEllipsoidVis();
        diffIKWidget->updateEndeffectorPoseVis();
    }
}

void DiffIKWidget::close() {
    delete diffIKWidget;
    diffIKWidget = nullptr;
}


Eigen::MatrixXd DiffIKWidget::readFollowManipulability() {
    try {
        Eigen::MatrixXd matrix;
        QString data = ui->followManipulability->toPlainText();
        // QTextEdit content
        QStringList strList = data.split(QRegExp("[\n]"), QString::SkipEmptyParts);

        if (strList.size() == 6) {
            matrix = Eigen::Matrix<double, 6, 6>();
        }
        else if (strList.size() == 3) {
            matrix = Eigen::Matrix3d::Identity();
        }
        else {
            return Eigen::Matrix<double, 0, 0>();
        }
        matrix.setZero();

        for (int i = 0; i < strList.size(); i++) {
            QStringList s = strList[i].split(QRegExp(" "), QString::SkipEmptyParts);
            for (int j = 0; j < s.size(); j++) {
                float value = simox::alg::to_<double>(s[j].toStdString());
                matrix(i,j) = value;
                if (j > matrix.cols()) break;
            }
            if (i > matrix.rows()) break;
        }
        return matrix;
    }
    catch (...) {
        return Eigen::Matrix<double, 0, 0>();
    }
}

VirtualRobot::AbstractManipulability::Mode DiffIKWidget::getMode(QComboBox *comboBox) {
    if (comboBox->currentText() == "Whole") {
        return VirtualRobot::AbstractManipulability::Whole;
    }
    else  if (comboBox->currentText() == "Position") {
        return VirtualRobot::AbstractManipulability::Position;
    }
    else if (comboBox->currentText() == "Rotation") {
        return VirtualRobot::AbstractManipulability::Orientation;
    }
    else {
        throw std::runtime_error("Wrong!");
    }
}

VirtualRobot::AbstractManipulability::Type DiffIKWidget::getManipulabilityType(QComboBox *comboBox) {
    if (comboBox->currentText() == "Velocity") {
        return VirtualRobot::AbstractManipulability::Velocity;
    }
    else  if (comboBox->currentText() == "Force") {
        return VirtualRobot::AbstractManipulability::Force;
    }
    else {
        throw std::runtime_error("Wrong!");
    }
}

Eigen::Matrix4f DiffIKWidget::getEndEffectorPos() {
    return simox::math::pos_rpy_to_mat4f(ui->xTarget->value(), ui->yTarget->value(), ui->zTarget->value(),
                                  ui->rollTarget->value(), ui->pitchTarget->value(), ui->yawTarget->value());
}

Eigen::Matrix4f DiffIKWidget::getEndEffectorPos2() {
    return simox::math::pos_rpy_to_mat4f(ui->xTarget2->value(), ui->yTarget2->value(), ui->zTarget2->value(),
                                         ui->rollTarget2->value(), ui->pitchTarget2->value(), ui->yawTarget2->value());
}

void DiffIKWidget::addRobotNodeSets() {
    ui->comboBoxRNS->clear();
    for (auto& robotNodeSet : robot->getRobotNodeSets())
    {
        if (robotNodeSet->getTCP())
            ui->comboBoxRNS->addItem(QString(robotNodeSet->getName().c_str()));
    }
    ui->comboBoxRNS->setCurrentIndex(0);
    ui->comboBoxRNS2->clear();
    for (auto& robotNodeSet : robot->getRobotNodeSets())
    {
        if (robotNodeSet->getTCP())
            ui->comboBoxRNS2->addItem(QString(robotNodeSet->getName().c_str()));
    }
    ui->comboBoxRNS2->setCurrentIndex(0);
}

void DiffIKWidget::updateCurrentManipulabilityEllipsoidVis() {
    manipSep->removeAllChildren();
    ui->currentManipulability->clear();
    if (ui->checkBoxVisManip->isChecked()) {
        auto manipTracking = getManipulabilityTracking();
        if (!manipTracking) return;
        auto manipulability = manipTracking->computeCurrentManipulability();
        VirtualRobot::VisualizationNodePtr visNode = manipTracking->getManipulabilityVis(manipulability, "", ui->elliosoidScaling->value());
        auto coinVisNode = std::dynamic_pointer_cast<VirtualRobot::CoinVisualizationNode>(visNode);
        if (coinVisNode) {
            auto coinVis = coinVisNode->getCoinVisualization();
            auto mat = new SoMaterial();
            mat->transparency.setValue(ui->ellipsoidTransparency->value());
            manipSep->addChild(mat);
            manipSep->addChild(coinVis);
        }
        std::stringstream ss;
        ss << manipulability;
        ui->currentManipulability->setText(QString::fromStdString(ss.str()));
    }
}

void DiffIKWidget::updateFollowManipulabilityEllipsoidVis() {
    followManipSep->removeAllChildren();
    if (ui->followBox->isChecked() && currentRobotNodeSet) {
        Eigen::MatrixXd followManip = readFollowManipulability();
        VirtualRobot::RobotNodeSetPtr rns = nullptr;
        VirtualRobot::AbstractManipulabilityTrackingPtr tracking = getManipulabilityTracking();
        if (tracking) {
            if (followManip.rows() != tracking->getTaskVars()) return;

            double distance = tracking->computeDistance(followManip);
            emit distanceUpdated(distance);

            VirtualRobot::VisualizationNodePtr visNode = tracking->getManipulabilityVis(followManip, "", ui->elliosoidScaling->value());
            auto coinVisNode = std::dynamic_pointer_cast<VirtualRobot::CoinVisualizationNode>(visNode);
            if (coinVisNode) {
                auto coinVis = coinVisNode->getCoinVisualization();
                auto mat = new SoMaterial();
                mat->transparency.setValue(ui->ellipsoidTransparency->value());
                followManipSep->addChild(mat);
                followManipSep->addChild(coinVis);
            }
        }
    }
}

void DiffIKWidget::updateEndeffectorPoseVis(VirtualRobot::RobotNodeSetPtr robotNodeSet, const Eigen::Matrix4f &pose) {
    if (robotNodeSet) {
        auto tcp = robotNodeSet->getTCP();
        if (tcp) {
            SoSeparator* sep = new SoSeparator();
            // Set the visualization stuff to milimeters
            SoUnits *u = new SoUnits();
            u->units = SoUnits::MILLIMETERS;
            sep->addChild(u);

            // set a transformation matrix for the visualization
            Eigen::Matrix4f transformation = robotNodeSet->getRobot()->getGlobalPose(pose);
            SoMatrixTransform* mt = new SoMatrixTransform();
            SbMatrix m_(reinterpret_cast<SbMat*>(transformation.data()));
            mt->matrix.setValue(m_);
            sep->addChild(mt);

            auto mat = new SoMaterial();
            mat->transparency.setValue(0.5);
            sep->addChild(mat);

            using Color = VirtualRobot::CoinVisualizationFactory::Color;

            if (ui->checkBoxOriIK->isChecked()) {
                auto visNode = tcp->getVisualization(VirtualRobot::SceneObject::Full);
                auto coinVisNode = std::dynamic_pointer_cast<VirtualRobot::CoinVisualizationNode>(visNode);
                if (coinVisNode) {
                    auto coinVis = coinVisNode->getCoinVisualization();
                    sep->addChild(coinVis);
                }
                else {
                    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateSphere(50, Color::Red()));
                }
            }
            else {
                sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateSphere(50, Color::Red()));
            }
            endeffectorSep->addChild(sep);
        }
    }
}

VirtualRobot::AbstractManipulabilityTrackingPtr DiffIKWidget::getManipulabilityTracking() {
    return getManipulabilityTracking(currentRobotNodeSet, currentRobotNodeSet2);
}

VirtualRobot::AbstractManipulabilityTrackingPtr DiffIKWidget::getManipulabilityTracking(VirtualRobot::RobotNodeSetPtr r1, VirtualRobot::RobotNodeSetPtr r2, bool setNewRNS) {
    VirtualRobot::AbstractManipulabilityTrackingPtr tracking = nullptr;
    if (r1 && r1->getTCP()) {
        auto mode = getMode(ui->comboBoxManip);
        if (!ui->checkBoxBimanual->isChecked()) {
            VirtualRobot::SingleRobotNodeSetManipulabilityPtr manip(new VirtualRobot::SingleRobotNodeSetManipulability(r1, mode, getManipulabilityType(ui->comboBoxManipType)));
            if (setNewRNS) newRNS = manip->getRobotNodeSet();
            tracking = VirtualRobot::SingleChainManipulabilityTrackingPtr(new VirtualRobot::SingleChainManipulabilityTracking(manip));
        }
        else if (r2 && r2->getTCP() && r1 != r2) {
            VirtualRobot::BimanualManipulabilityPtr manip(new VirtualRobot::BimanualManipulability(r1, r2, mode, getManipulabilityType(ui->comboBoxManipType)));
            if (setNewRNS) newRNS = manip->createRobotNodeSet();
            tracking = VirtualRobot::BimanualManipulabilityTrackingPtr(new VirtualRobot::BimanualManipulabilityTracking(manip));
        }
    }
    return tracking;
}

void DiffIKWidget::updateEndeffectorPoseVis() {
    endeffectorSep->removeAllChildren();
    if (ui->checkBoxVisTarget->isChecked()) {
        updateEndeffectorPoseVis(currentRobotNodeSet, getEndEffectorPos());
        updateEndeffectorPoseVis(currentRobotNodeSet2, getEndEffectorPos2());
    }
    if (ui->checkBoxSolveContinuous->isChecked()) {
        solveIK(true);
    }
}

void DiffIKWidget::stepFollowManip() {
    auto manipTracking = getManipulabilityTracking(currentRobotNodeSet, currentRobotNodeSet2, true);
    if (!manipTracking) return;
    Eigen::MatrixXd followManip = readFollowManipulability();
    if (followManip.rows() != manipTracking->getTaskVars()) {
        std::cout << "Wrong manipulability matrix!" << std::endl;
        return;
    }

    Eigen::VectorXf velocity = manipTracking->calculateVelocity(followManip, Eigen::MatrixXd(), true);
    std::cout << "Nullspace velocities without gain:\n" << velocity << "\n" << std::endl;
    Eigen::VectorXf jointValues = newRNS->getJointValuesEigen() + velocity * ui->kGain->value();;
    newRNS->setJointValues(jointValues);
    double distance = manipTracking->computeDistance(followManip);
    emit distanceUpdated(distance);
    emit currentManipUpdated();
}

void DiffIKWidget::followManip() {
    if (!currentRobotNodeSet) {
        std::cout << "RobotNodeSet is null" << std::endl;
        return;
    }

    clonedRobot = currentRobotNodeSet->getRobot()->clone();
    auto clonedRobotNodeSet = clonedRobot->getRobotNodeSet(currentRobotNodeSet->getName());
    auto manipTracking = getManipulabilityTracking(clonedRobot->getRobotNodeSet(currentRobotNodeSet->getName()),
                                                   currentRobotNodeSet2 ? clonedRobot->getRobotNodeSet(currentRobotNodeSet2->getName()) : nullptr,
                                                   true);
    if (!manipTracking) return;
    Eigen::MatrixXd followManip = readFollowManipulability();
    if (followManip.rows() != manipTracking->getTaskVars()) {
        std::cout << "Wrong manipulability matrix!" << std::endl;
        return;
    }
    float maxDistance = ui->maxDistance->value();
    float kGain = ui->kGain->value();

    ui->solveButton->setEnabled(false);
    ui->solveIKButton->setEnabled(false);
    emit followManipAsync(manipTracking, newRNS, followManip, kGain, maxDistance);
}

void DiffIKWidget::updateDistance(double distance) {
    ui->currentDistance->setText(QString::fromStdString(std::to_string(distance)));
}

void DiffIKWidget::updateJointValues(const std::map<std::string, float> &jointValues) {
    currentRobotNodeSet->getRobot()->setJointValues(jointValues);
    updateFollowManipulabilityEllipsoidVis();
}

void DiffIKWidget::solveIK(bool untilReached) {
    if (!currentRobotNodeSet || !currentRobotNodeSet->getTCP()) {
        std::cout << "RobotNodeSet or TCP is null" << std::endl;
        return;
    }

    std::cout << "Solving IK ..." << std::endl;

    clonedRobot = currentRobotNodeSet->getRobot()->clone();
    VirtualRobot::RobotNodeSetPtr clonedRobotNodeSet = nullptr;
    if (ui->checkBoxBimanual->isChecked()) {
        if (currentRobotNodeSet == currentRobotNodeSet2) return;
        std::vector<std::string> robotNodeNames = currentRobotNodeSet->getNodeNames();
        std::vector<std::string> robotNodeNames2 = currentRobotNodeSet2->getNodeNames();
        robotNodeNames.insert(robotNodeNames.end(), robotNodeNames2.begin(), robotNodeNames2.end());
        clonedRobotNodeSet = VirtualRobot::RobotNodeSet::createRobotNodeSet(clonedRobot, "New", robotNodeNames, clonedRobot->getRootNode()->getName());
    }
    else clonedRobotNodeSet = clonedRobot->getRobotNodeSet(currentRobotNodeSet->getName());
    auto tcp = currentRobotNodeSet->getTCP();
    if (!tcp) return;

    VirtualRobot::CompositeDiffIKPtr ik(new VirtualRobot::CompositeDiffIK(clonedRobotNodeSet));
    Eigen::Matrix4f pose = getEndEffectorPos();
    auto target1 = ik->addTarget(clonedRobot->getRobotNode(tcp->getName()), pose, ui->checkBoxOriIK->isChecked() ? VirtualRobot::IKSolver::All : VirtualRobot::IKSolver::Position);

    if (ui->checkBoxBimanual->isChecked()) {
        auto tcp = currentRobotNodeSet2->getTCP();
        if (tcp) {
            auto node = clonedRobot->getRobotNode(tcp->getName());
            Eigen::Matrix4f pose = getEndEffectorPos2();
            ik->addTarget(node, pose, ui->checkBoxOriIK2->isChecked() ? VirtualRobot::IKSolver::All : VirtualRobot::IKSolver::Position);
        }
        else return;
    }

    float jointLimitAvoidance = ui->jointLimitAvoidance->value();
    if (ui->checkBoxJointLimitAvoidance->isChecked() && jointLimitAvoidance > 0) {
        VirtualRobot::CompositeDiffIK::NullspaceJointLimitAvoidancePtr nsjla(new VirtualRobot::CompositeDiffIK::NullspaceJointLimitAvoidance(clonedRobotNodeSet));
        nsjla->kP = jointLimitAvoidance;
        for (auto node : clonedRobotNodeSet->getAllRobotNodes()) {
            if (node->isLimitless()) {
                nsjla->setWeight(node->getName(), 0);
            }
        }
        ik->addNullspaceGradient(nsjla);
    }

    VirtualRobot::NullspaceManipulabilityPtr nsman = nullptr;
    float kGain = ui->kGainNullspace->value();
    if (ui->checkBoxManipulabilityNullspace->isChecked() && kGain > 0) {
        std::cout << "Adding manipulability as nullspace target" << std::endl;
        auto manipTracking = getManipulabilityTracking(clonedRobot->getRobotNodeSet(currentRobotNodeSet->getName()),
                                                       currentRobotNodeSet2 ? clonedRobot->getRobotNodeSet(currentRobotNodeSet2->getName()) : nullptr);
        if (!manipTracking) {
            std::cout << "Manip tracking zero!" << std::endl;
            return;
        }
        Eigen::MatrixXd followManip = readFollowManipulability();
        if (followManip.rows() != manipTracking->getTaskVars()) {
            std::cout << "Wrong manipulability matrix!" << std::endl;
            return;
        }
        nsman = VirtualRobot::NullspaceManipulabilityPtr(new VirtualRobot::NullspaceManipulability(manipTracking, followManip, Eigen::MatrixXd(), true));
        nsman->kP = kGain;
        ik->addNullspaceGradient(nsman);
    }

    float kSoechting = ui->kSoechting->value();
    if (ui->checkBoxSoechtingNullspace->isChecked() && kSoechting > 0) {
        if (robot->getName() == "Armar6" && currentRobotNodeSet->getName() == "RightArm") {
            std::cout << "Adding soechting nullspace" << std::endl;
            VirtualRobot::SoechtingNullspaceGradient::ArmJoints armjoints;
            armjoints.clavicula = clonedRobot->getRobotNode("ArmR1_Cla1");
            armjoints.shoulder1 = clonedRobot->getRobotNode("ArmR2_Sho1");
            armjoints.shoulder2 = clonedRobot->getRobotNode("ArmR3_Sho2");
            armjoints.shoulder3 = clonedRobot->getRobotNode("ArmR4_Sho3");
            armjoints.elbow = clonedRobot->getRobotNode("ArmR5_Elb1");

            VirtualRobot::SoechtingNullspaceGradientPtr gradient(new VirtualRobot::SoechtingNullspaceGradient(target1, "ArmR2_Sho1", VirtualRobot::Soechting::ArmType::Right, armjoints));
            gradient->kP = kSoechting;
            ik->addNullspaceGradient(gradient);
        }
        else if (robot->getName() == "Armar6" && currentRobotNodeSet->getName() == "LeftArm") {
            std::cout << "Adding soechting nullspace" << std::endl;
            VirtualRobot::SoechtingNullspaceGradient::ArmJoints armjoints;
            armjoints.clavicula = clonedRobot->getRobotNode("ArmL1_Cla1");
            armjoints.shoulder1 = clonedRobot->getRobotNode("ArmL2_Sho1");
            armjoints.shoulder2 = clonedRobot->getRobotNode("ArmL3_Sho2");
            armjoints.shoulder3 = clonedRobot->getRobotNode("ArmL4_Sho3");
            armjoints.elbow = clonedRobot->getRobotNode("ArmL5_Elb1");

            VirtualRobot::SoechtingNullspaceGradientPtr gradient(new VirtualRobot::SoechtingNullspaceGradient(target1, "ArmL2_Sho1", VirtualRobot::Soechting::ArmType::Left, armjoints));
            gradient->kP = kSoechting;
            ik->addNullspaceGradient(gradient);
        }
        else std::cout << "Soechting currently supports only Armar6 and RightArm/LeftArm robot node set for first robot node set for demonstration" << std::endl;
    }

    ui->solveButton->setEnabled(false);
    ui->solveIKButton->setEnabled(false);
    emit solveIKAsync(ik, untilReached ? -1 : ui->ikSteps->value(), nsman);

}

void DiffIKWidget::setEndEffectorPose() {
    if (currentRobotNodeSet && currentRobotNodeSet->getTCP()) {
        auto tcp = currentRobotNodeSet->getTCP();
        auto pose = tcp->getPoseInRootFrame();
        auto position = simox::math::mat4f_to_pos(pose);
        ui->xTarget->setValue(position(0));
        ui->yTarget->setValue(position(1));
        ui->zTarget->setValue(position(2));
        auto rpy = simox::math::mat4f_to_rpy(pose);
        ui->rollTarget->setValue(rpy(0));
        ui->pitchTarget->setValue(rpy(1));
        ui->yawTarget->setValue(rpy(2));
    }
    if (currentRobotNodeSet2 && currentRobotNodeSet2->getTCP()) {
        auto tcp = currentRobotNodeSet2->getTCP();
        auto pose = tcp->getPoseInRootFrame();
        auto position = simox::math::mat4f_to_pos(pose);
        ui->xTarget2->setValue(position(0));
        ui->yTarget2->setValue(position(1));
        ui->zTarget2->setValue(position(2));
        auto rpy = simox::math::mat4f_to_rpy(pose);
        ui->rollTarget2->setValue(rpy(0));
        ui->pitchTarget2->setValue(rpy(1));
        ui->yawTarget2->setValue(rpy(2));
    }
}

void DiffIKWidget::workerFinished() {
    ui->solveButton->setEnabled(true);
    ui->solveIKButton->setEnabled(true);
    emit currentManipUpdated();
}

void DiffIKWidget::resetJointValues() {
    auto values = robot->getJointValues();
    for (auto &value : values) {
        value.second = 0;
    }
    robot->setJointValues(values);

    emit currentManipUpdated();
}

void DiffIKWidget::setAverageJointValues() {
    robot->setPropagatingJointValuesEnabled(false);
    for (auto node : robot->getRobotNodes()) {
        if (node->isJoint()) 
        {
            robot->setJointValue(node, (node->getJointLimitLo() + node->getJointLimitHi()) / 2.0f);
        }

    }
    robot->setPropagatingJointValuesEnabled(true);
    robot->updatePose();

    emit currentManipUpdated();
}

float randomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

void DiffIKWidget::setRandomJointValues() {
    robot->setPropagatingJointValuesEnabled(false);
    std::vector<std::string> robotNodeNames = currentRobotNodeSet->getNodeNames();
    if (ui->checkBoxBimanual && currentRobotNodeSet2) {
        auto robotNodeNames2 = currentRobotNodeSet2->getNodeNames();
        robotNodeNames.insert(robotNodeNames.end(), robotNodeNames2.begin(), robotNodeNames2.end());
    }
    for (const auto &name : robotNodeNames) {
        auto node = robot->getRobotNode(name);
        if (node->isJoint()) {
            float jointValue = randomFloat(node->getJointLimitLo(), node->getJointLimitHi());
            if (!std::isnan(jointValue))
                robot->setJointValue(node, jointValue);
        }
    }
    robot->setPropagatingJointValuesEnabled(true);
    robot->updatePose();

    emit currentManipUpdated();
}

void DiffIKWidget::setRobotNodeSet(QString name) {
    currentRobotNodeSet = robot->getRobotNodeSet(name.toStdString());
}

void DiffIKWidget::setRobotNodeSet2(QString name) {
    currentRobotNodeSet2 = robot->getRobotNodeSet(name.toStdString());
}

void printJacobi(VirtualRobot::RobotNodeSetPtr rns, bool scale) {
    if (!rns) return;
    VirtualRobot::DifferentialIKPtr diffIK(new VirtualRobot::DifferentialIK(rns, VirtualRobot::RobotNodePtr(), VirtualRobot::JacobiProvider::eSVDDamped));
    diffIK->convertModelScalingtoM(scale);
    Eigen::MatrixXd jacobian = diffIK->getJacobianMatrix(VirtualRobot::IKSolver::All).cast<double>();
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
    std::cout << "Jacobian matrix for " << rns->getName() << ":\n"
              << jacobian << "\n\n" << jacobian.format(CommaInitFmt) << "\n\n" << std::endl;
}

void DiffIKWidget::printJacobian() {
    printJacobi(currentRobotNodeSet, true);
}

void DiffIKWidget::printJacobian2() {
    printJacobi(currentRobotNodeSet, true);
}

void Worker::followManip(VirtualRobot::AbstractManipulabilityTrackingPtr manipTracking, VirtualRobot::RobotNodeSetPtr rns,
                         const Eigen::MatrixXd& followManip, float kGain, float maxDistance) {
    double distance = 1000;
    double lastDistance = 1000;
    int count = 0;
    while (distance > maxDistance) {
        Eigen::VectorXf velocity = manipTracking->calculateVelocity(followManip, Eigen::MatrixXd(), true);
        Eigen::VectorXf jointValues = rns->getJointValuesEigen() + velocity * kGain;
        rns->setJointValues(jointValues);
        distance = manipTracking->computeDistance(followManip);
        emit distanceUpdated(distance);
        if (distance / lastDistance < 0.99) {
            emit jointValuesUpdated(rns->getJointValueMap());
            emit currentManipUpdated();

            lastDistance = distance;
            count = 0;
        }
        else count++;
        if (count > 1000) break; // stop after 1000 iterations without large change
    }
    if (distance < maxDistance) {
        emit jointValuesUpdated(rns->getJointValueMap());
        emit distanceUpdated(distance);
        emit currentManipUpdated();
    }
    else emit distanceUpdated(lastDistance);
    emit finished();
}

void Worker::solveIK(VirtualRobot::CompositeDiffIKPtr ik, int steps, VirtualRobot::NullspaceManipulabilityPtr nsman) {
    VirtualRobot::CompositeDiffIK::Parameters cp;
    cp.resetRnsValues = false;
    cp.returnIKSteps = true;
    cp.steps = 1;
    VirtualRobot::CompositeDiffIK::SolveState state;
    ik->solve(cp, state);

    int i = 0;
    while (i < steps || (steps < 0 && !ik->getLastResult().reached && i < 1000)) {
        ik->step(cp, state, i);
        emit jointValuesUpdated(ik->getRobotNodeSet()->getJointValueMap());
        if (nsman) {
            emit distanceUpdated(nsman->computeDistance());
            emit currentManipUpdated();
        }
        i++;
    }
    emit jointValuesUpdated(ik->getRobotNodeSet()->getJointValueMap());
    if (nsman) {
        emit distanceUpdated(nsman->computeDistance());
        emit currentManipUpdated();
    }
    emit finished();
}
