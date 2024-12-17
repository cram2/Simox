#pragma once
#include <QComboBox>
#include <QDialog>
#include <QThread>
#include <QWidget>

#include <VirtualRobot/IK/CompositeDiffIK/CompositeDiffIK.h>
#include <VirtualRobot/IK/CompositeDiffIK/ManipulabilityNullspaceGradient.h>
#include <VirtualRobot/IK/IKSolver.h>
#include <VirtualRobot/Manipulability/AbstractManipulabilityTracking.h>
#include <VirtualRobot/VirtualRobot.h>

#include <Inventor/nodes/SoSeparator.h>

namespace Ui
{
    class DiffIKWidget;
}

Q_DECLARE_METATYPE(Eigen::MatrixXd)
Q_DECLARE_METATYPE(Eigen::VectorXf)
Q_DECLARE_METATYPE(VirtualRobot::AbstractManipulabilityTrackingPtr)
Q_DECLARE_METATYPE(VirtualRobot::CompositeDiffIKPtr)
Q_DECLARE_METATYPE(VirtualRobot::NullspaceManipulabilityPtr)
Q_DECLARE_METATYPE(VirtualRobot::RobotNodeSetPtr)

class Worker : public QObject
{
    Q_OBJECT

public slots:
    void followManip(VirtualRobot::AbstractManipulabilityTrackingPtr manip,
                     VirtualRobot::RobotNodeSetPtr rns,
                     const Eigen::MatrixXd& followManip,
                     float kGain,
                     float maxDistance);
    void solveIK(VirtualRobot::CompositeDiffIKPtr ik,
                 int steps,
                 VirtualRobot::NullspaceManipulabilityPtr = nullptr);

signals:
    void distanceUpdated(double distance);
    void currentManipUpdated();
    void jointValuesUpdated(const std::map<std::string, float>& jointValues);
    void finished();
};

class DiffIKWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DiffIKWidget(SoSeparator* sceneSep, QDialog* parent = nullptr);
    ~DiffIKWidget();

    QDialog* getDialog();

    static void open(QWidget* parent, SoSeparator* sceneSep);

    static void update(VirtualRobot::RobotPtr robot);

    static void close();

    static DiffIKWidget* diffIKWidget;

private slots:
    void updateCurrentManipulabilityEllipsoidVis();
    void updateFollowManipulabilityEllipsoidVis();
    void updateEndeffectorPoseVis();
    void stepFollowManip();
    void followManip();
    void updateDistance(double distance);
    void updateJointValues(const std::map<std::string, float>& jointValues);
    void solveIK(bool untilReached = false);
    void setEndEffectorPose();
    void workerFinished();
    void resetJointValues();
    void setAverageJointValues();
    void setRandomJointValues();
    void setRobotNodeSet(QString name);
    void setRobotNodeSet2(QString name);
    void printJacobian();
    void printJacobian2();

    Eigen::MatrixXd readFollowManipulability();

signals:
    void distanceUpdated(double distance);
    void currentManipUpdated();
    void followManipAsync(VirtualRobot::AbstractManipulabilityTrackingPtr manip,
                          VirtualRobot::RobotNodeSetPtr rns,
                          const Eigen::MatrixXd& followManip,
                          float kGain,
                          float maxDistance);
    void solveIKAsync(VirtualRobot::CompositeDiffIKPtr ik,
                      int steps,
                      VirtualRobot::NullspaceManipulabilityPtr nsman);

private:
    VirtualRobot::AbstractManipulability::Mode getMode(QComboBox* comboBox);
    VirtualRobot::AbstractManipulability::Type getManipulabilityType(QComboBox* comboBox);
    Eigen::Matrix4f getEndEffectorPos();
    Eigen::Matrix4f getEndEffectorPos2();

    void addRobotNodeSets();
    void updateEndeffectorPoseVis(VirtualRobot::RobotNodeSetPtr robotNodeSet,
                                  const Eigen::Matrix4f& pose);
    VirtualRobot::AbstractManipulabilityTrackingPtr getManipulabilityTracking();
    VirtualRobot::AbstractManipulabilityTrackingPtr
    getManipulabilityTracking(VirtualRobot::RobotNodeSetPtr r1,
                              VirtualRobot::RobotNodeSetPtr r2,
                              bool setNewRNS = false);

    QDialog* dialog;
    Ui::DiffIKWidget* ui;
    SoSeparator* manipSep;
    SoSeparator* followManipSep;
    SoSeparator* endeffectorSep;
    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
    VirtualRobot::RobotNodeSetPtr currentRobotNodeSet2;
    VirtualRobot::RobotNodeSetPtr newRNS;
    VirtualRobot::RobotPtr clonedRobot;
    QThread workerThread;
};
