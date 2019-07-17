
#include "PoseQualityMeasurement.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;

namespace VirtualRobot
{


    PoseQualityMeasurement::PoseQualityMeasurement(VirtualRobot::RobotNodeSetPtr rns)
        : rns(rns)
    {
        THROW_VR_EXCEPTION_IF((!rns || !rns->getTCP()), "NULL data");
        name = "PoseQualityMeasurement";
        considerObstacle = false;
        verbose = false;
    }


    PoseQualityMeasurement::~PoseQualityMeasurement()
    = default;

    float PoseQualityMeasurement::getPoseQuality()
    {
        VR_WARNING << "Please use derived classes..." << endl;
        return 0.0f;
    }

    float PoseQualityMeasurement::getPoseQuality(const Eigen::VectorXf& /*direction*/)
    {
        VR_ASSERT(direction.rows() == 3 || direction.rows() == 6);
        VR_WARNING << "Please use derived classes..." << endl;
        return 0.0f;
    }

    void PoseQualityMeasurement::setVerbose(bool v)
    {
        verbose = v;
    }

    std::string PoseQualityMeasurement::getName()
    {
        return name;
    }

    VirtualRobot::RobotNodeSetPtr PoseQualityMeasurement::getRNS()
    {
        return rns;
    }

    bool PoseQualityMeasurement::consideringJointLimits()
    {
        return false;
    }

    void PoseQualityMeasurement::setObstacleDistanceVector(const Eigen::Vector3f& directionSurfaceToObstance)
    {
        considerObstacle = true;
        obstacleDir = directionSurfaceToObstance;
    }

    void PoseQualityMeasurement::disableObstacleDistance()
    {
        considerObstacle = false;
    }

    PoseQualityMeasurementPtr PoseQualityMeasurement::clone(RobotPtr newRobot)
    {
        VR_ASSERT(newRobot);
        VR_ASSERT(newRobot->getRobotNodeSet(rns->getName()));
        VR_ASSERT(newRobot->getRobotNodeSet(rns->getName())->getSize() == rns->getSize());

        PoseQualityMeasurementPtr m(new PoseQualityMeasurement(newRobot->getRobotNodeSet(rns->getName())));
        return m;
    }

}
