/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*/
#include "BasicGraspQualityMeasure.h"
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>

using namespace std;
using namespace VirtualRobot;

namespace VirtualRobot
{


    BasicGraspQualityMeasure::BasicGraspQualityMeasure(VirtualRobot::SceneObjectPtr object)
        : object(object)
    {
        THROW_VR_EXCEPTION_IF(!object, "Need an object");
        THROW_VR_EXCEPTION_IF(!object->getCollisionModel(), "Need an object with collision model");
        THROW_VR_EXCEPTION_IF(!object->getCollisionModel()->getTriMeshModel(), "Need an object with trimeshmodel");

        //Member variable representing Grasp Quality ranging from 1 to 0
        graspQuality = 0.0;
        verbose = false;
        objectLength = 0.0f;

        centerOfModel = object->getCollisionModel()->getTriMeshModel()->getCOM();

        Eigen::Vector3f minS, maxS;
        object->getCollisionModel()->getTriMeshModel()->getSize(minS, maxS);
        maxS = maxS - minS;
        objectLength = maxS(0);

        if (maxS(1) > objectLength)
        {
            objectLength = maxS(1);
        }

        if (maxS(2) > objectLength)
        {
            objectLength = maxS(2);
        }

        graspQuality = 0;
        maxContacts = 5;
    }

    bool BasicGraspQualityMeasure::calculateGraspQuality()
    {
        graspQuality = static_cast<float>(contactPoints.size()) / static_cast<float>(maxContacts);

        if (graspQuality > 1.0f)
        {
            graspQuality = 1.0f;
        }

        return true;
    }

    float BasicGraspQualityMeasure::getGraspQuality()
    {
        calculateGraspQuality();
        return graspQuality;
    }

    BasicGraspQualityMeasure::~BasicGraspQualityMeasure()
    = default;

    void BasicGraspQualityMeasure::setContactPoints(const std::vector<VirtualRobot::MathTools::ContactPoint>& /*contactPoints6d*/)
    {
        this->contactPoints.clear();
        this->contactPointsM.clear();
        std::vector<MathTools::ContactPoint>::const_iterator objPointsIter;

        for (objPointsIter = contactPoints.begin(); objPointsIter != contactPoints.end(); objPointsIter++)
        {
            MathTools::ContactPoint point = (*objPointsIter);
            point.p -= centerOfModel;
            point.n.normalize();
            point.force = 1.0f;

            this->contactPoints.push_back(point);
        }

        MathTools::convertMM2M(this->contactPoints, this->contactPointsM);

        if (verbose)
        {
            VR_INFO << ": Nr of contact points:" << this->contactPoints.size() << std::endl;
        }
    }

    void BasicGraspQualityMeasure::setContactPoints(const EndEffector::ContactInfoVector& contactPoints)
    {
        this->contactPoints.clear();
        this->contactPointsM.clear();

        this->contactPoints.reserve(contactPoints.size());
        this->contactPointsM.reserve(contactPoints.size());

        for (const auto& objPoint : contactPoints)
        {
            this->contactPoints.emplace_back();
            MathTools::ContactPoint& point = this->contactPoints.back();

            point.p = objPoint.contactPointObstacleLocal;
            point.p -= centerOfModel;

            point.n = objPoint.contactPointFingerLocal - objPoint.contactPointObstacleLocal;
            point.n.normalize();

            // store force as projected component of approachDirection
            const Eigen::Vector3f nGlob = objPoint.contactPointObstacleGlobal - objPoint.contactPointFingerGlobal;

            if (nGlob.norm() > 1e-10f)
            {
                point.force = nGlob.dot(objPoint.approachDirectionGlobal) / nGlob.norm();
            }
            else
            {
                point.force = 0;
            }
        }

        VirtualRobot::MathTools::convertMM2M(this->contactPoints, this->contactPointsM);

        if (verbose)
        {
            VR_INFO << ": Nr of contact points:" << this->contactPoints.size() << std::endl;
        }
    }


    MathTools::ContactPoint BasicGraspQualityMeasure::getContactPointsCenter()
    {
        MathTools::ContactPoint p;
        p.p.setZero();
        p.n.setZero();

        if (contactPoints.size() == 0)
        {
            return p;
        }

        for (auto & contactPoint : contactPoints)
        {
            p.p += contactPoint.p;
            p.n += contactPoint.n;

        }

        p.p /= static_cast<float>(contactPoints.size());
        p.n /= static_cast<float>(contactPoints.size());

        return p;
    }

    void BasicGraspQualityMeasure::setVerbose(bool enable)
    {
        verbose = enable;
    }

    std::string BasicGraspQualityMeasure::getName()
    {
        std::string sName("BasicGraspQualityMeasure");
        return sName;
    }

    Eigen::Vector3f BasicGraspQualityMeasure::getCoM()
    {
        return centerOfModel;
    }

    VirtualRobot::SceneObjectPtr BasicGraspQualityMeasure::getObject()
    {
        return object;
    }

    bool BasicGraspQualityMeasure::isValid()
    {
        return (contactPoints.size() >= 2);

    }


} // namespace
