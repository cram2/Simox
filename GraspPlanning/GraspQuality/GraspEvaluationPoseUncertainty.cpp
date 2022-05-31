#include "GraspEvaluationPoseUncertainty.h"

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Random.h>
#include <VirtualRobot/math/Helpers.h>
#include <VirtualRobot/math/ClampedNormalDistribution.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <random>
#include <cfloat>
#include <cstdlib>

using namespace VirtualRobot;

namespace GraspStudio
{

    GraspEvaluationPoseUncertainty::GraspEvaluationPoseUncertainty(const PoseUncertaintyConfig& config)
    {
        this->_config = config;
    }


    GraspEvaluationPoseUncertainty::~GraspEvaluationPoseUncertainty()
        = default;


    std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(
        const Eigen::Matrix4f& objectGP, const Eigen::Matrix4f& graspCenterGP)
    {
        std::vector<Eigen::Matrix4f> result;
        Eigen::Matrix4f trafoGraspCenterToObjectCenter = objectGP * graspCenterGP.inverse();
        Eigen::Matrix4f initPose = graspCenterGP;
        Eigen::Vector3f rpy;
        MathTools::eigen4f2rpy(initPose, rpy);

        float initPoseRPY[6];
        initPoseRPY[0] = initPose(0, 3);
        initPoseRPY[1] = initPose(1, 3);
        initPoseRPY[2] = initPose(2, 3);
        initPoseRPY[3] = rpy(0);
        initPoseRPY[4] = rpy(1);
        initPoseRPY[5] = rpy(2);

        float start[6];
        float end[6];
        float step[6];
        float tmpPose[6];

        for (int i = 0; i < 6; i++)
        {
            if (_config.enableDimension[i])
            {
                start[i] = initPoseRPY[i] - _config.dimExtends[i];
                end[i] = initPoseRPY[i] + _config.dimExtends[i];
                step[i] = _config.stepSize[i];
            }
            else
            {
                start[i] = initPoseRPY[i];
                end[i] = initPoseRPY[i];
                step[i] = 1.0f;
            }
        }

        Eigen::Matrix4f m;

        for (float a = start[0]; a <= end[0]; a += step[0])
        {
            tmpPose[0] = a;
            for (float b = start[1]; b <= end[1]; b += step[1])
            {
                tmpPose[1] = b;
                for (float c = start[2]; c <= end[2]; c += step[2])
                {
                    tmpPose[2] = c;
                    for (float d = start[3]; d <= end[3]; d += step[3])
                    {
                        tmpPose[3] = d;
                        for (float e = start[4]; e <= end[4]; e += step[4])
                        {
                            tmpPose[4] = e;
                            for (float f = start[5]; f <= end[5]; f += step[5])
                            {
                                tmpPose[5] = f;
                                MathTools::posrpy2eigen4f(tmpPose, m);
                                m = m * trafoGraspCenterToObjectCenter;
                                result.push_back(m);
                            }
                        }
                    }
                }
            }
        }
        return result;
    }

    std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(const Eigen::Matrix4f& objectGP, const EndEffector::ContactInfoVector& contacts)
    {
        Eigen::Vector3f centerPos = getMean(contacts);
        if (centerPos.hasNaN())
        {
            VR_ERROR << "No contacts" << std::endl;
            return std::vector<Eigen::Matrix4f>();
        }

        if (_config.verbose)
        {
            std::cout << "using contact center pose:\n" << centerPos << std::endl;
        }

        return generatePoses(objectGP, math::Helpers::Pose(centerPos));
    }


    std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(
        const Eigen::Matrix4f& objectGP, const Eigen::Matrix4f& graspCenterGP, int numPoses)
    {
        std::vector<Eigen::Matrix4f> result;
        //Eigen::Matrix4f trafoGraspCenterToObjectCenter = objectGP * graspCenterGP.inverse();
        Eigen::Matrix4f trafoGraspCenterToObjectCenter = math::Helpers::InvertedPose(graspCenterGP) * objectGP;
        Eigen::Matrix4f initPose = graspCenterGP;
        Eigen::Vector3f rpy;
        MathTools::eigen4f2rpy(initPose, rpy);

        float initPoseRPY[6];
        initPoseRPY[0] = initPose(0, 3);
        initPoseRPY[1] = initPose(1, 3);
        initPoseRPY[2] = initPose(2, 3);
        initPoseRPY[3] = rpy(0);
        initPoseRPY[4] = rpy(1);
        initPoseRPY[5] = rpy(2);

        float start[6];
        float dist[6];
        float tmpPose[6];
        for (int i = 0; i < 6; i++)
        {
            start[i] = initPoseRPY[i];
            if (_config.enableDimension[i])
            {
                if (i < 3)
                {
                    dist[i] = _config.posDeltaMM;
                }
                else
                {
                    dist[i] = _config.oriDeltaDeg * static_cast<float>(M_PI / 180.0);
                }
            }
            else
            {
                dist[i] = 0.0f;
            }
        }

        Eigen::Matrix4f m;

        VirtualRobot::ClampedNormalDistribution<float> normalDistribution(-1, 1);
        std::uniform_real_distribution<float> uniformDistribution(-1, 1);

        for (int j = 0; j < numPoses; j++)
        {
            for (int i = 0; i < 6; i++)
            {
                float r = _config.useNormalDistribution ? normalDistribution(VirtualRobot::PRNG64Bit())
                          : uniformDistribution(VirtualRobot::PRNG64Bit());
                tmpPose[i] = start[i] + r * dist[i];
            }
            MathTools::posrpy2eigen4f(tmpPose, m);
            m = m * trafoGraspCenterToObjectCenter;
            result.push_back(m);
        }
        return result;
    }

    std::vector<Eigen::Matrix4f> GraspEvaluationPoseUncertainty::generatePoses(
        const Eigen::Matrix4f& objectGP,
        const VirtualRobot::EndEffector::ContactInfoVector& contacts,
        int numPoses)
    {
        Eigen::Vector3f centerPose = getMean(contacts);
        if (centerPose.hasNaN())
        {
            VR_ERROR << "No contacts" << std::endl;
            return std::vector<Eigen::Matrix4f>();
        }

        if (_config.verbose)
        {
            std::cout << "using contact center pose:\n" << centerPose << std::endl;
        }

        return generatePoses(objectGP, math::Helpers::Pose(centerPose), numPoses);
    }

    GraspEvaluationPoseUncertainty::PoseEvalResult GraspEvaluationPoseUncertainty::evaluatePose(
        EndEffectorPtr eef, GraspableSensorizedObjectPtr object, const Eigen::Matrix4f& objectPose,
        GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape,
        float closingStepSize, float stepSizeSpeedFactor)
    {
        PoseEvalResult result;
        result.forceClosure = false;
        result.quality = 0.0f;
        result.initialCollision = false;

        SceneObjectSetPtr eefColModel = eef->createSceneObjectSet();

        if (!eef || !qm)
        {
            VR_ERROR << "Missing parameters" << std::endl;
            return result;
        }

        if (preshape)
        {
            eef->getRobot()->setJointValues(preshape);
        }
        else
        {
            eef->openActors(nullptr, closingStepSize, stepSizeSpeedFactor);
        }
        object->setGlobalPose(objectPose);

        // check for initial collision
        if (object->getCollisionChecker()->checkCollision(object->getCollisionModel(), eefColModel))
        {
            result.initialCollision = true;
            return result;
        }

        // collision free
        EndEffector::ContactInfoVector cont = eef->closeActors(object, closingStepSize, stepSizeSpeedFactor);
        qm->setContactPoints(cont);

        result.quality = qm->getGraspQuality();
        result.forceClosure = qm->isGraspForceClosure();

        return result;
    }

    GraspEvaluationPoseUncertainty::PoseEvalResults GraspEvaluationPoseUncertainty::evaluatePoses(
        EndEffectorPtr eef, GraspableSensorizedObjectPtr object, const std::vector<Eigen::Matrix4f>& objectPoses,
        GraspQualityMeasurePtr qm, VirtualRobot::RobotConfigPtr preshape,
        float closingStepSize, float stepSizeSpeedFactor)
    {
        if (objectPoses.empty())
        {
            return {};
        }

        if (!eef || !qm)
        {
            VR_ERROR << "Missing parameters" << std::endl;
            return {};
        }

        if (!eef->getRobot())
        {
            VR_WARNING << "missing eef->robot" << std::endl;
            return {};
        }

        Eigen::Matrix4f eefRobotPoseInit = eef->getRobot()->getGlobalPose();
        Eigen::Matrix4f objectPoseInit = object->getGlobalPose();
        VirtualRobot::RobotConfigPtr initialConf = eef->getConfiguration();

        std::vector<PoseEvalResult> results;
        results.reserve(objectPoses.size());
        for (const auto& objectPose : objectPoses)
        {
            results.emplace_back(evaluatePose(eef, object, objectPose, qm, preshape, closingStepSize, stepSizeSpeedFactor));
        }

        PoseEvalResults res;
        res.numPosesTested = static_cast<int>(results.size());
        for (const auto& result : results)
        {
            if (result.initialCollision)
            {
                res.numColPoses++;
            }
            else
            {
                res.numValidPoses++;
                res.avgQuality += result.quality;
                res.avgQualityCol += result.quality;
                if (result.forceClosure)
                {
                    res.forceClosureRate += 1.0f;
                    res.forceClosureRateCol += 1.0f;
                    res.numForceClosurePoses++;
                }
            }
        }

        if (res.numValidPoses > 0)
        {
            res.forceClosureRate /= static_cast<float>(res.numValidPoses);
            res.avgQuality /= static_cast<float>(res.numValidPoses);
        }
        if (res.numPosesTested > 0)
        {
            res.forceClosureRateCol /= static_cast<float>(res.numPosesTested);
            res.avgQualityCol /= static_cast<float>(res.numPosesTested);
        }

        // restore setup
        eef->getRobot()->setGlobalPose(eefRobotPoseInit);
        object->setGlobalPose(objectPoseInit);
        eef->getRobot()->setConfig(initialConf);

        return res;
    }

    GraspEvaluationPoseUncertainty::PoseEvalResults GraspEvaluationPoseUncertainty::evaluateGrasp(
        VirtualRobot::GraspPtr grasp, VirtualRobot::EndEffectorPtr eef, VirtualRobot::GraspableSensorizedObjectPtr object,
        GraspQualityMeasurePtr qm, int numPoses,
        float closingStepSize, float stepSizeSpeedFactor)
    {
        PoseEvalResults res;
        res.avgQuality = 0.0f;
        res.forceClosureRate = 0.0f;
        res.numPosesTested = 0;
        res.numValidPoses = 0;
        res.numColPoses = 0;

        if (!grasp || !eef || !object || !qm)
        {
            VR_WARNING << "missing parameters" << std::endl;
            return res;
        }
        if (!eef->getRobot())
        {
            VR_WARNING << "missing eef->robot" << std::endl;
            return res;
        }

        Eigen::Matrix4f eefRobotPoseInit = eef->getRobot()->getGlobalPose();
        Eigen::Matrix4f objectPoseInit = object->getGlobalPose();
        VirtualRobot::RobotConfigPtr initialConf = eef->getConfiguration();

        std::string graspPreshapeName = grasp->getPreshapeName();
        VirtualRobot::RobotConfigPtr graspPS;
        if (eef->hasPreshape(graspPreshapeName))
        {
            graspPS = eef->getPreshape(graspPreshapeName);
        }

        Eigen::Matrix4f mGrasp = grasp->getTcpPoseGlobal(object->getGlobalPose());

        // apply grasp
        eef->getRobot()->setGlobalPoseForRobotNode(eef->getTcp(), mGrasp);

        if (graspPS)
        {
            eef->getRobot()->setJointValues(graspPS);
        }
        else
        {
            eef->openActors(nullptr, closingStepSize, stepSizeSpeedFactor);
        }



        auto contacts = eef->closeActors(object, closingStepSize, stepSizeSpeedFactor);
        if (contacts.empty())
        {
            VR_INFO << "No contacts for grasp " << grasp->getName() << " found" << std::endl;
            return res;
        }

        auto poses = generatePoses(object->getGlobalPose(), contacts, numPoses);
        if (poses.empty())
        {
            VR_INFO << "No poses for grasp found" << std::endl;
            return res;
        }
        res = evaluatePoses(eef, object, poses, qm, graspPS, closingStepSize, stepSizeSpeedFactor);

        // restore setup
        eef->getRobot()->setGlobalPose(eefRobotPoseInit);
        object->setGlobalPose(objectPoseInit);
        eef->getRobot()->setConfig(initialConf);

        return res;
    }

    Eigen::Vector3f GraspEvaluationPoseUncertainty::getMean(const EndEffector::ContactInfoVector& contacts) const
    {
        Eigen::Vector3f mean = mean.Zero();
        if (contacts.empty())
        {
            VR_ERROR << "No contacts" << std::endl;
            return Eigen::Vector3f::Constant(std::nanf(""));
        }

        for (size_t i = 0; i < contacts.size(); i++)
        {
            if (_config.verbose)
            {
                std::cout << "contact point:" << i << ": \n" << contacts[i].contactPointObstacleGlobal << std::endl;
            }
            mean += contacts[i].contactPointObstacleGlobal;
        }
        mean /= contacts.size();
        return mean;
    }


    GraspEvaluationPoseUncertainty::PoseUncertaintyConfig& GraspEvaluationPoseUncertainty::config()
    {
        return _config;
    }

    const GraspEvaluationPoseUncertainty::PoseUncertaintyConfig& GraspEvaluationPoseUncertainty::config() const
    {
        return _config;
    }


    std::ostream& operator<<(std::ostream& os, const GraspEvaluationPoseUncertainty::PoseEvalResults& rhs)
    {
        os << "Robustness analysis" << std::endl;
        os << "Num Poses Tested: \t" << rhs.numPosesTested << std::endl;
        os << "Num Poses Valid:  \t" << rhs.numValidPoses << std::endl;

        float colPercent = 0.0f;
        if (rhs.numPosesTested > 0)
        {
            colPercent = float(rhs.numColPoses) / float(rhs.numPosesTested) * 100.0f;
        }

        os << "Num Poses initially in collision:\t" << rhs.numColPoses << " (" << colPercent << "%)" << std::endl;
        os << "Avg Quality (only col freeposes):\t" << rhs.avgQuality << std::endl;
        os << "FC rate (only col free poses):   \t" << rhs.forceClosureRate * 100.0f << "%" << std::endl;
        os << "Avg Quality (all poses):         \t" << rhs.avgQualityCol << std::endl;
        os << "FC rate (all poses):             \t" << rhs.forceClosureRateCol * 100.0f << "%" << std::endl;

        return os;
    }

}
