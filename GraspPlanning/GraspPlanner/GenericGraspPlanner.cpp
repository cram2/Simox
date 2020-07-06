#include "GenericGraspPlanner.h"

#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotConfig.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/math/Helpers.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>


#include "../GraspQuality/GraspQualityMeasureWrenchSpace.h"
#include "../GraspQuality/GraspQualityMeasure.h"
#include "../ApproachMovementGenerator.h"


using namespace std;


namespace GraspStudio
{


    GenericGraspPlanner::GenericGraspPlanner(VirtualRobot::GraspSetPtr graspSet, GraspStudio::GraspQualityMeasurePtr graspQuality, GraspStudio::ApproachMovementGeneratorPtr approach, float minQuality, bool forceClosure)
        : GraspPlanner(graspSet), graspQuality(graspQuality), approach(approach), minQuality(minQuality), forceClosure(forceClosure)
    {
        THROW_VR_EXCEPTION_IF(!graspQuality, "NULL grasp quality...");
        THROW_VR_EXCEPTION_IF(!approach, "NULL approach...");
        THROW_VR_EXCEPTION_IF(!graspQuality->getObject(), "no object...");
        THROW_VR_EXCEPTION_IF(graspQuality->getObject() != approach->getObject(), "graspQuality and approach have to use the same object.");
        object = graspQuality->getObject();
        eef = approach->getEEF();
        THROW_VR_EXCEPTION_IF(!eef, "NULL eef in approach...");
        THROW_VR_EXCEPTION_IF(!graspSet, "NULL graspSet...");
        verbose = true;
        eval.fcCheck = forceClosure;
        eval.minQuality = minQuality;
        retreatOnLowContacts = true;
    }

    GenericGraspPlanner::~GenericGraspPlanner()
        = default;

    int GenericGraspPlanner::plan(int nrGrasps, int timeOutMS, VirtualRobot::SceneObjectSetPtr obstacles)
    {
        startTime = std::chrono::system_clock::now();
        this->timeOutDuration = std::chrono::milliseconds(timeOutMS);

        int nLoop = 0;
        int nGraspsCreated = 0;

        if (verbose)
        {
            GRASPSTUDIO_INFO << ": Searching " << nrGrasps << " grasps for EEF '" << approach->getEEF()->getName()
                             << "' and object '" << graspQuality->getObject()->getName() << "'.\n"
                             << "timeout set to " << timeOutMS << " ms.\n";
            GRASPSTUDIO_INFO << ": Approach movements are generated with " << approach->getName() << std::endl;
            GRASPSTUDIO_INFO << ": Grasps are evaluated with " << graspQuality->getName() << std::endl;
        }

        while (!timeout() && nGraspsCreated < nrGrasps)
        {
            VirtualRobot::GraspPtr g = planGrasp(obstacles);

            if (g)
            {
                if (graspSet)
                {
                    graspSet->addGrasp(g);
                }

                plannedGrasps.push_back(g);
                nGraspsCreated++;
            }

            nLoop++;
        }

        if (verbose)
        {
            const auto endt = std::chrono::system_clock::now();
            const auto dt = endt - startTime;
            const auto dtms = std::chrono::duration_cast<std::chrono::milliseconds>(dt);
            GRASPSTUDIO_INFO << ": created " << nGraspsCreated << " valid grasps in " << nLoop << " loops"
                              << "\n took " << dtms.count() << " ms " << std::endl;
        }

        return nGraspsCreated;
    }

    bool GenericGraspPlanner::moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops)
    {
        VR_ASSERT(eef);
        VR_ASSERT(approach);

        VirtualRobot::SceneObjectSetPtr sos = eef->createSceneObjectSet();

        if (!sos)
        {
            return false;
        }

        int loop = 0;
        Eigen::Vector3f delta = approachDir * step;
        bool finishedContactsOK = false;
        bool finishedCollision = false;

        while (loop < maxLoops && !finishedCollision && !finishedContactsOK)
        {
            approach->openHand();
            approach->updateEEFPose(delta);

            if (eef->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
            {
                finishedCollision = true;
                break;
            }
            auto contacts = eef->closeActors(object);
            if (contacts.size() >= 2)
            {
                approach->openHand();
                finishedContactsOK = true;
                break;
            }
            loop++;
        }
        return finishedContactsOK;
    }

    VirtualRobot::GraspPtr GenericGraspPlanner::planGrasp(VirtualRobot::SceneObjectSetPtr obstacles)
    {
        auto start_time = chrono::high_resolution_clock::now();
        std::string graspPlannerName = "Simox - GraspStudio - " + graspQuality->getName();
        std::string graspNameBase = "Grasp ";

        VirtualRobot::RobotPtr robot = approach->getEEFOriginal()->getRobot();
        VirtualRobot::RobotNodePtr tcp = eef->getTcp();

        VR_ASSERT(robot);
        VR_ASSERT(tcp);

        // GENERATE APPROACH POSE
        // Eigen::Matrix4f pose = approach->createNewApproachPose();
        // bRes = approach->setEEFPose(pose);

        bool bRes = approach->setEEFToRandomApproachPose();

        if (bRes && obstacles)
        {
            // CHECK VALID APPROACH POSE
            VirtualRobot::CollisionCheckerPtr colChecker = eef->getCollisionChecker();
            VR_ASSERT(eef->getRobot());
            VR_ASSERT(obstacles);

            if (colChecker->checkCollision(eef->createSceneObjectSet(), obstacles))
            {
                //                GRASPSTUDIO_INFO << ": Collision detected before closing fingers" << std::endl;
                //return VirtualRobot::GraspPtr();
                bRes = false;
            }
        }

        contacts.clear();

        // CHECK CONTACTS
        if (bRes)
        {
            contacts = eef->closeActors(object);

            eef->addStaticPartContacts(object, contacts, approach->getApproachDirGlobal());

            // low number of contacts: check if it helps to move away (small object)
            if (retreatOnLowContacts && contacts.size() < 2)
            {
                if (verbose)
                {
                    VR_INFO << "Low number of contacts, retreating hand (might be useful for a small object)" << std::endl;
                }
                if (moveEEFAway(approach->getApproachDirGlobal(), 5.0f, 10))
                {
                    contacts = eef->closeActors(object);
                    eef->addStaticPartContacts(object, contacts, approach->getApproachDirGlobal());
                }
            }

            if (obstacles)
            {
                VirtualRobot::CollisionCheckerPtr colChecker = eef->getCollisionChecker();
                VR_ASSERT(eef->getRobot());
                VR_ASSERT(obstacles);

                if (colChecker->checkCollision(eef->createSceneObjectSet(), obstacles))
                {
                    //              GRASPSTUDIO_INFO << ": Collision detected after closing fingers" << std::endl;
                    //return VirtualRobot::GraspPtr();
                    bRes = false;
                }
            }
        }

        // construct grasp
        std::stringstream graspName;
        graspName << graspNameBase << (graspSet->getSize() + 1);

        Eigen::Matrix4f poseObject = object->getGlobalPose();
        Eigen::Matrix4f poseTcp = tcp->toLocalCoordinateSystem(poseObject);

        VirtualRobot::GraspPtr grasp(new VirtualRobot::Grasp(
                                         graspName.str(), robot->getType(), eef->getName(), poseTcp,
                                         graspPlannerName, 0 /*score*/));

        // set joint config
        grasp->setConfiguration(eef->getConfiguration()->getRobotNodeJointValueMap());

        bool returnNullWhenInvalid = true;

        // eval data
        eval.graspTypePower.push_back(true);
        eval.nrGraspsGenerated++;

        auto getTimeMS = [&start_time]()
        {
            auto end_time = chrono::high_resolution_clock::now();
            float ms = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
            return ms;
        };

        if (!bRes)
        {
            // result not valid due to collision
            float ms = getTimeMS();
            eval.graspScore.push_back(0.0f);
            eval.graspValid.push_back(false);
            eval.nrGraspsInvalidCollision++;
            eval.timeGraspMS.push_back(ms);

            return returnNullWhenInvalid ? nullptr : grasp;
        }

        if (contacts.size() < 2)
        {
            if (verbose)
            {
                GRASPSTUDIO_INFO << ": ignoring grasp hypothesis, low number of contacts" << std::endl;
            }
            // result not valid due to low number of contacts
            float ms = getTimeMS();
            eval.graspScore.push_back(0.0f);
            eval.graspValid.push_back(false);
            eval.nrGraspsInvalidContacts++;
            eval.timeGraspMS.push_back(ms);

            return returnNullWhenInvalid ? nullptr : grasp;
        }

        graspQuality->setContactPoints(contacts);
        float score = graspQuality->getGraspQuality();
        grasp->setQuality(score);

        if (forceClosure && !graspQuality->isGraspForceClosure())
        {
            // not force closure
            float ms = getTimeMS();
            eval.graspScore.push_back(0.0f);
            eval.graspValid.push_back(false);
            eval.nrGraspsInvalidFC++;
            eval.timeGraspMS.push_back(ms);

            return returnNullWhenInvalid ? nullptr : grasp;
        }

        if (score < minQuality)
        {
            // min quality not reached
            float ms = getTimeMS();
            eval.graspScore.push_back(score);
            eval.graspValid.push_back(false);
            eval.nrGraspsInvalidFC++;
            eval.timeGraspMS.push_back(ms);

            return returnNullWhenInvalid ? nullptr : grasp;
        }

        // found valid grasp
        if (verbose)
        {
            GRASPSTUDIO_INFO << ": Found grasp with " << contacts.size() << " contacts, score: " << score << std::endl;
        }

        float ms = getTimeMS();

        eval.graspScore.push_back(score);
        eval.graspValid.push_back(true);
        eval.nrGraspsValid++;
        // only power grasps
        eval.nrGraspsValidPower++;
        eval.timeGraspMS.push_back(ms);

        return grasp;
    }

    VirtualRobot::EndEffector::ContactInfoVector GenericGraspPlanner::getContacts() const
    {
        return contacts;
    }

    void GenericGraspPlanner::setParameters(float minQuality, bool forceClosure)
    {
        this->minQuality = minQuality;
        this->forceClosure = forceClosure;
    }

    void GenericGraspPlanner::setRetreatOnLowContacts(bool enable)
    {
        retreatOnLowContacts = enable;
    }

    bool GenericGraspPlanner::timeout()
    {
        return timeOutDuration.count() <= 0
               || std::chrono::system_clock::now() > (startTime + timeOutDuration);
    }

} // namespace
