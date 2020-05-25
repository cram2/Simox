#include "ApproachMovementSurfaceNormal.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotConfig.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/SceneObjectSet.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>

#include <Eigen/Geometry>


using namespace std;
using namespace VirtualRobot;


namespace GraspStudio
{

    ApproachMovementSurfaceNormal::ApproachMovementSurfaceNormal(VirtualRobot::SceneObjectPtr object, VirtualRobot::EndEffectorPtr eef,
            const std::string& graspPreshape, float maxRetreatDist, bool useFaceAreaDistribution)
        : ApproachMovementGenerator(object, eef, graspPreshape),
          distribUniform(0, objectModel->faces.size() - 1),
          distribRetreatDistance(0, maxRetreatDist)
    {
        name = "ApproachMovementSurfaceNormal";

        if (useFaceAreaDistribution)
        {
            std::vector<float> areas = objectModel->getFaceAreas();
            distribFaceAreas = std::discrete_distribution<std::size_t>(areas.begin(), areas.end());
        }
    }

    ApproachMovementSurfaceNormal::~ApproachMovementSurfaceNormal()
        = default;

    bool ApproachMovementSurfaceNormal::getPositionOnObject(Eigen::Vector3f& storePos, Eigen::Vector3f& storeApproachDir)
    {
        if (!object || objectModel->faces.size() == 0)
        {
            return false;
        }

        std::size_t faceIndex = useFaceAreasDistrib ? distribFaceAreas(randomEngine)
                                : distribUniform(randomEngine);

        std::size_t nVert1 = (objectModel->faces[faceIndex]).id1;
        std::size_t nVert2 = (objectModel->faces[faceIndex]).id2;
        std::size_t nVert3 = (objectModel->faces[faceIndex]).id3;

        storePos = VirtualRobot::MathTools::randomPointInTriangle(objectModel->vertices[nVert1],
                   objectModel->vertices[nVert2],
                   objectModel->vertices[nVert3]);

        //storePos = (objectModel->vertices[nVert1] + objectModel->vertices[nVert2] + objectModel->vertices[nVert3]) / 3.0f;
        /*position(0) = (objectModel->vertices[nVert1].x + objectModel->vertices[nVert2].x + objectModel->vertices[nVert3].x) / 3.0f;
          position(1) = (objectModel->vertices[nVert1].y + objectModel->vertices[nVert2].y + objectModel->vertices[nVert3].y) / 3.0f;
          position(2) = (objectModel->vertices[nVert1].z + objectModel->vertices[nVert2].z + objectModel->vertices[nVert3].z) / 3.0f;*/

        storeApproachDir = (objectModel->faces[faceIndex]).normal;
        if (std::abs(storeApproachDir.squaredNorm() - 1) > 1e-6f)
        {
            if (verbose)
            {
                std::cout << "Normal in trimesh not normalized! (normalizing it now)\n";
            }
            storeApproachDir.normalize();
        }

        return true;
    }

    Eigen::Matrix4f ApproachMovementSurfaceNormal::createNewApproachPose()
    {
        // store current pose
        Eigen::Matrix4f pose = getEEFPose();
        openHand();
        Eigen::Vector3f position;
        Eigen::Vector3f approachDir;


        if (!getPositionOnObject(position, approachDir))
        {
            GRASPSTUDIO_ERROR << "no position on object?!" << std::endl;
            return pose;
        }

        this->approachDirGlobal = approachDir;

        // set new pose
        setEEFToApproachPose(position, approachDir);


        // move away until valid
        moveEEFAway(approachDir, 1.0f);

        Eigen::Matrix4f poseB = getEEFPose();


        // check if a random distance is wanted
        if (distribRetreatDistance.max() > 0)
        {
            float distance = distribRetreatDistance(randomEngine);
            Eigen::Vector3f delta = approachDir * distance;
            updateEEFPose(delta);

            if (!eef_cloned->getCollisionChecker()->checkCollision(object, eef->createSceneObjectSet()))
            {
                poseB = getEEFPose();
            } // else remain at original pose
        }

        // restore original pose
        setEEFPose(pose);

        return poseB;
    }

    bool ApproachMovementSurfaceNormal::setEEFToApproachPose(
        const Eigen::Vector3f& position, const Eigen::Vector3f& approachDir)
    {
        VirtualRobot::RobotNodePtr graspNode = eef_cloned->getGCP();

        // current pose
        //Eigen::Matrix4f pose = graspNode->getGlobalPose();

        // target pose
        Eigen::Matrix4f poseFinal = Eigen::Matrix4f::Identity();

        // position
        poseFinal.block(0, 3, 3, 1) = position;

        //target orientation
        Eigen::Vector3f z = approachDir;

        while (z.norm() < 1e-10f)
        {
            z.setRandom();
        }

        z.normalize();
        z *= -1.0f;

        Eigen::Vector3f y;
        Eigen::Vector3f x;

        // create a random rotation around approach vector
        bool bSuccess = false;
        int loop = 0;

        while (!bSuccess)
        {
            loop++;

            if (loop > 1000)
            {
                VR_ERROR << "INTERNAL ERROR, aborting..." << std::endl;
                return false;
            }

            //random y dir vector
            y.setRandom();

            if (y.norm() < 1e-8f)
            {
                continue;
            }

            y.normalize();

            x = y.cross(z);

            if (x.norm() < 1e-8f)
            {
                continue;
            }

            x.normalize();


            // now recalculate y again to obtain a correct base for a right handed coord system
            y = z.cross(x);

            if (y.norm() < 1e-8f)
            {
                continue;
            }

            y.normalize();

            bSuccess = true;
        }

        poseFinal.block(0, 0, 3, 1) = x;
        poseFinal.block(0, 1, 3, 1) = y;
        poseFinal.block(0, 2, 3, 1) = z;

        setEEFPose(poseFinal);

        return true;
    }

    void ApproachMovementSurfaceNormal::moveEEFAway(
        const Eigen::Vector3f& approachDir, float step, int maxLoops)
    {
        VirtualRobot::SceneObjectSetPtr sceneObjectSet = eef_cloned->createSceneObjectSet();
        if (!sceneObjectSet)
        {
            return;
        }

        CollisionModelPtr objectColModel = object->getCollisionModel();
        CollisionCheckerPtr eefCollChecker = eef_cloned->getCollisionChecker();

        Eigen::Vector3f delta = approachDir * step;
        int loop = 0;

        while (loop < maxLoops && eefCollChecker->checkCollision(objectColModel, sceneObjectSet))
        {
            updateEEFPose(delta);
            loop++;
        }
    }

    Eigen::Matrix4f ApproachMovementSurfaceNormal::getEEFPose()
    {
        RobotNodePtr tcp;
        if (!graspPreshape.empty() && eef_cloned->hasPreshape(graspPreshape) && eef_cloned->getPreshape(graspPreshape)->getTCP())
        {
            tcp = eef_cloned->getPreshape(graspPreshape)->getTCP();
        }
        else
        {
            tcp = eef_cloned->getGCP();
        }
        return tcp->getGlobalPose();
    }

    bool ApproachMovementSurfaceNormal::setEEFPose(const Eigen::Matrix4f& pose)
    {
        RobotNodePtr tcp;
        if (!graspPreshape.empty() && eef_cloned->hasPreshape(graspPreshape) && eef_cloned->getPreshape(graspPreshape)->getTCP())
        {
            tcp = eef_cloned->getPreshape(graspPreshape)->getTCP();
        }
        else
        {
            tcp = eef_cloned->getGCP();
        }
        eefRobot->setGlobalPoseForRobotNode(tcp, pose);
        return true;
    }

}
