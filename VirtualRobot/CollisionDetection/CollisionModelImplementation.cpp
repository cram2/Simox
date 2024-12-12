#include "CollisionModelImplementation.h"

namespace VirtualRobot
{

    CollisionModelImplementation::CollisionModelImplementation(
        const TriMeshModelPtr& modelData,
        const CollisionCheckerPtr& /*pColChecker*/,
        int id)
    {
        this->modelData = modelData;
        this->id = id;
    }


    void
    CollisionModelImplementation::setGlobalPose(const Eigen::Matrix4f& m)
    {
        globalPose = m;
    }

    const Eigen::Matrix4f&
    CollisionModelImplementation::getGlobalPose() const
    {
        return globalPose;
    }



    const TriMeshModelPtr&
    CollisionModelImplementation::getTriMeshModel()
    {
        return modelData;
    }

    CollisionModelImplementation::~CollisionModelImplementation()
    {
    }

    void
    CollisionModelImplementation::print()
    {
        std::cout << "Dummy Collision Model Implementation..." << std::endl;
    }
} // namespace VirtualRobot
