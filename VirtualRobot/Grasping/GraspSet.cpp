
#include "GraspSet.h"

#include <vector>

#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    GraspSet::GraspSet(const std::string& name,
                       const std::string& robotType,
                       const std::string& eef,
                       const std::vector<GraspPtr>& grasps) :
        grasps(grasps), name(name), robotType(robotType), eef(eef)
    {
    }

    GraspSet::~GraspSet() = default;

    void
    GraspSet::addGrasp(GraspPtr grasp)
    {
        VR_ASSERT_MESSAGE(grasp, "NULL grasp");
        VR_ASSERT_MESSAGE(!hasGrasp(grasp), "Grasp already added!");
        VR_ASSERT_MESSAGE(isCompatibleGrasp(grasp), "Grasp is not compatible with this grasp set");


        grasps.push_back(grasp);
    }

    bool
    GraspSet::hasGrasp(GraspPtr grasp) const
    {
        VR_ASSERT_MESSAGE(grasp, "NULL grasp");

        for (const auto& i : grasps)
            if (i == grasp)
            {
                return true;
            }

        return false;
    }

    bool
    GraspSet::hasGrasp(const std::string& name) const
    {
        for (auto& grasp : grasps)
        {
            if (grasp->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    void
    GraspSet::clear()
    {
        grasps.clear();
    }

    void
    GraspSet::includeGraspSet(GraspSetPtr grasps)
    {
        std::vector<GraspPtr> includedGrasp = grasps->getGrasps();

        for (const auto& i : includedGrasp)
        {
            if (!hasGrasp(i))
            {
                addGrasp(i);
            }
        }
    }

    void
    GraspSet::print() const
    {
        std::cout << "**** Grasp set ****" << std::endl;
        std::cout << "Name: " << name << std::endl;
        std::cout << "Robot Type: " << robotType << std::endl;
        std::cout << "End Effector: " << eef << std::endl;
        std::cout << "Grasps:" << std::endl;

        for (size_t i = 0; i < grasps.size(); i++)
        {
            std::cout << "** grasp " << i << ":" << std::endl;
            grasps[i]->print(false);
        }

        std::cout << std::endl;
    }

    bool
    GraspSet::isCompatibleGrasp(GraspPtr grasp) const
    {
        if (grasp->getRobotType() != robotType)
        {
            return false;
        }

        if (grasp->getEefName() != eef)
        {
            return false;
        }

        return true;
    }

    std::size_t
    GraspSet::getSize() const
    {
        return grasps.size();
    }

    VirtualRobot::GraspPtr
    GraspSet::getGrasp(std::size_t n) const
    {
        if (n >= grasps.size())
        {
            return GraspPtr();
        }

        return grasps[n];
    }

    VirtualRobot::GraspPtr
    GraspSet::getGrasp(const std::string& name) const
    {
        for (auto& grasp : grasps)
        {
            if (grasp->getName() == name)
            {
                return grasp;
            }
        }

        return GraspPtr();
    }

    std::string
    GraspSet::getName() const
    {
        return name;
    }

    std::string
    GraspSet::getRobotType() const
    {
        return robotType;
    }

    std::string
    GraspSet::getEndEffector() const
    {
        return eef;
    }

    std::string
    GraspSet::getXMLString(int tabs) const
    {
        std::stringstream ss;
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }

        ss << t << "<GraspSet name='" << name << "' RobotType='" << robotType << "' EndEffector='"
           << eef << "'>\n";

        for (auto& grasp : grasps)
        {
            ss << grasp->toXML(tabs + 1);
        }

        ss << t << "</GraspSet>\n";

        return ss.str();
    }

    VirtualRobot::GraspSetPtr
    GraspSet::clone() const
    {
        GraspSetPtr res(new GraspSet(name, robotType, eef));

        // clone grasps
        for (auto& grasp : grasps)
        {
            res->addGrasp(grasp->clone());
        }

        return res;
    }

    bool
    GraspSet::removeGrasp(GraspPtr grasp)
    {
        for (std::vector<GraspPtr>::iterator i = grasps.begin(); i != grasps.end(); i++)
        {
            if (*i == grasp)
            {
                grasps.erase(i);
                return true;
            }
        }

        return false;
    }

    bool
    GraspSet::removeGrasp(unsigned int i)
    {
        GraspPtr g = getGrasp(i);
        if (!g)
            return false;
        return removeGrasp(g);
    }

    void
    GraspSet::removeAllGrasps()
    {
        grasps.clear();
    }

    std::vector<GraspPtr>
    GraspSet::getGrasps() const
    {
        std::vector<GraspPtr> res;

        for (const auto& grasp : grasps)
        {
            res.push_back(grasp);
        }

        return res;
    }

    void
    GraspSet::setPreshape(const std::string& preshape)
    {
        for (auto& grasp : grasps)
        {
            grasp->setPreshape(preshape);
        }
    }


} // namespace VirtualRobot
