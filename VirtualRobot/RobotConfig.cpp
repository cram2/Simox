
#include "RobotConfig.h"
#include "Robot.h"
#include "VirtualRobotException.h"


namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    RobotConfig::RobotConfig(RobotWeakPtr robot, const std::string& name)
        : name(name),
          robot(robot)
    {
        THROW_VR_EXCEPTION_IF(!robot.lock(), "NULL robot in RobotConfig");
    }

    RobotConfig::RobotConfig(RobotWeakPtr robot, const std::string& name, const std::vector< Configuration >& configs)
        : name(name),
          robot(robot)
    {
        THROW_VR_EXCEPTION_IF(!robot.lock(), "NULL robot in RobotConfig");

        for (const auto& config : configs)
        {
            setConfig(config);
        }
    }

    RobotConfig::RobotConfig(RobotWeakPtr robot, const std::string& name, const std::map< RobotNodePtr, float >& configs)
        : name(name),
          robot(robot)
    {
        THROW_VR_EXCEPTION_IF(!robot.lock(), "NULL robot in RobotConfig");

        for (const auto& config : configs)
        {
            setConfig(config.first, config.second);
        }
    }

    RobotConfig::RobotConfig(RobotWeakPtr robot, const std::string& name, const std::vector< std::string >& robotNodes, const std::vector< float >& values)
        : name(name),
          robot(robot)
    {
        THROW_VR_EXCEPTION_IF(!robot.lock(), "NULL robot in RobotConfig");
        THROW_VR_EXCEPTION_IF(robotNodes.size() != values.size(), "Vector sizes have to be equal in RobotConfig");

        for (size_t i = 0; i < robotNodes.size(); i++)
        {
            setConfig(robotNodes[i], values[i]);
        }
    }

    RobotConfig::RobotConfig(RobotWeakPtr robot, const std::string& name, const std::vector< RobotNodePtr >& robotNodes, const std::vector< float >& values)
        : name(name),
          robot(robot)
    {
        THROW_VR_EXCEPTION_IF(!robot.lock(), "NULL robot in RobotConfig");
        THROW_VR_EXCEPTION_IF(robotNodes.size() != values.size(), "Vector sizes have to be equal in RobotConfig");

        for (size_t i = 0; i < robotNodes.size(); i++)
        {
            setConfig(robotNodes[i], values[i]);
        }
    }


    void RobotConfig::print() const
    {
        std::cout << "  Robot Config <" << name << ">" << std::endl;

        for (const auto& config : configs)
        {
            std::cout << "  * " << config.first->getName() << ":\t" << config.second << std::endl;
        }
    }

    bool RobotConfig::setConfig(const Configuration& c)
    {
        return setConfig(c.name, c.value);
    }

    bool RobotConfig::setConfig(const std::string& node, float value)
    {
        RobotPtr r = robot.lock();

        if (!r)
        {
            VR_WARNING << "Robot is already deleted, skipping update..." << std::endl;
            return false;
        }

        RobotNodePtr rn = r->getRobotNode(node);

        if (!rn)
        {
            VR_WARNING << "Did not find robot node with name " << node << std::endl;
            return false;
        }

        configs[rn] = value;
        return true;
    }

    bool RobotConfig::setConfig(RobotNodePtr node, float value)
    {
        THROW_VR_EXCEPTION_IF(!node, "Null data");

        RobotPtr r = robot.lock();

        if (!r)
        {
            VR_WARNING << "Robot is already deleted, skipping update..." << std::endl;
            return false;
        }

        THROW_VR_EXCEPTION_IF(!r->hasRobotNode(node), "RobotNode with name " << r->getName() << " does not belong to robot " << r->getName());

        configs[node] = value;
        return true;
    }

    VirtualRobot::RobotPtr RobotConfig::getRobot()
    {
        return robot.lock();
    }

    std::string RobotConfig::getName() const
    {
        return name;
    }

    RobotConfigPtr RobotConfig::clone(RobotPtr newRobot)
    {
        if (!newRobot)
        {
            newRobot = robot.lock();
        }

        VR_ASSERT(newRobot);

        std::map< RobotNodePtr, float > newConfigs;
        std::map< RobotNodePtr, float >::iterator i = configs.begin();

        while (i != configs.end())
        {
            RobotNodePtr rn = newRobot->getRobotNode(i->first->getName());

            if (!rn)
            {
                VR_WARNING << "Could not completely clone RobotConfig " << name << " because new robot does not know a RobotNode with name " << i->first->getName() << std::endl;
            }
            else
            {
                newConfigs[rn] = i->second;
            }

            i++;
        }

        RobotConfigPtr result(new RobotConfig(newRobot, name, newConfigs));
        if (tcpNode)
        {
            result->setTCP(tcpNode->getName());
        }
        return result;
    }


    bool RobotConfig::setJointValues()
    {
        RobotPtr r = robot.lock();

        if (!r)
        {
            VR_WARNING << "Robot is already deleted, skipping update..." << std::endl;
            return false;
        }

        WriteLockPtr lock = r->getWriteLock();

        for (std::map< RobotNodePtr, float >::const_iterator i = configs.begin(); i != configs.end(); i++)
        {
            i->first->setJointValueNoUpdate(i->second);
        }

        r->applyJointValues();
        return true;
    }

    bool RobotConfig::hasConfig(const std::string& name) const
    {
        for (const auto& config : configs)
        {
            if (config.first->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    float RobotConfig::getConfig(const std::string& name) const
    {
        if (!hasConfig(name))
        {
            return 0.0f;
        }

        RobotPtr r = robot.lock();

        if (!r)
        {
            VR_WARNING << "Robot is already deleted..." << std::endl;
            return 0.0f;
        }

        RobotNodePtr rn = r->getRobotNode(name);
        THROW_VR_EXCEPTION_IF(!rn, "Did not find robot node with name " << name);
        std::map< RobotNodePtr, float >::const_iterator i = configs.find(rn);

        if (i == configs.end())
        {
            VR_ERROR << "Internal error..." << std::endl;
            return 0.0f;
        }

        return i->second;
    }

    std::vector< RobotNodePtr > RobotConfig::getNodes() const
    {
        std::vector< RobotNodePtr > result;
        std::map< RobotNodePtr, float >::const_iterator i = configs.begin();

        while (i != configs.end())
        {
            result.push_back(i->first);
            i++;
        }

        return result;
    }

    std::map < std::string, float > RobotConfig::getRobotNodeJointValueMap()
    {
        std::map < std::string, float > result;
        std::map< RobotNodePtr, float >::const_iterator i = configs.begin();

        while (i != configs.end())
        {
            result[i->first->getName()] = i->second;
            i++;
        }

        return result;
    }

    bool RobotConfig::setJointValues(RobotPtr r)
    {
        if (!r)
        {
            return false;
        }

        WriteLockPtr lock = r->getWriteLock();

        std::map < std::string, float > jv = getRobotNodeJointValueMap();
        std::map< std::string, float >::const_iterator i = jv.begin();

        // first check if all nodes are present
        while (i != jv.end())
        {
            if (!r->hasRobotNode(i->first))
            {
                return false;
            }

            i++;
        }

        // apply jv
        i = jv.begin();

        while (i != jv.end())
        {
            RobotNodePtr rn = r->getRobotNode(i->first);

            if (!rn)
            {
                return false;
            }

            rn->setJointValueNoUpdate(i->second);
            i++;
        }

        r->applyJointValues();
        return true;
    }

    bool RobotConfig::setTCP(const std::string& tcpName)
    {
        RobotPtr r = robot.lock();

        if (!r)
        {
            VR_WARNING << "Robot is already deleted, skipping update..." << std::endl;
            return false;
        }

        RobotNodePtr rn = r->getRobotNode(tcpName);

        if (!rn)
        {
            VR_WARNING << "Did not find robot node with name " << tcpName << std::endl;
            return false;
        }

        tcpNode = rn;
        return true;
    }

    bool RobotConfig::setTCP(RobotNodePtr tcp)
    {
        tcpNode = tcp;
        return true;
    }

    bool RobotConfig::hasTCP() const
    {
        if (tcpNode)
        {
            return true;
        }
        return false;
    }

    RobotNodePtr RobotConfig::getTCP()
    {
        return tcpNode;
    }


    std::string RobotConfig::toXML(int tabs)
    {
        std::map < std::string, float > jv = getRobotNodeJointValueMap();
        std::string tcpName;
        if (tcpNode)
        {
            tcpName = tcpNode->getName();
        }
        return createXMLString(jv, name, tcpName, tabs);
    }

    std::string RobotConfig::createXMLString(const std::map< std::string, float >& config, const std::string& name, const std::string& tcpName, int tabs/*=0*/)
    {
        std::stringstream ss;
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }

        std::string tt = t;
        tt += "\t";
        std::string ttt = tt;
        ttt += "\t";

        ss << t << "<Configuration name='" << name << "'";

        if (!tcpName.empty())
        {
            ss << " tcp='" << tcpName << "'";
        }

        ss << ">\n";


        std::map< std::string, float >::const_iterator i = config.begin();

        while (i != config.end())
        {
            ss << tt << "<Node name='" << i->first << "' unit='radian' value='" << i->second << "'/>\n";
            i++;
        }

        ss << t << "</Configuration>\n";

        return ss.str();
    }

} // namespace VirtualRobot
