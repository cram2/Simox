/**
* @package    VirtualRobot
* @author     Manfred Kroehnert , Nikolaus Vahrenkamp
* @copyright  2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*/

#include "EndEffector.h"
#include "../VirtualRobotException.h"
#include "../Robot.h"
#include "../Obstacle.h"
#include "EndEffectorActor.h"
#include "../SceneObjectSet.h"
#include "../SceneObject.h"
#include "../RobotConfig.h"
#include "../CollisionDetection/CollisionChecker.h"



namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    EndEffector::EndEffector(const std::string& nameString, const std::vector<EndEffectorActorPtr>& actorsVector, const std::vector<RobotNodePtr>& staticPartVector, RobotNodePtr baseNodePtr, RobotNodePtr tcpNodePtr, RobotNodePtr gcpNodePtr, std::vector< RobotConfigPtr > preshapes) :
        name(nameString),
        actors(actorsVector),
        statics(staticPartVector),
        baseNode(baseNodePtr),
        tcpNode(tcpNodePtr)
    {
        THROW_VR_EXCEPTION_IF(!baseNode, "NULL base node not allowed!");
        THROW_VR_EXCEPTION_IF(!tcpNode, "NULL tcp node not allowed!");

        if (gcpNodePtr)
        {
            gcpNode = gcpNodePtr;
        }
        else
        {
            gcpNode = tcpNode;
        }

        for (const auto& preshape : preshapes)
        {
            registerPreshape(preshape);
        }
    }

    EndEffector::~EndEffector() = default;

    EndEffectorPtr EndEffector::clone(RobotPtr newRobot)
    {
        if (!newRobot)
        {
            VR_ERROR << "Attempting to clone EndEffector for invalid robot";
            return EndEffectorPtr();
        }

        RobotNodePtr newBase = newRobot->getRobotNode(baseNode->getName());
        RobotNodePtr newTCP = newRobot->getRobotNode(tcpNode->getName());
        RobotNodePtr newGCP = newRobot->getRobotNode(gcpNode->getName());
        THROW_VR_EXCEPTION_IF(!newBase, " New robot does not own a base node with name " << baseNode->getName());
        THROW_VR_EXCEPTION_IF(!newTCP, " New robot does not own a tcp node with name " << tcpNode->getName());
        THROW_VR_EXCEPTION_IF(!newGCP, " New robot does not own a gcp node with name " << gcpNode->getName());
        std::vector<RobotNodePtr> newStatics(statics.size());
        std::vector<EndEffectorActorPtr> newActors(actors.size());
        std::vector<RobotConfigPtr> newPreshapes;

        for (size_t i = 0; i < statics.size(); i++)
        {
            newStatics[i] = newRobot->getRobotNode(statics[i]->getName());
        }

        for (size_t i = 0; i < actors.size(); i++)
        {
            newActors[i] = actors[i]->clone(newRobot);
        }

        std::map< std::string, RobotConfigPtr >::iterator prI = preshapes.begin();

        while (prI != preshapes.end())
        {
            newPreshapes.push_back(prI->second->clone(newRobot));
            prI++;
        }



        EndEffectorPtr eef(new EndEffector(name, newActors, newStatics, newBase, newTCP, newGCP, newPreshapes));
        newRobot->registerEndEffector(eef);

        // set current config to new eef
        RobotConfigPtr currentConfig = getConfiguration();
        RobotConfigPtr newConfig = currentConfig->clone(newRobot);
        newRobot->setJointValues(newConfig);

        return eef;
    }


    std::string EndEffector::getName()
    {
        return name;
    }

    void EndEffector::getActors(std::vector<EndEffectorActorPtr>& actors)
    {
        actors = this->actors;
    }


    const std::vector<EndEffectorActorPtr>& EndEffector::getActors()
    {
        return actors;
    }

    void EndEffector::getStatics(std::vector<RobotNodePtr>& statics)
    {
        statics = this->statics;
    }
    const std::vector<RobotNodePtr>& EndEffector::getStatics()
    {
        return statics;
    }

    EndEffector::ContactInfoVector EndEffector::closeActors(SceneObjectSetPtr obstacles, float stepSize, float stepSizeSpeedFactor, std::uint64_t steps)
    {
        const bool hasStepLimit = steps;
        std::vector<char> actorFastMode(actors.size(), stepSizeSpeedFactor > 1);
        EndEffector::ContactInfoVector fastModeContacts;
        std::vector<char> actorCollisionStatus(actors.size(), false);
        EndEffector::ContactInfoVector result;

        bool finished = false;
        std::uint64_t loop = 0;

        const auto shared_this = shared_from_this();

        while (!finished)
        {
            loop++;
            --steps;
            finished = true;

            for (unsigned int i = 0; i < actors.size(); i++)
            {
                // Perform another step towards closing this actor
                if (!actorCollisionStatus[i])
                {
                    finished = false;

                    auto& fastMode = actorFastMode.at(i);
                    if (fastMode)
                    {
                        const auto fastStepSize = stepSize * stepSizeSpeedFactor;
                        fastMode = !actors[i]->moveActorCheckCollision(shared_this, fastModeContacts, obstacles, fastStepSize);
                        if (!fastMode)
                        {
                            //now in collision if fast mode is used
                            actors[i]->moveActor(-fastStepSize);
                            actorCollisionStatus[i] = actors[i]->moveActorCheckCollision(shared_this, result, obstacles, stepSize);
                        }
                    }
                    else
                    {
                        actorCollisionStatus[i] = actors[i]->moveActorCheckCollision(shared_this, result, obstacles, stepSize);
                    }
                }
            }

            // recheck if now the fingers can move (when the actors were blocked during closing, eventually they now can move again)
            if (finished && loop > 2)
            {
                loop = 0;
                finished = false;
                result.clear(); // we will gain contacts again

                for (unsigned int i = 0; i < actors.size(); i++)
                {
                    actorCollisionStatus[i] = false;
                }
            }
            if ((hasStepLimit && 0 == steps) || finished)
            {
                if (finished)
                {
                    return result;
                }
                return {};
            }
        }

        return result;
    }

    EndEffector::ContactInfoVector EndEffector::closeActors(SceneObjectPtr obstacle, float stepSize /*= 0.02*/, float stepSizeSpeedFactor, uint64_t steps)
    {
        if (!obstacle)
        {
            return closeActors(SceneObjectSetPtr(), stepSize, stepSizeSpeedFactor, steps);
        }

        SceneObjectSetPtr colModels(new SceneObjectSet("", obstacle->getCollisionChecker()));
        colModels->addSceneObject(obstacle);
        return closeActors(colModels, stepSize, stepSizeSpeedFactor, steps);
    }

    EndEffector::ContactInfoVector EndEffector::closeActors(std::nullptr_t, float stepSize, float stepSizeSpeedFactor, std::uint64_t steps)
    {
        return closeActors(SceneObjectSetPtr(), stepSize, stepSizeSpeedFactor, steps);
    }

    EndEffector::ContactInfoVector EndEffector::closeActors(float stepSize, float stepSizeSpeedFactor, std::uint64_t steps)
    {
        return closeActors(SceneObjectSetPtr(), stepSize, stepSizeSpeedFactor, steps);
    }

    bool EndEffector::allActorsClosed() const
    {
        for (const auto& actor : actors)
        {
            if (!actor->isAtHiLimit())
            {
                return false;
            }
        }
        return true;
    }

    bool EndEffector::allActorsOpen() const
    {
        for (const auto& actor : actors)
        {
            if (!actor->isAtLoLimit())
            {
                return false;
            }
        }
        return true;
    }

    void EndEffector::openActors(SceneObjectSetPtr obstacles, float stepSize, float stepSizeSpeedFactor)
    {
        closeActors(obstacles, -stepSize, stepSizeSpeedFactor);
    }

    VirtualRobot::SceneObjectSetPtr EndEffector::createSceneObjectSet(CollisionCheckerPtr colChecker)
    {
        SceneObjectSetPtr cms(new SceneObjectSet(name, colChecker));
        cms->addSceneObjects(statics);

        for (auto& actor : actors)
        {
            cms->addSceneObjects(actor->getRobotNodes());
        }

        return cms;

    }


    std::string EndEffector::getBaseNodeName()
    {
        return baseNode->getName();
    }


    std::string EndEffector::getTcpName()
    {
        return tcpNode->getName();
    }

    VirtualRobot::RobotPtr EndEffector::getRobot()
    {
        if (!baseNode)
        {
            return RobotPtr();
        }

        return baseNode->getRobot();
    }

    std::string EndEffector::getRobotType()
    {
        RobotPtr r = getRobot();

        if (!r)
        {
            return std::string("");
        }

        return r->getType();
    }

    VirtualRobot::RobotPtr EndEffector::createEefRobot(const std::string& newRobotType, const std::string& newRobotName, CollisionCheckerPtr /*collisionChecker=CollisionCheckerPtr()*/)
    {
        RobotPtr r = getRobot();
        THROW_VR_EXCEPTION_IF(!r, "No robot defined in EEF");
        RobotNodePtr baseNode = r->getRobotNode(getBaseNodeName());
        THROW_VR_EXCEPTION_IF(!baseNode, "no base node with name " << getBaseNodeName());

        // don't clone robotNodeSets and EEFs here
        RobotPtr robo = r->extractSubPart(baseNode, newRobotType, newRobotName, false, false);

        EndEffectorPtr eef = this->clone(robo);
        // now the eef is already registered at robo...

        Eigen::Matrix4f i;
        i.setIdentity();
        robo->setGlobalPose(i);
        return robo;
    }

    void EndEffector::print()
    {
        std::cout << " **** EndEffector ****" << std::endl;

        std::cout << " * Name: " << name << std::endl;
        std::cout << " * Base Node: ";

        if (baseNode)
        {
            std::cout << baseNode->getName() << std::endl;
        }
        else
        {
            std::cout << "<not set>" << std::endl;
        }

        std::cout << " * Static RobotNodes:" << std::endl;

        for (auto& i : statics)
        {
            std::cout << " ** " << i->getName() << std::endl;
        }

        std::cout << " * Actors:" << std::endl;

        for (auto& actor : actors)
        {
            actor->print();
        }

        std::cout << std::endl;
    }

    VirtualRobot::RobotNodePtr EndEffector::getTcp()
    {
        return tcpNode;
    }

    VirtualRobot::RobotNodePtr EndEffector::getGCP()
    {
        return gcpNode;
    }

    VirtualRobot::RobotNodePtr EndEffector::getBase()
    {
        return baseNode;
    }

    VirtualRobot::CollisionCheckerPtr EndEffector::getCollisionChecker()
    {
        if (!baseNode)
        {
            return CollisionChecker::getGlobalCollisionChecker();
        }

        return baseNode->getCollisionChecker();
    }

    void EndEffector::registerPreshape(RobotConfigPtr preshape)
    {
        THROW_VR_EXCEPTION_IF(!preshape, "NULL data...");
        std::vector< RobotNodePtr > nodes = preshape->getNodes();
        // don't be too strict!
        /*for (size_t i=0;i<nodes.size();i++)
        {
            THROW_VR_EXCEPTION_IF(!hasNode(nodes[i]), "Node " << nodes[i]->getName() << " is not part of EEF " << getName() );
        }*/
        preshapes[preshape->getName()] = preshape;
    }

    VirtualRobot::RobotConfigPtr EndEffector::getPreshape(const std::string& name)
    {
        if (!hasPreshape(name))
        {
            return RobotConfigPtr();
        }

        return (preshapes.find(name))->second;
    }

    bool EndEffector::hasPreshape(const std::string& name)
    {
        return (preshapes.find(name) != preshapes.end());
    }

    bool EndEffector::setPreshape(const std::string& name)
    {
        if (!hasPreshape(name))
        {
            return false;
        }

        RobotConfigPtr ps = getPreshape(name);

        if (ps)
        {
            RobotPtr robot = getRobot();
            VR_ASSERT(robot);
            robot->setJointValues(ps);
            return true;
        }

        return false;
    }

    bool EndEffector::hasNode(RobotNodePtr node)
    {
        if (node == baseNode || node == tcpNode || node == gcpNode)
        {
            return true;
        }

        std::vector<EndEffectorActorPtr>::iterator iA = actors.begin();

        while (iA != actors.end())
        {
            if ((*iA)->hasNode(node))
            {
                return true;
            }

            iA++;
        }

        std::vector<RobotNodePtr>::iterator iS = statics.begin();

        while (iS != statics.end())
        {
            if (*iS == node)
            {
                return true;
            }

            iS++;
        }

        return false;
    }

    VirtualRobot::RobotConfigPtr EndEffector::getConfiguration()
    {
        VirtualRobot::RobotConfigPtr result(new VirtualRobot::RobotConfig(getRobot(), getName()));
        std::vector< RobotNodePtr > rn = getAllNodes();

        for (auto& i : rn)
        {
            if (i->isRotationalJoint() || i->isTranslationalJoint())
            {
                result->setConfig(i->getName(), i->getJointValue());
            }
        }

        return result;
    }

    std::vector< RobotNodePtr > EndEffector::getAllNodes()
    {
        // avoid double entries
        std::map< RobotNodePtr, RobotNodePtr > mapR;

        if (baseNode)
        {
            mapR[baseNode] = baseNode;
            mapR[tcpNode] = tcpNode;
            mapR[gcpNode] = gcpNode;
        }

        std::vector<EndEffectorActorPtr>::iterator iA = actors.begin();

        while (iA != actors.end())
        {
            std::vector< RobotNodePtr > rn = (*iA)->getRobotNodes();

            for (const auto& i : rn)
            {
                mapR[i] = i;
            }

            iA++;
        }

        std::vector<RobotNodePtr>::iterator iS = statics.begin();

        while (iS != statics.end())
        {
            mapR[*iS] = *iS;
            iS++;
        }

        std::map< RobotNodePtr, RobotNodePtr >::iterator m = mapR.begin();
        std::vector< RobotNodePtr > result;

        while (m != mapR.end())
        {
            result.push_back(m->first);
            m++;
        }

        return result;
    }

    bool EndEffector::nodesSufficient(std::vector<RobotNodePtr> nodes) const
    {
        bool base = false;
        bool tcp = false;
        bool gcp = false;

        if (!baseNode)
        {
            base = true;
        }

        if (!tcpNode)
        {
            tcp = true;
        }

        if (!gcpNode)
        {
            gcp = true;
        }

        std::vector<RobotNodePtr>::const_iterator i = nodes.begin();

        while (i != nodes.end())
        {
            if (!base && (*i)->getName() == baseNode->getName())
            {
                base = true;
            }

            if (!tcp && (*i)->getName() == tcpNode->getName())
            {
                tcp = true;
            }

            if (!gcp && (*i)->getName() == gcpNode->getName())
            {
                gcp = true;
            }

            i++;
        }

        if (!base || !gcp || !tcp)
        {
            return false;
        }

        for (const auto& actor : actors)
        {
            if (!actor->nodesSufficient(nodes))
            {
                return false;
            }
        }

        return true;
    }

    float EndEffector::getApproximatedLength()
    {
        float maxActor = 0;

        for (auto& actor : actors)
        {
            float al = actor->getApproximatedLength();

            if (al > maxActor)
            {
                maxActor = al;
            }
        }


        BoundingBox bb_all;

        for (auto& j : statics)
        {
            if (j->getCollisionModel())
            {
                BoundingBox bb = j->getCollisionModel()->getBoundingBox();
                bb_all.addPoint(bb.getMin());
                bb_all.addPoint(bb.getMax());
            }
        }

        Eigen::Vector3f d = bb_all.getMax() - bb_all.getMin();
        float maxStatic = d.norm();

        return maxStatic + maxActor;
    }



    std::vector<std::string> EndEffector::getPreshapes()
    {
        std::vector<std::string> res;
        std::map< std::string, RobotConfigPtr >::iterator it = preshapes.begin();

        while (it != preshapes.end())
        {
            res.push_back(it->first);
            it++;
        }

        return res;
    }

    std::string EndEffector::toXML(int ident /*= 1*/)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < ident; i++)
        {
            pre += t;
        }

        std::string tt = pre + t;
        std::string ttt = tt + t;
        ss << pre << "<EndEffector name='" << name << "' ";

        if (baseNode)
        {
            ss << "base='" << baseNode->getName() << "' ";
        }

        if (tcpNode)
        {
            ss << "tcp='" << tcpNode->getName() << "' ";
        }

        if (gcpNode)
        {
            ss << "gcp='" << gcpNode->getName() << "' ";
        }

        ss << ">" << std::endl;

        // Preshapes
        std::map< std::string, RobotConfigPtr >::iterator itPre = preshapes.begin();

        while (itPre != preshapes.end())
        {
            ss << tt << "<Preshape name='" << itPre->first << "'>" << std::endl;
            std::map < std::string, float > jv = itPre->second->getRobotNodeJointValueMap();

            std::map< std::string, float >::const_iterator i = jv.begin();

            while (i != jv.end())
            {
                ss << ttt << "<Node name='" << i->first << "' unit='radian' value='" << i->second << "'/>\n";
                i++;
            }

            ss << tt << "</Preshape>" << std::endl;
            itPre++;
        }

        // Static
        if (statics.size() > 0)
        {
            ss << tt << "<Static>" << std::endl;

            for (auto& i : statics)
            {
                ss << ttt << "<Node name='" << i->getName() << "'/>" << std::endl;
            }

            ss << tt << "</Static>" << std::endl;
        }

        // Actors
        for (auto& actor : actors)
        {
            ss << actor->toXML(ident + 1);
        }

        ss << pre << "</EndEffector>" << std::endl;

        return ss.str();
    }

    int EndEffector::addStaticPartContacts(SceneObjectPtr obstacle, EndEffector::ContactInfoVector& contacts, const Eigen::Vector3f& approachDirGlobal, float maxDistance)
    {
        if (!obstacle)
        {
            return 0;
        }

        int contactCount = 0;

        for (auto n : statics)
        {
            if (!n->getCollisionModel())
            {
                continue;
            }
            int id1, id2;
            Eigen::Vector3f p1, p2;
            float dist = this->getCollisionChecker()->calculateDistance(n->getCollisionModel(), obstacle->getCollisionModel(), p1, p2, &id1, &id2);
            //VR_INFO << n->getName() << " - DIST: " << dist << std::endl;
            if (dist <= maxDistance)
            {
                EndEffector::ContactInfo ci;
                ci.eef = shared_from_this();
                //ci.actor = ;
                ci.robotNode = n;
                ci.obstacle = obstacle;

                ci.distance = dist;
                ci.contactPointFingerGlobal = p1;
                ci.contactPointObstacleGlobal = p2;
                ci.contactPointFingerLocal = ci.obstacle->toLocalCoordinateSystemVec(p1);
                ci.contactPointObstacleLocal = ci.obstacle->toLocalCoordinateSystemVec(p2);

                ci.approachDirectionGlobal = -approachDirGlobal;

                contacts.push_back(ci);
                contactCount++;
            }

        }

        return contactCount;
    }


} // namespace VirtualRobot
