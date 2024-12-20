#include "RobotMjcf.h"

#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Visualization/VisualizationNode.h>

#include "Logging.h"
#include "MeshConverter.h"
#include <VirtualRobotChecks.h>


namespace fs = std::filesystem;

namespace VirtualRobot::mujoco
{


    template <typename EnumT>
    static EnumT
    stringToEnum(const std::map<std::string, EnumT>& map, const std::string& string)
    {
        std::string lower = string;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        try
        {
            return map.at(lower);
        }
        catch (const std::out_of_range&)
        {
            std::stringstream msg;
            msg << "Invalid key '" << string << "'. Valid keys are: ";
            for (const auto& item : map)
            {
                msg << item.first << " ";
            }
            throw std::out_of_range(msg.str());
        }
    }

    ActuatorType
    toActuatorType(const std::string& string)
    {
        return stringToEnum<ActuatorType>(
            {
                {"none", ActuatorType::NONE},
                {"motor", ActuatorType::MOTOR},
                {"position", ActuatorType::POSITION},
                {"velocity", ActuatorType::VELOCITY},
            },
            string);
    }

    WorldMountMode
    toWorldMountMode(const std::string& string)
    {
        return stringToEnum<WorldMountMode>(
            {
                {"fixed", WorldMountMode::FIXED},
                {"free", WorldMountMode::FREE},
                {"mocap", WorldMountMode::MOCAP},
            },
            string);
    }

    RobotMjcf::RobotMjcf(RobotPtr robot) : robot(robot)
    {
        reset();
    }

    void
    RobotMjcf::reset()
    {
        document.reset(new mjcf::Document());
        robotBody = mjcf::Body();
        nodeBodies.clear();

        // Set model name.
        document->setModelName(robot->getName());

        const std::string className = robot->getName();

        // Add robot defaults class.
        document->default_().addClass(className);

        // Add robot body.
        robotBody = document->worldbody().addBody(robot->getName(), className);
        nodeBodies[robot->getName()] = robotBody;
    }

    mjcf::Document&
    RobotMjcf::getDocument()
    {
        return *document;
    }

    const mjcf::Document&
    RobotMjcf::getDocument() const
    {
        return *document;
    }

    float
    RobotMjcf::getLengthScale() const
    {
        return lengthScale;
    }

    void
    RobotMjcf::setLengthScale(float value)
    {
        lengthScale = value;
    }

    float
    RobotMjcf::getMeshScale() const
    {
        return meshScale;
    }

    void
    RobotMjcf::setMeshScale(float value)
    {
        meshScale = value;
    }

    float
    RobotMjcf::getMassScale() const
    {
        return massScale;
    }

    void
    RobotMjcf::setMassScale(float value)
    {
        massScale = value;
    }

    void
    RobotMjcf::setUseRelativePaths(bool useRelative)
    {
        this->useRelativePaths = useRelative;
    }

    void
    RobotMjcf::setOutputFile(const std::filesystem::path& filePath)
    {
        this->outputFile = filePath;
    }

    void
    RobotMjcf::setOutputMeshRelativeDirectory(const std::filesystem::path& meshRelativeDirectory)
    {
        this->outputMeshRelDirectory = meshRelativeDirectory;
    }

    void
    RobotMjcf::setOutputPaths(const std::filesystem::path& outputFile,
                              const std::filesystem::path& outputMeshRelativeDirectory)
    {
        setOutputFile(outputFile);
        setOutputMeshRelativeDirectory(outputMeshRelativeDirectory);
    }

    std::filesystem::path
    RobotMjcf::getOutputFile() const
    {
        return outputFile;
    }

    std::filesystem::path
    RobotMjcf::getOutputDirectory() const
    {
        return outputFile.parent_path();
    }

    std::filesystem::path
    RobotMjcf::getOutputMeshRelativeDirectory() const
    {
        return outputMeshRelDirectory;
    }

    std::filesystem::path
    RobotMjcf::getOutputMeshDirectory() const
    {
        return getOutputDirectory() / outputMeshRelDirectory;
    }

    RobotPtr
    RobotMjcf::getRobot() const
    {
        return robot;
    }

    mjcf::Body
    RobotMjcf::getRobotBody() const
    {
        return robotBody;
    }

    mjcf::Body
    RobotMjcf::getRobotNodeBody(const std::string& nodeName) const
    {
        try
        {
            return nodeBodies.at(nodeName);
        }
        catch (const std::range_error&)
        {
            throw error::NoBodyOfRobotNode(nodeName);
        }
    }

    bool
    RobotMjcf::hasRobotNodeBody(const std::string& nodeName) const
    {
        return nodeBodies.find(nodeName) != nodeBodies.end();
    }

    mjcf::DefaultClass
    RobotMjcf::getRobotDefaults() const
    {
        return document->default_().getClass(robot->getName());
    }

    void
    RobotMjcf::build(WorldMountMode worldMountMode, ActuatorType actuatorType)
    {
        // Reset MJCF.
        reset();

        // Add meta elements.
        addCompiler();

        addRobotDefaults();
        // document->setNewElementClass(robot->getName(), true);

        addSkybox();

        mjcf::Body mocapBody;
        switch (worldMountMode)
        {
            case WorldMountMode::FIXED:
                break;

            case WorldMountMode::FREE:
                robotBody.addFreeJoint();
                break;

            case WorldMountMode::MOCAP:
                std::cout << "Adding mocap body ..." << std::endl;
                robotBody.addFreeJoint();
                mocapBody = addMocapBodyWeldRobot();
                break;
        }

        std::cout << "Creating bodies structure ..." << std::endl;
        addNodeBodies();

        std::cout << "Adding meshes and geoms ..." << std::endl;
        addNodeBodyMeshes();

        bool verbose = false;
        if (verbose)
        {
            std::cout << "===========================" << std::endl
                      << "Current model: " << std::endl
                      << "--------------" << std::endl;
            std::cout << getDocument();
            std::cout << "===========================" << std::endl;
        }


        std::cout << "Adding contact excludes ..." << std::endl;
        addContactExcludes();

        if (worldMountMode == WorldMountMode::MOCAP)
        {
            std::cout << "Adding mocap body contact excludes ..." << std::endl;
            VR_CHECK(mocapBody);
            addMocapContactExcludes(mocapBody);
        }

        if (actuatorType != ActuatorType::NONE)
        {
            std::cout << "Adding actuators ..." << std::endl;
            addActuators(actuatorType);
        }

        std::cout << "Done." << std::endl;
    }

    void
    RobotMjcf::save() const
    {
        document->saveFile(outputFile);
    }

    void
    RobotMjcf::addCompiler(bool angleRadian, bool boundMass, bool balanceIneratia)
    {
        mjcf::CompilerSection compiler = document->compiler();
        compiler.angle = angleRadian ? "radian" : "degree";
        if (boundMass)
        {
            compiler.boundmass = document->getDummyMass();
        }
        compiler.balanceinertia = balanceIneratia;
    }

    void
    RobotMjcf::addRobotDefaults()
    {
        addRobotDefaults(meshScale);
    }

    void
    RobotMjcf::addRobotDefaults(float meshScale)
    {
        mjcf::DefaultClass defaultsClass = getRobotDefaults();

        defaultsClass.insertComment("Add default values for " + robot->getName() + " here.", true);

        mjcf::Joint joint = defaultsClass.getElement<mjcf::Joint>();
        joint.frictionloss = 1;
        joint.damping = 0;

        mjcf::Mesh mesh = defaultsClass.getElement<mjcf::Mesh>();
        mesh.scale = Eigen::Vector3f::Constant(meshScale);

        mjcf::Geom geom = defaultsClass.getElement<mjcf::Geom>();
        geom.condim = 4;

        mjcf::ActuatorPosition actPos = defaultsClass.getElement<mjcf::ActuatorPosition>();
        actPos.kp = 1;

        mjcf::ActuatorVelocity actVel = defaultsClass.getElement<mjcf::ActuatorVelocity>();
        actVel.kv = 1;
    }

    void
    RobotMjcf::addSkybox(const Eigen::Vector3f& rgb1, const Eigen::Vector3f& rgb2)
    {
        document->asset().addSkyboxTexture(rgb1, rgb2);
    }

    mjcf::Body
    RobotMjcf::addNodeBody(RobotNodePtr node, mjcf::Body parent, bool addJoint, bool addInertial)
    {
        mjcf::Body body = parent.addBody(node->getName());

        if (node->hasParent())
        {
            Eigen::Matrix4f pose = node->getTransformationFrom(node->getParent());
            math::Helpers::ScaleTranslation(pose, lengthScale);
            body.setPose(pose);
        }

        if (addJoint && (node->isRotationalJoint() || node->isTranslationalJoint()))
        {
            addNodeJoint(node, body);
        }

        if (addInertial)
        {
            addNodeInertial(node, body);
        }

        return body;
    }

    mjcf::Joint
    RobotMjcf::addNodeJoint(RobotNodePtr node, mjcf::Body body)
    {
        if (!(node->isRotationalJoint() || node->isTranslationalJoint()))
        {
            throw error::NodeIsNoJoint(node->getName());
        }
        VR_CHECK_HINT(!(node->isRotationalJoint() && node->isTranslationalJoint()),
                      "Node must not be both rotational and translational joint.");


        mjcf::Joint joint = body.addJoint();
        joint.name = node->getName();

        // get the axis
        Eigen::Vector3f axis;
        if (node->isRotationalJoint())
        {
            RobotNodeRevolutePtr revolute = std::dynamic_pointer_cast<RobotNodeRevolute>(node);
            VR_CHECK(revolute);
            axis = revolute->getJointRotationAxisInJointCoordSystem();
        }
        else if (node->isTranslationalJoint())
        {
            RobotNodePrismaticPtr prismatic = std::dynamic_pointer_cast<RobotNodePrismatic>(node);
            VR_CHECK(prismatic);
            axis = prismatic->getJointTranslationDirectionJointCoordSystem();
        }

        joint.type = node->isRotationalJoint() ? "hinge" : "slide";
        joint.axis = axis;
        joint.limited = !node->isLimitless();

        if (!node->isLimitless())
        {
            Eigen::Vector2f range = {node->getJointLimitLow(), node->getJointLimitHigh()};
            if (node->isTranslationalJoint())
            {
                range *= lengthScale;
            }

            // Mujoco does not like ranges where min >= max.
            if (std::abs(range(0) - range(1)) < 1e-6f)
            {
                range(1) = range(0) + 1e-6f;
            }
            joint.range = range;
        }

        return joint;
    }

    mjcf::Inertial
    RobotMjcf::addNodeInertial(RobotNodePtr node, mjcf::Body body)
    {
        const Eigen::Matrix3f matrix = node->getInertiaMatrix();
        if (matrix.isIdentity(document->getFloatCompPrecision()) &&
            node->getMass() < document->getFloatCompPrecision())
        {
            // Dont set an inertial element and let it be derived automatically.
            return {};
        }

        mjcf::Inertial inertial = body.addInertial();
        inertial.pos = node->getCoMLocal() * lengthScale;
        inertial.mass = node->getMass() * massScale;
        inertial.inertiaFromMatrix(matrix);

        return inertial;
    }

    mjcf::Body
    RobotMjcf::addNodeBody(RobotNodePtr node, bool addJoint, bool addInertial)
    {
        // See whether body for the node already exists.
        auto find = nodeBodies.find(node->getName());
        if (find != nodeBodies.end())
        {
            // Exists => break recursion.
            return find->second;
        }

        // Check whether body exists for parent node.
        mjcf::Body parent;

        if (node->getName() == robot->getRootNode()->getName())
        {
            parent = robotBody;
        }
        else
        {
            find = nodeBodies.find(node->getParent()->getName());
            if (find != nodeBodies.end())
            {
                // Parent exists.
                parent = find->second;
            }
            else
            {
                // Parent does not exist => create it first.
                parent = addNodeBody(
                    robot->getRobotNode(node->getParent()->getName()), addJoint, addInertial);
            }
        }

        // Add body as child of parent.
        mjcf::Body body = addNodeBody(node, parent, addJoint, addInertial);
        nodeBodies[node->getName()] = body;

        return body;
    }

    void
    RobotMjcf::addNodeBodies()
    {
        addNodeBodies(robot->getRobotNodeNames());
    }

    void
    RobotMjcf::addNodeBodies(RobotNodeSetPtr nodeSet)
    {
        addNodeBodies(nodeSet->getNodeNames());
    }

    void
    RobotMjcf::addNodeBodies(const std::vector<std::string>& nodeNames)
    {
        for (const std::string& nodeName : nodeNames)
        {
            addNodeBody(robot->getRobotNode(nodeName));
        }
    }

    void
    RobotMjcf::addNodeBodyMesh(RobotNodePtr node)
    {
        mjcf::Body body = getRobotNodeBody(node->getName());

        const fs::path meshFile = convertNodeMeshToSTL(node);

        if (meshFile.empty())
        {
            VR_ERROR << "Failed to add mesh to body for node '" << node->getName();
            return;
        }

        const fs::path meshPath = useRelativePaths ? getOutputMeshRelativeDirectory() / meshFile
                                                   : getOutputMeshDirectory() / meshFile;

        // Add asset.
        const std::string meshName = node->getName();
        document->asset().addMesh(meshName, meshPath);

        // Add mesh geom to body.
        mjcf::Geom geom = body.addGeomMesh(meshName);
        geom.name = node->getName();
    }

    std::filesystem::path
    RobotMjcf::convertNodeMeshToSTL(RobotNodePtr node)
    {
        VisualizationNodePtr visualization =
            node->getVisualization(SceneObject::VisualizationType::Full);

        if (!visualization)
        {
            VR_INFO << "Node '" << node->getName() << "': No visualization." << std::endl;
            return "";
        }

        std::cout << "Node '" << node->getName() << "':\t";

        const fs::path sourceFile = visualization->getFilename();

        if (sourceFile.empty())
        {
            VR_INFO << "Node '" << node->getName() << "': No visualization file." << std::endl;
        }
        if (!fs::is_regular_file(sourceFile))
        {
            VR_INFO << "Node '" << node->getName() << "': Visualization file " << sourceFile
                    << " does not exist." << std::endl;
            return "";
        }

        fs::path targetFilename = sourceFile.filename();
        targetFilename.replace_extension("stl");
        const fs::path targetFile = getOutputMeshDirectory() / targetFilename;

        MeshConverter::toSTL(sourceFile, targetFile, true);

        return targetFilename;
    }

    void
    RobotMjcf::addNodeBodyMeshes()
    {
        addNodeBodyMeshes(robot->getRobotNodeNames());
    }

    void
    RobotMjcf::addNodeBodyMeshes(RobotNodeSetPtr nodeSet)
    {
        addNodeBodyMeshes(nodeSet->getNodeNames());
    }

    void
    RobotMjcf::addNodeBodyMeshes(const std::vector<std::string>& nodeNames)
    {
        for (const std::string& nodeName : nodeNames)
        {
            addNodeBodyMesh(robot->getRobotNode(nodeName));
        }
    }

    mjcf::Body
    RobotMjcf::addMocapBody(const std::string& bodyName,
                            const std::string& className,
                            float geomSize)
    {
        if (!className.empty())
        {
            // add defaults class
            mjcf::DefaultClass defaultClass = document->default_().getClass(className);

            mjcf::Geom geom = defaultClass.getElement<mjcf::Geom>();
            geom.rgba = Eigen::Vector4f(.9f, .5f, .5f, .5f);
        }

        // Add body.
        mjcf::Body mocap = document->worldbody().addMocapBody(bodyName, geomSize);
        if (!className.empty())
        {
            mocap.childclass = className;
        }

        return mocap;
    }

    mjcf::Body
    RobotMjcf::addMocapBodyWeld(const std::string& weldBodyName,
                                const std::string& bodyName,
                                const std::string& className,
                                float geomSize)
    {
        mjcf::Body mocap = addMocapBody(bodyName, className, geomSize);

        if (!className.empty())
        {
            // Get defaults class.
            mjcf::DefaultClass defaultClass = document->default_().getClass(className);

            // Add equality defaults.
            mjcf::EqualityDefaults equality = defaultClass.getElement<mjcf::EqualityDefaults>();
            Eigen::Vector5f solimp = equality.solimp;
            solimp(1) = 0.99f;
            equality.solimp = solimp;
            //equality.solref = Eigen::Vector2f(.02f, 1.f);
        }

        // Add equality weld constraint.
        mjcf::EqualityWeld weld = document->equality().addWeld(bodyName, weldBodyName, bodyName);
        if (!className.empty())
        {
            weld.class_ = className;
        }

        // Add contact exclude.
        document->contact().addExclude(mocap.name, weldBodyName);

        return mocap;
    }

    mjcf::Body
    RobotMjcf::addMocapBodyWeldRobot(const std::string& bodyName, const std::string& className)
    {
        if (!robotBody.hasFreeJoint())
        {
            robotBody.addFreeJoint();
        }
        return addMocapBodyWeld(bodyName.empty() ? robot->getName() + "_mocap" : bodyName,
                                className);
    }

    struct ParentChildContactExcludeVisitor : public mjcf::Visitor
    {
        ParentChildContactExcludeVisitor(mjcf::Document& document) : mjcf::Visitor(document)
        {
        }

        virtual ~ParentChildContactExcludeVisitor() override = default;

        // bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
        bool visitEnter(const mjcf::AnyElement& element) override;

        std::vector<std::pair<std::string, std::string>> excludePairs;
        bool firstSkipped = false; ///< Used to skip the root element.
    };

    bool
    ParentChildContactExcludeVisitor::visitEnter(const mjcf::AnyElement& element)
    {
        if (!element.is<mjcf::Body>())
        {
            return true;
        }

        const mjcf::Body body = element.as<mjcf::Body>();

        if (!firstSkipped)
        {
            firstSkipped = true;
            return true;
        }

        const mjcf::Body parent = body.parent<mjcf::Body>();
        VR_CHECK(parent);
        excludePairs.emplace_back(parent.name.get(), body.name.get());

        return true;
    }

    void
    RobotMjcf::addContactExcludes(bool addParentChildExcludes)
    {
        addContactExcludes(robot->getRobotNodeNames(), addParentChildExcludes);
    }

    void
    RobotMjcf::addContactExcludes(RobotNodeSetPtr nodeSet, bool addParentChildExcludes)
    {
        addContactExcludes(nodeSet->getNodeNames(), addParentChildExcludes);
    }

    void
    RobotMjcf::addContactExcludes(const std::vector<std::string>& nodeNames,
                                  bool addParentChildExcludes)
    {
        std::vector<std::pair<std::string, std::string>> excludePairs;

        for (const std::string& nodeName : nodeNames)
        {
            RobotNodePtr node = robot->getRobotNode(nodeName);

            for (const std::string& ignoreNode : node->getPhysics().ignoreCollisions)
            {
                // I found an <IgnoreCollision> element referring to a non-existing node.
                // => check node existence here
                if (robot->hasRobotNode(ignoreNode))
                {
                    excludePairs.push_back({node->getName(), ignoreNode});
                }
            }
        }

        // Resolve body names and add exludes.
        for (const auto& excludePair : excludePairs)
        {
            const std::string body1 = excludePair.first;
            const std::string body2 = excludePair.second;

            VR_CHECK(!body1.empty());
            VR_CHECK(!body2.empty());
            document->contact().addExclude(body1, body2);
        }

        if (addParentChildExcludes)
        {
            // Add excludes between parent and child elemenets.
            // This should actually not be necessary?
            ParentChildContactExcludeVisitor visitor(*document);
            robotBody.accept(visitor);
            for (const auto& excludePair : visitor.excludePairs)
            {
                document->contact().addExclude(excludePair.first, excludePair.second);
            }
        }
    }

    void
    RobotMjcf::addMocapContactExcludes(mjcf::Body mocap)
    {
        if (!mocap)
        {
            throw std::invalid_argument("Passed uninitialized mocap body to " +
                                        std::string(__FUNCTION__) + "()");
        }
        for (const auto& nodeBody : nodeBodies)
        {
            document->contact().addExclude(mocap, nodeBody.second);
        }
    }

    mjcf::AnyElement
    RobotMjcf::addJointActuator(mjcf::Joint joint, ActuatorType type)
    {
        mjcf::AnyElement actuator;

        const std::string jointName = joint.name;
        switch (type)
        {
            case ActuatorType::NONE:
                // Nothing to do.
                break;

            case ActuatorType::MOTOR:
            {
                mjcf::ActuatorMotor act = document->actuator().addMotor(jointName);
                actuator = act;
                break;
            }

            case ActuatorType::POSITION:
            {
                mjcf::ActuatorPosition act = document->actuator().addPosition(jointName);
                actuator = act;

                if (joint.limited)
                {
                    act.ctrllimited = joint.limited;
                    act.ctrlrange = joint.range;
                }
            }
            break;

            case ActuatorType::VELOCITY:
            {
                mjcf::ActuatorVelocity act = document->actuator().addVelocity(jointName);
                actuator = act;
            }
            break;
        }

        std::string actuatorName = joint.name;
        actuator.setAttribute("name", actuatorName);

        return actuator;
    }

    mjcf::AnyElement
    RobotMjcf::addNodeActuator(RobotNodePtr node, ActuatorType type)
    {
        if (!(node->isRotationalJoint() || node->isTranslationalJoint()))
        {
            throw error::NodeIsNoJoint(node->getName());
        }

        mjcf::Body body = getRobotNodeBody(node->getName());

        mjcf::Joint joint;
        if (body.hasChild<mjcf::Joint>())
        {
            joint = body.firstChild<mjcf::Joint>();
        }
        else
        {
            joint = addNodeJoint(node, body);
        }

        return addJointActuator(joint, type);
    }

    void
    RobotMjcf::addActuators(ActuatorType type)
    {
        addActuators(robot->getRobotNodeNames(), type);
    }

    void
    RobotMjcf::addActuators(RobotNodeSetPtr nodeSet, ActuatorType type)
    {
        addActuators(nodeSet->getNodeNames(), type);
    }

    void
    RobotMjcf::addActuators(const std::vector<std::string>& nodeNames, ActuatorType type)
    {
        for (const std::string& nodeName : nodeNames)
        {
            RobotNodePtr node = robot->getRobotNode(nodeName);
            // Ignore non-joint nodes.
            if (node->isRotationalJoint() || node->isTranslationalJoint())
            {
                addNodeActuator(robot->getRobotNode(nodeName), type);
            }
        }
    }

    void
    RobotMjcf::addActuators(const std::map<std::string, ActuatorType>& nodeTypeMap)
    {
        for (const auto& item : nodeTypeMap)
        {
            const std::string& nodeName = item.first;

            RobotNodePtr node = robot->getRobotNode(nodeName);
            // Ignore non-joint nodes.
            if (node->isRotationalJoint() || node->isTranslationalJoint())
            {
                addNodeActuator(robot->getRobotNode(nodeName), item.second);
            }
        }
    }

} // namespace VirtualRobot::mujoco
