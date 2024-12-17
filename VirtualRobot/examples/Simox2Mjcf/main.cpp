#include <filesystem>

#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/mujoco/RobotMjcf.h>


using namespace VirtualRobot;
using VirtualRobot::RuntimeEnvironment;

namespace fs = std::filesystem;

/**
 * Loads a Simox robot and converts it to Mujoco's XML format (MJCF).
 * The converted file is stored in a directory mjcf/ placed in the directory
 * of the input file.
 */
int
main(int argc, char* argv[])
{
    RuntimeEnvironment::setCaption("Convert Simox XML to MJCF (Mujoco XML)");

    RuntimeEnvironment::considerKey("robot", "The Simox robot model to convert. (required)");
    RuntimeEnvironment::considerKey("outdir", "The output directory. (default: 'mjcf')");
    RuntimeEnvironment::considerKey("mesh_rel_dir",
                                    "The mesh directory relative to outdir. (default: 'mesh')");

    RuntimeEnvironment::considerKey(
        "actuator", "The actuator type to add (motor, position, velocity). (default: motor)");
    RuntimeEnvironment::considerFlag(
        "mount", "How to mount the robot at the world body (fixed, free, mocap). (default: fixed)");

    RuntimeEnvironment::considerFlag(
        "rel_paths",
        "Store relative mesh paths instead of absolute ones. (This can "
        "cause problems when including the generated model from another directory.)");

    RuntimeEnvironment::considerKey(
        "scale_length",
        "Scaling of lengths and distances (to m). For meshes, see 'scale_mesh'. (default: 1.0)");
    RuntimeEnvironment::considerKey("scale_mesh", "Scaling of meshes (to m). (default: 1.0)");
    RuntimeEnvironment::considerKey("scale_mass", "Scaling of masses (to kg). (default: 1.0)");

    // RuntimeEnvironment::considerFlag(
    //            "verbose", "Enable verbose output.");

    RuntimeEnvironment::processCommandLine(argc, argv);


    if (RuntimeEnvironment::hasHelpFlag() || !RuntimeEnvironment::hasValue("robot"))
    {
        RuntimeEnvironment::printOptions();
        return 0;
    }

    fs::path inputFilename;
    {
        std::string robFile = RuntimeEnvironment::getValue("robot");

        if (RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            inputFilename = robFile;
        }
        else
        {
            std::cout << "Something is wrong with " << robFile;
        }
    }

    const fs::path outputDir =
        RuntimeEnvironment::checkParameter("outdir", (inputFilename.parent_path() / "mjcf"));
    const std::string meshRelDir = RuntimeEnvironment::checkParameter("mesh_rel_dir", "mesh");


    const std::string actuatorTypeStr = RuntimeEnvironment::checkParameter("actuator", "motor");
    mujoco::ActuatorType actuatorType;
    const std::string worldMountModeStr = RuntimeEnvironment::checkParameter("mount", "fixed");
    mujoco::WorldMountMode worldMountMode;

    try
    {
        actuatorType = mujoco::toActuatorType(actuatorTypeStr);
        worldMountMode = mujoco::toWorldMountMode(worldMountModeStr);
    }
    catch (const std::out_of_range& e)
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    // const bool verbose = RuntimeEnvironment::hasFlag("verbose");

    const bool useRelativePaths = RuntimeEnvironment::hasFlag("rel_paths");


    auto parseFloatParameter = [](const std::string& key, const std::string& default_)
    {
        const std::string string = RuntimeEnvironment::checkParameter(key, default_);
        try
        {
            return std::stof(string);
        }
        catch (const std::invalid_argument& e)
        {
            std::cerr << "Could not parse value of argument " << key << ": '" << string << "' \n"
                      << "Reason: " << e.what() << std::endl;
            throw e;
        }
    };

    float scaleLength;
    float scaleMesh;
    float scaleMass;
    try
    {
        scaleLength = parseFloatParameter("scale_length", "1");
        scaleMesh = parseFloatParameter("scale_mesh", "1");
        scaleMass = parseFloatParameter("scale_mass", "1");
    }
    catch (const std::invalid_argument&)
    {
        return -1;
    }


    std::cout << "Input file:         " << inputFilename << std::endl;
    std::cout << "Output dir:         " << outputDir << std::endl;
    std::cout << "Output mesh dir:    " << outputDir / meshRelDir << std::endl;
    std::cout << "Actuator type:      " << actuatorTypeStr << std::endl;
    std::cout << "World Mount Mode:   " << worldMountModeStr << std::endl;

    std::cout << "Scaling: " << std::endl
              << "  - length: " << scaleLength << std::endl
              << "  - mesh:   " << scaleMesh << std::endl
              << "  - mass:   " << scaleMass << std::endl;

    std::cout << "Loading robot ..." << std::endl;

    RobotPtr robot;
    try
    {
        std::cout << "Loading robot from " << inputFilename << " ..." << std::endl;
        robot = RobotIO::loadRobot(inputFilename, RobotIO::eFull);
        assert(robot);
    }
    catch (const VirtualRobotException&)
    {
        throw; // Rethrow
    }

    if (/* DISABLES CODE */ (false))
    {
        // Using RobotIO.
        RobotIO::saveMJCF(robot, inputFilename.filename(), outputDir, meshRelDir);
    }
    else
    {
        // Direct API.
        mujoco::RobotMjcf mjcf(robot);

        mjcf.setUseRelativePaths(useRelativePaths);

        mjcf.setOutputFile(outputDir / inputFilename.filename());
        mjcf.setOutputMeshRelativeDirectory(meshRelDir);

        mjcf.setLengthScale(scaleLength);
        mjcf.setMeshScale(scaleMesh);
        mjcf.setMassScale(scaleMass);

        // mjcf.setVerbose(verbose);

        mjcf.build(worldMountMode, actuatorType);
        mjcf.save();
    }
}
