#include "SimoxPath.h"

#include <stdlib.h>

#include <functional>

namespace simox
{

    bool SimoxPath::simoxPathInitialized = false;
    std::filesystem::path SimoxPath::simoxRootPath = "";

    void
    SimoxPath::init()
    {
        namespace fs = std::filesystem;
        // Adapted from VirtualRobot::RuntimeEnvironment

        if (simoxPathInitialized)
        {
            return;
        }
        SimoxPath::simoxRootPath = "";

        std::vector<std::function<void(fs::path & vrDataPath)>> sources;

        // Test special environment variables.
        sources.push_back(
            [](fs::path& vrDataPath)
            {
                char* vrDataPathEnv = getenv("SIMOX_DATA_PATH");
                if (!vrDataPathEnv)
                {
                    vrDataPathEnv = getenv("VIRTUAL_ROBOT_DATA_PATH");
                }
                if (vrDataPathEnv && checkPath(vrDataPathEnv))
                {
                    vrDataPath = fs::path(vrDataPathEnv);
                }
            });

        // Test for Simox_DIR
        sources.push_back(
            [](fs::path& vrDataPath)
            {
                char* simox_dir = getenv("Simox_DIR");
                if (simox_dir)
                {
                    const std::vector<std::string> candidates = {
                        "data", "VirtualRobot/data", "../VirtualRobot/data"};
                    vrDataPath = checkCandidatePaths(candidates, std::filesystem::path(simox_dir));
                }
            });

        auto ConstantSource = [](const std::string& value)
        {
            return [value](fs::path& vrDataPath)
            {
                if (checkPath(value))
                {
                    vrDataPath = value;
                };
            };
        };

#ifdef Simox_DATA_PATH
        sources.push_back(ConstantSource(Simox_DATA_PATH));
#endif
#ifdef VirtualRobot_DATA_PATH
        sources.push_back(ConstantSource(VirtualRobot_DATA_PATH));
#endif
#ifdef VirtualRobot_SRC_DATA_PATH
        sources.push_back(ConstantSource(VirtualRobot_SRC_DATA_PATH));
#endif

        // Check standard linux install path
        sources.push_back(ConstantSource("/usr/local/data"));
        sources.push_back(ConstantSource("/usr/data"));


        // Last chance, check for inbuild paths
        sources.push_back(
            [](fs::path& vrDataPath)
            {
                const std::vector<std::string> candidates = {"../VirtualRobot/data",
                                                             "../../VirtualRobot/data",
                                                             "../../../VirtualRobot/data",
                                                             "../../../../VirtualRobot/data"};
                vrDataPath = checkCandidatePaths(candidates, std::filesystem::current_path());
            });


        fs::path virtualRobotDataPath;
        for (auto source : sources)
        {
            source(virtualRobotDataPath);
            if (!virtualRobotDataPath.empty())
            {
                break;
            }
        }

        SimoxPath::simoxRootPath =
            virtualRobotDataPath.parent_path().parent_path().lexically_normal();
        simoxPathInitialized = true;
    }

    bool
    SimoxPath::checkPath(const char* path)
    {
        if (!path)
        {
            return false;
        }
        std::filesystem::path p(path);
        return checkPath(p);
    }

    bool
    SimoxPath::checkPath(const std::filesystem::path& path)
    {
        return std::filesystem::is_directory(path) || std::filesystem::is_symlink(path);
    }

    std::filesystem::path
    SimoxPath::checkCandidatePaths(const std::vector<std::string>& candidates,
                                   std::filesystem::path prefix)
    {
        for (const std::string& cand : candidates)
        {
            if (checkPath(prefix / cand))
            {
                return prefix / cand;
            }
        }
        return "";
    }

} // namespace simox
