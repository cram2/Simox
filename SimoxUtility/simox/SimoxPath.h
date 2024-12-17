#include <filesystem>
#include <vector>

namespace simox
{
    class SimoxPath
    {
    public:
        static std::filesystem::path
        getSimoxDir()
        {
            init();
            return simoxRootPath;
        }

        static std::filesystem::path
        getSimoxUtilityDir()
        {
            return getSimoxDir() / "SimoxUtility";
        }

        static std::filesystem::path
        getVirtualRobotDir()
        {
            return getSimoxDir() / "VirtualRobot";
        }

        static std::filesystem::path
        getVirtualRobotDataDir()
        {
            return getVirtualRobotDir() / "data";
        }

        static std::filesystem::path
        getSimDynamicsDir()
        {
            return getSimoxDir() / "SimDynamics";
        }

        static std::filesystem::path
        getGraspPlanningDir()
        {
            return getSimoxDir() / "GraspPlanning";
        }

        static std::filesystem::path
        getMotionPlanningDir()
        {
            return getSimoxDir() / "MotionPlanning";
        }


    private:
        SimoxPath();

        static void init();
        static bool checkPath(const char* path);
        static bool checkPath(const std::filesystem::path& path);
        static std::filesystem::path checkCandidatePaths(const std::vector<std::string>& candidates,
                                                         std::filesystem::path prefix = "");

    private:
        static bool simoxPathInitialized;
        static std::filesystem::path simoxRootPath;
    };
} // namespace simox
