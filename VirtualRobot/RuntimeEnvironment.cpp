#include "Logging.h"
#ifdef WIN32
#pragma warning(                                                                                   \
    disable : 4275) // non dll-interface class 'std::logic_error' used as base for dll-interface class 'boost::program_options::error'
#pragma warning(                                                                                   \
    disable : 4251) // class 'std::vector<_Ty>' needs to have dll-interface to be used by clients of class 'boost::program_options::ambiguous_option'
#pragma warning(                                                                                   \
    disable : 4996) // warning on insecure char* usage, that arises when using boost::split
#endif

#include "RuntimeEnvironment.h"

#include <filesystem>

#include <boost/program_options.hpp>

#include <SimoxUtility/algorithm/string/string_tools.h>

#include "VirtualRobotException.h"
#include "Visualization/VisualizationFactory.h"

namespace VirtualRobot
{
    static bool RuntimeEnvironment_pathInitialized = false;

    static std::string RuntimeEnvironment_caption = "Simox runtime options";

    /// Pairs of (key, description). If not given, description is empty.
    static std::vector<std::pair<std::string, std::string>> RuntimeEnvironment_processKeys;
    /// Pairs of (flag, description). If not given, description is empty.
    static std::vector<std::pair<std::string, std::string>> RuntimeEnvironment_processFlags;

    static std::vector<std::string> RuntimeEnvironment_dataPaths;
    static std::vector<std::string> RuntimeEnvironment_unrecognizedOptions;

    static std::map<std::string, std::string> RuntimeEnvironment_keyValues;
    static std::set<std::string> RuntimeEnvironment_flags;
    static bool RuntimeEnvironment_helpFlag = false;

    /// Make the options description based on the added keys and flags.
    static boost::program_options::options_description
    RuntimeEnvironment_makeOptionsDescription()
    {
        // Declare the supported options.

        boost::program_options::options_description desc(RuntimeEnvironment_caption);
        desc.add_options()("help", "Simox command line parser: Set options with '--key value'\n")(
            "data-path",
            boost::program_options::value<std::vector<std::string>>()->composing(),
            "Set data path. Multiple data paths are allowed.");

        for (const auto& item : RuntimeEnvironment_processKeys)
        {
            desc.add_options()(item.first.c_str(),
                               boost::program_options::value<std::vector<std::string>>(),
                               item.second.c_str());
        }

        for (const auto& item : RuntimeEnvironment_processFlags)
        {
            desc.add_options()(item.first.c_str(), item.second.c_str());
        }
        return desc;
    }

    static void
    RuntimeEnvironment_processParsedOptions(const boost::program_options::parsed_options& parsed)
    {
        boost::program_options::variables_map vm;
        //boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::store(parsed, vm);
        boost::program_options::notify(vm);

        // process data-path entries
        if (vm.count("data-path"))
        {
            //VR_INFO << "Data paths are: " << std::endl;
            std::vector<std::string> dp = vm["data-path"].as<std::vector<std::string>>();

            for (const auto& i : dp)
            {
                RuntimeEnvironment::addDataPath(i);
                //VR_INFO << dp[i] << "\n";
            }
        }

        // process generic keys
        for (const auto& processKey : RuntimeEnvironment_processKeys)
        {
            const std::string& key = processKey.first;
            if (vm.count(key.c_str()) > 0)
            {
                std::vector<std::string> dp = vm[key.c_str()].as<std::vector<std::string>>();

                if (dp.size() > 1)
                {
                    VR_WARNING << "More than one parameter for key '" << key
                               << "'. Using only first one..." << std::endl;
                }

                if (dp.size() > 0)
                {
                    RuntimeEnvironment::addKeyValuePair(key, dp[0]); // take the first one...
                }
            }
        }

        for (const auto& flag : RuntimeEnvironment_processFlags)
        {
            if (vm.count(flag.first.c_str()) > 0)
            {
                RuntimeEnvironment_flags.insert(flag.first);
            }
        }
        RuntimeEnvironment_helpFlag = vm.count("help") > 0;

        // collect unrecognized arguments
        std::vector<std::string> options = boost::program_options::collect_unrecognized(
            parsed.options, boost::program_options::include_positional);

        for (const auto& option : options)
        {
            RuntimeEnvironment_unrecognizedOptions.push_back(option);
        }
    }

    void
    RuntimeEnvironment::init()
    {
        if (!RuntimeEnvironment_pathInitialized)
        {
            RuntimeEnvironment_pathInitialized = true;
            bool pathFound = false;
            char* simox_data_path = getenv("SIMOX_DATA_PATH");

            if (simox_data_path)
            {
                pathFound = addDataPath(std::string(simox_data_path), true);
            }

            char* vr_data_path = getenv("VIRTUAL_ROBOT_DATA_PATH");

            if (vr_data_path)
            {
                pathFound = addDataPath(std::string(vr_data_path), true);
            }

            if (!pathFound)
            {
                // test for Simox_DIR
                simox_data_path = getenv("Simox_DIR");

                if (simox_data_path)
                {
                    std::string sd(simox_data_path);
                    sd += std::string("/data");
                    pathFound = addDataPath(sd, true);

                    if (!pathFound)
                    {
                        std::string sd(simox_data_path);
                        sd += std::string("/VirtualRobot/data");
                        pathFound = addDataPath(sd, true);
                    }

                    if (!pathFound)
                    {
                        std::string sd(simox_data_path);
                        sd += std::string("../VirtualRobot/data");
                        pathFound = addDataPath(sd, true);
                    }
                }
            }

#ifdef Simox_DATA_PATH
            pathFound = pathFound | addDataPath(std::string(Simox_DATA_PATH), true);
#endif
#ifdef VirtualRobot_DATA_PATH
            pathFound = pathFound | addDataPath(std::string(VirtualRobot_DATA_PATH), true);
#endif
#ifdef VirtualRobot_SRC_DATA_PATH
            pathFound = pathFound | addDataPath(std::string(VirtualRobot_SRC_DATA_PATH), true);
#endif

            // check standard linux install path
            if (!pathFound)
            {
                pathFound = addDataPath(std::string("/usr/local/data"), true);
            }

            if (!pathFound)
            {
                pathFound = addDataPath(std::string("/usr/data"), true);
            }

            // last chance, check for inbuild paths
            if (!pathFound)
            {
                std::filesystem::path p(std::filesystem::current_path());
                std::filesystem::path p1 = p / "../VirtualRobot/data";
                std::filesystem::path p2 = p / "../../VirtualRobot/data";
                std::filesystem::path p3 = p / "../../../VirtualRobot/data";
                std::filesystem::path p4 = p / "../../../../VirtualRobot/data";
                pathFound = pathFound | addDataPath(p1.string(), true);
                pathFound = pathFound | addDataPath(p2.string(), true);
                pathFound = pathFound | addDataPath(p3.string(), true);
                pathFound = pathFound | addDataPath(p4.string(), true);
            }
        }
    }

    bool
    RuntimeEnvironment::getDataFileAbsolute(std::string& fileName)
    {
        if (!RuntimeEnvironment_pathInitialized)
        {
            init();
        }

        std::filesystem::path fn(fileName);

        try
        {
            // first check current path
            if (std::filesystem::exists(fn) && std::filesystem::is_regular_file(fn))
            {
                fileName = fn.string();
                return true;
            }
        }
        catch (...) //const std::filesystem::filesystem_error& /*ex*/)
        {
            //cout << ex.what() << '\n';
            // silently skip this error (e.g. device not ready, permission denied etc)
        }

        for (auto& dataPath : RuntimeEnvironment_dataPaths)
        {
            std::filesystem::path p(dataPath);

            std::filesystem::path fnComplete = p / fn;

            try
            {
                if (std::filesystem::exists(fnComplete) &&
                    std::filesystem::is_regular_file(fnComplete))
                {
                    // check for permissions (todo)
                    //std::filesystem::file_status s = std::filesystem::status(fnComplete);
                    //printf("%o\n",s.permissions());

                    fileName = fnComplete.string();
                    return true;
                }
            }
            catch (...) //const std::filesystem::filesystem_error& ex)
            {
                //cout << "EX:" << ex.what() << '\n';
                // silently skip this error (e.g. device not ready, permission denied etc)
            }
        }

        return false;
    }

    void
    RuntimeEnvironment::processCommandLine(int argc, char* argv[])
    {
        if (!RuntimeEnvironment_pathInitialized)
        {
            init();
        }

        const boost::program_options::options_description description =
            RuntimeEnvironment_makeOptionsDescription();

        const boost::program_options::parsed_options parsed =
            boost::program_options::command_line_parser(argc, argv)
                .options(description)
                .allow_unregistered()
                .run();

        RuntimeEnvironment_processParsedOptions(parsed);
    }

    void
    RuntimeEnvironment::addKeyValuePair(const std::string& key, const std::string& value)
    {
        RuntimeEnvironment_keyValues[key] = value;
    }

    std::string
    RuntimeEnvironment::getValue(const std::string& key, const std::string& defaultValue)
    {
        auto it = RuntimeEnvironment_keyValues.find(key);
        if (it != RuntimeEnvironment_keyValues.end())
        {
            return it->second;
        }

        return defaultValue;
    }

    std::map<std::string, std::string>
    RuntimeEnvironment::getKeyValuePairs()
    {
        return RuntimeEnvironment_keyValues;
    }

    std::vector<std::string>
    RuntimeEnvironment::getUnrecognizedOptions()
    {
        return RuntimeEnvironment_unrecognizedOptions;
    }

    std::vector<std::string>
    RuntimeEnvironment::getDataPaths()
    {
        if (!RuntimeEnvironment_pathInitialized)
        {
            init();
        }

        return RuntimeEnvironment_dataPaths;
    }

    void
    RuntimeEnvironment::setCaption(const std::string& caption)
    {
        RuntimeEnvironment_caption = caption;
    }

    bool
    RuntimeEnvironment::addDataPath(const std::string& path, bool quiet)
    {
        try
        {
            std::filesystem::path p(path);

            if (std::filesystem::is_directory(p) || std::filesystem::is_symlink(p))
            {
                RuntimeEnvironment_dataPaths.push_back(path);
                return true;
            }
        }
        catch (...)
        {
        }

        if (!quiet)
        {
            VR_ERROR << "Trying to add non-existing data path: " << path << std::endl;
        }

        return false;
    }

    static std::size_t
    getMaxLength(const std::vector<std::pair<std::string, std::string>>& items)
    {
        std::size_t max = 0;
        for (const auto& key : items)
        {
            max = std::max(max, key.first.size());
        }
        return max;
    }

    static std::string
    padding(std::size_t current, std::size_t target, char c = ' ')
    {
        std::stringstream ss;
        while (target-- > current)
        {
            ss << c;
        }
        return ss.str();
    }

    void
    RuntimeEnvironment::print()
    {
        if (!RuntimeEnvironment_pathInitialized)
        {
            init();
        }

        std::cout << " *********** Simox RuntimeEnvironment ************* " << std::endl;
        std::cout << "Data paths:" << std::endl;

        for (const auto& dataPath : RuntimeEnvironment_dataPaths)
        {
            std::cout << " * " << dataPath << std::endl;
        }

        const std::size_t descriptionOffset =
            std::max(getMaxLength(RuntimeEnvironment_processKeys),
                     getMaxLength(RuntimeEnvironment_processFlags)) +
            4;

        auto printDescriptions =
            [&descriptionOffset](const std::vector<std::pair<std::string, std::string>>& items,
                                 const std::string& name)
        {
            if (items.size() > 0)
            {
                std::cout << "Known " << name << ":" << std::endl;

                for (const auto& item : items)
                {
                    std::cout << " * " << item.first
                              << padding(item.first.size(), descriptionOffset) << item.second
                              << std::endl;
                }
            }
        };

        printDescriptions(RuntimeEnvironment_processKeys, "keys");
        printDescriptions(RuntimeEnvironment_processFlags, "flags");


        if (RuntimeEnvironment_keyValues.size() > 0)
        {
            std::cout << "Parsed options:" << std::endl;
            for (const auto& item : RuntimeEnvironment_keyValues)
            {
                std::cout << " * " << item.first << ": " << item.second << std::endl;
            }
        }

        if (RuntimeEnvironment_flags.size() > 0)
        {
            std::cout << "Parsed flags:" << std::endl;
            for (const auto& flag : RuntimeEnvironment_flags)
            {
                std::cout << " * " << flag << std::endl;
            }
        }

        if (RuntimeEnvironment_unrecognizedOptions.size() > 0)
        {
            std::cout << "Unrecognized options:" << std::endl;

            for (const auto& unrecognizedOption : RuntimeEnvironment_unrecognizedOptions)
            {
                std::cout << " * <" << unrecognizedOption << ">" << std::endl;
            }
        }
    }

    void
    RuntimeEnvironment::printOptions(std::ostream& os)
    {
        os << RuntimeEnvironment_makeOptionsDescription() << std::endl;
    }

    void
    RuntimeEnvironment::considerKey(const std::string& key, const std::string& description)
    {
        RuntimeEnvironment_processKeys.emplace_back(key, description);
    }

    void
    RuntimeEnvironment::considerFlag(const std::string& flag, const std::string& description)
    {
        RuntimeEnvironment_processFlags.emplace_back(flag, description);
    }

    bool
    RuntimeEnvironment::hasValue(const std::string& key)
    {
        return RuntimeEnvironment_keyValues.find(key) != RuntimeEnvironment_keyValues.end();
    }

    bool
    RuntimeEnvironment::hasFlag(const std::string& flag)
    {
        return RuntimeEnvironment_flags.find(flag) != RuntimeEnvironment_flags.end();
    }

    bool
    RuntimeEnvironment::hasHelpFlag()
    {
        return RuntimeEnvironment_helpFlag;
    }

    float
    RuntimeEnvironment::toFloat(const std::string& s)
    {
        return std::stof(s);
    }

    int
    RuntimeEnvironment::toInt(const std::string& s)
    {
        return std::stoi(s);
    }

    bool
    RuntimeEnvironment::toVector3f(const std::string& string, Eigen::Vector3f& storeResult)
    {
        if (string.length() < 3)
        {
            return false;
        }

        if (string[0] != '(' || string[string.length() - 1] != ')')
        {
            VR_WARNING << "Expecting string to start and end with brackets (): " << string
                       << std::endl;
            return false;
        }

        const std::string stringTrimmed = string.substr(1, string.size() - 1);
        const std::string delimiter = ",";

        std::vector<std::string> stringSplit = simox::alg::split(stringTrimmed, delimiter);

        if (stringSplit.size() != 3)
        {
            VR_WARNING << "Expecting values of string to be separated with a ',': " << string
                       << std::endl;
            return false;
        }

        Eigen::Vector3f result;

        for (int i = 0; i < result.SizeAtCompileTime; ++i)
        {
            const std::string& string = stringSplit[static_cast<std::size_t>(i)];
            bool error = false;
            float a;
            try
            {
                a = std::stof(string);
            }
            catch (const std::invalid_argument&)
            {
                error = true;
            }
            if (error || std::isinf(a) || std::isinf(-a) || std::isnan(a))
            {
                VR_WARNING << "Could not convert '" << string << "' to a number." << std::endl;
                return false;
            }
            result(i) = a;
        }

        storeResult = result;
        return true;
    }

    std::string
    RuntimeEnvironment::checkValidFileParameter(const std::string& key,
                                                const std::string& standardFilename)
    {
        if (VirtualRobot::RuntimeEnvironment::hasValue(key))
        {
            std::string f = RuntimeEnvironment::getValue(key);

            if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(f))
            {
                return f;
            }
        }

        // don't check for empty files
        if (standardFilename.empty())
        {
            return standardFilename;
        }

        std::string s = standardFilename;

        if (!RuntimeEnvironment::getDataFileAbsolute(s))
        {
            VR_WARNING << "Could not determine path to file " << standardFilename << std::endl;
            return standardFilename;
        }

        return s;
    }

    std::string
    RuntimeEnvironment::checkParameter(const std::string& key,
                                       const std::string& standardValue /*= ""*/)
    {
        if (RuntimeEnvironment::hasValue(key))
        {
            return RuntimeEnvironment::getValue(key);
        }

        return standardValue;
    }

    void
    RuntimeEnvironment::cleanup()
    {
        VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(nullptr);

        if (visualizationFactory)
        {
            visualizationFactory->cleanup();
        }
    }


} // namespace VirtualRobot
