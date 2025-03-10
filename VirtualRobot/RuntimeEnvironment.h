/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include "VirtualRobotImportExport.h"

namespace VirtualRobot
{
    /*!
     * The runtime environment holds data paths and program arguments.
     * Data files can be located by the getDataFileAbsolute method.
     * Here, the environment variable SIMOX_DATA_PATH and VIRTUAL_ROBOT_DATA_PATH are considered.
     * In addition, the install data directory is added at the end of the list of data paths for convenient access.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT RuntimeEnvironment
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //! Return vector of all data paths.
        static std::vector<std::string> getDataPaths();

        /// Set the runtime environment caption.
        static void setCaption(const std::string& caption);

        /*!
         * Add a path to global data path vector.
         * These paths are searched within the getDataFileAbsolute() method.
         * Only valid paths are processed.
         * \param path The path to add.
         * \param quiet If set, invalid paths are quietly ignored. Otherwise an error is printed.
         * \return true on success (path has to be a directory or a symlink).
         */
        static bool addDataPath(const std::string& path, bool quiet = false);

        /*!
         * \brief Enable the command line search for given key.
         * Only keys that are enabled can later be accessed with the getValue() method.
         */
        static void considerKey(const std::string& key, const std::string& description = "");
        /*!
         * \brief Enable the command line search for given flag.
         * A flag does not take an argument but is either present or not.
         */
        static void considerFlag(const std::string& flag, const std::string& description = "");

        /*!
         * Tries to find a file with name fileName. Therefore the working directory followed by all data paths are checked if the file can be found.
         * \param fileName Could be a filename or a relative path. In case the file could be located, the absolute path is stored in place.
         * \return True when the file could be located (the result will be stored in fileName).
         */
        static bool getDataFileAbsolute(std::string& fileName);

        /*!
         * The command line parameters can be passed in order to generate a map of key/value pairs.
         * All "--key value" pairs for which the key was enabled with the allowCommandLineOption() method are processed and stored as std::strings in a std::map.
         * In addition, all "--data-path <path>" entries are extracted and the according data paths are stored.
         * All unrecognized options are also stored.
         */
        static void processCommandLine(int argc, char* argv[]);

        //! Manually add a key/value pair.
        static void addKeyValuePair(const std::string& key, const std::string& value);

        //! Return the corresponding vale to key. If key cannot be found, defaultValue is returned.
        static std::string getValue(const std::string& key, const std::string& defaultValue = "");
        static bool hasValue(const std::string& key);

        /// Indicate whether the given flag was specified.
        static bool hasFlag(const std::string& flag);
        /// Indicate whether the 'help' flag was specified.
        static bool hasHelpFlag();

        //! Return all key value pairs
        static std::map<std::string, std::string> getKeyValuePairs();

        //! Return all unrecognized options
        static std::vector<std::string> getUnrecognizedOptions();

        //! Convert strings as '(a,b,c)' to a 3 dimensional vector.
        static bool toVector3f(const std::string& s, Eigen::Vector3f& storeResult);

        //! Get the given string as float.
        static float toFloat(const std::string& s);
        //! Get the given string as int.
        static int toInt(const std::string& s);


        /*!
         * Check if command line parameters specify a valid filename and
         * in case the key is not present in the command line arguments or the file was not found, the standardFilename is used.
         * Additionally the absolute filenames are considered by calling getDataFileAbsolute().
         * \param key  The key which is checked for a filename. It is checked if the key is present and if so, it is tried to
         *             construct a valid filename with getDataFileAbsolute().
         * \param standardFilename In case a valid file could not be determined, this file will be returned
         *                         (additionally it is made absolute by calling getDataFileAbsolute().
         *                         Hence, a filename with a relative path can be passed
         *                         and all datapaths are searched for it.
         * \return A valid filename if it can be found, otherwise the standardFilename is returned.
        */
        static std::string checkValidFileParameter(const std::string& key,
                                                   const std::string& standardFilename);

        /*!
         * Checks command line arguments for parameter key.
         * If present the corresponding value is returned, otherwise standardValue will be returned.
         */
        static std::string checkParameter(const std::string& key,
                                          const std::string& standardValue = "");

        //! Print status.
        static void print();

        //! Print the command line options (as printed by boost::program_options) to os.
        static void printOptions(std::ostream& os = std::cout);


        /*!
         * \brief Free all resources.
         * Usually not not needed, since on application exit all resources are freed automatically.
         */
        static void cleanup();


    protected:
        RuntimeEnvironment()
        {
        }

        virtual ~RuntimeEnvironment()
        {
        }

        static void init();
    };

} // namespace VirtualRobot
