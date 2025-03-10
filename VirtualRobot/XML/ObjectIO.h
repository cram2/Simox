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


#include <filesystem>
#include <fstream>

#include "../VirtualRobot.h"
#include "BaseIO.h"

// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template <class Ch>
    class xml_node;
}

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT ObjectIO : public BaseIO
    {
        friend class SceneIO;

    public:
        /*!
            Load Obstacle from file.
            @param xmlFile The file
            @return Returns an empty pointer, when file access failed.
        */
        static ObstaclePtr loadObstacle(const std::string& xmlFile);

        /*!
            Load Obstacle from a file stream.
            @param xmlFile The file stream
            @param basePath If file tags are given, the base path for searching the object files can be specified.
            @return Returns an empty pointer, when file access failed.
        */
        static ObstaclePtr loadObstacle(const std::ifstream& xmlFile,
                                        const std::string& basePath = "");

        /*!
            Load ManipulationObject from file.
            @param xmlFile The file
            @return Returns an empty pointer, when file access failed.
        */
        static ManipulationObjectPtr loadManipulationObject(const std::string& xmlFile);

        static ManipulationObjectPtr
        loadManipulationObject(const char* xmlFile)
        {
            return loadManipulationObject(std::string{xmlFile});
        }

        static ManipulationObjectPtr
        loadManipulationObject(const std::filesystem::path& xmlFile)
        {
            return loadManipulationObject(xmlFile.string());
        }

        /*!
            Load ManipulationObject from a file stream.
            @param xmlFile The file stream
            @param basePath If file tags are given, the base path for searching the object files can be specified.
            @return Returns an empty pointer, when file access failed.
        */
        static ManipulationObjectPtr loadManipulationObject(const std::ifstream& xmlFile,
                                                            const std::string& basePath = "");

        /*!
            Save ManipulationObject to file.
            @param object The object
            @param xmlFile The file
            @return Returns true on success.
        */
        static bool saveManipulationObject(ManipulationObjectPtr object,
                                           const std::string& xmlFile);

        /*!
            Creates ManipulationObject from string.
            @param xmlString The input string.
            @param basePath If file tags are given, the base path for searching the object files can be specified.
        */
        static ManipulationObjectPtr
        createManipulationObjectFromString(const std::string& xmlString,
                                           const std::string& basePath = "");

        /*!
            Creates Obstacle from string.
            @param xmlString The input string.
            @param basePath If file tags are given, the base path for searching the object files can be specified.
        */
        static ObstaclePtr createObstacleFromString(const std::string& xmlString,
                                                    const std::string& basePath = "");

        static ObstaclePtr processObstacle(rapidxml::xml_node<char>* objectXMLNode,
                                           const std::string& basePath);
        static ManipulationObjectPtr
        processManipulationObject(rapidxml::xml_node<char>* objectXMLNode,
                                  const std::string& basePath);

        /*!
         * \brief writeSTL Write ascii stl file.
         * \param t
         * \param filename
         * \param objectName
         * \param scaling Usually we scale from the internal mm format to m.
         * \return true on success
         */
        static bool writeSTL(TriMeshModelPtr t,
                             const std::string& filename,
                             const std::string& objectName,
                             float scaling = 0.001f);

    protected:
        // instantiation not allowed
        ObjectIO();
        ~ObjectIO() override;

        static bool processSceneRobot(rapidxml::xml_node<char>* sceneXMLNode,
                                      ScenePtr scene,
                                      const std::string& basePath);
    };

} // namespace VirtualRobot
