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
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <fstream>
#include <vector>

#include <eigen3/Eigen/Core>

#include <VirtualRobot/VirtualRobotImportExport.h>

namespace VirtualRobot
{
    namespace FileIO
    {

        template <typename T>
        inline T
        read(std::ifstream& file)
        {
            T t;
            file.read((char*)&t, sizeof(T));
            return t;
        }

        template <typename T>
        inline void
        readArray(T* res, int num, std::ifstream& file)
        {
            file.read((char*)res, num * sizeof(T));
        }

        template <typename T>
        inline void
        write(std::ofstream& file, T value)
        {
            file.write((char*)&value, sizeof(T));
        }

        template <typename T>
        inline void
        writeArray(std::ofstream& file, const T* value, int num)
        {
            file.write((char*)value, num * sizeof(T));
        }

        bool readString(std::string& res, std::ifstream& file);

        bool readMatrix4f(Eigen::Matrix4f& res, std::ifstream& file);

        void writeMatrix4f(std::ofstream& file, const Eigen::Matrix4f& m);

        void writeString(std::ofstream& file, const std::string& value);

        /*!
            Read points form ascii file.
            Each row defines one point triple.
            \param filename The absolute filename.
            \param separator Separator character. Standard space, but comma or semicolon could be passed here.
            \return Vector of points.
        */
        std::vector<Eigen::Vector3f> VIRTUAL_ROBOT_IMPORT_EXPORT
        readPts(const std::string& filename, const char separator = ' ');
    } // namespace FileIO

} // namespace VirtualRobot
