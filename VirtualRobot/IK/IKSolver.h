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

#include <memory>

#include "VirtualRobot/VirtualRobotImportExport.h"

namespace VirtualRobot
{
    /*!
        Abstract IK solver interface.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT IKSolver : public std::enable_shared_from_this<IKSolver>
    {
    public:
        /*!
        @brief Flags for the selection of the target components.
        @details The flags can be combined with the +-operator.
        */
        enum CartesianSelection
        {
            X = 1,
            Y = 2,
            Z = 4,
            Position = 7,
            Orientation = 8,
            All = 15
        };

        IKSolver();
    };

    using IKSolverPtr = std::shared_ptr<IKSolver>;

} // namespace VirtualRobot
