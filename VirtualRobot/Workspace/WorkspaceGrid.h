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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../VirtualRobot.h"
#include <string>
#include <vector>
#include "WorkspaceRepresentation.h"
#include "../Grasping/Grasp.h"

namespace VirtualRobot
{
    /*!
    * A 2D grid which represents a quality distribution (e.g. the reachability) at 2D positions w.r.t. one or multiple grasp(s).
    * Internally the inverse workspace data (@see WorkspaceRepresentation), which encodes the
    * transformation between robot's base and grasping position, is used.
    * This data is useful to quickly sample positions from where the probability that a grasp is reachable is high (see \func getRandomPos).
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT WorkspaceGrid
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Setup the 2D grid with given extends and discretization parameter.
        */
        WorkspaceGrid(float minX, float maxX, float minY, float maxY, float discretizeSize, bool checkNeighbors = false);
        ~WorkspaceGrid();

        //! returns entry of position in x/y plane (in world coords)
        int getEntry(float x, float y);
        /*!
            Gets the corresponding entry of a world / global 2d position.
            \param x The x coordinate (in global coordinate system)
            \param y The y coordinate (in global coordinate system)
            \param storeEntry The corresponding entry is stored.
            \param storeGrasp A random grasp of corresponding cell is stored.
            \return True if x/y is inside this grid and an entry>0 was found, false otherwise.
        */
        bool getEntry(float x, float y, int& storeEntry, GraspPtr& storeGrasp);
        bool getEntry(float x, float y, int& nStoreEntry, std::vector<GraspPtr>& storeGrasps);

        //! returns entry of discretized square (x/y)
        int getCellEntry(int cellX, int cellY);
        bool getCellEntry(int cellX, int cellY, int& nStoreEntry, GraspPtr& storeGrasp);
        bool getCellEntry(int cellX, int cellY, int& nStoreEntry, std::vector<GraspPtr>& storeGrasps);
        /*!
            sets the entry to value, if the current value is lower
        */
        void setEntry(float x, float y, int value, GraspPtr grasp);
        void setCellEntry(int cellX, int cellY, int value, GraspPtr pGrasp);


        int getMaxEntry();

        /*!
            This method sets the grid value to nValue and checks if the neighbors have a lower value and in case the value of the neighbors is set to nValue.
        */
        void setEntryCheckNeighbors(float x, float y, int value, GraspPtr grasp);

        //! tries to find a random position with a entry >= minEntry
        bool getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, GraspPtr& storeGrasp, int maxLoops = 50, int* entries = NULL);
        bool getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, std::vector<GraspPtr>& storeGrasps, int maxLoops = 50, int* entries = NULL);

        /*!
            Clear all entries.
        */
        void reset();

        /*!
            Fill the grid with inverse reachability data generated from grasp g and object o.
        */
        bool fillGridData(WorkspaceRepresentationPtr ws, ManipulationObjectPtr o, GraspPtr g, RobotNodePtr baseRobotNode, float baseOrientation = 0, float maxAngle = M_PIf32);
        bool fillGridData(WorkspaceRepresentationPtr ws, const Eigen::Matrix4f &graspGlobal, GraspPtr g, RobotNodePtr baseRobotNode, float baseOrientation = 0, float maxAngle = M_PIf32, float minCenterDistance = 0);


        /*!
            Move the grid to (x,y), given in global coordinate system. Sets the new center.
        */
        void setGridPosition(float x, float y);

        /*!
            Get extends in global coord system.
        */
        void getExtends(float& storeMinX, float& storeMaxX, float& storeMinY, float& storeMaxY);

        struct Extends
        {
            float minX;
            float maxX;

            float minY;
            float maxY;
        };

        Extends getExtends() const
        {
            return Extends
            {
                .minX = minX,
                .maxX = maxX,
                .minY = minY,
                .maxY = maxY
            };
        }

        enum CellUpdateMode
        {
            MIN,
            MAX
        };

        void setCellUpdateMode(const CellUpdateMode mode)
        {
            this->mode = mode;
        }


        /*!
            Number of cells in x and y
        */
        void getCells(int& storeCellsX, int& storeCellsY);

        struct Size
        {
            int x,y;
        };

        Size getCells() const
        {
            return Size{.x = gridSizeX, .y = gridSizeY};
        }


        Eigen::Vector2f getPosition(int cellX, int cellY) const;


        float getDiscretizeSize() const;
        Eigen::Vector2f getMin() const;
        Eigen::Vector2f getMax() const;

        /**
         * @brief Creates the intersection between multiple grids into one grid by considering for each x,y position the worst values of all grids.
         * @param reachGrids grids for different grasp poses or object poses
         * @return new, merged grid
         */
        static WorkspaceGridPtr MergeWorkspaceGrids(const std::vector<WorkspaceGridPtr>& reachGrids);
    protected:
        /*!
            Adds data stored in reachability transformations. This data defines transformations from robot base system to grasping pose,
            so when defining a grasping pose, the inverse reachability can be represented by this grid
            Therefor the "world coordinates" of the inverse reachability distributions are computed by T_grasp * ReachTransformation^-1
        */
        void setEntries(std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr>& wsData, const Eigen::Matrix4f& graspGlobal, GraspPtr grasp);

        void checkAndReplaceValue(int& val, int newVal);



        inline int getDataPos(int x, int y)
        {
            return (x * gridSizeY + y);
        };
        float minX, maxX; // in global coord system
        float minY, maxY; // in global coord system
        float discretizeSize;

        int gridSizeX, gridSizeY;

        float gridExtendX, gridExtendY;

        int* data;                              // stores the quality values
        std::vector<GraspPtr>* graspLink;       // points to list of all reachable grasps

        const bool checkNeighbors;

    private:
        CellUpdateMode mode = CellUpdateMode::MAX;
    };

}
