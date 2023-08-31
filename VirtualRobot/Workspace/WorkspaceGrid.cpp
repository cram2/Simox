
#include "WorkspaceGrid.h"
#include "../VirtualRobotException.h"
#include "../ManipulationObject.h"
#include "../Nodes/RobotNode.h"
#include "../Robot.h"
#include "VirtualRobot.h"
#include "Workspace/WorkspaceData.h"
#include <cmath>
#include <iostream>
#include <algorithm>


using namespace std;

#define MIN_VALUES_STORE_GRASPS 100

namespace VirtualRobot
{
    constexpr int INVALID_VALUE = -1;

    WorkspaceGrid::WorkspaceGrid(float fMinX, float fMaxX, float fMinY, float fMaxY, float fDiscretizeSize, const bool checkNeighbors) 
        : minX(fMinX), maxX(fMaxX), minY(fMinY), maxY(fMaxY), discretizeSize(fDiscretizeSize), data(nullptr), checkNeighbors(checkNeighbors)
    {
        if (fMinX >= fMaxX || fMinY >= fMaxY)
        {
            THROW_VR_EXCEPTION("ERROR min >= max");
        }

        gridExtendX = maxX - minX;
        gridExtendY = maxY - minY;
        gridSizeX = (int)((maxX - minX) / discretizeSize) + 1;
        gridSizeY = (int)((maxY - minY) / discretizeSize) + 1;


        if (gridSizeY <= 0 || gridSizeX <= 0 || gridExtendX <= 0 || gridExtendY <= 0)
        {
            THROW_VR_EXCEPTION("ERROR negative grid size");
        }

        // VR_INFO << ": creating grid with " << gridSizeX << "x" << gridSizeY << " = " << gridSizeX* gridSizeY << " entries" << std::endl;
        data = new int[gridSizeX * gridSizeY];
        graspLink = new std::vector<GraspPtr>[gridSizeX * gridSizeY];
        memset(data, INVALID_VALUE, sizeof(int)*gridSizeX * gridSizeY);
    }

    WorkspaceGrid::~WorkspaceGrid()
    {
        delete []data;
        delete []graspLink;
    }

    void WorkspaceGrid::reset()
    {
        delete []graspLink;
        graspLink = new std::vector<GraspPtr>[gridSizeX * gridSizeY];
        memset(data, INVALID_VALUE, sizeof(int)*gridSizeX * gridSizeY);
    }

    int WorkspaceGrid::getEntry(float x, float y)
    {
        if (!data)
        {
            return INVALID_VALUE;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);

        if (nPosX < 0 || nPosY < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << std::endl;
            return INVALID_VALUE;
        }

        return data[getDataPos(nPosX, nPosY)];
    }

    bool WorkspaceGrid::getEntry(float x, float y, int& storeEntry, GraspPtr& storeGrasp)
    {
        if (!data)
        {
            return false;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);

        if (nPosX < 0 || nPosY < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << std::endl;
            return false;
        }

        storeEntry = data[getDataPos(nPosX, nPosY)];
        size_t nLinks = graspLink[getDataPos(nPosX, nPosY)].size();

        if (nLinks > 0)
        {
            storeGrasp = graspLink[getDataPos(nPosX, nPosY)][rand() % nLinks];    // get random grasp
        }
        else
        {
            storeGrasp.reset();
        }

        return true;
    }

    bool WorkspaceGrid::getEntry(float x, float y, int& storeEntry, std::vector<GraspPtr>& storeGrasps)
    {
        if (!data)
        {
            return false;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);

        if (nPosX < 0 || nPosY < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << std::endl;
            return false;
        }

        storeEntry = data[getDataPos(nPosX, nPosY)];
        storeGrasps = graspLink[getDataPos(nPosX, nPosY)];
        return true;
    }

    int WorkspaceGrid::getCellEntry(int cellX, int cellY)
    {
        if (!data)
        {
            return INVALID_VALUE;
        }

        if (cellX < 0 || cellY < 0 || cellX >= gridSizeX || cellY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << std::endl;
            return INVALID_VALUE;
        }

        return data[getDataPos(cellX, cellY)];
    }

    bool WorkspaceGrid::getCellEntry(int cellX, int cellY, int& storeEntry, GraspPtr& storeGrasp)
    {
        if (!data)
        {
            return false;
        }

        if (cellX < 0 || cellY < 0 || cellX >= gridSizeX || cellY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << std::endl;
            return false;
        }

        storeEntry = data[getDataPos(cellX, cellY)];
        size_t nLinks = graspLink[getDataPos(cellX, cellY)].size();

        if (nLinks > 0)
        {
            storeGrasp = graspLink[getDataPos(cellX, cellY)][rand() % nLinks];
        }
        else
        {
            storeGrasp.reset();
        }

        return true;
    }

    bool WorkspaceGrid::getCellEntry(int cellX, int cellY, int& storeEntry, std::vector<GraspPtr>& storeGrasps)
    {
        if (!data)
        {
            return false;
        }

        if (cellX < 0 || cellY < 0 || cellX >= gridSizeX || cellY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nX << "," << nY << std::endl;
            return false;
        }

        storeEntry = data[getDataPos(cellX, cellY)];
        storeGrasps = graspLink[getDataPos(cellX, cellY)];
        return true;
    }

    void WorkspaceGrid::setEntry(float x, float y, int value, GraspPtr grasp)
    {
        if (!data)
        {
            return;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);
        setCellEntry(nPosX, nPosY, value, grasp);
    }

    void WorkspaceGrid::setCellEntry(int cellX, int cellY, const int value, GraspPtr grasp)
    {
        if (!data)
        {
            return;
        }

        if (cellX < 0 || cellY < 0 || cellX >= gridSizeX || cellY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << nPosX << "," << nPosY << std::endl;
            return;
        }

        int& lastVal = data[getDataPos(cellX, cellY)];
        checkAndReplaceValue(lastVal, value);
        
        if (value >= MIN_VALUES_STORE_GRASPS)
        {
            if (grasp && find(graspLink[getDataPos(cellX, cellY)].begin(), graspLink[getDataPos(cellX, cellY)].end(), grasp) == graspLink[getDataPos(cellX, cellY)].end())
            {
                graspLink[getDataPos(cellX, cellY)].push_back(grasp);
            }
        }
    }

    void WorkspaceGrid::checkAndReplaceValue(int& val, int newVal)
    {
        // check if cell is uninitialized
        if(val == INVALID_VALUE)
        {
            val = newVal;
            return;
        }

        switch(mode)
        {
            case CellUpdateMode::MIN:
                val = std::min(val, newVal);
                break;
            case CellUpdateMode::MAX:
                val = std::max(val, newVal);
                break;
        }
    }


    Eigen::Vector2f WorkspaceGrid::getPosition(int cellX, int cellY) const
    {
        float x = minX + cellX / gridSizeX * gridExtendX;
        float y = minY + cellY / gridSizeY * gridExtendY;

        return {x,y};

    }

    void WorkspaceGrid::setEntryCheckNeighbors(float x, float y, int value, GraspPtr grasp)
    {
        if (!data)
        {
            return;
        }

        int nPosX = (int)(((x - minX) / gridExtendX) * gridSizeX);
        int nPosY = (int)(((y - minY) / gridExtendY) * gridSizeY);

        if (nPosX < 0 || nPosY < 0 || nPosX >= gridSizeX || nPosY >= gridSizeY)
        {
            //cout << __PRETTY_FUNCTION__ << " internal error: " << fX << "," << fY << std::endl;
            return;
        }

        setCellEntry(nPosX, nPosY, value, grasp);

        // update neighbor cells as well
        if(checkNeighbors)
        {
            if (nPosX > 0 && nPosX < (gridSizeX - 1) && nPosY > 0 && nPosY < (gridSizeY - 1))
            {
                setCellEntry(nPosX - 1, nPosY, value, grasp);
                setCellEntry(nPosX - 1, nPosY - 1, value, grasp);
                setCellEntry(nPosX - 1, nPosY + 1, value, grasp);
                setCellEntry(nPosX, nPosY - 1, value, grasp);
                setCellEntry(nPosX, nPosY + 1, value, grasp);
                setCellEntry(nPosX + 1, nPosY - 1, value, grasp);
                setCellEntry(nPosX + 1, nPosY, value, grasp);
                setCellEntry(nPosX + 1, nPosY + 1, value, grasp);
            }
        }
        
    }

    void WorkspaceGrid::setEntries(std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr>& wsData, const Eigen::Matrix4f& graspGlobal, GraspPtr grasp)
    {
        if (!data)
        {
            return;
        }

        const Eigen::Isometry3f global_T_grasp(graspGlobal);

        for (auto & i : wsData)
        {
            const auto pos = global_T_grasp * Eigen::Isometry3f(i->transformation).inverse().translation();
            setEntryCheckNeighbors(pos.x(), pos.y(), i->value, grasp);
        }
    }

    int WorkspaceGrid::getMaxEntry()
    {
        if (!data)
        {
            return 0;
        }

        int nMax = 0;

        for (int i = 0; i < gridSizeX; i++)
            for (int j = 0; j < gridSizeY; j++)
                if (data[getDataPos(i, j)] > nMax)
                {
                    nMax = data[getDataPos(i, j)];
                }

        return nMax;
    }
    /*
    void WorkspaceGrid::visualize(SoSeparator *pStoreResult)
    {
        if (!data || !pStoreResult)
            return;
        float sizeX,sizeY;
        sizeX = gridExtendX / (float)gridSizeX;
        sizeY = gridExtendY / (float)gridSizeY;

        / *SoCube *cube = new SoCube();
        cube->width = sizeX*1.1f;
        cube->height = sizeY*1.1f;
        cube->depth = 1.0f;* /
        SoSphere *cube = new SoSphere();
        cube->radius.setValue(sizeX*1.0f);
        float x,y;

        int nMaxValue = getMaxEntry();
        if (nMaxValue==0)
            nMaxValue = 1;
        int value;

        for (int i=0;i<gridSizeX;i++)
        {
            for (int j=0;j<gridSizeY;j++)
            {
                if (data[getDataPos(i,j)]>0)
                {
                    value = data[getDataPos(i,j)];

                    SoSeparator *pSep1 = new SoSeparator();
                    x = minX + ((float)i+0.5f)*discretizeSize;
                    y = minY + ((float)j+0.5f)*discretizeSize;
                    SoTranslation *tr = new SoTranslation();
                    tr->translation.setValue(x,y,0);
                    float mat1 = (float)value/(float)nMaxValue;
                    mat1 *= 1.0f;
                    if (mat1<0.0f)
                        mat1 = 0.0f;
                    if (mat1>0.8f)
                        mat1 = 0.8f;
                    mat1 = 1.0f-mat1;
                    SoMaterial *mat= new SoMaterial();
                    mat->diffuseColor.setValue(1.0f,mat1,mat1);
                    mat->ambientColor.setValue(1.0f,mat1,mat1);

                    pSep1->addChild(tr);
                    pSep1->addChild(mat);
                    pSep1->addChild(cube);

                    pStoreResult->addChild(pSep1);
                }
            }
        }
    }*/

    bool WorkspaceGrid::getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, GraspPtr& storeGrasp, int maxLoops /*= 50*/, int* entries)
    {
        if (!data)
        {
            return false;
        }

        int nLoop = 0;
        int nEntry = 0;
        int x, y;

        do
        {
            x = rand() % gridSizeX;
            y = rand() % gridSizeY;
            getCellEntry(x, y, nEntry, storeGrasp);
            if(entries)
                *entries = nEntry;
            if (nEntry >= minEntry)
            {
                storeXGlobal = minX + ((float)x + 0.5f) * discretizeSize;
                storeYGlobal = minY + ((float)y + 0.5f) * discretizeSize;

                return true;
            }

            nLoop++;
        }
        while (nLoop < maxLoops);

        return false;
    }


    bool WorkspaceGrid::getRandomPos(int minEntry, float& storeXGlobal, float& storeYGlobal, std::vector<GraspPtr>& storeGrasps, int maxLoops /*= 50*/, int *entries)
    {
        if (!data)
        {
            return false;
        }

        int nLoop = 0;
        int nEntry = 0;
        int x, y;

        do
        {
            x = rand() % gridSizeX;
            y = rand() % gridSizeY;
            getCellEntry(x, y, nEntry, storeGrasps);
            if(entries)
                *entries = nEntry;
            if (nEntry >= minEntry)
            {
                storeXGlobal = minX + ((float)x + 0.5f) * discretizeSize;
                storeYGlobal = minY + ((float)y + 0.5f) * discretizeSize;
                return true;
            }

            nLoop++;
        }
        while (nLoop < maxLoops);

        return false;
    }

    void WorkspaceGrid::setGridPosition(float x, float y)
    {
        minX = x - gridExtendX / 2.0f;
        maxX = x + gridExtendX / 2.0f;
        minY = y - gridExtendY / 2.0f;
        maxY = y + gridExtendY / 2.0f;
    }

    bool WorkspaceGrid::fillGridData(WorkspaceRepresentationPtr ws, ManipulationObjectPtr o, GraspPtr g, RobotNodePtr baseRobotNode, const float baseOrientation, const float maxAngle)
    {
        if (!ws || !o || !g)
        {
            return false;
        }

        Eigen::Matrix4f graspGlobal = g->getTcpPoseGlobal(o->getGlobalPose());

        return fillGridData(ws, graspGlobal, g, baseRobotNode, baseOrientation, maxAngle);
    }

    bool WorkspaceGrid::fillGridData(WorkspaceRepresentationPtr ws, const Eigen::Matrix4f &global_T_grasp_orig, GraspPtr g, RobotNodePtr baseRobotNode, float baseOrientation, const float maxAngle, const float minCenterDistance)
    {
        (void) maxAngle;  // Unused.

        if (!ws)
        {
            return false;
        }

        // ensure robot is at identity
        Eigen::Matrix4f gpOrig = Eigen::Matrix4f::Identity();

        VR_ASSERT(baseRobotNode);
        if (baseRobotNode)
        {
            gpOrig = baseRobotNode->getRobot()->getGlobalPose();
            baseRobotNode->getRobot()->setGlobalPose(Eigen::Matrix4f::Identity());
        }


        Eigen::Isometry3f global_T_grasp(global_T_grasp_orig);
        global_T_grasp.linear() = Eigen::AngleAxisf(-baseOrientation, Eigen::Vector3f::UnitZ()) * global_T_grasp.linear();


        WorkspaceRepresentation::WorkspaceCut2DPtr cutXY = ws->createCut(global_T_grasp.matrix(), discretizeSize, false);

        std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> transformations = ws->createCutTransformations(cutXY, baseRobotNode, M_PI/*, isFlipped*/);
        
        // cut out a cylinder around the robot if minCenterDistance is specified
        if(minCenterDistance > 0)
        {
            transformations.erase(std::remove_if(transformations.begin(), transformations.end(), [&](const WorkspaceRepresentation::WorkspaceCut2DTransformationPtr& tp){
                
                const Eigen::Isometry3f robot_T_tcp(tp->transformation);
                return robot_T_tcp.translation().head<2>().norm() < minCenterDistance; 
            }), transformations.end());
        }

        setEntries(transformations, global_T_grasp_orig.matrix(), g);

        if (baseRobotNode)
        {
            baseRobotNode->getRobot()->setGlobalPose(gpOrig);
        }

        return true;
    }

    void WorkspaceGrid::getExtends(float& storeMinX, float& storeMaxX, float& storeMinY, float& storeMaxY)
    {
        storeMinX = minX;
        storeMaxX = maxX;
        storeMinY = minY;
        storeMaxY = maxY;
    }

    void WorkspaceGrid::getCells(int& storeCellsX, int& storeCellsY)
    {
        storeCellsX = gridSizeX;
        storeCellsY = gridSizeY;
    }

    float WorkspaceGrid::getDiscretizeSize() const
    {
        return discretizeSize;
    }

    Eigen::Vector2f WorkspaceGrid::getMin() const
    {
        return Eigen::Vector2f(minX, minY);
    }

    Eigen::Vector2f WorkspaceGrid::getMax() const
    {
        return Eigen::Vector2f(maxX, maxY);
    }

    WorkspaceGridPtr WorkspaceGrid::MergeWorkspaceGrids(const std::vector<WorkspaceGridPtr> &grids)
    {
        VR_ASSERT(grids.size() >= 2);
        float totalMaxX = std::numeric_limits<float>::min();
        float totalMinX = std::numeric_limits<float>::max();
        float totalMaxY = std::numeric_limits<float>::min();
        float totalMinY = std::numeric_limits<float>::max();
        for(auto& grid : grids)
        {
//            VR_INFO << "min: " << grid->getMin() << " max: " << grid->getMax() << std::endl;
            totalMinX = std::min(grid->getMin()(0), totalMinX);
            totalMinY = std::min(grid->getMin()(1), totalMinY);
            totalMaxX = std::max(grid->getMax()(0), totalMaxX);
            totalMaxY = std::max(grid->getMax()(1), totalMaxY);
        }
        WorkspaceGridPtr resultGrid(new WorkspaceGrid(totalMinX, totalMaxX, totalMinY, totalMaxY, grids.at(0)->getDiscretizeSize()));
//        VR_INFO << "minx : " << totalMinX << " maxX: " << totalMaxX << " step size: " << resultGrid->getDiscretizeSize() << std::endl;
//        int sameValueCount =  0;
//        int totalValueCount = 0;
        for(float x = totalMinX; x < totalMaxX; x += resultGrid->getDiscretizeSize())
        {
            for(float y = totalMinY; y < totalMaxY; y += resultGrid->getDiscretizeSize())
            {
                int min = std::numeric_limits<int>::max();
                GraspPtr grasp;
                for(auto& grid : grids)
                {
                    int score;
                    GraspPtr tmpGrasp;
                    if(grid->getEntry(x, y, score, tmpGrasp))
                    {
                        grasp = tmpGrasp;
                    }
                    min = std::min(score, min);
                }
//                totalValueCount++;
                resultGrid->setEntry(x, y, min, grasp);
            }
        }
//        VR_INFO << "Same value percentage: " << (100.f * sameValueCount/totalValueCount) << std::endl;
        return resultGrid;

    }

} //  namespace
