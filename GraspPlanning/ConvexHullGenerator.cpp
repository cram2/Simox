


#include "ConvexHullGenerator.h"

#include <cfloat>
#include <cmath>
#include <iostream>

#include <VirtualRobot/Visualization/TriMeshModel.h>

//#define CONVEXHULL_DEBUG_OUTPUT

using namespace std;
using namespace VirtualRobot;
using namespace VirtualRobot::MathTools;

namespace GraspStudio
{
    bool
    ConvexHullGenerator::ConvertPoints(const std::vector<Eigen::Vector3f>& points,
                                       double* storePointsQHull)
    {
        for (int i = 0; i < (int)points.size(); i++)
        {
            storePointsQHull[i * 3 + 0] = points[i][0];
            storePointsQHull[i * 3 + 1] = points[i][1];
            storePointsQHull[i * 3 + 2] = points[i][2];
        }
        return true;
    }

    bool
    ConvexHullGenerator::ConvertPoints(const std::vector<ContactPoint>& points,
                                       double* storePointsQHull)
    {
        for (int i = 0; i < (int)points.size(); i++)
        {
            storePointsQHull[i * 6 + 0] = points[i].p[0];
            storePointsQHull[i * 6 + 1] = points[i].p[1];
            storePointsQHull[i * 6 + 2] = points[i].p[2];
            storePointsQHull[i * 6 + 3] = points[i].n[0];
            storePointsQHull[i * 6 + 4] = points[i].n[1];
            storePointsQHull[i * 6 + 5] = points[i].n[2];
        }
        return true;
    }

    VirtualRobot::MathTools::ConvexHull3DPtr
    ConvexHullGenerator::CreateConvexHull(VirtualRobot::TriMeshModelPtr pointsInput)
    {
        return CreateConvexHull(pointsInput->vertices);
    }

    /*
    bool createPoints( SoSeparator *pInputIVModel, std::vector<Vec3D> &vStorePoints)
    {
        if (!pInputIVModel)
            return false;

        vStorePoints.clear();
        SoCallbackAction ca;
        ca.addTriangleCallback(SoShape::getClassTypeId(), &CConvexHullGenerator_triangleCB, &vStorePoints);
        ca.apply(pInputIVModel);
        return true;
    }*/

    /*
    bool createConvexHull(SoSeparator *pInputIVModel, ConvexHull3D &storeResult)
    {
        vector<Vec3D> points;
        if (!CreatePoints(pInputIVModel,points,false))
            return false;
        bool bRes = CreateConvexHull(points,storeResult,false);
        return bRes;
    }*/

    /*
    bool createIVModel( ConvexHull3D &convexHull, SoSeparator *pStoreResult)
    {
        if (!pStoreResult || convexHull.vertices.size()<=0 || convexHull.faces.size()<=0)
            return false;

        SoCoordinate3* pCoords = new SoCoordinate3();
        SoFaceSet* pFaceSet = new SoFaceSet();

        int nFaces = (int)convexHull.faces.size();
        int nVertices = nFaces*3;
        Face3D f;
        Vec3D v1,v2,v3;

        SbVec3f *pVertexArray = new SbVec3f[nVertices];

        int nVertexCount = 0;

        for (int i=0;i<nFaces;i++)
        {
            f = convexHull.faces.at(i);
            v1 = convexHull.vertices.at(f.id[0]);
            v2 = convexHull.vertices.at(f.id[1]);
            v3 = convexHull.vertices.at(f.id[2]);

            bool bNeedFlip = GraspStudioHelpers::checkVerticeOrientation(v1,v2,v3,f.normal);

            // COUNTER CLOCKWISE
            if (bNeedFlip)
                pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
            else
                pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
            nVertexCount++;
            pVertexArray[nVertexCount].setValue((float)v2.x,(float)v2.y,(float)v2.z);
            nVertexCount++;
            if (bNeedFlip)
                pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
            else
                pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
            nVertexCount++;

        }
        pCoords->point.setValues(0,nVertices,pVertexArray);
        long *nNumVertices = new long[nFaces];
        for (int i=0;i<nFaces;i++)
            nNumVertices[i] = 3;
        pFaceSet->numVertices.setValues(0,nFaces,(const int32_t*)nNumVertices);

        pStoreResult->addChild(pCoords);
        pStoreResult->addChild(pFaceSet);
        delete []pVertexArray;
        delete []nNumVertices;

        return true;
    }*/

    /*
    void addVertex(Vec3D &v1,Vec3D &v2,Vec3D &v3,Vec3D &normal,SbVec3f *pVertexArray, int& nVertexCount)
    {
        bool bNeedFlip = GraspStudioHelpers::checkVerticeOrientation(v1,v2,v3,normal);

        // COUNTER CLOCKWISE
        if (bNeedFlip)
            pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
        else
            pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
        nVertexCount++;

        pVertexArray[nVertexCount].setValue((float)v2.x,(float)v2.y,(float)v2.z);
        nVertexCount++;

        if (bNeedFlip)
            pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
        else
            pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
        nVertexCount++;
    }*/

    /*
    bool createIVModel(ConvexHull6D &convHull, SoSeparator *pStoreResult, bool buseFirst3Coords)
    {
        if (!pStoreResult || convHull.vertices.size()<=0 || convHull.faces.size()<=0)
            return false;

        Face6D f;
        Vec3D v1,v2,v3,v4,v5,v6;


        int nFaces = (int)convHull.faces.size();


        // project points to 3d, then create hull of these points to visualize it
        std::vector<Vec3D> vProjectedPoints;
        for (int i=0;i<nFaces;i++)
        {
            f = convHull.faces.at(i);
            if (buseFirst3Coords)
            {
                v1.x = convHull.vertices.at(f.id[0]).x;
                v1.y = convHull.vertices.at(f.id[0]).y;
                v1.z = convHull.vertices.at(f.id[0]).z;
                v2.x = convHull.vertices.at(f.id[1]).x;
                v2.y = convHull.vertices.at(f.id[1]).y;
                v2.z = convHull.vertices.at(f.id[1]).z;
                v3.x = convHull.vertices.at(f.id[2]).x;
                v3.y = convHull.vertices.at(f.id[2]).y;
                v3.z = convHull.vertices.at(f.id[2]).z;
                v4.x = convHull.vertices.at(f.id[3]).x;
                v4.y = convHull.vertices.at(f.id[3]).y;
                v4.z = convHull.vertices.at(f.id[3]).z;
                v5.x = convHull.vertices.at(f.id[4]).x;
                v5.y = convHull.vertices.at(f.id[4]).y;
                v5.z = convHull.vertices.at(f.id[4]).z;
                v6.x = convHull.vertices.at(f.id[5]).x;
                v6.y = convHull.vertices.at(f.id[5]).y;
                v6.z = convHull.vertices.at(f.id[5]).z;
            } else
            {
                v1.x = convHull.vertices.at(f.id[0]).nx;
                v1.y = convHull.vertices.at(f.id[0]).ny;
                v1.z = convHull.vertices.at(f.id[0]).nz;
                v2.x = convHull.vertices.at(f.id[1]).nx;
                v2.y = convHull.vertices.at(f.id[1]).ny;
                v2.z = convHull.vertices.at(f.id[1]).nz;
                v3.x = convHull.vertices.at(f.id[2]).nx;
                v3.y = convHull.vertices.at(f.id[2]).ny;
                v3.z = convHull.vertices.at(f.id[2]).nz;
                v4.x = convHull.vertices.at(f.id[3]).nx;
                v4.y = convHull.vertices.at(f.id[3]).ny;
                v4.z = convHull.vertices.at(f.id[3]).nz;
                v5.x = convHull.vertices.at(f.id[4]).nx;
                v5.y = convHull.vertices.at(f.id[4]).ny;
                v5.z = convHull.vertices.at(f.id[4]).nz;
                v6.x = convHull.vertices.at(f.id[5]).nx;
                v6.y = convHull.vertices.at(f.id[5]).ny;
                v6.z = convHull.vertices.at(f.id[5]).nz;
            }
            vProjectedPoints.push_back(v1);
            vProjectedPoints.push_back(v2);
            vProjectedPoints.push_back(v3);
            vProjectedPoints.push_back(v4);
            vProjectedPoints.push_back(v5);
            vProjectedPoints.push_back(v6);
        }
        ConvexHull3D projectedHull;
        if (!CreateConvexHull(vProjectedPoints, projectedHull, false))
        {
            std::cout << __FUNCTION__ << " Could not create hull of projected points, aborting..." << std::endl;
            return false;
        }
        bool bRes = CreateIVModel(projectedHull, pStoreResult, false);
        return bRes;
        / *
        // creates 3d-projection of all 6d facets
        int nVertices = nFaces*12;
        SoCoordinate3* pCoords = new SoCoordinate3();
        SoFaceSet* pFaceSet = new SoFaceSet();
        Face6d f;
        Vec3d v1,v2,v3,v4,v5,v6;
        Vec3d normal;

        SbVec3f *pVertexArray = new SbVec3f[nVertices];

        int nVertexCount = 0;
        bool bNeedFlip = false;

        for (int i=0;i<nFaces;i++)
        {
            f = convHull.faces.at(i);
            if (buseFirst3Coords)
            {
                v1.x = convHull.vertices.at(f.id[0]).x;
                v1.y = convHull.vertices.at(f.id[0]).y;
                v1.z = convHull.vertices.at(f.id[0]).z;
                v2.x = convHull.vertices.at(f.id[1]).x;
                v2.y = convHull.vertices.at(f.id[1]).y;
                v2.z = convHull.vertices.at(f.id[1]).z;
                v3.x = convHull.vertices.at(f.id[2]).x;
                v3.y = convHull.vertices.at(f.id[2]).y;
                v3.z = convHull.vertices.at(f.id[2]).z;
                v4.x = convHull.vertices.at(f.id[3]).x;
                v4.y = convHull.vertices.at(f.id[3]).y;
                v4.z = convHull.vertices.at(f.id[3]).z;
                v5.x = convHull.vertices.at(f.id[4]).x;
                v5.y = convHull.vertices.at(f.id[4]).y;
                v5.z = convHull.vertices.at(f.id[4]).z;
                v6.x = convHull.vertices.at(f.id[5]).x;
                v6.y = convHull.vertices.at(f.id[5]).y;
                v6.z = convHull.vertices.at(f.id[5]).z;
                normal.x = f.normal.x;
                normal.y = f.normal.y;
                normal.z = f.normal.z;
            } else
            {
                v1.x = convHull.vertices.at(f.id[0]).nx;
                v1.y = convHull.vertices.at(f.id[0]).ny;
                v1.z = convHull.vertices.at(f.id[0]).nz;
                v2.x = convHull.vertices.at(f.id[1]).nx;
                v2.y = convHull.vertices.at(f.id[1]).ny;
                v2.z = convHull.vertices.at(f.id[1]).nz;
                v3.x = convHull.vertices.at(f.id[2]).nx;
                v3.y = convHull.vertices.at(f.id[2]).ny;
                v3.z = convHull.vertices.at(f.id[2]).nz;
                v4.x = convHull.vertices.at(f.id[3]).nx;
                v4.y = convHull.vertices.at(f.id[3]).ny;
                v4.z = convHull.vertices.at(f.id[3]).nz;
                v5.x = convHull.vertices.at(f.id[4]).nx;
                v5.y = convHull.vertices.at(f.id[4]).ny;
                v5.z = convHull.vertices.at(f.id[4]).nz;
                v6.x = convHull.vertices.at(f.id[5]).nx;
                v6.y = convHull.vertices.at(f.id[5]).ny;
                v6.z = convHull.vertices.at(f.id[5]).nz;
                normal.x = f.normal.nx;
                normal.y = f.normal.ny;
                normal.z = f.normal.nz;
            }
            bool bNeedFlip = GraspStudioHelpers::checkVerticeOrientation(v1,v2,v3,normal);
            if (bNeedFlip)
            {
                pVertexArray[nVertexCount].setValue((float)v6.x,(float)v6.y,(float)v6.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v5.x,(float)v5.y,(float)v5.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v4.x,(float)v4.y,(float)v4.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v2.x,(float)v2.y,(float)v2.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
                nVertexCount++;
            } else
            {
                pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v2.x,(float)v2.y,(float)v2.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v4.x,(float)v4.y,(float)v4.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v5.x,(float)v5.y,(float)v5.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v6.x,(float)v6.y,(float)v6.z);
                nVertexCount++;
            }
        }
        pCoords->point.setValues(0,nVertices,pVertexArray);
        long *nNumVertices = new long[nFaces];
        for (int i=0;i<nFaces;i++)
            nNumVertices[i] = 6;
        pFaceSet->numVertices.setValues(0,nFaces,(const int32_t*)nNumVertices);

        pStoreResult->addChild(pCoords);
        pStoreResult->addChild(pFaceSet);
        delete []pVertexArray;
        delete []nNumVertices;


        return true;
        * /
    }*/

    void
    ConvexHullGenerator::PrintVertices(std::vector<ContactPoint>& pointsInput)
    {
        for (std::size_t i = 0; i < pointsInput.size(); i++)
        {
            std::cout << "v" << i << ": " << pointsInput[i].p[0] << "," << pointsInput[i].p[1]
                      << "," << pointsInput[i].p[2] << "," << pointsInput[i].n[0] << ","
                      << pointsInput[i].n[1] << "," << pointsInput[i].n[2] << std::endl;
        }
    }

    void
    ConvexHullGenerator::PrintStatistics(VirtualRobot::MathTools::ConvexHull6DPtr convHull)
    {
        if (!convHull)
        {
            GRASPSTUDIO_ERROR << " Null data to print" << std::endl;
            return;
        }

        float minValue[6];
        float maxValue[6];

        for (std::size_t i = 0; i <= 5; i++)
        {
            minValue[i] = FLT_MAX;
            maxValue[i] = -FLT_MAX;
        }

        for (std::size_t i = 0; i < convHull->vertices.size(); i++)
        {
            if (convHull->vertices[i].p[0] < minValue[0])
            {
                minValue[0] = convHull->vertices[i].p[0];
            }

            if (convHull->vertices[i].p[0] > maxValue[0])
            {
                maxValue[0] = convHull->vertices[i].p[0];
            }

            if (convHull->vertices[i].p[1] < minValue[1])
            {
                minValue[1] = convHull->vertices[i].p[1];
            }

            if (convHull->vertices[i].p[1] > maxValue[1])
            {
                maxValue[1] = convHull->vertices[i].p[1];
            }

            if (convHull->vertices[i].p[2] < minValue[2])
            {
                minValue[2] = convHull->vertices[i].p[2];
            }

            if (convHull->vertices[i].p[2] > maxValue[2])
            {
                maxValue[2] = convHull->vertices[i].p[2];
            }

            if (convHull->vertices[i].n[0] < minValue[3])
            {
                minValue[3] = convHull->vertices[i].n[0];
            }

            if (convHull->vertices[i].n[0] > maxValue[3])
            {
                maxValue[3] = convHull->vertices[i].n[0];
            }

            if (convHull->vertices[i].n[1] < minValue[4])
            {
                minValue[4] = convHull->vertices[i].n[1];
            }

            if (convHull->vertices[i].n[1] > maxValue[4])
            {
                maxValue[4] = convHull->vertices[i].n[1];
            }

            if (convHull->vertices[i].n[2] < minValue[5])
            {
                minValue[5] = convHull->vertices[i].n[2];
            }

            if (convHull->vertices[i].n[2] > maxValue[5])
            {
                maxValue[5] = convHull->vertices[i].n[2];
            }
        }

        std::cout << "Conv Hull Bounds:" << std::endl;
        std::cout << "\t\t x : " << minValue[0] << "," << maxValue[0] << std::endl;
        std::cout << "\t\t y : " << minValue[1] << "," << maxValue[1] << std::endl;
        std::cout << "\t\t z : " << minValue[2] << "," << maxValue[2] << std::endl;
        std::cout << "\t\t nx: " << minValue[3] << "," << maxValue[3] << std::endl;
        std::cout << "\t\t ny: " << minValue[4] << "," << maxValue[4] << std::endl;
        std::cout << "\t\t nz: " << minValue[5] << "," << maxValue[5] << std::endl;
    }

    bool
    ConvexHullGenerator::checkVerticeOrientation(const Eigen::Vector3f& v1,
                                                 const Eigen::Vector3f& v2,
                                                 const Eigen::Vector3f& v3,
                                                 const Eigen::Vector3f& n)
    {
        Eigen::Vector3f tmp;
        Eigen::Vector3f v1v2;
        Eigen::Vector3f v1v3;
        v1v2(0) = v2(0) - v1(0);
        v1v2(1) = v2(1) - v1(1);
        v1v2(2) = v2(2) - v1(2);
        v1v3(0) = v3(0) - v1(0);
        v1v3(1) = v3(1) - v1(1);
        v1v3(2) = v3(2) - v1(2);
        tmp = v1v2.cross(v1v3);
        float tmpF = tmp.dot(n);
        return (tmpF < 0);
        /*crossProduct(v1v2,v1v3,tmp);
        float tmpF = dotProduct(tmp,n);*/
    }


} // namespace GraspStudio
