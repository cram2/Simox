// **************************************************************
// Implementation of class GraspQualityMeasure
// **************************************************************
// Author: Niko Vahrenkamp
// Date: 26.10.2011
// **************************************************************


// **************************************************************
// includes
// **************************************************************

#include "GraspQualityMeasureWrenchSpace.h"
#include <cmath>
#include <cstdio>
#include <cassert>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cfloat>
// if defined, the inverted contact normals are used
//#define INVERT_NORMALS

using namespace std;
using namespace VirtualRobot;

namespace GraspStudio
{


    GraspQualityMeasureWrenchSpace::GraspQualityMeasureWrenchSpace(VirtualRobot::SceneObjectPtr object, float unitForce, float frictionConeCoeff, int frictionConeSamples)
        : GraspQualityMeasure(object, unitForce, frictionConeCoeff, frictionConeSamples)
    {
        OWSCalculated = false;
        GWSCalculated = false;
        minOffsetOWS = 0.0f;
        volumeOWS = 0.0f;
    }

    GraspQualityMeasureWrenchSpace::~GraspQualityMeasureWrenchSpace() = default;


    void GraspQualityMeasureWrenchSpace::setContactPoints(const std::vector<VirtualRobot::MathTools::ContactPoint>& contactPoints)
    {
        GraspQualityMeasure::setContactPoints(contactPoints);
        GWSCalculated = false;
    }

    void GraspQualityMeasureWrenchSpace::setContactPoints(const VirtualRobot::EndEffector::ContactInfoVector& contactPoints)
    {
        GraspQualityMeasure::setContactPoints(contactPoints);
        GWSCalculated = false;
    }


    float GraspQualityMeasureWrenchSpace::getGraspQuality()
    {
        calculateObjectProperties();
        calculateGraspQuality();
        return graspQuality;
    }

    bool GraspQualityMeasureWrenchSpace::isGraspForceClosure()
    {
        if (!GWSCalculated)
        {
            calculateGWS();
        }

        return isOriginInGWSHull();
    }

    void GraspQualityMeasureWrenchSpace::calculateOWS(int samplePoints)
    {
        bool printAll = true;
        bool bRes = sampleObjectPoints(samplePoints);

        if (!bRes)
        {
            return;
        }

        //Rotate generic friction cone to align with object normals
        std::vector<VirtualRobot::MathTools::ContactPoint>::iterator objPointsIter;
        std::vector<VirtualRobot::MathTools::ContactPoint> conePoints;

        if (verbose && printAll)
        {
            cout << "OWS contact points:" << endl;
        }

        for (objPointsIter = sampledObjectPointsM.begin(); objPointsIter != sampledObjectPointsM.end(); objPointsIter++)
        {
            if (verbose && printAll)
            {
                MathTools::print((*objPointsIter));
            }

            coneGenerator->computeConePoints((*objPointsIter), conePoints);
        }

        //Generate convex hull from rotated object friction cones
        convexHullOWS = calculateConvexHull(conePoints);
        //calculateHullCenter(convexHullOWS, convexHullCenterOWS);
        convexHullCenterOWS = convexHullOWS->center;

        if (verbose && printAll)
        {
            GRASPSTUDIO_INFO << " CENTER of OWS: " << endl;
            MathTools::print(convexHullCenterOWS);
        }

        minOffsetOWS = minOffset(convexHullOWS);
        volumeOWS = convexHullOWS->volume;

        if (verbose)
        {
            GRASPSTUDIO_INFO << ": MinDistance to Center of OWS: " << minOffsetOWS << endl;
            GRASPSTUDIO_INFO << ": Volume of OWS: " << volumeOWS << endl;
        }

        OWSCalculated = true;
    }

    void GraspQualityMeasureWrenchSpace::calculateGWS()
    {
        bool printAll = false;

        if (contactPointsM.empty())
        {
            if (verbose && printAll)
            {
                printf("Contact points not set.\n");
            }
            return;
        }

        //Rotate generic friction cone to align with object normals
        std::vector<VirtualRobot::MathTools::ContactPoint>::iterator objPointsIter;
        std::vector<VirtualRobot::MathTools::ContactPoint> conePoints;

        if (verbose && printAll)
        {
            cout << "GWS contact points:" << endl;
        }

        for (objPointsIter = contactPointsM.begin(); objPointsIter != contactPointsM.end(); objPointsIter++)
        {
            if (verbose && printAll)
            {
                MathTools::print((*objPointsIter));
            }

            coneGenerator->computeConePoints((*objPointsIter), conePoints);
        }

        //Generate convex hull from rotated contact friction cones
        convexHullGWS = calculateConvexHull(conePoints);
        //calculateHullCenter(convexHullGWS, convexHullCenterGWS);
        convexHullCenterGWS = convexHullGWS->center;

        if (verbose && printAll)
        {
            GRASPSTUDIO_INFO << " CENTER of GWS: " << endl;
            MathTools::print(convexHullCenterGWS);
        }

        GWSCalculated = true;
    }

    VirtualRobot::MathTools::ConvexHull6DPtr GraspQualityMeasureWrenchSpace::calculateConvexHull(std::vector<VirtualRobot::MathTools::ContactPoint>& points)
    {
        bool printAll = true;

        if (verbose && printAll)
        {
            cout << "Convex hull points for wrench calculation:" << endl;
            printContacts(points);
        }

        // create wrench
        std::vector<VirtualRobot::MathTools::ContactPoint> wrenchPoints = createWrenchPoints(points, Eigen::Vector3f::Zero(), objectLength); // contact points are already moved so that com is at origin

        if (verbose && printAll)
        {
            cout << "Wrench points:" << endl;
            printContacts(wrenchPoints);
        }

        return ConvexHullGenerator::CreateConvexHull(wrenchPoints);
    }

    std::vector<VirtualRobot::MathTools::ContactPoint> GraspQualityMeasureWrenchSpace::createWrenchPoints(std::vector<VirtualRobot::MathTools::ContactPoint>& points, const Eigen::Vector3f& centerOfModel, float objectLengthMM)
    {
        std::vector<VirtualRobot::MathTools::ContactPoint> result;
        std::vector<VirtualRobot::MathTools::ContactPoint>::iterator iter = points.begin();
        VirtualRobot::MathTools::ContactPoint p;
        Eigen::Vector3f normal;

        // convert objLength from mm to m
        bool convertMM2M = true;

        float factor = 1.0f;

        if (objectLengthMM != 0)
        {
            if (convertMM2M)
            {
                factor = 2.0f / (objectLengthMM / 1000.0f);    // == max distance from center ( 1 / (length/2) )
            }
            else
            {
                factor = 2.0f / objectLengthMM;    // == max distance from center ( 1 / (length/2) )
            }
        }

        VirtualRobot::MathTools::ContactPoint tmpP;

        while (iter != points.end())
        {
#ifdef INVERT_NORMALS
            /*p.p(0) = -(*iter).n(0);
                    p.p(1) = -(*iter).n(1);
                    p.p(2) = -(*iter).n(2);*/
            p.p = -1.0f * (iter->n);
#else
            p.p = iter->n;
#endif
            tmpP.p = iter->p - centerOfModel;
            tmpP.n = -(iter->n);

            //normal = crossProductPosNormalInv(tmpP);
            //p.n = factor * normal;
            p.n = factor * tmpP.p.cross(tmpP.n);

            result.push_back(p);
            iter++;
        }

        return result;
    }

    void GraspQualityMeasureWrenchSpace::printContacts(std::vector<VirtualRobot::MathTools::ContactPoint>& points)
    {
        std::vector<VirtualRobot::MathTools::ContactPoint>::iterator iter = points.begin();

        while (iter != points.end())
        {
            cout << "# ";
            MathTools::print((*iter));
            iter++;
        }
    }

    VirtualRobot::MathTools::ContactPoint GraspQualityMeasureWrenchSpace::calculateHullCenter(VirtualRobot::MathTools::ConvexHull6DPtr hull)
    {
        if (!hull)
        {
            GRASPSTUDIO_ERROR << "NULL data?!" << endl;
            return VirtualRobot::MathTools::ContactPoint();
        }

        VirtualRobot::MathTools::ContactPoint resultCenter;
        std::vector<VirtualRobot::MathTools::ContactPoint>::iterator iter;
        resultCenter.p.setZero();
        resultCenter.n.setZero();

        if (hull->vertices.size() == 0)
        {
            cout << __FUNCTION__ << ": error, no vertices..." << endl;
            return resultCenter;
        }

        for (iter = hull->vertices.begin(); iter != hull->vertices.end(); iter++)
        {
            resultCenter.p += iter->p;
            resultCenter.n += iter->n;
        }

        resultCenter.p /= (float)hull->vertices.size();
        resultCenter.n /= (float)hull->vertices.size();
        return resultCenter;
    }

    float GraspQualityMeasureWrenchSpace::minDistanceToGWSHull(VirtualRobot::MathTools::ContactPoint& point)
    {
        float minDist = FLT_MAX;
        float dist[6];
        float currentDist2;
        std::vector<MathTools::TriangleFace6D>::iterator faceIter;

        for (faceIter = convexHullGWS->faces.begin(); faceIter != convexHullGWS->faces.end(); faceIter++)
        {
            VirtualRobot::MathTools::ContactPoint faceCenter;
            faceCenter.p.setZero();
            faceCenter.n.setZero();

            for (int j : faceIter->id)
            {
                faceCenter.p += (convexHullGWS->vertices)[j].p;
                faceCenter.n += (convexHullGWS->vertices)[j].n;
            }

            faceCenter.p /= 6.0f;
            faceCenter.n /= 6.0f;

            currentDist2 = 0;

            for (int j = 0; j < 3; j++)
            {
                dist[j] = (faceCenter.p(j) - point.p(j));
                dist[j + 3] = (faceCenter.n(j) - point.n(j));
                currentDist2 += dist[j] * dist[j];
                currentDist2 += dist[j + 3] * dist[j + 3];
            }

            if (currentDist2 < minDist)
            {
                minDist = currentDist2;
            }
        }

        return sqrtf(minDist);
    }
    bool GraspQualityMeasureWrenchSpace::isOriginInGWSHull()
    {
        if (!GWSCalculated || !convexHullGWS)
        {
            return false;
        }

        for (const auto& face : convexHullGWS->faces)
        {
            // ignore rounding errors
            if (face.distPlaneZero > 1e-4)
            {
                return false;
            }
        }

        return true;
    }



    float GraspQualityMeasureWrenchSpace::minOffset(VirtualRobot::MathTools::ConvexHull6DPtr ch)
    {
        if (!ch)
        {
            return 0.0f;
        }

        float fRes = FLT_MAX;
        int nWrongFacets = 0;

        for (auto& face : ch->faces)
        {
            const auto dist = face.distNormCenter;
            if (dist > 0)
            {
                //outside
                nWrongFacets++;
                continue;
            }
            fRes = std::min(fRes, -dist);
        }

        if (nWrongFacets > 0)
        {
            cout << __FUNCTION__ << " Warning: offset of " << nWrongFacets << " facets >0 (# of facets:" << ch->faces.size() << ")" << endl;
        }

        return fRes;
    }

    float GraspQualityMeasureWrenchSpace::getVolumeGraspMeasure()
    {
        if (!GWSCalculated)
        {
            calculateGWS();
        }

        if (!convexHullGWS || convexHullGWS->vertices.size() == 0)
        {
            cout << __FUNCTION__ << "No vertices in Grasp Wrench Space?! Maybe I was not initialized correctly..." << endl;
            return 0.0;
        }

        if (volumeOWS <= 0 || convexHullGWS->volume <= 0)
        {
            cout << __FUNCTION__ << "No Volumes in Wrench Space?! Maybe I was not initialized correctly..." << endl;
            return 0.0;
        }

        float fRes = convexHullGWS->volume;

        if (volumeOWS > 1e-10)
        {
            fRes /= volumeOWS;
        }

        return fRes;
    }

    void GraspQualityMeasureWrenchSpace::preCalculatedOWS(float fMinDist, float volume)
    {
        minOffsetOWS = fMinDist;
        volumeOWS = volume;
        OWSCalculated = true;
    }

    bool GraspQualityMeasureWrenchSpace::calculateObjectProperties()
    {
        if (!OWSCalculated)
        {
            calculateOWS();
        }

        return true;
    }

    bool GraspQualityMeasureWrenchSpace::calculateGraspQuality()
    {
        if (!GWSCalculated)
        {
            calculateGWS();
        }

        graspQuality = 0.0f;

        if (!convexHullGWS || convexHullGWS->vertices.size() == 0)
        {
            cout << __FUNCTION__ << "No vertices in Grasp Wrench Space?! Maybe I was not initialized correctly..." << endl;
            return 0.0;
        }

        if (minOffsetOWS <= 0 || convexHullGWS->volume <= 0)
        {
            cout << __FUNCTION__ << "No Volumes in Wrench Space?! Maybe I was not initialized correctly..." << endl;
            return 0.0;
        }

        float fResOffsetGWS = minOffset(convexHullGWS);
        float fResOffsetOWS = minOffsetOWS;


        if (fResOffsetOWS != 0)
        {
            graspQuality = fResOffsetGWS / fResOffsetOWS;
        }
        else
        {
            graspQuality = fResOffsetGWS;
        }

        if (verbose)
        {
            GRASPSTUDIO_INFO << endl;
            cout << ": GWS volume    : " << convexHullGWS->volume << endl;
            cout << ": GWS min Offset: " << fResOffsetGWS << endl;
            cout << ": OWS min Offset: " << fResOffsetOWS << endl;
            cout << ": GraspQuality  : " << graspQuality << endl;
        }

        return true;
    }

    std::string GraspQualityMeasureWrenchSpace::getName()
    {
        return "GraspWrenchSpace";
    }


    Eigen::Vector3f GraspQualityMeasureWrenchSpace::crossProductPosNormalInv(const VirtualRobot::MathTools::ContactPoint& v1)
    {
        Eigen::Vector3f res;
        res(0) = v1.p(1) * (-v1.n(2)) - v1.p(2) * (-v1.n(1));
        res(1) = v1.p(2) * (-v1.n(0)) - v1.p(0) * (-v1.n(2));
        res(2) = v1.p(0) * (-v1.n(1)) - v1.p(1) * (-v1.n(0));
        return res;
    }

}
