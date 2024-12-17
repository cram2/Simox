// **************************************************************
// Implementation of class ContactConeGenerator
// **************************************************************
// Author: Niko Vahrenkamp, Martin Do
// Date: 27.10.2011
// **************************************************************


// **************************************************************
// includes
// **************************************************************

#include "ContactConeGenerator.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;
using namespace VirtualRobot;

namespace GraspStudio
{

    ContactConeGenerator::ContactConeGenerator(int coneSamples,
                                               float frictionCoeff,
                                               float unitForce)
    {

        this->unitForce = unitForce;
        //Generation of generic friction cone discretized by an 8-sided polyhedron
        this->frictionCoeff = frictionCoeff;
        this->frictionConeAngle = atan(frictionCoeff);
        this->frictionConeRad = unitForce * sin(frictionConeAngle);
        this->frictionConeHeight = unitForce * cos(frictionConeAngle);
        this->frictionConeSamples = coneSamples;

        for (int i = 0; i < frictionConeSamples; i++)
        {
            Eigen::Vector3f p;
            p(0) = unitForce *
                   (float)(cos(frictionConeAngle) * cos(i * 2.0 * M_PI / frictionConeSamples));
            p(1) = unitForce *
                   (float)(cos(frictionConeAngle) * sin(i * 2.0 * M_PI / frictionConeSamples));
            p(2) = (float)frictionConeHeight;
            frictionConeRimPoints.push_back(p);
        }
    }

    ContactConeGenerator::~ContactConeGenerator()
    {
        frictionConeRimPoints.clear();
    }

    void
    ContactConeGenerator::computeConePoints(
        const VirtualRobot::MathTools::ContactPoint& point,
        std::vector<VirtualRobot::MathTools::ContactPoint>& storeConePoints)
    {
        bool printInfo = false;

        if (printInfo)
        {
            std::cout << "Compute Cone Points" << std::endl;
            std::cout << "Point.p:" << std::endl;
            std::cout << point.p << std::endl;
            std::cout << "Point.n:" << std::endl;
            std::cout << point.n << std::endl;
            std::cout << "Point.force:" << std::endl;
            std::cout << point.force << std::endl;
            std::cout << "storeConePoints.size():" << storeConePoints.size() << std::endl;
        }

        //Rotate generic friction cone to align with object normals
        Eigen::Vector3f upRightNormal(0.0f, 0.0f, 1.0f);
        MathTools::Quaternion objNormalRot = MathTools::getRotation(upRightNormal, point.n);
        Eigen::Matrix4f objNormalTrafo = MathTools::quat2eigen4f(objNormalRot);

        Eigen::Vector3f conePoint;

        float scaleFactor = point.force;

        if (printInfo)
        {
            std::cout << "frictionConeSamples:" << frictionConeSamples << std::endl;
        }

        for (int i = 0; i < frictionConeSamples; i++)
        {
            VirtualRobot::MathTools::ContactPoint newConePoint;
            Eigen::Vector3f conePointOrg = frictionConeRimPoints[i] * scaleFactor;
            conePoint = MathTools::transformPosition(conePointOrg, objNormalTrafo);
            newConePoint.p = conePoint + point.p;
            newConePoint.n = conePoint;
            newConePoint.n.normalize();
            newConePoint.force = point.force;

            if (printInfo)
            {
                std::cout << "Loop " << i << std::endl;
                std::cout << "newConePoint.p:" << std::endl;
                std::cout << newConePoint.p << std::endl;
                std::cout << "newConePoint.n:" << std::endl;
                std::cout << newConePoint.n << std::endl;
                std::cout << "newConePoint.force:" << std::endl;
                std::cout << newConePoint.force << std::endl;
            }

            storeConePoints.push_back(newConePoint);
        }
    }

    void
    ContactConeGenerator::computeConePoints(const VirtualRobot::MathTools::ContactPoint& point,
                                            std::vector<Eigen::Vector3f>& storeConePoints)
    {
        bool printInfo = false;

        if (printInfo)
        {
            std::cout << "Compute Cone Points" << std::endl;
            std::cout << "Point.p:" << std::endl;
            std::cout << point.p << std::endl;
            std::cout << "Point.n:" << std::endl;
            std::cout << point.n << std::endl;
            std::cout << "Point.force:" << std::endl;
            std::cout << point.force << std::endl;
            std::cout << "storeConePoints.size():" << storeConePoints.size() << std::endl;
        }

        //Rotate generic friction cone to align with object normals
        Eigen::Vector3f upRightNormal(0.0f, 0.0f, 1.0f);
        MathTools::Quaternion objNormalRot =
            MathTools::getRotation(upRightNormal, point.n); // invert?!
        Eigen::Matrix4f objNormalTrafo = MathTools::quat2eigen4f(objNormalRot);

        Eigen::Vector3f conePoint;

        float scaleFactor = point.force;

        if (printInfo)
        {
            std::cout << "frictionConeSamples:" << frictionConeSamples << std::endl;
        }

        for (int i = 0; i < frictionConeSamples; i++)
        {
            Eigen::Vector3f newConePoint;
            Eigen::Vector3f conePointOrg = frictionConeRimPoints[i] * scaleFactor;
            conePoint = MathTools::transformPosition(conePointOrg, objNormalTrafo);
            newConePoint = conePoint + point.p;

            if (printInfo)
            {
                std::cout << "Loop " << i << std::endl;
                std::cout << "newConePoint:" << std::endl;
                std::cout << newConePoint << std::endl;
            }

            storeConePoints.push_back(newConePoint);
        }
    }

    float
    ContactConeGenerator::getConeAngle()
    {
        return (float)frictionConeAngle;
    }

    float
    ContactConeGenerator::getConeRadius()
    {
        return (float)frictionConeRad;
    }

    float
    ContactConeGenerator::getConeHeight()
    {
        return (float)frictionConeHeight;
    }

} // namespace GraspStudio
