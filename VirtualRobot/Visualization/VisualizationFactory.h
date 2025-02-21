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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <string>

#include <eigen3/Eigen/Core>

#include "SimoxUtility/color/Color.h"
#include "VirtualRobot/AbstractFactoryMethod.h"
#include "VirtualRobot/BoundingBox.h"
#include "VirtualRobot/Primitive.h"
#include "VirtualRobot/VirtualRobot.h"

namespace VirtualRobot
{
    class VisualizationNode;

    class VIRTUAL_ROBOT_IMPORT_EXPORT VisualizationFactory :
        public ::AbstractFactoryMethod<VisualizationFactory, void*>
    {
    public:
        struct Color
        {
            Color() = default;

            Color(float r, float g, float b, float transparency = 0.0f) :
                r(r), g(g), b(b), transparency(transparency)
            {
            }

            float r = 0.5f, g = 0.5f, b = 0.5f;
            float transparency = 1;

            bool
            isNone() const
            {
                return transparency >= 1.0f;
            }

            static Color
            Blue(float transparency = 0.0f)
            {
                return Color(0.2f, 0.2f, 1.0f, transparency);
            }

            static Color
            Red(float transparency = 0.0f)
            {
                return Color(1.0f, 0.2f, 0.2f, transparency);
            }

            static Color
            Green(float transparency = 0.0f)
            {
                return Color(0.2f, 1.0f, 0.2f, transparency);
            }

            static Color
            Black(float transparency = 0.0f)
            {
                return Color(0, 0, 0, transparency);
            }

            static Color
            Gray()
            {
                return Color(0.5f, 0.5f, 0.5f, 0);
            }

            static Color
            None()
            {
                return Color(0.0f, 0.0f, 0.0f, 1.0f);
            }

            Color(const simox::Color& sc) :
                Color(sc.r / 255., sc.g / 255., sc.b / 255., 1. - sc.a / 255.)
            {
            }
        };

        struct PhongMaterial
        {
            PhongMaterial() = default;
            Color emission;
            Color ambient;
            Color diffuse;
            Color specular;
            float shininess{0};
            Color reflective;
            float reflectivity{0};
            Color transparent;
            float transparency{0};
            float refractionIndex{0};
        };

        VisualizationFactory() = default;
        virtual ~VisualizationFactory() = default;

        virtual void
        init(int& /*argc*/, char* /*argv*/[], const std::string& /*appName*/)
        {
        }

        virtual VisualizationNodePtr
        getVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& /*primitives*/,
                                       bool /*boundingBox*/ = false,
                                       const Color& /*color*/ = Color::Gray())
        {
            return nullptr;
        }

        virtual VisualizationNodePtr
        getVisualizationFromFile(const std::string& /*filename*/,
                                 bool /*boundingBox*/ = false,
                                 float /*scaleX*/ = 1.0f,
                                 float /*scaleY*/ = 1.0f,
                                 float /*scaleZ*/ = 1.0f)
        {
            return nullptr;
        }

        virtual VisualizationNodePtr
        getVisualizationFromFile(const std::ifstream& /*ifs*/,
                                 bool /*boundingBox*/ = false,
                                 float /*scaleX*/ = 1.0f,
                                 float /*scaleY*/ = 1.0f,
                                 float /*scaleZ*/ = 1.0f)
        {
            return nullptr;
        }

        // virtual VisualizationNodePtr createPlane(const MathTools::Plane& plane, float extend, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        // {
        //     return createPlane(plane.p, plane.n, extend, transparency, colorR, colorG, colorB);
        // }


        virtual VisualizationNodePtr
        createBox(float width, float height, float depth, const Color& color = Color::Gray());

        virtual VisualizationNodePtr createLine(const Eigen::Vector3f& from,
                                                const Eigen::Vector3f& to,
                                                float width = 1.0f,
                                                const Color& color = Color::Gray());

        virtual VisualizationNodePtr createSphere(float radius, const Color& color = Color::Gray());

        virtual VisualizationNodePtr
        createCylinder(float radius, float height, const Color& color = Color::Gray());

        virtual VisualizationNodePtr createCircle(float radius,
                                                  float circleCompletion,
                                                  float width,
                                                  const Color& color = Color::Gray(),
                                                  size_t numberOfCircleParts = 30);

        virtual VisualizationNodePtr createCoordSystem(float /*scaling*/ = 1.0f,
                                                       std::string* text = nullptr,
                                                       float axisLength = 100.0f,
                                                       float axisSize = 3.0f,
                                                       int nrOfBlocks = 10);

        virtual VisualizationNodePtr createBoundingBox(const BoundingBox& bbox,
                                                       bool wireFrame = false);

        virtual VisualizationNodePtr createVertexVisualization(const Eigen::Vector3f& position,
                                                               float radius,
                                                               const Color& color = Color::Gray());

        virtual VisualizationNodePtr
        createTriMeshModelVisualization(const TriMeshModelPtr& model,
                                        const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity(),
                                        float scaleX = 1.0f,
                                        float scaleY = 1.0f,
                                        float scaleZ = 1.0f);

        virtual VisualizationNodePtr createTriMeshModelVisualization(const TriMeshModelPtr& model,
                                                                     bool showNormals,
                                                                     const Eigen::Matrix4f& pose,
                                                                     bool /*showLines*/ = true);

        virtual VisualizationNodePtr createPlane(const Eigen::Vector3f& position,
                                                 const Eigen::Vector3f& normal,
                                                 float extend,
                                                 const Color& color = Color::Gray());

        virtual VisualizationNodePtr createArrow(const Eigen::Vector3f& n,
                                                 float length = 50.0f,
                                                 float width = 2.0f,
                                                 const Color& color = Color::Gray());

        virtual VisualizationNodePtr createCircleArrow(float radius,
                                                       float tubeRadius,
                                                       float completion = 1,
                                                       const Color& color = Color::Gray(),
                                                       int sides = 8,
                                                       int rings = 30);

        virtual VisualizationNodePtr createTrajectory(TrajectoryPtr t,
                                                      const Color& colorNode = Color::Blue(),
                                                      const Color& colorLine = Color::Gray(),
                                                      float nodeSize = 15.0f,
                                                      float lineSize = 4.0f);

        virtual VisualizationNodePtr createText(const std::string& text,
                                                bool billboard = false,
                                                float scaling = 1.0f,
                                                const Color& c = Color::Black(),
                                                float offsetX = 20.0f,
                                                float offsetY = 20.0f,
                                                float offsetZ = 0.0f);

        virtual VisualizationNodePtr createTorus(float radius,
                                                 float tubeRadius,
                                                 float completion = 1,
                                                 const Color& color = Color::Gray(),
                                                 int sides = 8,
                                                 int rings = 30);

        /*!
            Creates an coordinate axis aligned ellipse
            \param x The extend in x direction must be >= 1e-6
            \param y The extend in y direction must be >= 1e-6
            \param z The extend in z direction must be >= 1e-6
            \param showAxes If true, the axes are visualized
            \param axesHeight The height of the axes (measured from the body surface)
            \param axesWidth The width of the axes.
            \return A VisualizationNode containing the visualization.
        */
        virtual VisualizationNodePtr
        createEllipse(float /*x*/,
                      float /*y*/,
                      float /*z*/,
                      bool /*showAxes*/ = true,
                      float /*axesHeight*/ = 4.0f,
                      float /*axesWidth*/ = 8.0f)
        {
            return nullptr;
        }

        /*!
            Move local visualization by homogeneous matrix m. (MM)
        */
        virtual void applyDisplacement(VisualizationNodePtr visu,
                                       const Eigen::Matrix4f& displacement);

        /*!
            Create an empty VisualizationNode.
        */
        virtual VisualizationNodePtr
        createVisualization()
        {
            return nullptr;
        }

        /*!
            Create a united visualization.
        */
        virtual VisualizationNodePtr
        createUnitedVisualization(const std::vector<VisualizationNodePtr>& /*visualizations*/) const
        {
            return nullptr;
        }

        /*!
            Here, a manual cleanup can be called, no Coin3D access possible after this.
            Usually no need to call cleanup explicitly, since cleanup is performed automatically at application exit.
        */
        virtual void
        cleanup()
        {
        }
    };

    using ColorPtr = std::shared_ptr<VisualizationFactory::Color>;

} // namespace VirtualRobot
