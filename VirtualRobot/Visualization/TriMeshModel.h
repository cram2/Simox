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

#include <utility>
#include <vector>

#include <eigen3/Eigen/Core>

#include "../BoundingBox.h"
#include "../MathTools.h"
#include "../VirtualRobot.h"
#include "../Visualization/VisualizationFactory.h"

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT TriMeshModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static TriMeshModelPtr FromFile(const std::string& str);

        static TriMeshModel MakeBox(float a, float b, float c);
        static TriMeshModel MakePoint(float x, float y, float z);
        static TriMeshModelPtr MakePointPtr(float x, float y, float z);
        static TriMeshModelPtr MakePointPtr(const Eigen::Vector3f& p);

        /// Constructor.
        TriMeshModel();
        TriMeshModel(TriMeshModel&&) = default;
        TriMeshModel(const TriMeshModel&) = default;

        TriMeshModel& operator=(TriMeshModel&&) = default;
        TriMeshModel& operator=(const TriMeshModel&) = default;

        struct triangle
        {
            triangle() = default;

            triangle(const Eigen::Vector3f& v1,
                     const Eigen::Vector3f& v2,
                     const Eigen::Vector3f& v3) :
                vertex1(v1), vertex2(v2), vertex3(v3)
            {
            }

            triangle(const Eigen::Vector3f& v) : triangle(v, v, v)
            {
            }

            triangle(float x, float y, float z) : triangle(Eigen::Vector3f{x, y, z})
            {
            }

            Eigen::Vector3f vertex1;
            Eigen::Vector3f vertex2;
            Eigen::Vector3f vertex3;
        };

        /// Construct from vector of triangles.
        TriMeshModel(const std::vector<triangle>& triangles);

        TriMeshModel(const MathTools::ConvexHull3D& ch);
        /// Virtual destructor.
        virtual ~TriMeshModel() = default;

        template <typename T>
        Eigen::Vector3f
        nonUniformSampleSurface(T& gen) const
        {
            if (faces.empty())
            {
                return Eigen::Vector3f::Zero();
            }
            std::uniform_int_distribution<std::size_t> d{0, faces.size() - 1};
            const auto& f = faces.at(d(gen));
            const float f0 = 1 + d(gen);
            const float f1 = 1 + d(gen);
            const float f2 = 1 + d(gen);
            const Eigen::Vector3f factors = Eigen::Vector3f{f0, f1, f2}.normalized();
            return factors(0) * vertices.at(f.id1) + factors(1) * vertices.at(f.id2) +
                   factors(2) * vertices.at(f.id3);
        }

        void addTriangleWithFace(const Eigen::Vector3f& vertex1,
                                 const Eigen::Vector3f& vertex2,
                                 const Eigen::Vector3f& vertex3);
        void addTriangleWithFace(
            const Eigen::Vector3f& vertex1,
            const Eigen::Vector3f& vertex2,
            const Eigen::Vector3f& vertex3,
            Eigen::Vector3f normal,
            const VisualizationFactory::Color& color1 = VisualizationFactory::Color::Gray(),
            const VisualizationFactory::Color& color2 = VisualizationFactory::Color::Gray(),
            const VisualizationFactory::Color& color3 = VisualizationFactory::Color::Gray());
        void addTriangleWithFace(const Eigen::Vector3f& vertex1,
                                 const Eigen::Vector3f& vertex2,
                                 const Eigen::Vector3f& vertex3,
                                 const Eigen::Vector4f& vertexColor1,
                                 const Eigen::Vector4f& vertexColor2,
                                 const Eigen::Vector4f& vertexColor3);
        void addMesh(const TriMeshModel& mesh);
        static Eigen::Vector3f CreateNormal(const Eigen::Vector3f& vertex1,
                                            const Eigen::Vector3f& vertex2,
                                            const Eigen::Vector3f& vertex3);
        void addFace(const MathTools::TriangleFace& face);
        int addVertex(const Eigen::Vector3f& vertex);

        int
        addVertex(float x, float y, float z)
        {
            return addVertex({x, y, z});
        }

        unsigned int addNormal(const Eigen::Vector3f& normal);

        unsigned int
        addNormal(float x, float y, float z)
        {
            return addNormal({x, y, z});
        }

        unsigned int addColor(const VisualizationFactory::Color& color);
        unsigned int addColor(const Eigen::Vector4f& color);

        unsigned int
        addColor(float r, float g, float b, float a)
        {
            return addColor(Eigen::Vector4f{r, g, b, a});
        }

        unsigned int addMaterial(const VisualizationFactory::PhongMaterial& material);
        void addFace(unsigned int id0, unsigned int id1, unsigned int id2);
        void clear();
        /**
         * @brief Checks all faces for existence of normals. Creates the normals in case they are missing.
         * @return Number of created normals
         */
        unsigned int addMissingNormals();

        /**
         * @brief  Checks all faces for existence of colors.
         * First, mergeVertices() is called. Then, if no color is found,
         * a face with the same vertex is searched and that color is used (if available).
         * Otherwise sets color to the given color.
         * @param color Default Color
         * @return Number of created colors entries
         */
        unsigned int addMissingColors(
            const VisualizationFactory::Color& color = VisualizationFactory::Color::Gray());

        /**
         * @brief Smoothes the normal surface by calculating the average of all face-normals of one vertex.
         * Calls first mergeVertices().
         */
        void smoothNormalSurface();
        void flipVertexOrientations();
        /**
         * @brief Merges vertices that are close together (mergeThreshold).
         * Usually, vertices that are close together should be one vertex. Otherwise the mesh
         * could consist of many individual triangles.
         * All vertex ids stored in faces are updated. This function is quite efficient due to a kd-tree and an inverted face-vertex mapping.
         * @param mergeThreshold If squared Euclidan distance of two points is belong this threshold, two vertices are merged.
         * @param removeVertices If set, the vertex vextor is chekced for unused vertices. May result in a reassembled vertex vector.
         */
        void mergeVertices(float mergeThreshold = 0.0001f, bool removeVertices = true);

        /**
         * @brief fatten or shrink this trimesh. Done by moving a vertex along a normal calculated from the normals
         * of all the faces the vertex is used in.
         * @param offset All vertexes are moved about this offset in mm.
         * @param updateNormals If true, all normals will be updated with the average normal of all normals beloning to one vertex (from multiple faces)
         */
        void fattenShrink(float offset, bool updateNormals = false);

        /*!
         * \brief removeUnusedVertices Checks if vertices are used by faces. May rearrange vertices vector!
         * @return Number of removed vertices
         */
        size_t removeUnusedVertices();

        /// Get the areas of all faces.
        std::vector<float> getFaceAreas() const;

        //! Computes the volume of the mesh. Works only for closed meshes
        float getVolume() const;

        void rotate(const Eigen::Matrix3f& mx);

        // Overwrite all colors
        void setColor(VisualizationFactory::Color color);

        void print();
        void printNormals();
        void printVertices();
        void printFaces();
        Eigen::Vector3f getCOM();
        bool getSize(Eigen::Vector3f& storeMinSize, Eigen::Vector3f& storeMaxSize);
        bool checkFacesHaveSameEdge(const MathTools::TriangleFace& face1,
                                    const MathTools::TriangleFace& face2,
                                    std::vector<std::pair<int, int>>& commonVertexIds) const;
        unsigned int checkAndCorrectNormals(bool inverted);

        Eigen::Vector3f getNormalOfFace(std::size_t faceId) const;

        virtual void scale(const Eigen::Vector3f& scaleFactor);
        virtual void scale(float scaleFactor);
        TriMeshModelPtr clone() const;
        TriMeshModelPtr clone(const Eigen::Vector3f& scaleFactor) const;
        TriMeshModelPtr clone(float x, float y, float z) const;

        std::vector<Eigen::Vector3f> normals;
        std::vector<Eigen::Vector3f> vertices;
        std::vector<VisualizationFactory::Color> colors;
        std::vector<MathTools::TriangleFace> faces;
        std::vector<VisualizationFactory::PhongMaterial> materials;
        BoundingBox boundingBox;
    };
} // namespace VirtualRobot
