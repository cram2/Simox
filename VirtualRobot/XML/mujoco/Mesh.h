/*
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    MujocoX::ArmarXObjects::Mesh
 * @author     Rainer Kartmann ( rainer dot kartmann at student dot kit dot edu )
 * @date       2019
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

namespace VirtualRobot::mujoco
{
    class Mesh
    {
    public:
        static Mesh fromFile(const std::string& filename);

    public:
        /// Constructor.
        Mesh();

        /// Read from a file. Sets the filename.
        void read(const std::string& filename);
        /// Read from an in stream.
        void read(std::istream& is);

        /// Write to a file.
        void write(const std::string& filename) const;
        /// Write to an out stream.
        void write(std::ostream& os) const;

        /// Get the number of vertices.
        int32_t
        nvertex() const
        {
            return _nvertex;
        }

        /// Get the number of normals.
        int32_t
        nnormal() const
        {
            return _nnormal;
        }

        /// Get the number of texture coordinates.
        int32_t
        ntexcoord() const
        {
            return _ntexcoord;
        }

        /// Get the number of faces.
        int32_t
        nface() const
        {
            return _nface;
        }

        /// Get the vertex positions.
        std::vector<float>&
        vertexPositions()
        {
            return _vertex_positions;
        }

        const std::vector<float>&
        vertexPositions() const
        {
            return _vertex_positions;
        }

        /// Get the vertex normals.
        std::vector<float>&
        vertexNormals()
        {
            return _vertex_normals;
        }

        const std::vector<float>&
        vertexNormals() const
        {
            return _vertex_normals;
        }

        /// Get the vertex texture coordinates.
        std::vector<float>&
        vertexTexcoords()
        {
            return _vertex_texcoords;
        }

        const std::vector<float>&
        vertexTexcoords() const
        {
            return _vertex_texcoords;
        }

        /// Get the face vertex indices.
        std::vector<int32_t>&
        faceVertexIndices()
        {
            return _face_vertex_indices;
        }

        const std::vector<int32_t>&
        faceVertexIndices() const
        {
            return _face_vertex_indices;
        }

        /// Get the position of vertex i.
        Eigen::Vector3f vertexPosition(int32_t i) const;
        /// Get the normal of vertex i.
        Eigen::Vector3f vertexNormal(int32_t i) const;
        /// Get the texture coordinates of vertex i.
        Eigen::Vector2f vertexTexcoord(int32_t i) const;
        /// Get the vertex indices of face i.
        Eigen::Vector3i faceVertexIndex(int32_t i) const;

        /// Indicate whether this mesh has vertex normals.
        bool
        hasNormals() const
        {
            return _nnormal > 0;
        }

        /// Indicate whether this mesh has vertex texture coordinates.
        bool
        hasTexcoords() const
        {
            return _ntexcoord > 0;
        }

        /// Indicate whether this mesh has faces.
        bool
        hasFaces() const
        {
            return _nface > 0;
        }

        /// Add a vertex position.
        void addVertexPosition(const Eigen::Vector3f& position);
        /// Add a vertex normal.
        void addVertexNormal(const Eigen::Vector3f& normal);
        /// Add vertex texture coordinates.
        void addVertexTexcoord(const Eigen::Vector2f& texcoord);
        /// Add a face as vertex indices.
        void addFaceVertexIndex(const Eigen::Vector3i& face);


        /**
         * @brief Get the bounding box of vertex positions.
         * The bounding box is returned as a 3x2 matrix:
         *    min   max
         * ( x_min x_max )  < x limits
         * ( y_min y_max )  < y limits
         * ( z_min z_max )  < z limits
         */
        Eigen::Matrix<float, 3, 2> getVertexPositionsBoundingBox() const;


        /// Get a texture file.
        std::string getTextureFile() const;
        /// Set the texture file.
        void setTextureFile(const std::string& path);

        /// Get the filename.
        std::string getFilename() const;
        /// Set a filename.
        void setFilename(const std::string& path);


        /**
         * @brief Split this mesh into `num` meshes by randomly assigning its
         * faces to the new meshes.
         */
        std::vector<Mesh> splitRandomly(std::size_t num) const;


    private:
        /// Resize the vectors to the according number of elements.
        /// _n* values must be nonnegative.
        void resizeVectors();

        /// Number of vertices.
        int32_t _nvertex = 0;
        /// Number of normals.
        int32_t _nnormal = 0;
        /// Number of texture coordinates.
        int32_t _ntexcoord = 0;
        /// Number of faces.
        int32_t _nface = 0;

        /// Vertex positions. Must have size `3 * _nvertex`.
        std::vector<float> _vertex_positions; // 3*nvertex
        /// Vertex normals. Must have size `3 * _nnormal`.
        std::vector<float> _vertex_normals; // 3*nnormal
        /// Vertex texture coordinates. Must have size `3 * _ntexcoord`.
        std::vector<float> _vertex_texcoords; // 2*ntexcoord
        /// Face vertex indices. Must have size `3 * _nface`.
        std::vector<int32_t> _face_vertex_indices; // 3*nface

        /// The stored texture file.
        std::string _texture_file;
        /// The stored file name.
        std::string _filename;
    };
} // namespace VirtualRobot::mujoco
