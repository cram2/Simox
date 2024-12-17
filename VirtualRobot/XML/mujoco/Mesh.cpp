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

#include "Mesh.h"

#include <fstream>
#include <random>
#include <stdexcept>

#include <VirtualRobot/VirtualRobotChecks.h>

namespace VirtualRobot::mujoco
{


    Mesh
    Mesh::fromFile(const std::string& filename)
    {
        Mesh mesh;
        mesh.read(filename);
        return mesh;
    }

    Mesh::Mesh()
    {
    }

    static void
    readInt32(std::istream& is, int32_t& value)
    {
        is.read(reinterpret_cast<char*>(&value), sizeof(int32_t));
    }

    static void
    writeInt32(std::ostream& os, const int32_t& value)
    {
        os.write(reinterpret_cast<const char*>(&value), sizeof(int32_t));
    }

    template <typename ValueT>
    static void
    readVector(std::istream& is, std::vector<ValueT>& vector)
    {
        is.read(reinterpret_cast<char*>(vector.data()),
                static_cast<std::streamsize>(vector.size() * sizeof(ValueT)));
    }

    template <typename ValueT>
    static void
    writeVector(std::ostream& os, const std::vector<ValueT>& vector)
    {
        os.write(reinterpret_cast<const char*>(vector.data()),
                 static_cast<std::streamsize>(vector.size() * sizeof(ValueT)));
    }

    void
    Mesh::read(const std::string& filename)
    {
        std::ifstream ifs(filename, std::ios::binary);
        if (!ifs.is_open())
        {
            throw std::runtime_error("Could not open file: " + filename);
        }
        read(ifs);
        setFilename(filename);
    }

    void
    Mesh::read(std::istream& is)
    {
        readInt32(is, _nvertex);
        readInt32(is, _nnormal);
        readInt32(is, _ntexcoord);
        readInt32(is, _nface);

        resizeVectors();

        readVector<float>(is, _vertex_positions);
        readVector<float>(is, _vertex_normals);
        readVector<float>(is, _vertex_texcoords);
        readVector<int32_t>(is, _face_vertex_indices);
    }

    void
    Mesh::write(const std::string& filename) const
    {
        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs.is_open())
        {
            throw std::runtime_error("Could not open file: " + filename);
        }
        write(ofs);
    }

    void
    Mesh::write(std::ostream& os) const
    {
        VR_CHECK_EQUAL(_nvertex * 3, static_cast<int>(_vertex_positions.size()));
        VR_CHECK_EQUAL(_nnormal * 3, static_cast<int>(_vertex_normals.size()));
        VR_CHECK_EQUAL(_ntexcoord * 2, static_cast<int>(_vertex_texcoords.size()));
        VR_CHECK_EQUAL(_nface * 3, static_cast<int>(_face_vertex_indices.size()));

        writeInt32(os, _nvertex);
        writeInt32(os, _nnormal);
        writeInt32(os, _ntexcoord);
        writeInt32(os, _nface);

        writeVector<float>(os, _vertex_positions);
        writeVector<float>(os, _vertex_normals);
        writeVector<float>(os, _vertex_texcoords);
        writeVector<int32_t>(os, _face_vertex_indices);
    }

    template <typename EigenVectorT, typename ValueT>
    EigenVectorT
    getEntry(const std::vector<ValueT>& vector, int32_t i)
    {
        VR_CHECK_NONNEGATIVE(i);
        return EigenVectorT(&vector.data()[EigenVectorT::SizeAtCompileTime * i]);
    }

    Eigen::Vector3f
    Mesh::vertexPosition(int32_t i) const
    {
        return getEntry<Eigen::Vector3f>(_vertex_positions, i);
    }

    Eigen::Vector3f
    Mesh::vertexNormal(int32_t i) const
    {
        return getEntry<Eigen::Vector3f>(_vertex_normals, i);
    }

    Eigen::Vector2f
    Mesh::vertexTexcoord(int32_t i) const
    {
        return getEntry<Eigen::Vector2f>(_vertex_texcoords, i);
    }

    Eigen::Vector3i
    Mesh::faceVertexIndex(int32_t i) const
    {
        return getEntry<Eigen::Vector3i>(_face_vertex_indices, i);
    }

    template <typename Derived>
    void
    addEntries(std::vector<typename Eigen::MatrixBase<Derived>::Scalar>& vector,
               const Eigen::MatrixBase<Derived>& values)
    {
        for (long i = 0; i < values.size(); ++i)
        {
            vector.push_back(values(i));
        }
    }

    void
    Mesh::addVertexPosition(const Eigen::Vector3f& position)
    {
        addEntries(_vertex_positions, position);
        ++_nvertex;
    }

    void
    Mesh::addVertexNormal(const Eigen::Vector3f& normal)
    {
        addEntries(_vertex_normals, normal.normalized());
        ++_nnormal;
    }

    void
    Mesh::addVertexTexcoord(const Eigen::Vector2f& texcoord)
    {
        addEntries(_vertex_texcoords, texcoord);
        ++_ntexcoord;
    }

    void
    Mesh::addFaceVertexIndex(const Eigen::Vector3i& face)
    {
        addEntries(_face_vertex_indices, face);
        ++_nface;
    }

    Eigen::Matrix<float, 3, 2>
    Mesh::getVertexPositionsBoundingBox() const
    {
        Eigen::Array3f min;
        Eigen::Array3f max;
        min.setConstant(std::numeric_limits<float>::max());
        max.setConstant(std::numeric_limits<float>::min());

        VR_CHECK_NONNEGATIVE(_nvertex);

        for (int i = 0; i < _nvertex; ++i)
        {
            Eigen::Vector3f pos = vertexPosition(i);
            min = min.min(pos.array());
            max = max.max(pos.array());
        }

        Eigen::Matrix<float, 3, 2> bb;
        bb.col(0) = min.matrix();
        bb.col(1) = max.matrix();
        return bb;
    }

    void
    Mesh::setTextureFile(const std::string& path)
    {
        _texture_file = path;
    }

    std::string
    Mesh::getTextureFile() const
    {
        return _texture_file;
    }

    void
    Mesh::setFilename(const std::string& path)
    {
        _filename = path;
    }

    std::string
    Mesh::getFilename() const
    {
        return _filename;
    }

    std::vector<Mesh>
    Mesh::splitRandomly(std::size_t num) const
    {
        std::vector<Mesh> meshes(num);

        std::default_random_engine gen(std::random_device{}());
        std::uniform_int_distribution<std::size_t> distrib(0, num - 1);

        for (int faceIndex = 0; faceIndex < nface(); ++faceIndex)
        {
            // select mesh receiving the face
            std::size_t meshIndex = distrib(gen);
            Mesh& mesh = meshes[meshIndex];

            const Eigen::Vector3i faceVertexIndexIn = faceVertexIndex(faceIndex);

            // copy the face
            Eigen::Vector3i faceVertexIndexOut = faceVertexIndex(faceIndex);
            for (int i = 0; i < faceVertexIndexIn.size(); ++i)
            {
                int vertexIndex = faceVertexIndexIn(i);

                // store index of new vertex
                faceVertexIndexOut(i) = mesh.nvertex();

                // add vertex
                mesh.addVertexPosition(vertexPosition(vertexIndex));

                if (hasNormals())
                {
                    mesh.addVertexNormal(vertexPosition(vertexIndex));
                }
                if (hasTexcoords())
                {
                    mesh.addVertexTexcoord(vertexTexcoord(vertexIndex));
                }
            }
            // add face vertex index
            mesh.addFaceVertexIndex(faceVertexIndexOut);
        }

        return meshes;
    }

    void
    Mesh::resizeVectors()
    {
        VR_CHECK_NONNEGATIVE(_nvertex);
        VR_CHECK_NONNEGATIVE(_nnormal);
        VR_CHECK_NONNEGATIVE(_ntexcoord);
        VR_CHECK_NONNEGATIVE(_nface);

        _vertex_positions.resize(static_cast<std::size_t>(3 * _nvertex));
        _vertex_normals.resize(static_cast<std::size_t>(3 * _nnormal));
        _vertex_texcoords.resize(static_cast<std::size_t>(2 * _ntexcoord));
        _face_vertex_indices.resize(static_cast<std::size_t>(3 * _nface));
    }


} // namespace VirtualRobot::mujoco
