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

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotMujocoMeshTest

#define ARMARX_BOOST_TEST

#include <filesystem>

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/mujoco/Mesh.h>


namespace fs = std::filesystem;

// Defined by CMake.
static const fs::path meshFileName = TEST_MESH_FILE; // "VirtualRobotMujocoMeshTestMesh.msh";

struct Fixture
{
    Fixture()
    {
        BOOST_CHECK(fs::exists(meshFileName) && fs::is_regular_file(meshFileName));
        mesh.read(meshFileName);
    }

    VirtualRobot::mujoco::Mesh mesh;
};

BOOST_FIXTURE_TEST_CASE(test_consistency_after_read, Fixture)
{
    BOOST_CHECK_GE(mesh.nvertex(), 0);
    BOOST_CHECK_GE(mesh.nnormal(), 0);
    BOOST_CHECK_GE(mesh.ntexcoord(), 0);
    BOOST_CHECK_GE(mesh.nface(), 0);

    BOOST_CHECK_EQUAL(mesh.vertexPositions().size(), 3 * mesh.nvertex());
    BOOST_CHECK_EQUAL(mesh.vertexNormals().size(), 3 * mesh.nnormal());
    BOOST_CHECK_EQUAL(mesh.vertexTexcoords().size(), 2 * mesh.ntexcoord());
    BOOST_CHECK_EQUAL(mesh.faceVertexIndices().size(), 3 * mesh.nface());
}

BOOST_FIXTURE_TEST_CASE(test_write_consistency, Fixture)
{
    Eigen::Matrix<float, 3, 2> bb = Eigen::Matrix<float, 3, 2>::Zero();
    BOOST_CHECK_NO_THROW(bb = mesh.getVertexPositionsBoundingBox());

    Eigen::Vector3f min = bb.col(0);
    Eigen::Vector3f max = bb.col(1);

    BOOST_CHECK_GE(mesh.nvertex(), 0);

    for (int i = 0; i < mesh.nvertex(); ++i)
    {
        Eigen::Vector3f pos = mesh.vertexPosition(i);
        for (int i = 0; i < 3; ++i)
        {
            BOOST_CHECK_LE(min(0), pos(0));
            BOOST_CHECK_GE(max(0), pos(0));
        }
    }
}
