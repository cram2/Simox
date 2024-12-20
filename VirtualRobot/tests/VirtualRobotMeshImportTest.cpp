/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2014 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotMeshImportTest

#include <sstream>
#include <string>

#include <VirtualRobot/Import/MeshImport/AssimpReader.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>

using namespace VirtualRobot;
using namespace std;

BOOST_AUTO_TEST_SUITE(VirtualRobotMeshImport)

BOOST_AUTO_TEST_CASE(testParseSTL)
{
    const char* stlcontent = "solid block100\n"
                             "facet normal -1.000000e+000 0.000000e+000 0.000000e+000\n"
                             "outer loop\n"
                             "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
                             "vertex 0.000000e+000 1.000000e+002 0.000000e+000\n"
                             "vertex 0.000000e+000 0.000000e+000 1.000000e+002\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal -1.000000e+000 0.000000e+000 0.000000e+000\n"
                             "outer loop\n"
                             "vertex 0.000000e+000 0.000000e+000 1.000000e+002\n"
                             "vertex 0.000000e+000 1.000000e+002 0.000000e+000\n"
                             "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 0.000000e+000 0.000000e+000 1.000000e+000\n"
                             "outer loop\n"
                             "vertex 1.000000e+002 1.000000e+002 1.000000e+002\n"
                             "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
                             "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 0.000000e+000 0.000000e+000 1.000000e+000\n"
                             "outer loop\n"
                             "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
                             "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
                             "vertex 0.000000e+000 0.000000e+000 1.000000e+002\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 1.000000e+000 0.000000e+000 0.000000e+000\n"
                             "outer loop\n"
                             "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
                             "vertex 1.000000e+002 1.000000e+002 1.000000e+002\n"
                             "vertex 1.000000e+002 0.000000e+000 0.000000e+000\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 1.000000e+000 0.000000e+000 0.000000e+000\n"
                             "outer loop\n"
                             "vertex 1.000000e+002 0.000000e+000 0.000000e+000\n"
                             "vertex 1.000000e+002 1.000000e+002 1.000000e+002\n"
                             "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 0.000000e+000 0.000000e+000 -1.000000e+000\n"
                             "outer loop\n"
                             "vertex 0.000000e+000 1.000000e+002 0.000000e+000\n"
                             "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
                             "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 0.000000e+000 0.000000e+000 -1.000000e+000\n"
                             "outer loop\n"
                             "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
                             "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
                             "vertex 1.000000e+002 0.000000e+000 0.000000e+000\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 0.000000e+000 1.000000e+000 0.000000e+000\n"
                             "outer loop\n"
                             "vertex 1.000000e+002 1.000000e+002 1.000000e+002\n"
                             "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
                             "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 0.000000e+000 1.000000e+000 0.000000e+000\n"
                             "outer loop\n"
                             "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
                             "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
                             "vertex 0.000000e+000 1.000000e+002 0.000000e+000\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 0.000000e+000 -1.000000e+000 0.000000e+000\n"
                             "outer loop\n"
                             "vertex 1.000000e+002 0.000000e+000 0.000000e+000\n"
                             "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
                             "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
                             "endloop\n"
                             "endfacet\n"
                             "facet normal 0.000000e+000 -1.000000e+000 0.000000e+000\n"
                             "outer loop\n"
                             "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
                             "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
                             "vertex 0.000000e+000 0.000000e+000 1.000000e+002\n"
                             "endloop\n"
                             "endfacet\n"
                             "endsolid\n";

    TriMeshModelPtr t2(new TriMeshModel());
    {
        bool readSTLok = AssimpReader{}.readBufferAsTriMesh(stlcontent, t2);
        BOOST_REQUIRE(readSTLok);
        BOOST_REQUIRE(t2);
        BOOST_CHECK_EQUAL(t2->vertices.size(),
                          8); // identical vertices are mapped to one vertex instance
        BOOST_CHECK_EQUAL(t2->faces.size(), 12);
    }
}

BOOST_AUTO_TEST_CASE(testLoadSTL)
{

    std::string filename = "objects/stl/Piggy6.stl";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    TriMeshModelPtr t2(new TriMeshModel());
    {
        bool readSTLok = AssimpReader{}.readFileAsTriMesh(filename, t2);
        BOOST_REQUIRE(readSTLok);
        BOOST_REQUIRE(t2);
        BOOST_CHECK_GT(int(t2->vertices.size()), 20);
        BOOST_CHECK_GT(int(t2->faces.size()), 20);
    }
}


BOOST_AUTO_TEST_SUITE_END()
