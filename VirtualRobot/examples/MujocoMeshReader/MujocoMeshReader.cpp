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
 * @package    MujocoX::ArmarXObjects::MujocoMJCF
 * @author     Rainer Kartmann ( rainer dot kartmann at student dot kit dot edu )
 * @date       2019
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>

#include <VirtualRobot/XML/mujoco/Mesh.h>


namespace fs = std::filesystem;
namespace po = boost::program_options;

using namespace VirtualRobot;


static po::variables_map parseOptions(int argc, char** argv);

static std::string
printN(int32_t n, int32_t mult)
{
    std::stringstream ss;
    ss << n << " (x" << mult << " = " << mult * n << ")";
    return ss.str();
}

template <int stepSize = 3, typename ValueT>
static void printVector(const std::vector<ValueT>& vector, const std::string& name);

static void printFaces(const mujoco::Mesh& mesh);

int
main(int argc, char** argv)
{
    po::variables_map opt = parseOptions(argc, argv);
    fs::path file = opt["file"].as<std::string>();

    mujoco::Mesh mesh;
    try
    {
        mesh.read(file.string());
    }
    catch (const std::runtime_error& e)
    {
        std::cout << e.what() << std::endl;
        return 2;
    }

    std::cout << "File: " << file << std::endl;

    std::cout << "\n- nvertex:   " << printN(mesh.nvertex(), 3)
              << "\n- nnormal:   " << printN(mesh.nnormal(), 3)
              << "\n- ntexcoord: " << printN(mesh.ntexcoord(), 2)
              << "\n- nface:     " << printN(mesh.nface(), 3) << std::endl;

    std::cout << "\n- #Vertex Positions:    " << mesh.vertexPositions().size()
              << "\n- #Vertex Normals:      " << mesh.vertexNormals().size()
              << "\n- #Vertex Texcoords:    " << mesh.vertexTexcoords().size()
              << "\n- #Face vertex indices: " << mesh.faceVertexIndices().size() << std::endl;

    if (opt.count("pos") || opt.count("all"))
    {
        printVector<3>(mesh.vertexPositions(), "Vertex positions");
    }
    if (opt.count("normal") || opt.count("all"))
    {
        printVector<3>(mesh.vertexNormals(), "Vertex normals");
    }
    if (opt.count("tex") || opt.count("all"))
    {
        printVector<2>(mesh.vertexTexcoords(), "Vertex texcoords");
    }
    if (opt.count("indices") || opt.count("all"))
    {
        printVector<3>(mesh.faceVertexIndices(), "Face vertex indices");
    }
    if (opt.count("faces"))
    {
        printFaces(mesh);
    }

    return 0;
}

po::variables_map
parseOptions(int argc, char** argv)
{
    po::positional_options_description posOpts;
    posOpts.add("file", -1);

    po::options_description opts("Allowed options");
    opts.add_options()("help,h", "Show this help message")(
        "file", po::value<std::string>(), "the mesh file to read")(
        "pos,p", "Print vertex positions")("normal,n", "Print vertex normals")(
        "tex,t", "Print vertex texcoords")("indices,i", "Print face vertex indices")(
        "all,a", "pos, normal, tex and indices")("faces,f",
                                                 "Print faces with vertex positions and normals");

    bool doPrintHelp = false;
    po::variables_map vm;

    try
    {
        po::store(po::command_line_parser(argc, argv).options(opts).positional(posOpts).run(), vm);
        po::notify(vm);
    }
    catch (const po::multiple_occurrences& e)
    {
        doPrintHelp = true;
        std::cout << e.what() << std::endl;
    }

    if (doPrintHelp || vm.count("help") || !vm.count("file"))
    {
        std::cout << "Usage: " << argv[0] << " <file>" << std::endl;
        std::cout << opts << std::endl;
        std::exit(1);
    }

    return vm;
}

template <int stepSize, typename ValueT>
void
printVector(const std::vector<ValueT>& vector, const std::string& name)
{
    static const Eigen::IOFormat iof(4, 0, "\t", "\t", "", "", "[ ", " ]");

    std::cout << "\n" << name << ": " << std::endl;

    if (vector.empty())
    {
        std::cout << "(none)" << std::endl;
        return;
    }

    std::cout.precision(4);

    int width = static_cast<int>(std::ceil(std::log10(vector.size())));
    auto setw = std::setw(width);

    for (std::size_t i = 0; i < vector.size(); i += stepSize)
    {
        Eigen::Matrix<ValueT, stepSize, 1> pos(vector.data() + i);
        std::cout << std::right << "(" << setw << i << ") " << pos.format(iof) << std::endl;
    }
}

template <typename Derived>
std::string
printVector(const Eigen::MatrixBase<Derived>& matrix)
{
    std::stringstream os;
    os << "[ ";
    for (int i = 0; i < matrix.size(); ++i)
    {
        if (matrix(i) >= 0)
        {
            os << " ";
        }
        os << std::right << matrix(i) << " ";
    }
    os << "]";
    return os.str();
}

int
width(int size)
{
    return static_cast<int>(std::ceil(std::log10(size)));
}

void
printFaces(const mujoco::Mesh& mesh)
{
    static const Eigen::IOFormat iof(4, 0, "\t", "\t", "", "", "[ ", " ]");

    std::cout << "\nFaces: " << std::endl;

    if (mesh.faceVertexIndices().empty())
    {
        std::cout << "(none)" << std::endl;
        return;
    }

    std::cout.precision(4);

    auto setwV = std::setw(width(mesh.nvertex()));
    auto setwF = std::setw(width(mesh.nface()));

    for (int fi = 0; fi < mesh.nface(); ++fi)
    {
        std::cout << "Face " << std::right << setwF << fi << ": \n";

        Eigen::Vector3i vertexIndex = mesh.faceVertexIndex(fi);

        for (int i = 0; i < vertexIndex.size(); ++i)
        {
            int index = vertexIndex(i);

            std::cout << "- vertex (" << std::right << setwV << index << "):";

            std::cout << "  -  "
                      << "pos " << std::left << std::setw(31)
                      << printVector(mesh.vertexPosition(index)) << "  -  "
                      << "nrm " << printVector(mesh.vertexNormal(index));

            std::cout << std::endl;
        }
    }
}
