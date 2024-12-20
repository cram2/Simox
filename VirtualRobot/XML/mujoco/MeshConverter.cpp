#include "MeshConverter.h"

#include <fstream>

#include <VirtualRobot/VirtualRobotChecks.h>

#include "Logging.h"

namespace fs = std::filesystem;

namespace VirtualRobot::mujoco
{

    const std::string MeshConverter::MESHLABSERVER = "meshlabserver";

    VirtualRobot::TriMeshModelPtr
    MeshConverter::toVirtualRobotPtr(const Mesh& mesh, float scaling)
    {
        return VirtualRobot::TriMeshModelPtr(
            new VirtualRobot::TriMeshModel(toVirtualRobot(mesh, scaling)));
    }

    VirtualRobot::TriMeshModel
    MeshConverter::toVirtualRobot(const Mesh& mj, float scaling)
    {
        VirtualRobot::TriMeshModel vr;

        // copy vertices
        for (int i = 0; i < mj.nvertex(); ++i)
        {
            vr.addVertex(mj.vertexPosition(i) * scaling);
        }

        // copy normals
        for (int i = 0; i < mj.nnormal(); ++i)
        {
            vr.addNormal(mj.vertexNormal(i));
        }

        // copy faces
        for (int i = 0; i < mj.nface(); ++i)
        {
            Eigen::Vector3i faceVertexIndex = mj.faceVertexIndex(i);

            // check before casting
            VR_CHECK_NONNEGATIVE(faceVertexIndex.minCoeff());
            VR_CHECK_LESS(faceVertexIndex.maxCoeff(), static_cast<int>(vr.vertices.size()));

            Eigen::Matrix<unsigned int, 3, 1> vertexIDs = faceVertexIndex.cast<unsigned int>();

            VirtualRobot::MathTools::TriangleFace triFace;

            // set position indices
            triFace.set(vertexIDs(0), vertexIDs(1), vertexIDs(2));

            // set normals (if present)
            if (vertexIDs.maxCoeff() < vr.normals.size())
            {
                triFace.setNormal(vertexIDs(0), vertexIDs(1), vertexIDs(2));

                Eigen::Vector3f normal = Eigen::Vector3f::Zero();
                for (int i = 0; i < vertexIDs.size(); ++i)
                {
                    normal += vr.normals[vertexIDs(i)];
                }
                triFace.normal = normal / vertexIDs.size();
            }
            else
            {
                // create a normal from the vertices
                triFace.normal = vr.CreateNormal(
                    vr.vertices[triFace.id1], vr.vertices[triFace.id2], vr.vertices[triFace.id3]);
            }

            vr.addFace(triFace);
        }

        return vr;
    }

    template <typename ValueT>
    static ValueT
    max3(ValueT a, ValueT b, ValueT c)
    {
        return std::max(a, std::max(b, c));
    }

    Mesh
    MeshConverter::fromVirtualRobot(const VirtualRobot::TriMeshModel& vr, float scaling)
    {
        Mesh mj;

        const auto& vertices = vr.vertices;
        const auto& normals = vr.normals;

        for (const auto& face : vr.faces)
        {
            VR_CHECK_EQUAL(mj.nvertex(), mj.nnormal());

            int index = static_cast<int>(mj.nvertex());

            VR_CHECK_LESS_EQUAL(max3(face.id1, face.id2, face.id3), vertices.size());
            mj.addVertexPosition(vertices.at(face.id1) * scaling);
            mj.addVertexPosition(vertices.at(face.id2) * scaling);
            mj.addVertexPosition(vertices.at(face.id3) * scaling);

            if (max3(face.idNormal1, face.idNormal2, face.idNormal3) < normals.size())
            {
                mj.addVertexNormal(normals.at(face.idNormal1));
                mj.addVertexNormal(normals.at(face.idNormal2));
                mj.addVertexNormal(normals.at(face.idNormal3));
            }
            else
            {
                // take anything in face.normal, normalize it and use it
                Eigen::Vector3f normal = face.normal;
                if (normal.isZero(1e-6f))
                {
                    VR_WARNING << "No normal information in TriMeshModel";
                    normal = Eigen::Vector3f::UnitX();
                }
                normal.normalize();

                // add it 3 times (once for each vertex position)
                int i = 3;
                while (i-- > 0) // use convergence operator
                {
                    mj.addVertexNormal(normal);
                }
            }

            mj.addFaceVertexIndex({index, index + 1, index + 2});
        }

        return mj;
    }

    Mesh
    MeshConverter::toMujoco(const VirtualRobot::TriMeshModel& triMeshModel, float scaling)
    {
        return fromVirtualRobot(triMeshModel, scaling);
    }

    void
    MeshConverter::toSTL(const std::filesystem::path& sourceFile,
                         const std::filesystem::path& _targetPath,
                         bool skipIfExists,
                         float scaling)
    {
        fs::path targetFile = _targetPath;

        // Add a file name if none was passed.
        if (!targetFile.has_filename())
        {
            fs::path filename = sourceFile.filename();
            filename.replace_extension("stl");
            targetFile = targetFile / filename;
        }

        // Make sure parent directory exists.
        if (!fs::exists(targetFile.parent_path()))
        {
            fs::create_directories(targetFile.parent_path());
        }

        // Check if file already exists.
        if (skipIfExists && fs::exists(targetFile))
        {
            VR_INFO << "Skipping (" << targetFile << " already exists).";
            return;
        }

        // Check if file has to be converted.
        if (sourceFile.extension() == ".stl")
        {
            VR_INFO << "Copying: " << sourceFile << std::endl << "     to: " << targetFile;
            fs::copy_file(sourceFile, targetFile);
            return;
        }


        VR_INFO << "Converting to .stl: " << sourceFile << std::endl
                << "(to: " << targetFile << ")" << std::endl;

        const bool meshlabserverAvailable = checkMeshlabserverAvailable();
        bool notAvailableReported = false;

        if (!meshlabserverAvailable)
        {
            if (!notAvailableReported)
            {
                VR_ERROR << "Command '" << MESHLABSERVER
                         << "' not available, cannot convert meshes."
                         // << " (This error is reported only once.)"
                         << std::endl;
                notAvailableReported = true;
            }

            return;
        }

        // Meshlabserver available.
        runMeshlabserverCommand(sourceFile, targetFile, scaling);
    }

    bool
    MeshConverter::checkMeshlabserverAvailable()
    {
        std::stringstream ss;
        ss << "which " << MESHLABSERVER << " > /dev/null 2>&1";
        return system(ss.str().c_str()) == 0;
    }

    fs::path
    createScript(float scaling)
    {
        fs::path path = fs::temp_directory_path() /
                        ("meshlabserver_scaling_" + std::to_string(scaling) + ".mlx");
        // if (!fs::exists(path))
        {
            std::stringstream filterScript;
            filterScript << "<!DOCTYPE FilterScript>";
            filterScript << "<FilterScript>";
            filterScript << "<filter name=\"Transform: Scale\">";
            filterScript << "<Param type=\"RichDynamicFloat\" value=\"" << scaling
                         << "\" min=\"0.1\" name=\"axisX\" max=\"10\"/>";
            filterScript << "<Param type=\"RichDynamicFloat\" value=\"" << scaling
                         << "\" min=\"0.1\" name=\"axisY\" max=\"10\"/>";
            filterScript << "<Param type=\"RichDynamicFloat\"value=\"" << scaling
                         << "\" min=\"0.1\" name=\"axisZ\" max=\"10\"/>";
            filterScript << "<Param type=\"RichBool\" value=\"true\" name=\"uniformFlag\"/>";
            filterScript << "<Param enum_val0=\"origin\" enum_val1=\"barycenter\" "
                            "enum_cardinality=\"3\" enum_val2=\"custom point\" type=\"RichEnum\" "
                            "value=\"0\" name=\"scaleCenter\"/>";
            filterScript
                << "<Param x=\"0\" y=\"0\" z=\"0\" type=\"RichPoint3f\" name=\"customCenter\"/>";
            filterScript << "<Param type=\"RichBool\" value=\"false\" name=\"unitFlag\"/>";
            filterScript << "<Param type=\"RichBool\" value=\"true\" name=\"Freeze\"/>";
            filterScript << "<Param type=\"RichBool\" value=\"false\" name=\"ToAll\"/>";
            filterScript << "</filter>";
            filterScript << "</FilterScript>";
            std::ofstream o(path);
            o << filterScript.str();
            o.close();
        }
        return path;
    }

    bool
    MeshConverter::runMeshlabserverCommand(const std::filesystem::path& sourceFile,
                                           const std::filesystem::path& targetFile,
                                           float scaling)
    {
        std::stringstream convertCommand;
        convertCommand << MESHLABSERVER << " -i " << sourceFile << " -o " << targetFile;

        if (scaling != 1.0f)
        {
            convertCommand << " -s " << createScript(scaling);
        }

        // run command
        VR_INFO << "----------------------------------------------------------" << std::endl;
        VR_INFO << "Running command: " << convertCommand.str() << std::endl;
        const int r = system(convertCommand.str().c_str());
        VR_INFO << "----------------------------------------------------------" << std::endl;

        if (r == 0)
        {
            // Success.
            return true;
        }
        else
        {
            VR_INFO << "Command returned with error: " << r << "\n"
                    << "Command was: " << convertCommand.str() << std::endl;
            return false;
        }
    }

    MeshConverter::MeshConverter() = default;

    /*
float MeshConverter::getScaling() const
{
    return scaling;
}

void MeshConverter::setScaling(float value)
{
    this->scaling = value;
}

TriMeshModel MeshConverter::toVirtualRobot(const Mesh& mesh)
{
    return toVirtualRobot(mesh, scaling);
}

TriMeshModelPtr MeshConverter::toVirtualRobotPtr(const Mesh& mesh)
{
    return toVirtualRobotPtr(mesh, scaling);
}

Mesh MeshConverter::fromVirtualRobot(const TriMeshModel& triMeshModel)
{
    return fromVirtualRobot(triMeshModel, scaling);
}

Mesh MeshConverter::toMujoco(const TriMeshModel& triMeshModel)
{
    return toMujoco(triMeshModel, scaling);
}
*/

} // namespace VirtualRobot::mujoco
