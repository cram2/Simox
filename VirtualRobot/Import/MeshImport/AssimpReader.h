#pragma once

#include <stdio.h>

#include <cfloat>
#include <string>

#include <VirtualRobot/VirtualRobot.h>

struct aiScene;
class SoNode;

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT AssimpReader
    {
    public:
        AssimpReader(float eps = FLT_MIN, float scaling = 1);
        virtual ~AssimpReader() = default;

        static std::string
        get_description()
        {
            return "Open Asset Import Library";
        }

        static std::string get_extensions();
        static bool can_load(const std::string& file);

        // read data and store it to trimesh
        bool readFileAsTriMesh(const std::string& _filename, const TriMeshModelPtr& t);
        bool readBufferAsTriMesh(const std::string_view& v, const TriMeshModelPtr& t);

        TriMeshModelPtr readFileAsTriMesh(const std::string& filename);
        TriMeshModelPtr readBufferAsTriMesh(const std::string_view& v);
        static SoNode* readFileAsSoNode(const std::string& filename);

        ManipulationObjectPtr readFileAsManipulationObject(const std::string& filename,
                                                           const std::string& name = "");
        ManipulationObjectPtr readBufferAsManipulationObject(const std::string_view& v,
                                                             const std::string& name = "");

        struct Parameters
        {
            float scaling = 1;
            /// Returns the threshold to be used for considering two point to be equal.
            float eps = FLT_MIN;
            bool mergeMultipleMeshes = false;
            bool ignoreMissingNormals = false;
            bool skipInvalidFaces = false;
            bool verbose = true;
        };

        Parameters parameters;

        struct ResultMetaData
        {
            std::size_t skippedFaces = 0;
            bool loadingSuccessful = false;
            bool regeneratedNormals = false;
        };

        ResultMetaData resultMetaData;
    };

    typedef std::shared_ptr<AssimpReader> AssimpReaderPtr;
} // namespace VirtualRobot
