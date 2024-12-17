#pragma once

#include <filesystem>

#include <VirtualRobot/Visualization/TriMeshModel.h>

#include "Mesh.h"

namespace VirtualRobot::mujoco
{
    /**
     * @brief Converts from and to MuJoCo mesh formats.
     */
    class MeshConverter
    {
    public:
        // STATELESS API (static).

        // From mujoco::Mesh to VirtualRobot::TriMeshModel

        /// Construct a VirtualRobot::TriMeshModel from mujoco::Mesh.
        static VirtualRobot::TriMeshModel toVirtualRobot(const mujoco::Mesh& mesh,
                                                         float scaling = 1.0);
        static VirtualRobot::TriMeshModelPtr toVirtualRobotPtr(const mujoco::Mesh& mesh,
                                                               float scaling = 1.0);


        // To mujoco::Mesh (from VirtualRobot::TriMeshModel)

        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        static mujoco::Mesh fromVirtualRobot(const VirtualRobot::TriMeshModel& triMeshModel,
                                             float scaling = 1.0);

        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        static mujoco::Mesh toMujoco(const VirtualRobot::TriMeshModel& triMeshModel,
                                     float scaling = 1.0);


        // To STL.

        /**
         * @brief Convert a mesh file to STL using meshlabserver.
         *
         * Checks whether the command "meshlabserver" is available. If it is,
         *
         *
         * @param sourceFile The source mesh file (anything usable by meshlabserver).
         * @param targetPath
         *  The target directory or filename. If a directory, the source file
         *  name with replaced extension (.stl) is used.
         * @param skipIfExists If true and the target file exists, do nothing.
         */
        static void toSTL(const std::filesystem::path& sourceFile,
                          const std::filesystem::path& targetPath,
                          bool skipIfExists = true,
                          float scaling = 1.0f);


    private:
        /// Check whether the command `MESHLABSERVER` is available using `system("which ...")`.
        static bool checkMeshlabserverAvailable();

        /**
         * @brief Run the command converting `sourceFile` to `targetFile`.
         * @return True if the command returned without error, false otherwise.
         */
        static bool runMeshlabserverCommand(const std::filesystem::path& sourceFile,
                                            const std::filesystem::path& targetFile,
                                            float scaling = 1.0f);


        /// Command used to convert mesh files to STL.
        static const std::string MESHLABSERVER;


    private:
        /// Private constructor.
        MeshConverter();


        /* Disabled stateful API because its methods are ambiguous to the
         * static API.

    public:

        // STATEFUL API.


        /// Constructor.
        MeshConverter();


        /// Set the scaling.
        float getScaling() const;
        /// Get the scaling.
        void setScaling(float value);


        // From mujoco::Mesh to VirtualRobot::TriMeshModel

        /// Construct a VirtualRobot::TriMeshModel from mujoco::Mesh.
        VirtualRobot::TriMeshModel toVirtualRobot(const mujoco::Mesh& mesh);
        VirtualRobot::TriMeshModelPtr toVirtualRobotPtr(const mujoco::Mesh& mesh);


        // To mujoco::Mesh (from VirtualRobot::TriMeshModel)

        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        mujoco::Mesh fromVirtualRobot(const VirtualRobot::TriMeshModel& triMeshModel);

        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        mujoco::Mesh toMujoco(const VirtualRobot::TriMeshModel& triMeshModel);



    private:

        /// The scaling.
        float scaling = 1.0f;
        */
    };

} // namespace VirtualRobot::mujoco
