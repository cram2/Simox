#include "AssimpReader.h"

#include <filesystem>

#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>

#include <Inventor/SbImage.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoNormalBinding.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTexture2.h>
#include <Inventor/nodes/SoTextureCoordinate2.h>
#include <assimp/Importer.hpp>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace
{
    bool
    readScene(const aiScene* scene,
              const VirtualRobot::TriMeshModelPtr& t,
              const std::string& filename,
              const VirtualRobot::AssimpReader::Parameters& param,
              VirtualRobot::AssimpReader::ResultMetaData& meta)
    {
        meta.loadingSuccessful = false;
        meta.regeneratedNormals = false;
        meta.skippedFaces = 0;
        if (!t)
        {
            VR_ERROR << "Tri mesh is null\n";
            return false;
        }
        if (!scene)
        {
            VR_ERROR << "unable to load the file '" << filename << "'\n";
            return false;
        }
        if (!scene->mRootNode)
        {
            VR_ERROR << "mesh from '" << filename << "' has no root node\n";
            return false;
        }
        if (scene->mNumMeshes == 0)
        {
            VR_ERROR << "mesh from '" << filename << "' has no mesh\n";
            return false;
        }
        if (scene->mNumMeshes > 1 && !param.mergeMultipleMeshes)
        {
            VR_ERROR << "mesh from '" << filename << "' has not exactly one mesh"
                     << " It has " << scene->mNumMeshes << " meshes\n";
            return false;
        }
        if (!scene->mMeshes[0])
        {
            VR_ERROR << "mesh from '" << filename << "' is null\n";
            return false;
        }

        t->clear();
        for (std::size_t idxMesh = 0; idxMesh < scene->mNumMeshes; ++idxMesh)
        {
#define error_log                                                                                  \
    VR_ERROR << "mesh[" << idxMesh + 1 << " / " << scene->mNumMeshes << "] from '" << filename     \
             << "'"

            const long vertexIdxOffset = t->vertices.size();
            const aiMesh& m = *(scene->mMeshes[idxMesh]);
            if (!(m.mVertices && m.mNumVertices))
            {
                error_log << " has no vertices\n";
                return false;
            }
            if (!(m.mFaces && m.mNumFaces))
            {
                error_log << " has no vertices\n";
                return false;
            }
            if (!m.mNormals)
            {
                if (!param.ignoreMissingNormals)
                {
                    error_log << " has no normals (and none were generated when loading it)\n";
                    return false;
                }
                if (param.verbose)
                {
                    error_log << " has no normals (and none were generated when loading it) "
                                 "(ignoring this)\n";
                }
                meta.regeneratedNormals = true;
            }

            for (unsigned i = 0; i < m.mNumVertices; ++i)
            {
                const auto& v = m.mVertices[i];
                t->addVertex({v.x, v.y, v.z});
                if (m.mNormals)
                {
                    const auto& n = m.mNormals[i];
                    t->addNormal(Eigen::Vector3f{n.x, n.y, n.z}.normalized());
                }
            }
            for (unsigned i = 0; i < m.mNumFaces; ++i)
            {
                const auto& f = m.mFaces[i];
                if (f.mNumIndices != 3)
                {
                    if (param.verbose || !param.skipInvalidFaces)
                        error_log << " has face (# " << i << ") with the wrong number of vertices ("
                                  << f.mNumIndices << ")\n";
                    if (param.skipInvalidFaces)
                    {
                        ++meta.skippedFaces;
                        continue;
                    }

                    return false;
                }
                if (f.mIndices[0] >= m.mNumVertices || f.mIndices[1] >= m.mNumVertices ||
                    f.mIndices[2] >= m.mNumVertices)
                {
                    error_log << " has vertex index out of bounds for face # " << i << " \n";
                    return false;
                }
                VirtualRobot::MathTools::TriangleFace fc;
                fc.id1 = vertexIdxOffset + f.mIndices[0];
                fc.id2 = vertexIdxOffset + f.mIndices[1];
                fc.id3 = vertexIdxOffset + f.mIndices[2];
                fc.idNormal1 = vertexIdxOffset + f.mIndices[0];
                fc.idNormal2 = vertexIdxOffset + f.mIndices[1];
                fc.idNormal3 = vertexIdxOffset + f.mIndices[2];
                t->addFace(fc);
            }
#undef error_log
        }

        t->scale(param.scaling);
        t->mergeVertices(param.eps);
        t->addMissingNormals();
        meta.loadingSuccessful = true;
        return true;
    }
} // namespace

namespace VirtualRobot
{
    AssimpReader::AssimpReader(float eps, float scaling)
    {
        parameters.scaling = scaling;
        parameters.eps = eps;
    }

    std::string
    AssimpReader::get_extensions()
    {
        static const auto extensions = []
        {
            const std::string upper = "3D 3DS 3MF AC AC3D ACC AMJ ASE ASK B3D BLEND BVH CMS COB "
                                      "DAE DXF ENFF FBX LWO LWS LXO MD2 MD3 MD5 MDC MDL MESH MOT "
                                      "MS3D NDO NFF OBJ OFF OGEX PLY PMX PRJ Q3O Q3S RAW SCN SIB "
                                      "SMD STP STL STLA STLB TER UC VTA X X3D XGL ZGL";
            std::string both = upper + upper;
            std::transform(both.begin(),
                           both.begin() + upper.size(),
                           both.begin(),
                           [](unsigned char c) { return std::tolower(c); });
            return both;
        }();
        return extensions;
    }

    bool
    AssimpReader::can_load(const std::string& file)
    {
        std::filesystem::path p{file};
        if (!p.has_extension())
        {
            return false;
        }
        std::string ext = p.extension().string().substr(1);
        for (auto& c : ext)
        {
            c = std::toupper(c);
        }
        if (ext.empty())
        {
            return false;
        }
        return get_extensions().find(ext) != std::string::npos;
    }

    bool
    AssimpReader::readFileAsTriMesh(const std::string& filename, const TriMeshModelPtr& t)
    {
        //code adapted from http://assimp.sourceforge.net/lib_html/usage.html
        Assimp::Importer importer;
        // And have it read the given file with some example postprocessing
        // Usually - if speed is not the most important aspect for you - you'll
        // propably to request more postprocessing than we do in this example.
        const aiScene* scene =
            importer.ReadFile(filename,
                              aiProcess_JoinIdenticalVertices | aiProcess_Triangulate |
                                  aiProcess_GenSmoothNormals | aiProcess_SortByPType);
        return readScene(scene, t, filename, parameters, resultMetaData);
    }

    bool
    AssimpReader::readBufferAsTriMesh(const std::string_view& v, const TriMeshModelPtr& t)
    {
        //code adapted from http://assimp.sourceforge.net/lib_html/usage.html
        Assimp::Importer importer;
        // And have it read the given file with some example postprocessing
        // Usually - if speed is not the most important aspect for you - you'll
        // propably to request more postprocessing than we do in this example.
        const aiScene* scene =
            importer.ReadFileFromMemory(v.data(),
                                        v.size(),
                                        aiProcess_JoinIdenticalVertices | aiProcess_Triangulate |
                                            aiProcess_GenSmoothNormals | aiProcess_SortByPType);
        return readScene(scene, t, "<MEMORY_BUFFER>", parameters, resultMetaData);
    }

    TriMeshModelPtr
    AssimpReader::readFileAsTriMesh(const std::string& filename)
    {
        auto tri = std::make_shared<TriMeshModel>();
        if (!readFileAsTriMesh(filename, tri))
        {
            return nullptr;
        }
        return tri;
    }

    TriMeshModelPtr
    AssimpReader::readBufferAsTriMesh(const std::string_view& v)
    {
        auto tri = std::make_shared<TriMeshModel>();
        if (!readBufferAsTriMesh(v, tri))
        {
            return nullptr;
        }
        return tri;
    }

    static void
    addPerVertexColorMaterial(aiColor4D* colors, unsigned int numColors, SoSeparator* result)
    {
        SoMaterialBinding* materialBinding = new SoMaterialBinding;
        materialBinding->value = SoMaterialBinding::PER_VERTEX_INDEXED;
        result->addChild(materialBinding);

        SoMaterial* materialNode = new SoMaterial;
        materialNode->diffuseColor.setNum(numColors);

        SbColor* diffuseData = materialNode->diffuseColor.startEditing();

        for (unsigned int i = 0; i < numColors; ++i)
        {
            aiColor4D color = colors[i];
            // Cannot define alpha value per vertex in Coin!
            diffuseData[i].setValue(color.r, color.g, color.b);
        }

        materialNode->diffuseColor.finishEditing();
        //materialNode->ambientColor = materialNode->diffuseColor;
        //materialNode->specularColor = materialNode->diffuseColor;

        result->addChild(materialNode);
    }

    static void
    addOverallMaterial(aiMaterial* material, SoSeparator* result)
    {
        SoMaterialBinding* materialBinding = new SoMaterialBinding;
        materialBinding->value = SoMaterialBinding::OVERALL;
        result->addChild(materialBinding);

        SoMaterial* materialNode = new SoMaterial;

        aiColor3D diffuseColor(0.0f, 0.0f, 0.0f);
        bool diffuseOk = material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuseColor) == aiReturn_SUCCESS;
        if (diffuseOk)
        {
            // std::cout << "diffuse: " << diffuseColor.r << ", " << diffuseColor.g << ", " << diffuseColor.b << std::endl;
            materialNode->diffuseColor.setValue(diffuseColor.r, diffuseColor.g, diffuseColor.b);
        }
        else
        {
            materialNode->diffuseColor.setValue(1.0f, 1.0f, 1.0f);
        }

        aiColor3D ambientColor(0.0f, 0.0f, 0.0f);
        bool ambientOk = material->Get(AI_MATKEY_COLOR_AMBIENT, ambientColor) == aiReturn_SUCCESS;
        if (ambientOk)
        {
            // std::cout << "diffuse: " << ambientColor.r << ", " << ambientColor.g << ", " << ambientColor.b << std::endl;
            materialNode->ambientColor.setValue(ambientColor.r, ambientColor.g, ambientColor.b);
        }

        aiColor3D specularColor(0.0f, 0.0f, 0.0f);
        bool specularOk =
            material->Get(AI_MATKEY_COLOR_SPECULAR, specularColor) == aiReturn_SUCCESS;
        if (specularOk)
        {
            // std::cout << "diffuse: " << specularColor.r << ", " << specularColor.g << ", " << specularColor.b << std::endl;
            materialNode->specularColor.setValue(specularColor.r, specularColor.g, specularColor.b);
        }

        aiColor3D emissiveColor(0.0f, 0.0f, 0.0f);
        bool emissiveOk =
            material->Get(AI_MATKEY_COLOR_EMISSIVE, emissiveColor) == aiReturn_SUCCESS;
        if (emissiveOk)
        {
            // std::cout << "diffuse: " << emissiveColor.r << ", " << emissiveColor.g << ", " << emissiveColor.b << std::endl;
            materialNode->emissiveColor.setValue(emissiveColor.r, emissiveColor.g, emissiveColor.b);
        }

        float shininess = 0.0f;
        bool shininessOk = material->Get(AI_MATKEY_SHININESS, shininess) == aiReturn_SUCCESS;
        if (shininessOk && shininess)
        {
            // std::cout << "shininess: " << shininess << std::endl;
            materialNode->shininess = shininess;
        }

        result->addChild(materialNode);
    }

    static void
    addTextureMaterial(aiMaterial* material,
                       aiTextureType textureType,
                       std::filesystem::path const& meshPath,
                       SoSeparator* result)
    {
        aiString path;
        aiTextureMapMode mapMode[3] = {
            aiTextureMapMode_Wrap, aiTextureMapMode_Wrap, aiTextureMapMode_Wrap};
        aiReturn pathOk = material->Get(_AI_MATKEY_TEXTURE_BASE, textureType, 0, path);
        aiReturn mapUOk = material->Get(_AI_MATKEY_MAPPINGMODE_U_BASE, textureType, 0, mapMode[0]);
        aiReturn mapVOk = material->Get(_AI_MATKEY_MAPPINGMODE_V_BASE, textureType, 0, mapMode[1]);

        if (pathOk == aiReturn_SUCCESS)
        {
            SoTexture2* textureNode = new SoTexture2();
            std::filesystem::path texturePath = meshPath.parent_path() / path.C_Str();
            textureNode->filename.set(texturePath.c_str());

            // Texture map mode for the first coordinate (assimp U, Coin S)
            if (mapUOk)
            {
                if (mapMode[0] == aiTextureMapMode_Wrap)
                {
                    textureNode->wrapS = SoTexture2::REPEAT;
                }
                else if (mapMode[0] == aiTextureMapMode_Clamp)
                {
                    textureNode->wrapS = SoTexture2::CLAMP;
                }
            }

            // Texture map mode for the second coordinate (assimp V, Coin T)
            if (mapVOk)
            {
                if (mapMode[1] == aiTextureMapMode_Wrap)
                {
                    textureNode->wrapT = SoTexture2::REPEAT;
                }
                else if (mapMode[1] == aiTextureMapMode_Clamp)
                {
                    textureNode->wrapT = SoTexture2::CLAMP;
                }
            }

            SbImage const& image = textureNode->image.getValue();
            SbVec3s size = image.getSize();

            VR_INFO << "Tried to load image from file '" << texturePath << "', size: " << size[0]
                    << " x " << size[1] << "\n";

            result->addChild(textureNode);
        }
        else
        {
            VR_WARNING << "Could not get texture data: " << pathOk << "In file: " << meshPath
                       << "\n";
        }
    }

    static void
    addVertices(unsigned int numVertices, aiVector3D* vertices, SoSeparator* result)
    {
        SoCoordinate3* vertexNode = new SoCoordinate3;
        vertexNode->point.setNum(numVertices);
        SbVec3f* vertexData = vertexNode->point.startEditing();

        for (unsigned int i = 0; i < numVertices; ++i)
        {
            vertexData[i][0] = vertices[i].x;
            vertexData[i][1] = vertices[i].y;
            vertexData[i][2] = vertices[i].z;
        }
        vertexNode->point.finishEditing();

        result->addChild(vertexNode);
    }

    static void
    addNormals(unsigned int numNormals, aiVector3D* normals, SoSeparator* result)
    {
        SoNormal* normalNode = new SoNormal;
        normalNode->vector.setNum(numNormals);
        SbVec3f* normalsData = normalNode->vector.startEditing();

        for (unsigned int i = 0; i < numNormals; ++i)
        {
            normalsData[i][0] = normals[i].x;
            normalsData[i][1] = normals[i].y;
            normalsData[i][2] = normals[i].z;
        }
        normalNode->vector.finishEditing();

        result->addChild(normalNode);

        SoNormalBinding* normalBinding = new SoNormalBinding;
        normalBinding->value = SoNormalBinding::PER_VERTEX_INDEXED;
        result->addChild(normalBinding);
    }

    static void
    addTextureCoordinates(unsigned int numCoordinates, aiVector3D* coordinates, SoSeparator* result)
    {
        SoTextureCoordinate2* texCoord = new SoTextureCoordinate2;
        texCoord->point.setNum(numCoordinates);
        SbVec2f* uvData = texCoord->point.startEditing();
        for (unsigned int i = 0; i < numCoordinates; ++i)
        {
            aiVector3D uv = coordinates[i];
            uvData[i][0] = uv[0];
            uvData[i][1] = uv[1];
        }
        texCoord->point.finishEditing();
        result->addChild(texCoord);
    }

    static void
    addIndexedFaceSet(unsigned int numFaces, aiFace* faces, SoSeparator* result)
    {
        SoIndexedFaceSet* faceSetNode = new SoIndexedFaceSet;
        faceSetNode->coordIndex.setNum(numFaces * 4);
        int32_t* indexData = faceSetNode->coordIndex.startEditing();
        for (unsigned int i = 0; i < numFaces; ++i)
        {
            unsigned int* indices = faces[i].mIndices;
            indexData[0] = indices[0];
            indexData[1] = indices[1];
            indexData[2] = indices[2];
            //Coin requires -1 as end of face marker
            indexData[3] = -1;

            indexData += 4;
        }
        faceSetNode->coordIndex.finishEditing();

        // Assimp: Coord indices are equivalent with the normal indices
        faceSetNode->normalIndex = faceSetNode->coordIndex;

        result->addChild(faceSetNode);
    }

    SoNode*
    AssimpReader::readFileAsSoNode(const std::string& filename)
    {
        std::filesystem::path meshPath = filename;

        // For assimp to work correctly, we need to set the locale to something english
        setlocale(LC_ALL, "C");

        Assimp::Importer importer;
        // Triangulate, so that we can assume faces with 3 vertices (aiProcess_Triangulate)
        // Generate normals if they are not present so that the subsequent code
        // can assume that per vertex normals exist (aiProcess_GenSmoothNormals)
        // Generate UV coordinates if possible (for primitive shapes like boxes, spheres, ...)
        const aiScene* scene = importer.ReadFile(
            filename,
            // aiProcess_JoinIdenticalVertices |
            aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_GenUVCoords |
                aiProcess_TransformUVCoords | aiProcess_SortByPType);

        if (!scene)
        {
            VR_WARNING << "Could not load mesh file: '" << filename << "'\n"
                       << importer.GetErrorString() << std::endl;
            return nullptr;
        }

        unsigned int numMeshes = scene->mNumMeshes;
        if (numMeshes < 1)
        {
            VR_INFO << "No mesh found in file: " << filename << std::endl;
            return nullptr;
        }

        SoSeparator* group = new SoSeparator;
        for (unsigned int meshIndex = 0; meshIndex < numMeshes; ++meshIndex)
        {
            // We only load the first mesh
            // If there are files with multiple meshes inside, we would need to loop over them
            aiMesh* mesh = scene->mMeshes[meshIndex];
            unsigned int numVertices = mesh->mNumVertices;
            unsigned int numFaces = mesh->mNumFaces;

            if (!mesh->HasNormals())
            {
                VR_WARNING << "Expected assimp mesh to have normals since we requested to generate "
                              "them if they are not present\n"
                           << "Mesh index: " << meshIndex << "File: " << filename << std::endl;
                continue;
            }

            // Each mesh has a corresponding material (via mMaterialIndex)
            // We use the material to lookup textures
            unsigned int materialIndex = mesh->mMaterialIndex;
            if (materialIndex >= scene->mNumMaterials)
            {
                VR_WARNING << "Material index is invalid: " << materialIndex
                           << " >= " << scene->mNumMaterials << "\nMesh index: " << meshIndex
                           << "File: " << filename << std::endl;
                continue;
            }
            aiMaterial* material = scene->mMaterials[materialIndex];
            // We only lookup diffuse color textures
            // Maybe other texture types should be supported as well?
            aiTextureType textureType = aiTextureType_DIFFUSE;
            unsigned int numTextures = material->GetTextureCount(textureType);

#if 0
            VR_INFO << "Loaded mesh from file '" << filename << "' with "
                    << "mesh index " << meshIndex << "(total #meshes " << numMeshes << "), "
                    << numVertices << " vertices, "
                    << numFaces << " faces, "
                    << numTextures << " texture(s), "
                    << (mesh->HasVertexColors(0) ? "has" : "no") << " vertex colors, "
                    << (mesh->HasTextureCoords(0) ? "has" : "no") << " texture coordinates\n";
#endif


            SoSeparator* result = new SoSeparator;

            addOverallMaterial(material, result);

            // We only look at the first vertex color attribute (index 0)
            // Coin does not support multiple colors per vertex
            if (mesh->HasVertexColors(0))
            {
                addPerVertexColorMaterial(mesh->mColors[0], numVertices, result);
            }

            if (numTextures > 0)
            {
                addTextureMaterial(material, textureType, meshPath, result);
            }

            addVertices(numVertices, mesh->mVertices, result);
            addNormals(numVertices, mesh->mNormals, result);

            if (mesh->HasTextureCoords(0))
            {
                addTextureCoordinates(numVertices, mesh->mTextureCoords[0], result);
            }

            addIndexedFaceSet(numFaces, mesh->mFaces, result);

            group->addChild(result);
        }

        return group;
    }

    ManipulationObjectPtr
    AssimpReader::readFileAsManipulationObject(const std::string& filename, const std::string& name)
    {
        if (auto tri = readFileAsTriMesh(filename))
        {
            return std::make_shared<ManipulationObject>(name, tri, filename);
        }
        return nullptr;
    }

    ManipulationObjectPtr
    AssimpReader::readBufferAsManipulationObject(const std::string_view& v, const std::string& name)
    {
        if (auto tri = readBufferAsTriMesh(v))
        {
            return std::make_shared<ManipulationObject>(name, tri);
        }
        return nullptr;
    }
} // namespace VirtualRobot
