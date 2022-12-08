#include "stdafx.h"

#include "Model.h"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <iostream>

Model::Model(const std::string& path)
{
    Assimp::Importer importer;
    m_scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_ValidateDataStructure);

    // DUDU support multiple meshes in single model
    if (!m_scene || m_scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !m_scene->mRootNode || m_scene->mNumMeshes != 1)
    {
        std::string msg = std::string("ERROR::ASSIMP::") + importer.GetErrorString();
        throw std::runtime_error(msg);
    }

    for (int meshIdx = 0; meshIdx < m_scene->mNumMeshes; ++meshIdx)
    {
        aiMesh* mesh = m_scene->mMeshes[meshIdx];
        for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
        {
            const aiVector3D& vertex = mesh->mVertices[i];
            if (!mesh->HasNormals())
            {
                throw std::runtime_error("no normals at mesh:" + std::to_string(i));
            }
            const aiVector3D& normal = mesh->mNormals[i];
            Vertex v;
            v.position = XMFLOAT3{ vertex.x, vertex.y, vertex.z };
            v.normal = XMFLOAT3{ normal.x, normal.y, normal.z };
            m_vertices.push_back(v);
        }
        for (unsigned int i = 0; i < mesh->mNumFaces; i++)
        {
            aiFace face = mesh->mFaces[i];
            for (unsigned int j = 0; j < face.mNumIndices; j++)
            {
                m_indices.push_back(face.mIndices[j]);
            }
        }
    }
}