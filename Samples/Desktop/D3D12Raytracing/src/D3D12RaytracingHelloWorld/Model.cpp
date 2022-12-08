#include "stdafx.h"

#include "Model.h"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <iostream>

Model::Model()
{
    Assimp::Importer importer;
    m_scene = importer.ReadFile("SphereRad1.obj", aiProcess_Triangulate | /*aiProcess_ConvertToLeftHanded |*/
                                                      aiProcess_ValidateDataStructure);

    if (!m_scene || m_scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !m_scene->mRootNode || m_scene->mNumMeshes != 1)
    {
        std::string msg = std::string("ERROR::ASSIMP::") + importer.GetErrorString();
        throw std::runtime_error(msg);
    }

    aiMesh* mesh = m_scene->mMeshes[0];
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
    {
        const aiVector3D& vertex = mesh->mVertices[i];
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