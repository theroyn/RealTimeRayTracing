#include "stdafx.h"

#include "Model.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

Model::Model()
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile("", aiProcess_Triangulate | aiProcess_FlipUVs);
}