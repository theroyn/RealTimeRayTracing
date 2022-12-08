#pragma once

#include "RaytracingHlslCompat.h"

struct aiScene;

class Model
{
public:
    const aiScene* m_scene = nullptr;
    std::vector<Index> m_indices;
    std::vector<Vertex> m_vertices;

    Model(const std::string& path);

};