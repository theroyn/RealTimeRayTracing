#pragma once

#include "Model.h"
#include <map>
#include <vector>

class Scene
{
public:
    Scene() {}
    // return model idx
    size_t LoadModel(const std::string& path);
    void AddInstance(size_t modelIdx, const DirectX::XMMATRIX& transform, unsigned int materialIndex);
    void GetInstances(D3D12_GPU_VIRTUAL_ADDRESS BLASAddress,
                      std::vector<D3D12_RAYTRACING_INSTANCE_DESC>& instanceDescs);
    void GetModelData(size_t modelIdx, std::vector<Vertex>& verticesData, std::vector<Index>& indicesData);

private:
    struct SceneInstance
    {
        DirectX::XMMATRIX transform;
        unsigned int materialIndex;
    };

private:
    std::vector<Model> _models;
    std::map<size_t /**model idx*/, std::vector<SceneInstance> /** model instance transforms*/> _instances;
};
