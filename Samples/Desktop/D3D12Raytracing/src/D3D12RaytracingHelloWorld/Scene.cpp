#include "stdafx.h"

#include "Scene.h"

using namespace DirectX;

size_t Scene::LoadModel(const std::string& path)
{
    _models.emplace_back(path);
    size_t modelIdx = _models.size() - 1;
    _instances.emplace(modelIdx, std::vector<SceneInstance>{});
    return modelIdx;
}

void Scene::AddInstance(size_t modelIdx, const DirectX::XMMATRIX& transform, unsigned int materialIndex)
{
    _instances.at(modelIdx).push_back(SceneInstance{ transform, materialIndex });
}

void Scene::GetInstances(D3D12_GPU_VIRTUAL_ADDRESS BLASAddress,
                         std::vector<D3D12_RAYTRACING_INSTANCE_DESC>& instanceDescs)
{
    instanceDescs.clear();
    for (int i = 0; i < _models.size(); ++i)
    {
        auto& modelInstances = _instances.at(i);
        for (int j = 0; j < modelInstances.size(); ++j)
        {
            const SceneInstance& instance = modelInstances.at(j);
            const DirectX::XMMATRIX& instanceTransform = instance.transform;
            XMMATRIX transposed = XMMatrixTranspose(instanceTransform);
            D3D12_RAYTRACING_INSTANCE_DESC instanceDesc = {};
            instanceDesc.InstanceMask = 1;
            instanceDesc.InstanceID = instance.materialIndex;
            instanceDesc.AccelerationStructure = BLASAddress;
            for (int rowIdx = 0; rowIdx < 3; ++rowIdx)
            {
                XMFLOAT4 rowFloat4;
                XMStoreFloat4(&rowFloat4, transposed.r[rowIdx]);
                instanceDesc.Transform[rowIdx][0] = rowFloat4.x;
                instanceDesc.Transform[rowIdx][1] = rowFloat4.y;
                instanceDesc.Transform[rowIdx][2] = rowFloat4.z;
                instanceDesc.Transform[rowIdx][3] = rowFloat4.w;
            }

            instanceDescs.push_back(instanceDesc);
        }
    }
}

void Scene::GetModelData(size_t modelIdx, std::vector<Vertex>& verticesData, std::vector<Index>& indicesData)
{
    verticesData = _models.at(modelIdx).m_vertices;
    indicesData = _models.at(modelIdx).m_indices;
}