//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************
#include "stdafx.h"

#include "CompiledShaders\Raytracing.hlsl.h"
#include "D3D12RaytracingHelloWorld.h"
#include "DirectXRaytracingHelper.h"

#include <chrono>
using namespace std;
using namespace DX;

const wchar_t* D3D12RaytracingHelloWorld::c_hitGroupName = L"MyHitGroup";
const wchar_t* D3D12RaytracingHelloWorld::c_raygenShaderName = L"MyRaygenShader";
const wchar_t* D3D12RaytracingHelloWorld::c_closestHitShaderName = L"MyClosestHitShader";
const wchar_t* D3D12RaytracingHelloWorld::c_missShaderName = L"MyMissShader";

D3D12RaytracingHelloWorld::D3D12RaytracingHelloWorld(UINT width, UINT height, std::wstring name)
    : DXSample(width, height, name), m_raytracingOutputResourceUAVDescriptorHeapIndex(UINT_MAX)
{
    m_rayGenCB.viewport = { -1.0f, -1.0f, 1.0f, 1.0f };

    InitializeCamera();
    UpdateForSizeChange(width, height);
}

void D3D12RaytracingHelloWorld::InitializeCamera()
{
    XMVECTOR lookfrom{ std::sin(m_rayGenCB.timeNow * .01f), 0.f, 1.f, 1.f };
    XMVECTOR lookat{ 0.f, 0.f, -1.f, 1.f };
    XMVECTOR vup{ 0., 1., 0. };
    m_cam = std::make_shared<Camera>(lookfrom, lookat, vup);
}

void D3D12RaytracingHelloWorld::UpdateCamera()
{
    using namespace DirectX;
    XMVECTOR lookfrom = m_cam->GetLookFrom();
    XMVECTOR forward = m_cam->GetForward();
    XMVECTOR up = m_cam->GetUp();
    XMVECTOR right = m_cam->GetRight();

    float aspectRatio = (float)GetWidth() / (float)GetHeight();
    float vpHeight = 2.f;
    float vpWidth = aspectRatio * vpHeight;

    XMVECTOR vpHorizontal = vpWidth * right;
    XMVECTOR vpVertical = vpHeight * up;

    XMVECTOR leftCorner = lookfrom + 1.f * forward - 0.5f * vpHorizontal - 0.5 * vpVertical;

    XMStoreFloat3(&m_rayGenCB.origin, lookfrom);
    XMStoreFloat3(&m_rayGenCB.leftCorner, leftCorner);
    XMStoreFloat3(&m_rayGenCB.vpHorizontal, vpHorizontal);
    XMStoreFloat3(&m_rayGenCB.vpVertical, vpVertical);
}

void D3D12RaytracingHelloWorld::OnInit()
{
    InitializeScene();

    m_deviceResources = std::make_unique<DeviceResources>(
        DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_FORMAT_UNKNOWN, FrameCount, D3D_FEATURE_LEVEL_11_0,
        // Sample shows handling of use cases with tearing support, which is OS dependent and has been supported since
        // TH2. Since the sample requires build 1809 (RS5) or higher, we don't need to handle non-tearing cases.
        DeviceResources::c_RequireTearingSupport, m_adapterIDoverride);
    m_deviceResources->RegisterDeviceNotify(this);
    m_deviceResources->SetWindow(Win32Application::GetHwnd(), m_width, m_height);
    m_deviceResources->InitializeDXGIAdapter();

    ThrowIfFalse(IsDirectXRaytracingSupported(m_deviceResources->GetAdapter()),
                 L"ERROR: DirectX Raytracing is not supported by your OS, GPU and/or driver.\n\n");

    m_deviceResources->CreateDeviceResources();
    m_deviceResources->CreateWindowSizeDependentResources();

    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();
}

// Create resources that depend on the device.
void D3D12RaytracingHelloWorld::CreateDeviceDependentResources()
{
    // Initialize raytracing pipeline.

    // Create raytracing interfaces: raytracing device and commandlist.
    CreateRaytracingInterfaces();

    // Create root signatures for the shaders.
    CreateRootSignatures();

    // Create a raytracing pipeline state object which defines the binding of shaders, state and resources to be used
    // during raytracing.
    CreateRaytracingPipelineStateObject();

    // Create a heap for descriptors.
    CreateDescriptorHeap();

    // Build geometry to be used in the sample.
    BuildGeometry();

    // Build raytracing acceleration structures from the generated geometry.
    BuildAccelerationStructures();

    // Build shader tables, which define shaders and their local root arguments.
    BuildShaderTables();

    // Create an output 2D texture to store the raytracing result to.
    CreateRaytracingOutputResource();
}

void D3D12RaytracingHelloWorld::SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc,
                                                                          ComPtr<ID3D12RootSignature>* rootSig)
{
    auto device = m_deviceResources->GetD3DDevice();
    ComPtr<ID3DBlob> blob;
    ComPtr<ID3DBlob> error;

    ThrowIfFailed(D3D12SerializeRootSignature(&desc, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &error),
                  error ? static_cast<wchar_t*>(error->GetBufferPointer()) : nullptr);
    ThrowIfFailed(
        device->CreateRootSignature(1, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&(*rootSig))));
}

void D3D12RaytracingHelloWorld::CreateRootSignatures()
{
    // Global Root Signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    {
        CD3DX12_DESCRIPTOR_RANGE UAVDescriptor;
        CD3DX12_DESCRIPTOR_RANGE SRVDescriptor;
        UAVDescriptor.Init(D3D12_DESCRIPTOR_RANGE_TYPE_UAV, 1, 0);
        SRVDescriptor.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 1);
        CD3DX12_ROOT_PARAMETER rootParameters[GlobalRootSignatureParams::Count];
        rootParameters[GlobalRootSignatureParams::OutputViewSlot].InitAsDescriptorTable(1, &UAVDescriptor);
        rootParameters[GlobalRootSignatureParams::VertexBuffers].InitAsDescriptorTable(1, &SRVDescriptor);
        rootParameters[GlobalRootSignatureParams::AccelerationStructureSlot].InitAsShaderResourceView(0);
        CD3DX12_ROOT_SIGNATURE_DESC globalRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        SerializeAndCreateRaytracingRootSignature(globalRootSignatureDesc, &m_raytracingGlobalRootSignature);
    }

    // Local Root Signature
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.
    {
        CD3DX12_ROOT_PARAMETER rootParameters[LocalRootSignatureParams::Count];
        rootParameters[LocalRootSignatureParams::ViewportConstantSlot].InitAsConstants(SizeOfInUint32(m_rayGenCB), 0,
                                                                                       0);
        CD3DX12_ROOT_SIGNATURE_DESC localRootSignatureDesc(ARRAYSIZE(rootParameters), rootParameters);
        localRootSignatureDesc.Flags = D3D12_ROOT_SIGNATURE_FLAG_LOCAL_ROOT_SIGNATURE;
        SerializeAndCreateRaytracingRootSignature(localRootSignatureDesc, &m_raytracingLocalRootSignature);
    }
}

// Create raytracing device and command list.
void D3D12RaytracingHelloWorld::CreateRaytracingInterfaces()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();

    ThrowIfFailed(device->QueryInterface(IID_PPV_ARGS(&m_dxrDevice)),
                  L"Couldn't get DirectX Raytracing interface for the device.\n");
    ThrowIfFailed(commandList->QueryInterface(IID_PPV_ARGS(&m_dxrCommandList)),
                  L"Couldn't get DirectX Raytracing interface for the command list.\n");
}

// Local root signature and shader association
// This is a root signature that enables a shader to have unique arguments that come from shader tables.
void D3D12RaytracingHelloWorld::CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline)
{
    // Hit group and miss shaders in this sample are not using a local root signature and thus one is not associated
    // with them.

    // Local root signature to be used in a ray gen shader.
    {
        auto localRootSignature = raytracingPipeline->CreateSubobject<CD3DX12_LOCAL_ROOT_SIGNATURE_SUBOBJECT>();
        localRootSignature->SetRootSignature(m_raytracingLocalRootSignature.Get());
        // Shader association
        auto rootSignatureAssociation =
            raytracingPipeline->CreateSubobject<CD3DX12_SUBOBJECT_TO_EXPORTS_ASSOCIATION_SUBOBJECT>();
        rootSignatureAssociation->SetSubobjectToAssociate(*localRootSignature);
        rootSignatureAssociation->AddExport(c_raygenShaderName);
    }
}

// Create a raytracing pipeline state object (RTPSO).
// An RTPSO represents a full set of shaders reachable by a DispatchRays() call,
// with all configuration options resolved, such as local signatures and other state.
void D3D12RaytracingHelloWorld::CreateRaytracingPipelineStateObject()
{
    // Create 7 subobjects that combine into a RTPSO:
    // Subobjects need to be associated with DXIL exports (i.e. shaders) either by way of default or explicit
    // associations. Default association applies to every exported shader entrypoint that doesn't have any of the same
    // type of subobject associated with it. This simple sample utilizes default shader association except for local
    // root signature subobject which has an explicit association specified purely for demonstration purposes. 1 - DXIL
    // library 1 - Triangle hit group 1 - Shader config 2 - Local root signature and association 1 - Global root
    // signature 1 - Pipeline config
    CD3DX12_STATE_OBJECT_DESC raytracingPipeline{ D3D12_STATE_OBJECT_TYPE_RAYTRACING_PIPELINE };

    // DXIL library
    // This contains the shaders and their entrypoints for the state object.
    // Since shaders are not considered a subobject, they need to be passed in via DXIL library subobjects.
    auto lib = raytracingPipeline.CreateSubobject<CD3DX12_DXIL_LIBRARY_SUBOBJECT>();
    D3D12_SHADER_BYTECODE libdxil = CD3DX12_SHADER_BYTECODE((void*)g_pRaytracing, ARRAYSIZE(g_pRaytracing));
    lib->SetDXILLibrary(&libdxil);
    // Define which shader exports to surface from the library.
    // If no shader exports are defined for a DXIL library subobject, all shaders will be surfaced.
    // In this sample, this could be omitted for convenience since the sample uses all shaders in the library.
    {
        lib->DefineExport(c_raygenShaderName);
        lib->DefineExport(c_closestHitShaderName);
        lib->DefineExport(c_missShaderName);
    }

    // Triangle hit group
    // A hit group specifies closest hit, any hit and intersection shaders to be executed when a ray intersects the
    // geometry's triangle/AABB. In this sample, we only use triangle geometry with a closest hit shader, so others are
    // not set.
    auto hitGroup = raytracingPipeline.CreateSubobject<CD3DX12_HIT_GROUP_SUBOBJECT>();
    hitGroup->SetClosestHitShaderImport(c_closestHitShaderName);
    hitGroup->SetHitGroupExport(c_hitGroupName);
    hitGroup->SetHitGroupType(D3D12_HIT_GROUP_TYPE_TRIANGLES);

    // Shader config
    // Defines the maximum sizes in bytes for the ray payload and attribute structure.
    auto shaderConfig = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_SHADER_CONFIG_SUBOBJECT>();
    UINT payloadSize = 4 * sizeof(float);   // float4 color
    UINT attributeSize = 2 * sizeof(float); // float2 barycentrics
    shaderConfig->Config(payloadSize, attributeSize);

    // Local root signature and shader association
    CreateLocalRootSignatureSubobjects(&raytracingPipeline);
    // This is a root signature that enables a shader to have unique arguments that come from shader tables.

    // Global root signature
    // This is a root signature that is shared across all raytracing shaders invoked during a DispatchRays() call.
    auto globalRootSignature = raytracingPipeline.CreateSubobject<CD3DX12_GLOBAL_ROOT_SIGNATURE_SUBOBJECT>();
    globalRootSignature->SetRootSignature(m_raytracingGlobalRootSignature.Get());

    // Pipeline config
    // Defines the maximum TraceRay() recursion depth.
    auto pipelineConfig = raytracingPipeline.CreateSubobject<CD3DX12_RAYTRACING_PIPELINE_CONFIG_SUBOBJECT>();
    // PERFOMANCE TIP: Set max recursion depth as low as needed
    // as drivers may apply optimization strategies for low recursion depths.
    UINT maxRecursionDepth = 1; // ~ primary rays only.
    pipelineConfig->Config(maxRecursionDepth);

#if _DEBUG
    PrintStateObjectDesc(raytracingPipeline);
#endif

    // Create the state object.
    ThrowIfFailed(m_dxrDevice->CreateStateObject(raytracingPipeline, IID_PPV_ARGS(&m_dxrStateObject)),
                  L"Couldn't create DirectX Raytracing state object.\n");
}

// Create 2D output texture for raytracing.
void D3D12RaytracingHelloWorld::CreateRaytracingOutputResource()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto backbufferFormat = m_deviceResources->GetBackBufferFormat();

    // Create the output resource. The dimensions and format should match the swap-chain.
    auto uavDesc = CD3DX12_RESOURCE_DESC::Tex2D(backbufferFormat, m_width, m_height, 1, 1, 1, 0,
                                                D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    auto defaultHeapProperties = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
    ThrowIfFailed(device->CreateCommittedResource(&defaultHeapProperties, D3D12_HEAP_FLAG_NONE, &uavDesc,
                                                  D3D12_RESOURCE_STATE_UNORDERED_ACCESS, nullptr,
                                                  IID_PPV_ARGS(&m_raytracingOutput)));
    NAME_D3D12_OBJECT(m_raytracingOutput);

    D3D12_CPU_DESCRIPTOR_HANDLE uavDescriptorHandle;
    m_raytracingOutputResourceUAVDescriptorHeapIndex =
        AllocateDescriptor(&uavDescriptorHandle, m_raytracingOutputResourceUAVDescriptorHeapIndex);
    D3D12_UNORDERED_ACCESS_VIEW_DESC UAVDesc = {};
    UAVDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
    device->CreateUnorderedAccessView(m_raytracingOutput.Get(), nullptr, &UAVDesc, uavDescriptorHandle);
    m_raytracingOutputResourceUAVGpuDescriptor =
        CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(),
                                      m_raytracingOutputResourceUAVDescriptorHeapIndex, m_descriptorSize);
}

void D3D12RaytracingHelloWorld::CreateDescriptorHeap()
{
    auto device = m_deviceResources->GetD3DDevice();

    D3D12_DESCRIPTOR_HEAP_DESC descriptorHeapDesc = {};
    // Allocate a heap for 3 descriptors:
    // 2 - vertex and index buffer SRVs
    // 1 - raytracing output texture SRV
    descriptorHeapDesc.NumDescriptors = 3;
    descriptorHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    descriptorHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    descriptorHeapDesc.NodeMask = 0;
    device->CreateDescriptorHeap(&descriptorHeapDesc, IID_PPV_ARGS(&m_descriptorHeap));
    NAME_D3D12_OBJECT(m_descriptorHeap);

    m_descriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
}

// Create a SRV for a buffer.
UINT D3D12RaytracingHelloWorld::CreateBufferSRV(D3DBuffer* buffer, UINT numElements, UINT elementSize)
{
    auto device = m_deviceResources->GetD3DDevice();

    // SRV
    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Buffer.NumElements = numElements;
    if (elementSize == 0)
    {
        srvDesc.Format = DXGI_FORMAT_R32_TYPELESS;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_RAW;
        srvDesc.Buffer.StructureByteStride = 0;
    }
    else
    {
        srvDesc.Format = DXGI_FORMAT_UNKNOWN;
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_NONE;
        srvDesc.Buffer.StructureByteStride = elementSize;
    }
    UINT descriptorIndex = AllocateDescriptor(&buffer->cpuDescriptorHandle);
    device->CreateShaderResourceView(buffer->resource.Get(), &srvDesc, buffer->cpuDescriptorHandle);
    buffer->gpuDescriptorHandle = CD3DX12_GPU_DESCRIPTOR_HANDLE(m_descriptorHeap->GetGPUDescriptorHandleForHeapStart(),
                                                                descriptorIndex, m_descriptorSize);
    return descriptorIndex;
}

// Build geometry used in the sample.
void D3D12RaytracingHelloWorld::BuildGeometry()
{
    auto device = m_deviceResources->GetD3DDevice();

    std::vector<Vertex> verticesData;
    std::vector<Index> indicesData;
    m_scene.GetModelData(0, verticesData, indicesData);

    Vertex* vertices = verticesData.data();
    UINT verticesCount = static_cast<UINT>(verticesData.size());
    size_t verticesSizeBytes = sizeof(vertices[0]) * verticesCount;
    Index* indices = indicesData.data();
    UINT indicesCount = static_cast<UINT>(indicesData.size());
    size_t indicesSizeBytes = sizeof(indices[0]) * indicesCount;

    AllocateUploadBuffer(device, indices, indicesSizeBytes, &m_indexBuffer.resource);
    AllocateUploadBuffer(device, vertices, verticesSizeBytes, &m_vertexBuffer.resource);

    // Vertex buffer is passed to the shader along with index buffer as a descriptor range.
    UINT descriptorIndexIB = CreateBufferSRV(&m_indexBuffer, static_cast<UINT>(indicesSizeBytes / 4), 0);
    UINT descriptorIndexVB = CreateBufferSRV(&m_vertexBuffer, (UINT)verticesCount, sizeof(vertices[0]));
    ThrowIfFalse(descriptorIndexVB == descriptorIndexIB + 1,
                 L"Vertex Buffer descriptor index must follow that of Index Buffer descriptor index");
}

// Build acceleration structures needed for raytracing.
void D3D12RaytracingHelloWorld::BuildAccelerationStructures()
{
    auto device = m_deviceResources->GetD3DDevice();
    auto commandList = m_deviceResources->GetCommandList();
    auto commandQueue = m_deviceResources->GetCommandQueue();
    auto commandAllocator = m_deviceResources->GetCommandAllocator();

    // Reset the command list for the acceleration structure construction.
    commandList->Reset(commandAllocator, nullptr);

    D3D12_RAYTRACING_GEOMETRY_DESC geometryDesc = {};
    geometryDesc.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
    geometryDesc.Triangles.IndexBuffer = m_indexBuffer.resource->GetGPUVirtualAddress();
    geometryDesc.Triangles.IndexCount = static_cast<UINT>(m_indexBuffer.resource->GetDesc().Width) / sizeof(Index);
    geometryDesc.Triangles.IndexFormat = DXGI_FORMAT_R16_UINT;
    geometryDesc.Triangles.Transform3x4 = 0;
    geometryDesc.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
    geometryDesc.Triangles.VertexCount = static_cast<UINT>(m_vertexBuffer.resource->GetDesc().Width) / sizeof(Vertex);
    geometryDesc.Triangles.VertexBuffer.StartAddress = m_vertexBuffer.resource->GetGPUVirtualAddress();
    geometryDesc.Triangles.VertexBuffer.StrideInBytes = sizeof(Vertex);

    // Mark the geometry as opaque.
    // PERFORMANCE TIP: mark geometry as opaque whenever applicable as it can enable important ray processing
    // optimizations. Note: When rays encounter opaque geometry an any hit shader will not be executed whether it is
    // present or not.
    geometryDesc.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
    // Get required sizes for an acceleration structure.
    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS buildFlags =
        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS topLevelInputs = {};
    topLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    topLevelInputs.Flags = buildFlags;
    topLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO topLevelPrebuildInfo = {};
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&topLevelInputs, &topLevelPrebuildInfo);
    ThrowIfFalse(topLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS bottomLevelInputs = {};
    bottomLevelInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
    bottomLevelInputs.Flags = buildFlags;
    bottomLevelInputs.NumDescs = 1;
    bottomLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO bottomLevelPrebuildInfo = {};
    bottomLevelInputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
    bottomLevelInputs.pGeometryDescs = &geometryDesc;
    m_dxrDevice->GetRaytracingAccelerationStructurePrebuildInfo(&bottomLevelInputs, &bottomLevelPrebuildInfo);
    ThrowIfFalse(bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes > 0);

    ComPtr<ID3D12Resource> scratchResource;
    AllocateUAVBuffer(device,
                      max(topLevelPrebuildInfo.ScratchDataSizeInBytes, bottomLevelPrebuildInfo.ScratchDataSizeInBytes),
                      &scratchResource, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, L"ScratchResource");

    // Allocate resources for acceleration structures.
    // Acceleration structures can only be placed in resources that are created in the default heap (or custom heap
    // equivalent). Default heap is OK since the application doesn’t need CPU read/write access to them. The resources
    // that will contain acceleration structures must be created in the state
    // D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, and must have resource flag
    // D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS. The ALLOW_UNORDERED_ACCESS requirement simply acknowledges both:
    //  - the system will be doing this type of access in its implementation of acceleration structure builds behind the
    //  scenes.
    //  - from the app point of view, synchronization of writes/reads to acceleration structures is accomplished using
    //  UAV barriers.
    {
        D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE;

        AllocateUAVBuffer(device, bottomLevelPrebuildInfo.ResultDataMaxSizeInBytes, &m_bottomLevelAccelerationStructure,
                          initialResourceState, L"BottomLevelAccelerationStructure");
        AllocateUAVBuffer(device, topLevelPrebuildInfo.ResultDataMaxSizeInBytes, &m_topLevelAccelerationStructure,
                          initialResourceState, L"TopLevelAccelerationStructure");
    }

    // Create an instance desc for the bottom-level acceleration structure.
    ComPtr<ID3D12Resource> instanceDescsResource;

    std::vector<D3D12_RAYTRACING_INSTANCE_DESC> instanceDescs;
    m_scene.GetInstances(m_bottomLevelAccelerationStructure->GetGPUVirtualAddress(), instanceDescs);

    UINT64 bufferSize = static_cast<UINT64>(instanceDescs.size() * sizeof(instanceDescs[0]));
    AllocateUploadBuffer(device, instanceDescs.data(), bufferSize, &instanceDescsResource, L"InstanceDescs");

    // Bottom Level Acceleration Structure desc
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC bottomLevelBuildDesc = {};
    {
        bottomLevelBuildDesc.Inputs = bottomLevelInputs;
        bottomLevelBuildDesc.ScratchAccelerationStructureData = scratchResource->GetGPUVirtualAddress();
        bottomLevelBuildDesc.DestAccelerationStructureData = m_bottomLevelAccelerationStructure->GetGPUVirtualAddress();
    }

    // Top Level Acceleration Structure desc
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC topLevelBuildDesc = {};
    {
        topLevelInputs.InstanceDescs = instanceDescsResource->GetGPUVirtualAddress();
        topLevelInputs.NumDescs = (UINT)instanceDescs.size();
        topLevelBuildDesc.Inputs = topLevelInputs;
        topLevelBuildDesc.DestAccelerationStructureData = m_topLevelAccelerationStructure->GetGPUVirtualAddress();
        topLevelBuildDesc.ScratchAccelerationStructureData = scratchResource->GetGPUVirtualAddress();
    }

    auto BuildAccelerationStructure = [&](auto* raytracingCommandList)
    {
        raytracingCommandList->BuildRaytracingAccelerationStructure(&bottomLevelBuildDesc, 0, nullptr);
        commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::UAV(m_bottomLevelAccelerationStructure.Get()));
        raytracingCommandList->BuildRaytracingAccelerationStructure(&topLevelBuildDesc, 0, nullptr);
    };

    // Build acceleration structure.
    BuildAccelerationStructure(m_dxrCommandList.Get());

    // Kick off acceleration structure construction.
    m_deviceResources->ExecuteCommandList();

    // Wait for GPU to finish as the locally created temporary GPU resources will get released once we go out of scope.
    m_deviceResources->WaitForGpu();
}

void D3D12RaytracingHelloWorld::InitRayGenTable()
{
    struct RootArguments
    {
        RayGenConstantBuffer cb;
    } rootArguments;
    rootArguments.cb = m_rayGenCB;

    UINT numShaderRecords = 1;
    UINT shaderRecordSize = m_shaderIdentifierSize + sizeof(rootArguments);
    m_actualRayGenShaderTable->ModifyFirstRecord(
        ShaderRecord(m_rayGenShaderIdentifier, m_shaderIdentifierSize, &rootArguments, sizeof(rootArguments)));
}
// Build shader tables.
// This encapsulates all shader records - shaders and the arguments for their local root signatures.
void D3D12RaytracingHelloWorld::BuildShaderTables()
{
    auto device = m_deviceResources->GetD3DDevice();

    // void* rayGenShaderIdentifier;
    void* missShaderIdentifier;
    void* hitGroupShaderIdentifier;

    auto GetShaderIdentifiers = [&](auto* stateObjectProperties)
    {
        m_rayGenShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_raygenShaderName);
        missShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_missShaderName);
        hitGroupShaderIdentifier = stateObjectProperties->GetShaderIdentifier(c_hitGroupName);
    };

    // Get shader identifiers.
    {
        ComPtr<ID3D12StateObjectProperties> stateObjectProperties;
        ThrowIfFailed(m_dxrStateObject.As(&stateObjectProperties));
        GetShaderIdentifiers(stateObjectProperties.Get());
        m_shaderIdentifierSize = D3D12_SHADER_IDENTIFIER_SIZE_IN_BYTES;
    }

    // Ray gen shader table
    {
        struct RootArguments
        {
            RayGenConstantBuffer cb;
        } rootArguments;
        rootArguments.cb = m_rayGenCB;

        UINT numShaderRecords = 1;
        UINT shaderRecordSize = m_shaderIdentifierSize + sizeof(rootArguments);
        m_actualRayGenShaderTable.reset(
            new ShaderTable(device, numShaderRecords, shaderRecordSize, L"RayGenShaderTable"));
        // ShaderTable rayGenShaderTable(device, numShaderRecords, shaderRecordSize, L"RayGenShaderTable");
        m_actualRayGenShaderTable->push_back(
            ShaderRecord(m_rayGenShaderIdentifier, m_shaderIdentifierSize, &rootArguments, sizeof(rootArguments)));
        m_rayGenShaderTable = m_actualRayGenShaderTable->GetResource();
    }

    // Miss shader table
    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = m_shaderIdentifierSize;
        ShaderTable missShaderTable(device, numShaderRecords, shaderRecordSize, L"MissShaderTable");
        missShaderTable.push_back(ShaderRecord(missShaderIdentifier, m_shaderIdentifierSize));
        m_missShaderTable = missShaderTable.GetResource();
    }

    // Hit group shader table
    {
        UINT numShaderRecords = 1;
        UINT shaderRecordSize = m_shaderIdentifierSize;
        ShaderTable hitGroupShaderTable(device, numShaderRecords, shaderRecordSize, L"HitGroupShaderTable");
        hitGroupShaderTable.push_back(ShaderRecord(hitGroupShaderIdentifier, m_shaderIdentifierSize));
        m_hitGroupShaderTable = hitGroupShaderTable.GetResource();
    }
}

// Update frame-based values.
void D3D12RaytracingHelloWorld::OnUpdate()
{
    m_timer.Tick();
    CalculateFrameStats();

    float timeInMilli = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                               std::chrono::high_resolution_clock::now().time_since_epoch())
                                               .count());
    m_rayGenCB.timeNow = timeInMilli;
    UpdateCamera();
    // m_cam->RotateRight(std::sin(timeInMilli * .0001f));
    // m_rayGenCB.origin = XMFLOAT3{ std::sin(timeInMilli * .001f)*3.f, 0.f, 0.f };
    InitRayGenTable();
    /*   ReleaseWindowSizeDependentResources();
       CreateWindowSizeDependentResources();*/
}

void D3D12RaytracingHelloWorld::DoRaytracing()
{
    auto commandList = m_deviceResources->GetCommandList();

    auto DispatchRays = [&](auto* commandList, auto* stateObject, auto* dispatchDesc)
    {
        // Since each shader table has only one shader record, the stride is same as the size.
        dispatchDesc->HitGroupTable.StartAddress = m_hitGroupShaderTable->GetGPUVirtualAddress();
        dispatchDesc->HitGroupTable.SizeInBytes = m_hitGroupShaderTable->GetDesc().Width;
        dispatchDesc->HitGroupTable.StrideInBytes = dispatchDesc->HitGroupTable.SizeInBytes;
        dispatchDesc->MissShaderTable.StartAddress = m_missShaderTable->GetGPUVirtualAddress();
        dispatchDesc->MissShaderTable.SizeInBytes = m_missShaderTable->GetDesc().Width;
        dispatchDesc->MissShaderTable.StrideInBytes = dispatchDesc->MissShaderTable.SizeInBytes;
        dispatchDesc->RayGenerationShaderRecord.StartAddress = m_rayGenShaderTable->GetGPUVirtualAddress();
        dispatchDesc->RayGenerationShaderRecord.SizeInBytes = m_rayGenShaderTable->GetDesc().Width;
        dispatchDesc->Width = m_width;
        dispatchDesc->Height = m_height;
        dispatchDesc->Depth = 1;
        commandList->SetPipelineState1(stateObject);
        commandList->DispatchRays(dispatchDesc);
    };

    commandList->SetComputeRootSignature(m_raytracingGlobalRootSignature.Get());

    // Bind the heaps, acceleration structure and dispatch rays.
    D3D12_DISPATCH_RAYS_DESC dispatchDesc = {};
    commandList->SetDescriptorHeaps(1, m_descriptorHeap.GetAddressOf());
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::VertexBuffers,
                                               m_indexBuffer.gpuDescriptorHandle);
    commandList->SetComputeRootDescriptorTable(GlobalRootSignatureParams::OutputViewSlot,
                                               m_raytracingOutputResourceUAVGpuDescriptor);
    commandList->SetComputeRootShaderResourceView(GlobalRootSignatureParams::AccelerationStructureSlot,
                                                  m_topLevelAccelerationStructure->GetGPUVirtualAddress());
    DispatchRays(m_dxrCommandList.Get(), m_dxrStateObject.Get(), &dispatchDesc);
}

// Update the application state with the new resolution.
void D3D12RaytracingHelloWorld::UpdateForSizeChange(UINT width, UINT height)
{
    DXSample::UpdateForSizeChange(width, height);

    float border = 0.1f;
    if (m_width <= m_height)
    {
        m_rayGenCB.stencil = { -1 + border, -1 + border * m_aspectRatio, 1.0f - border, 1 - border * m_aspectRatio };
    }
    else
    {
        m_rayGenCB.stencil = { -1 + border / m_aspectRatio, -1 + border, 1 - border / m_aspectRatio, 1.0f - border };
    }
    UpdateCamera();
}

// Copy the raytracing output to the backbuffer.
void D3D12RaytracingHelloWorld::CopyRaytracingOutputToBackbuffer()
{
    auto commandList = m_deviceResources->GetCommandList();
    auto renderTarget = m_deviceResources->GetRenderTarget();

    D3D12_RESOURCE_BARRIER preCopyBarriers[2];
    preCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_RENDER_TARGET,
                                                              D3D12_RESOURCE_STATE_COPY_DEST);
    preCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(
        m_raytracingOutput.Get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE);
    commandList->ResourceBarrier(ARRAYSIZE(preCopyBarriers), preCopyBarriers);

    commandList->CopyResource(renderTarget, m_raytracingOutput.Get());

    D3D12_RESOURCE_BARRIER postCopyBarriers[2];
    postCopyBarriers[0] = CD3DX12_RESOURCE_BARRIER::Transition(renderTarget, D3D12_RESOURCE_STATE_COPY_DEST,
                                                               D3D12_RESOURCE_STATE_PRESENT);
    postCopyBarriers[1] = CD3DX12_RESOURCE_BARRIER::Transition(
        m_raytracingOutput.Get(), D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    commandList->ResourceBarrier(ARRAYSIZE(postCopyBarriers), postCopyBarriers);
}

// Create resources that are dependent on the size of the main window.
void D3D12RaytracingHelloWorld::CreateWindowSizeDependentResources()
{
    CreateRaytracingOutputResource();

    // For simplicity, we will rebuild the shader tables.
    BuildShaderTables();
}

// Release resources that are dependent on the size of the main window.
void D3D12RaytracingHelloWorld::ReleaseWindowSizeDependentResources()
{
    m_rayGenShaderTable.Reset();
    m_missShaderTable.Reset();
    m_hitGroupShaderTable.Reset();
    m_raytracingOutput.Reset();
}

// Release all resources that depend on the device.
void D3D12RaytracingHelloWorld::ReleaseDeviceDependentResources()
{
    m_raytracingGlobalRootSignature.Reset();
    m_raytracingLocalRootSignature.Reset();

    m_dxrDevice.Reset();
    m_dxrCommandList.Reset();
    m_dxrStateObject.Reset();

    m_descriptorHeap.Reset();
    m_descriptorsAllocated = 0;
    m_raytracingOutputResourceUAVDescriptorHeapIndex = UINT_MAX;
    m_indexBuffer.resource.Reset();
    m_vertexBuffer.resource.Reset();

    m_accelerationStructure.Reset();
    m_bottomLevelAccelerationStructure.Reset();
    m_topLevelAccelerationStructure.Reset();
}

void D3D12RaytracingHelloWorld::RecreateD3D()
{
    // Give GPU a chance to finish its execution in progress.
    try
    {
        m_deviceResources->WaitForGpu();
    }
    catch (HrException&)
    {
        // Do nothing, currently attached adapter is unresponsive.
    }
    m_deviceResources->HandleDeviceLost();
}

// Render the scene.
void D3D12RaytracingHelloWorld::OnRender()
{
    if (!m_deviceResources->IsWindowVisible())
    {
        return;
    }

    m_deviceResources->Prepare();
    DoRaytracing();
    CopyRaytracingOutputToBackbuffer();

    m_deviceResources->Present(D3D12_RESOURCE_STATE_PRESENT);
}

void D3D12RaytracingHelloWorld::OnDestroy()
{
    // Let GPU finish before releasing D3D resources.
    m_deviceResources->WaitForGpu();
    OnDeviceLost();
}

// Release all device dependent resouces when a device is lost.
void D3D12RaytracingHelloWorld::OnDeviceLost()
{
    ReleaseWindowSizeDependentResources();
    ReleaseDeviceDependentResources();
}

// Create all device dependent resources when a device is restored.
void D3D12RaytracingHelloWorld::OnDeviceRestored()
{
    CreateDeviceDependentResources();
    CreateWindowSizeDependentResources();
}

// Compute the average frames per second and million rays per second.
void D3D12RaytracingHelloWorld::CalculateFrameStats()
{
    static int frameCnt = 0;
    static double elapsedTime = 0.0f;
    double totalTime = m_timer.GetTotalSeconds();
    frameCnt++;

    // Compute averages over one second period.
    if ((totalTime - elapsedTime) >= 1.0f)
    {
        float diff = static_cast<float>(totalTime - elapsedTime);
        float fps = static_cast<float>(frameCnt) / diff; // Normalize to an exact second.

        frameCnt = 0;
        elapsedTime = totalTime;

        float MRaysPerSecond = (m_width * m_height * fps) / static_cast<float>(1e6);

        wstringstream windowText;

        windowText << setprecision(2) << fixed << L"    fps: " << fps << L"     ~Million Primary Rays/s: "
                   << MRaysPerSecond << L"    GPU[" << m_deviceResources->GetAdapterID() << L"]: "
                   << m_deviceResources->GetAdapterDescription();
        SetCustomWindowText(windowText.str().c_str());
    }
}

// Handle OnSizeChanged message event.
void D3D12RaytracingHelloWorld::OnSizeChanged(UINT width, UINT height, bool minimized)
{
    if (!m_deviceResources->WindowSizeChanged(width, height, minimized))
    {
        return;
    }

    UpdateForSizeChange(width, height);

    ReleaseWindowSizeDependentResources();
    CreateWindowSizeDependentResources();
}

// Allocate a descriptor and return its index.
// If the passed descriptorIndexToUse is valid, it will be used instead of allocating a new one.
UINT D3D12RaytracingHelloWorld::AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor,
                                                   UINT descriptorIndexToUse)
{
    auto descriptorHeapCpuBase = m_descriptorHeap->GetCPUDescriptorHandleForHeapStart();
    if (descriptorIndexToUse >= m_descriptorHeap->GetDesc().NumDescriptors)
    {
        descriptorIndexToUse = m_descriptorsAllocated++;
    }
    *cpuDescriptor = CD3DX12_CPU_DESCRIPTOR_HANDLE(descriptorHeapCpuBase, descriptorIndexToUse, m_descriptorSize);
    return descriptorIndexToUse;
}

void D3D12RaytracingHelloWorld::OnMouseMove(UINT x, UINT y)
{
    static float lastX = (float)x;
    static float lastY = (float)y;
    float deltaX = x - lastX;
    float deltaY = y - lastY;
    lastX = (float)x;
    lastY = (float)y;

    if (mouseRotateMode)
    {
        m_cam->RotateRight(deltaX);
        m_cam->RotateUp(deltaY);
    }
}

void D3D12RaytracingHelloWorld::OnKeyDown(UINT8 key)
{
    switch (key)
    {
    case 'W':
    case 'w':
    {
        m_cam->MoveForward(1.f);
    }
    break;
    case 'S':
    case 's':
    {
        m_cam->MoveForward(-1.f);
    }
    break;
    case 'D':
    case 'd':
    {
        m_cam->MoveRight(1.f);
    }
    break;
    case 'A':
    case 'a':
    {
        m_cam->MoveRight(-1.f);
    }
    break;
    case VK_CONTROL:
        mouseRotateMode = true;
        break;
    default:
        break;
    }
}

void D3D12RaytracingHelloWorld::OnKeyUp(UINT8 key)
{
    switch (key)
    {
    case VK_CONTROL:
        mouseRotateMode = false;
        break;
    default:
        break;
    }
}

void D3D12RaytracingHelloWorld::InitializeScene()
{
    size_t sphereIdx = m_scene.LoadModel("SphereRad1.obj");
    DirectX::XMMATRIX mat = DirectX::XMMatrixIdentity();
    m_scene.AddInstance(sphereIdx, mat);
    mat = DirectX::XMMatrixScaling(.5f, .5f, .5f) * DirectX::XMMatrixTranslation(2.f, 0.f, 0.f);
    m_scene.AddInstance(sphereIdx, mat);
}