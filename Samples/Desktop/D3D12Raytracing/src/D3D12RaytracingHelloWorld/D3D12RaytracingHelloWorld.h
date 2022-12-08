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

#pragma once

#include "Camera.h"
#include "DXSample.h"
#include "RaytracingHlslCompat.h"
#include "StepTimer.h"
#include "Model.h"
#include "Scene.h"

namespace GlobalRootSignatureParams
{
enum Value
{
    OutputViewSlot = 0,
    AccelerationStructureSlot,
    VertexBuffers,
    Count
};
}

namespace LocalRootSignatureParams
{
enum Value
{
    ViewportConstantSlot = 0,
    Count
};
}
class ShaderTable;
class D3D12RaytracingHelloWorld : public DXSample
{
public:
    D3D12RaytracingHelloWorld(UINT width, UINT height, std::wstring name);

    // IDeviceNotify
    virtual void OnDeviceLost() override;
    virtual void OnDeviceRestored() override;

    // Messages
    virtual void OnInit() override;
    virtual void OnUpdate() override;
    virtual void OnRender() override;
    virtual void OnSizeChanged(UINT width, UINT height, bool minimized) override;
    virtual void OnDestroy() override;
    virtual void OnMouseMove(UINT x, UINT y) override;
    virtual void OnKeyDown(UINT8 key) override;
    virtual void OnKeyUp(UINT8 key) override;
    virtual IDXGISwapChain* GetSwapchain() override
    {
        return m_deviceResources->GetSwapChain();
    }

private:
    static const UINT FrameCount = 3;

    // DirectX Raytracing (DXR) attributes
    ComPtr<ID3D12Device5> m_dxrDevice;
    ComPtr<ID3D12GraphicsCommandList4> m_dxrCommandList;
    ComPtr<ID3D12StateObject> m_dxrStateObject;

    // Root signatures
    ComPtr<ID3D12RootSignature> m_raytracingGlobalRootSignature;
    ComPtr<ID3D12RootSignature> m_raytracingLocalRootSignature;

    // Descriptors
    ComPtr<ID3D12DescriptorHeap> m_descriptorHeap;
    UINT m_descriptorsAllocated;
    UINT m_descriptorSize;

    // Raytracing scene
    RayGenConstantBuffer m_rayGenCB;

    // Geometry
    D3DBuffer m_indexBuffer;
    D3DBuffer m_vertexBuffer;

    // Acceleration structure
    ComPtr<ID3D12Resource> m_accelerationStructure;
    ComPtr<ID3D12Resource> m_bottomLevelAccelerationStructure;
    ComPtr<ID3D12Resource> m_topLevelAccelerationStructure;

    // Raytracing output
    ComPtr<ID3D12Resource> m_raytracingOutput;
    D3D12_GPU_DESCRIPTOR_HANDLE m_raytracingOutputResourceUAVGpuDescriptor;
    UINT m_raytracingOutputResourceUAVDescriptorHeapIndex;

    // Shader tables
    static const wchar_t* c_hitGroupName;
    static const wchar_t* c_raygenShaderName;
    static const wchar_t* c_closestHitShaderName;
    static const wchar_t* c_missShaderName;
    ComPtr<ID3D12Resource> m_missShaderTable;
    ComPtr<ID3D12Resource> m_hitGroupShaderTable;
    ComPtr<ID3D12Resource> m_rayGenShaderTable;
    std::shared_ptr<ShaderTable> m_actualRayGenShaderTable;
    std::shared_ptr<Camera> m_cam;
    bool mouseRotateMode = false;
    void* m_rayGenShaderIdentifier;
    UINT m_shaderIdentifierSize;
    void InitRayGenTable();
    void UpdateCamera();
    void InitializeCamera();
    // Application state
    StepTimer m_timer;
    Scene m_scene;

    void RecreateD3D();
    void DoRaytracing();
    void CreateDeviceDependentResources();
    void CreateWindowSizeDependentResources();
    void ReleaseDeviceDependentResources();
    void ReleaseWindowSizeDependentResources();
    void CreateRaytracingInterfaces();
    void SerializeAndCreateRaytracingRootSignature(D3D12_ROOT_SIGNATURE_DESC& desc,
                                                   ComPtr<ID3D12RootSignature>* rootSig);
    void CreateRootSignatures();
    void CreateLocalRootSignatureSubobjects(CD3DX12_STATE_OBJECT_DESC* raytracingPipeline);
    void CreateRaytracingPipelineStateObject();
    void CreateDescriptorHeap();
    void CreateRaytracingOutputResource();
    void BuildGeometry();
    void BuildAccelerationStructures();
    void BuildShaderTables();
    void UpdateForSizeChange(UINT clientWidth, UINT clientHeight);
    void CopyRaytracingOutputToBackbuffer();
    void CalculateFrameStats();
    UINT AllocateDescriptor(D3D12_CPU_DESCRIPTOR_HANDLE* cpuDescriptor, UINT descriptorIndexToUse = UINT_MAX);
    UINT CreateBufferSRV(D3DBuffer* buffer, UINT numElements, UINT elementSize);
    void InitializeScene();
};
