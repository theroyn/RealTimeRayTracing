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

#ifndef RAYTRACINGHLSLCOMPAT_H
#define RAYTRACINGHLSLCOMPAT_H
#ifdef HLSL
#include "HlslCompat.h"
#else
#include <DirectXMath.h>

// Shader will use byte encoding to access vertex indices.
using Index = UINT16;

using XMFLOAT4 = DirectX::XMFLOAT4;
using XMFLOAT3 = DirectX::XMFLOAT3;
using XMVECTOR = DirectX::XMVECTOR;
#endif
struct Viewport
{
    float left;
    float top;
    float right;
    float bottom;
};
// origin, leftCorner, vpHorizontal, vpVertical
struct RayGenConstantBuffer
{
    Viewport viewport;

    Viewport stencil;

    XMFLOAT3 origin;
    float timeNow;

    XMFLOAT3 leftCorner;
    float pad1;

    XMFLOAT3 vpHorizontal;
    float pad2;

    XMFLOAT3 vpVertical;
    //  float pad3;
    // XMFLOAT3 padding;
    float pad3[9];
};

struct Vertex
{
    XMFLOAT3 position;
    XMFLOAT3 normal;
};

#ifndef HLSL
//#pragma pack(16)
#endif

#endif // RAYTRACINGHLSLCOMPAT_H