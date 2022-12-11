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
using XMFLOAT2 = DirectX::XMFLOAT2;
using XMVECTOR = DirectX::XMVECTOR;
#endif

struct Viewport
{
    float left;
    float top;
    float right;
    float bottom;
};

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
    float pad3[9];
};

struct Vertex
{
    XMFLOAT3 position;
    XMFLOAT3 normal;
};

namespace MaterialType
{
enum Type
{
    Default,
    Matte,  // Lambertian scattering
    Mirror, // Specular reflector that isn't modified by the Fresnel equations.
    AnalyticalCheckerboardTexture
};
}

struct PrimitiveMaterialBuffer
{
    XMFLOAT3 Ks;
    XMFLOAT3 Kr;
    XMFLOAT3 Kt;
    XMFLOAT3 albedo;
    XMFLOAT3 opacity;
    XMFLOAT3 eta;
    float fuzz;
    float refractionIndex;
    BOOL hasDiffuseTexture;
    BOOL hasNormalTexture;
    BOOL hasPerVertexTangents;
    MaterialType::Type type;
};

struct RayPayload
{
    XMFLOAT4 color;

    float timeVal;
    int currentRecursionDepth;
    XMFLOAT2 padding;
};

#endif // RAYTRACINGHLSLCOMPAT_H