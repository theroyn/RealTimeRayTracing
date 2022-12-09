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

#ifndef RAYTRACING_HLSL
#define RAYTRACING_HLSL

#define HLSL
#include "RaytracingHlslCompat.h"
#include "RaytracingShaderHelper.hlsli"

RaytracingAccelerationStructure Scene : register(t0, space0);
RWTexture2D<float4> RenderTarget : register(u0);
ConstantBuffer<RayGenConstantBuffer> g_rayGenCB : register(b0);

// Triangle resources
ByteAddressBuffer g_indices : register(t1, space0);
StructuredBuffer<Vertex> g_vertices : register(t2, space0);
StructuredBuffer<PrimitiveMaterialBuffer> g_materials : register(t3);

typedef BuiltInTriangleIntersectionAttributes MyAttributes;

struct RayPayload
{
    float4 color;
};

float3 GetNormal()
{
    uint indexSizeInBytes = 2;
    uint indicesPerTriangle = 3;
    uint triangleIndexStride = indicesPerTriangle * indexSizeInBytes;
    uint baseIndex = PrimitiveIndex() * triangleIndexStride;

    // Load up three 16 bit indices for the triangle.
    const uint3 indices = Load3x16BitIndices(baseIndex, g_indices);

    // Retrieve corresponding vertex normals for the triangle vertices.
    float3 triangleNormal = g_vertices[indices[0]].normal;

    return triangleNormal;
}

[shader("raygeneration")] void MyRaygenShader()
{
    float timeVal = sin(g_rayGenCB.timeNow * .01f) * .5f + .5f;
    float2 dims = DispatchRaysDimensions().xy;
    float2 lerpValues = (float2)DispatchRaysIndex() / dims;

    float3 currentPixel = g_rayGenCB.leftCorner.xyz + lerpValues.x * g_rayGenCB.vpHorizontal.xyz +
                          (1.f - lerpValues.y) * g_rayGenCB.vpVertical.xyz;

    // Trace the ray.
    // Set the ray's extents.
    RayDesc ray;
    ray.Origin = g_rayGenCB.origin;
    ray.Direction = normalize(currentPixel - g_rayGenCB.origin);
    // Set TMin to a non-zero small value to avoid aliasing issues due to floating - point errors.
    // TMin should be kept small to prevent missing geometry at close contact areas.
    ray.TMin = 0.001;
    ray.TMax = 10000.0;

    RayPayload payload = { float4(0, 0, 0, 0) };

    TraceRay(Scene, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);

    // Write the raytraced color to the output texture.
    RenderTarget[DispatchRaysIndex().xy] = payload.color;
}

    [shader("closesthit")] void MyClosestHitShader(inout RayPayload payload, in MyAttributes attr)
{
    float4 lightPos = float4(4.f, 10.f, -1.f, 1.f);
    float3 lightDiffuse = float3(.4f, .4f, .4f);
    float3 lightAmbience = float3(.05f, .05f, .05f);
    float3 lightSpecular = float3(.5f, .5f, .5f);
    float3 lightColor = float3(1.f, 1.f, 1.f);
    uint instanceID = InstanceID();
    float objectShinines = 8.f;
    PrimitiveMaterialBuffer material = g_materials[instanceID];
    float3 objectColor = material.albedo;

    // assuming object is centered around the origin
    float3 worldCenter = mul(float4(0.f,0.f,0.f,1.f), ObjectToWorld4x3()).xyz;
    float3 worldPos = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
    float3 realNormal = normalize(worldPos - worldCenter);
    float3 outColor;

    // Retrieve corresponding vertex normals for the triangle vertices.
    float3 triangleNormal = GetNormal();
    float3 barycentrics =
        float3(1 - attr.barycentrics.x - attr.barycentrics.y, attr.barycentrics.x, attr.barycentrics.y);
    float3 currentNormal = barycentrics * triangleNormal;
    float3 objectToLight = normalize(lightPos.xyz - worldPos);

    float diffuse = max(0.f, dot(objectToLight, realNormal));
    float3 reflected = reflect(WorldRayDirection(), realNormal);
    float specular = pow(objectShinines, max(0.f, (dot(reflected, objectToLight))));

    outColor = (lightAmbience + specular * lightSpecular + diffuse * lightDiffuse) * lightColor * objectColor;
    payload.color = float4(outColor, 1);
}

[shader("miss")] void MyMissShader(inout RayPayload payload) { payload.color = float4(0, 0, 0, 1); }

#endif // RAYTRACING_HLSL