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

RaytracingAccelerationStructure Scene : register(t0, space0);
RWTexture2D<float4> RenderTarget : register(u0);
ConstantBuffer<RayGenConstantBuffer> g_rayGenCB : register(b0);

typedef BuiltInTriangleIntersectionAttributes MyAttributes;
struct RayPayload
{
    float4 color;
};

bool IsInsideViewport(float2 p, Viewport viewport)
{
    return true;
    return (p.x >= viewport.left && p.x <= viewport.right) && (p.y >= viewport.top && p.y <= viewport.bottom);
}

[shader("raygeneration")] void MyRaygenShader()
{
    float timeVal = sin(g_rayGenCB.timeNow * .01f) * .5f + .5f;
    float2 dims = DispatchRaysDimensions().xy;
    float2 lerpValues = (float2)DispatchRaysIndex() / dims;

    float3 currentPixel = g_rayGenCB.leftCorner.xyz + lerpValues.x * g_rayGenCB.vpHorizontal.xyz +
                          lerpValues.y * g_rayGenCB.vpVertical.xyz;

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
    float3 barycentrics =
        float3(1 - attr.barycentrics.x - attr.barycentrics.y, attr.barycentrics.x, attr.barycentrics.y);
    payload.color = float4(barycentrics, 1);
}

[shader("miss")] void MyMissShader(inout RayPayload payload) { payload.color = float4(0, 0, 0, 1); }

#endif // RAYTRACING_HLSL