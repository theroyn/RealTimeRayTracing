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
    float3 rayDir;
    float3 origin;
    {
        float3 lookfrom = g_rayGenCB.origin.xyz;
        // float3 lookfrom = float3(0., 0., 1.);
        /*   lookfrom.y = 0.f;
           lookfrom.z = 0.f;*/
        float3 lookat = float3(0., 0., -1.);

        float3 forward = normalize(lookat - lookfrom);
        float3 vup = float3(0., 1., 0.);

        float3 right = normalize(cross(forward, vup));
        float3 up = normalize(cross(right, forward));

        float aspectRatio = dims.x / dims.y;
        float vpHeight = 2.f;
        float vpWidth = aspectRatio * vpHeight;

        float3 vpHorizontal = vpWidth * right;
        float3 vpVertical = vpHeight * up;

        float3 leftCorner = lookfrom + 1.f * forward - 0.5f * vpHorizontal - 0.5 * vpVertical;

        float3 currentPixel = g_rayGenCB.leftCorner.xyz + lerpValues.x * g_rayGenCB.vpHorizontal.xyz +
                              lerpValues.y * g_rayGenCB.vpVertical.xyz;
        rayDir = normalize(currentPixel - lookfrom);
        origin = lookfrom;
    }
    /* float3 currentPixel =
         g_rayGenCB.leftCorner + lerpValues.x * g_rayGenCB.vpHorizontal + lerpValues.y * g_rayGenCB.vpVertical;
     rayDir = normalize(currentPixel - g_rayGenCB.origin);
     origin = g_rayGenCB.origin;*/

    // Orthographic projection since we're raytracing in screen space.
    // float3 rayDir = float3(0, 0, -1);
    // float3 origin = float3(lerp(g_rayGenCB.viewport.left, g_rayGenCB.viewport.right, lerpValues.x) /** + timeVal*/,
    //    lerp(g_rayGenCB.viewport.top, g_rayGenCB.viewport.bottom, lerpValues.y),
    //    0.0f);

    /*  if (IsInsideViewport(origin.xy, g_rayGenCB.stencil))
      {*/
    // Trace the ray.
    // Set the ray's extents.
    RayDesc ray;
    ray.Origin = origin;
    ray.Direction = rayDir;
    // Set TMin to a non-zero small value to avoid aliasing issues due to floating - point errors.
    // TMin should be kept small to prevent missing geometry at close contact areas.
    ray.TMin = 0.001;
    ray.TMax = 10000.0;
    RayPayload payload = { float4(0, 0, 0, 0) };
    TraceRay(Scene, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);

    // Write the raytraced color to the output texture.
    // RenderTarget[DispatchRaysIndex().xy] = float4(timeVal, 0.f,0.f,1.f);
    // payload.color.x *= timeVal;
    // RenderTarget[DispatchRaysIndex().xy] = float4(g_rayGenCB.origin, 1.f);
    RenderTarget[DispatchRaysIndex().xy] = payload.color;
    /*       if (DispatchRaysIndex().x > 1270)
           {
               RenderTarget[DispatchRaysIndex().xy] = float4(1., 0., 0., 1.);
           }
           else
           {
               RenderTarget[DispatchRaysIndex().xy] = float4(0., 1., 0., 1.);
           }
           if (DispatchRaysIndex().y > 710)
           {
               RenderTarget[DispatchRaysIndex().xy] = float4(0., 0., 1., 1.);
           }*/
    /*   else
       {
           RenderTarget[DispatchRaysIndex().xy] = float4(0., 1., 0., 1.);
       }*/
    //}
    // else
    //{
    //    // Render interpolated DispatchRaysIndex outside the stencil window
    //    RenderTarget[DispatchRaysIndex().xy] = float4(lerpValues, 0, 1);
    //}
}

    [shader("closesthit")] void MyClosestHitShader(inout RayPayload payload, in MyAttributes attr)
{
    float3 barycentrics =
        float3(1 - attr.barycentrics.x - attr.barycentrics.y, attr.barycentrics.x, attr.barycentrics.y);
    payload.color = float4(barycentrics, 1);
}

[shader("miss")] void MyMissShader(inout RayPayload payload) { payload.color = float4(0, 0, 0, 1); }

#endif // RAYTRACING_HLSL