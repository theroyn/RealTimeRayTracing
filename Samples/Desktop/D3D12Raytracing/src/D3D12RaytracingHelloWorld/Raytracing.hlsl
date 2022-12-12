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

[shader("raygeneration")] void MyRaygenShader()
{
    float timeVal = sin(g_rayGenCB.timeNow * .01f) * .5f + .5f;
    float2 dims = DispatchRaysDimensions().xy;
    float2 uv = (float2)DispatchRaysIndex() / dims;
    int samples_per_pixel = 10;
    float3 color = float3(0.f, 0.f, 0.f);
    for (int s = 0; s < samples_per_pixel; ++s)
    {
        RayPayload payload;
        float stepPart = (float)s / (float)samples_per_pixel;
        float2 step = gold2(DispatchRaysDimensions().xy, fract(g_rayGenCB.timeNow) + stepPart);
        float2 lerpValues = (DispatchRaysIndex().xy + step) / (dims - 1.f);

        float3 currentPixel = g_rayGenCB.leftCorner.xyz + lerpValues.x * g_rayGenCB.vpHorizontal +
                              (1.f - lerpValues.y) * g_rayGenCB.vpVertical;

        payload.color = float4(0.f, 0.f, 0.f, 0.f);
        payload.timeVal = g_rayGenCB.timeNow + stepPart;
        payload.currentRecursionDepth = 0;

        RayDesc ray;
        ray.Origin = g_rayGenCB.origin;
        ray.Direction = normalize(currentPixel - g_rayGenCB.origin);
        // Set TMin to a non-zero small value to avoid aliasing issues due to floating - point errors.
        // TMin should be kept small to prevent missing geometry at close contact areas.
        ray.TMin = 0.001;
        ray.TMax = 10000.0;

        TraceRay(Scene, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, payload);

        color += payload.color.xyz;
    }

    float scale = scale = 1.f / (float)samples_per_pixel;

    color = sqrt(scale * color);

    RenderTarget[DispatchRaysIndex().xy] = float4(color, 1.f);
}

#define MAX_RECURSION 25

    [shader("closesthit")] void MyClosestHitShader(inout RayPayload payload, in MyAttributes attr)
{
    if (++payload.currentRecursionDepth == MAX_RECURSION)
    {
        payload.color = float4(0.f, 0.f, 0.f, 1.f);
        return;
    }

    // assuming object is a sphere centered around the origin
    float3 p = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();

    float3 pInObjectSpace = mul(WorldToObject3x4(), float4(p, 1.f)).xyz;

    float3 normal = normalize(pInObjectSpace);

    RayPayload currentPayload = payload;
    RayDesc ray;
    float3 attenuation = float3(0.f, 0.f, 0.f);
    PrimitiveMaterialBuffer material = g_materials[InstanceID()];
    bool scatter = false;
    if (material.type == MaterialType::Lambertian)
    {
        attenuation = material.albedo;
        float2 dims = DispatchRaysDimensions().xy;

        float2 uv = (float2)DispatchRaysIndex() / dims;

        // float3 target = randomInHemisphere(normal, DispatchRaysIndex().xy, fract(payload.timeVal));
        // Catch degenerate scatter direction
        float3 target = normal + randomInUnitVector(DispatchRaysIndex().xy, fract(payload.timeVal));
        if (nearZero(target))
        {
            target = normal;
        }

        ray.Origin = p;
        ray.Direction = normalize(target);
        scatter = true;
    }
    else if (material.type == MaterialType::Metal)
    {
        //         vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
        // scattered = ray(rec.p, reflected);
        attenuation = material.albedo;
        float3 reflected = reflect(WorldRayDirection(), normal);
        ray.Origin = p;
        ray.Direction = normalize(reflected);
        scatter = (dot(ray.Direction, normal) > 0);
    }

    if (scatter)
    {
        currentPayload.color = float4(0.f, 0.f, 0.f, 0.f);
        ray.TMin = 0.0001;
        ray.TMax = 10000.0;

        TraceRay(Scene, RAY_FLAG_CULL_BACK_FACING_TRIANGLES, ~0, 0, 1, 0, ray, currentPayload);

        payload.color = float4(attenuation, 1.f) * currentPayload.color;
    }
    else
    {
        payload.color = float4(0.f, 0.f, 0.f, 0.f);
    }
}

[shader("miss")] void MyMissShader(inout RayPayload payload)
{
    float3 unit_direction = WorldRayDirection();
    float t = 0.5 * (unit_direction.y + 1.0);
    float3 color = (1.f - t) * float3(1.f, 1.f, 1.f) + t * float3(.5f, .7f, 1.f);
    payload.color = float4(color, 1.f);
}

#endif // RAYTRACING_HLSL