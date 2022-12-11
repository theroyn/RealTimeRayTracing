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

#ifndef RAYTRACINGSHADERHELPER_H
#define RAYTRACINGSHADERHELPER_H

#include "RayTracingHlslCompat.h"

bool IsInsideViewport(float2 p, Viewport viewport)
{
    return true;
    return (p.x >= viewport.left && p.x <= viewport.right) && (p.y >= viewport.top && p.y <= viewport.bottom);
}

// Load three 16 bit indices.
static uint3 Load3x16BitIndices(uint offsetBytes, ByteAddressBuffer Indices)
{
    uint3 indices;

    // ByteAdressBuffer loads must be aligned at a 4 byte boundary.
    // Since we need to read three 16 bit indices: { 0, 1, 2 }
    // aligned at a 4 byte boundary as: { 0 1 } { 2 0 } { 1 2 } { 0 1 } ...
    // we will load 8 bytes (~ 4 indices { a b | c d }) to handle two possible index triplet layouts,
    // based on first index's offsetBytes being aligned at the 4 byte boundary or not:
    //  Aligned:     { 0 1 | 2 - }
    //  Not aligned: { - 0 | 1 2 }
    const uint dwordAlignedOffset = offsetBytes & ~3;
    const uint2 four16BitIndices = Indices.Load2(dwordAlignedOffset);

    // Aligned: { 0 1 | 2 - } => retrieve first three 16bit indices
    if (dwordAlignedOffset == offsetBytes)
    {
        indices.x = four16BitIndices.x & 0xffff;
        indices.y = (four16BitIndices.x >> 16) & 0xffff;
        indices.z = four16BitIndices.y & 0xffff;
    }
    else // Not aligned: { - 0 | 1 2 } => retrieve last three 16bit indices
    {
        indices.x = (four16BitIndices.x >> 16) & 0xffff;
        indices.y = four16BitIndices.y & 0xffff;
        indices.z = (four16BitIndices.y >> 16) & 0xffff;
    }

    return indices;
}


//////// DUDU REMOVE ALL THE DEAD RANDOM CODE WHEN SATISFIED WITH CURRENT CODE
#if 0

// exact glsl version
//#define mod(x, y) ((x) - (y)*floor((x) / (y)))
// should be faster
#define mod(x, y) ((x) % (y))

#define fract(x) ((x)-floor(x))

// Input: It uses texture coords as the random number seed.
// Input variables: p - [0, 1).
// Output: Random number: [0,1), that is between 0.0 and 0.999999... inclusive.
// Author: Michael Pohoreski
// Copyright: Copyleft 2012 :-)
float random(float2 p)
{
    // We need irrationals for pseudo randomness.
    // Most (all?) known transcendental numbers will (generally) work.
    const float2 r = float2(23.1406926327792690, // e^pi (Gelfond's constant)
                            2.6651441426902251); // 2^sqrt(2) (Gelfond–Schneider constant)
    return frac(cos(mod(123456789., 1e-7 + 256. * dot(p, r))) + 1.f);
}

float random(float2 p, float min, float max)
{
    return (random(p) * (max - min)) + min;
}

// Input variables: uv - [0, 1). timeVal - [0, 1] (sine of current time).
float2 rand2(float2 uv, float timeVal)
{
    float2 ins = (uv + timeVal) * .5f; // [0..1]
    float2 ins2 = 1.f - ins * ins;     // [0..1]

    return float2(random(ins), random(ins2));
}

#define KKKK 1103515245
// const uint k = 134775813U;   // Delphi and Turbo Pascal
// const uint k = 20170906U;    // Today's date (use three days ago's dateif you want a prime)
// const uint k = 1664525U;     // Numerical Recipes

float3 hash(uint3 x)
{
    x = ((x >> 8) ^ x.yzx) * KKKK;
    x = ((x >> 8) ^ x.yzx) * KKKK;
    x = ((x >> 8) ^ x.yzx) * KKKK;

    return float3(x) * (1.f / float(0xffffffff));
}

float3 hash3(uint2 pixelIndices, float timeNowMilli)
{
    return hash(uint3(pixelIndices, timeNowMilli));
}
float3 hash3(uint2 pixelIndices, float timeNowMilli, float min, float max)
{
    return hash3(pixelIndices, timeNowMilli) * (max - min) + min;
}

float pcg_hash(inout uint seed)
{
    uint state = seed * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    seed++;
    return float(((word >> 22u) ^ word) / 4294967295.f);
}

float3 newRand(float timeNow)
{
    uint seed = DispatchRaysIndex().x + DispatchRaysDimensions().x * DispatchRaysIndex().y + floor(timeNow);
    // Generate a few numbers...
    float r0 = pcg_hash(seed);
    float r1 = pcg_hash(seed);
    float r2 = pcg_hash(seed);

    return float3(r0, r1, r2);
}

float3 newRand(float timeNow, float min, float max)
{
    return newRand(timeNow) * (max - min) + min;
}

// Input variables: uv - [0, 1). timeVal - [0, 1] (sine of current time).
float3 rand3(float2 uv, float timeVal)
{
    float2 ins = (uv + timeVal) * .5f; // [0..1]
    float2 ins2 = 1 - ins * ins;       // [0..1]
    float2 ins3 = 1 - ins2 * ins2;     // [0..1]

    return float3(random(ins), random(ins2), random(ins3));
}

// Input variables: uv - [0, 1). timeVal - [0, 1] (sine of current time).
float3 rand3(float2 uv, float timeVal, float min, float max)
{
    float2 ins = (uv + timeVal) * .5f; // [0..1]
    float2 ins2 = 1 - ins * ins;       // [0..1]
    float2 ins3 = 1 - ins2 * ins2;     // [0..1]

    return float3(random(ins, min, max), random(ins2, min, max), random(ins3, min, max));
}
#endif

#define lengthSquared(vec) dot((vec), (vec))
#define fract frac
// phi = Golden Ratio
#define PHI 1.61803398874989484820459f

float gold_noise(in float2 xy, in float seed)
{
    return fract(tan(distance(xy * PHI, xy) * seed) * xy.x);
}
float2 gold2(in float2 xy, in float seed)
{
    return float2(gold_noise(DispatchRaysIndex().xy, seed + 0.1f), gold_noise(DispatchRaysIndex().xy, seed + 0.2f));
}
float3 gold3(in float2 xy, in float seed)
{
    return float3(gold_noise(DispatchRaysIndex().xy, seed + 0.1f), gold_noise(DispatchRaysIndex().xy, seed + 0.2f),
                  gold_noise(DispatchRaysIndex().xy, seed + 0.3f));
}

float3 gold3(in float2 xy, in float seed, float min, float max)
{
    return gold3(xy, seed) * (max - min) + min;
}

float3 randomInUnitSphere(float2 uv, float timeVal)
{
    while (true)
    {
        float3 p = gold3(uv, timeVal, -1.f, 1.f);

        if (lengthSquared(p) >= 1.f)
            continue;
        return p;
    }
}

float3 randomInUnitVector(float2 uv, float timeVal)
{
    return normalize(randomInUnitSphere(uv, timeVal));
}

float3 randomInHemisphere(const float3 normal, float2 uv, float timeVal)
{
    float3 inUnitSphere = randomInUnitSphere(uv, timeVal);
    if (dot(inUnitSphere, normal) > 0.f) // In the same hemisphere as the normal
        return inUnitSphere;
    else
        return -inUnitSphere;
}
#endif // RAYTRACINGSHADERHELPER_H