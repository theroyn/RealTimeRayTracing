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
    return fract(cos(mod(123456789., 1e-7 + 256. * dot(p, r))));
}

// Input variables: uv - [0, 1). timeVal - [0, 1] (sine of current time).
float2 rand2(float2 uv, float timeVal)
{
    float2 ins = (uv + timeVal) * .5f; // [0..1]
    float2 ins2 = 1 - ins * ins;       // [0..1]

    return float2(random(ins), random(ins2));
}

// Input variables: uv - [0, 1). timeVal - [0, 1] (sine of current time).
float3 rand3(float2 uv, float timeVal)
{
    float2 ins = (uv + timeVal) * .5f; // [0..1]
    float2 ins2 = 1 - ins * ins;       // [0..1]
    float2 ins3 = 1 - ins2 * ins2;     // [0..1]

    return float3(random(ins), random(ins2), random(ins3));
}

#endif // RAYTRACINGSHADERHELPER_H