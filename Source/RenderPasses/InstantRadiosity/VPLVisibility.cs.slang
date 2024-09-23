#include "Scene/SceneDefines.slangh"
#include "Utils/Math/MathConstants.slangh"

import Rendering.Lights.EmissiveLightSamplerHelpers;
import Scene.Scene;
import Scene.RaytracingInline;
import Scene.Intersection;
import Utils.Geometry.GeometryHelpers;
import Utils.Math.MathHelpers;
import Utils.Sampling.SampleGenerator;
import Rendering.Lights.LightHelpers;
import VPLData;

// Inputs
StructuredBuffer<VPLData> gVPL;
ByteAddressBuffer gVPLCounter;

// Outputs
RWTexture2D<float> gVisibility;
RWTexture2D<float4> gPosW0;
RWTexture2D<float4> gPosW1;

// Static configurations
static const bool kRandomSampleVPL = RANDOM_SAMPLE_VPL;

static const bool kUseAlphaTest = true;

cbuffer CB
{
    uint2 frameDim = { 0, 0 };
    uint vplSampleSeed;
}

bool traceShadowRay(float3 origin, float3 dir, float distance, out float hitT)
{
    SceneRayQuery<kUseAlphaTest> sceneRayQuery;
    HitInfo hit;

    Ray ray = Ray(origin, dir, 1e-4f, distance - 1e-4f);

    return !sceneRayQuery.traceRay(ray, hit, hitT, RAY_FLAG_NONE, 0xff);
}

void execute(uint2 pixel)
{
    if (any(pixel >= frameDim))
    {
        return;
    }
    SampleGenerator sg = SampleGenerator(pixel, vplSampleSeed);

    const uint VPLCount = gVPLCounter.Load(0);

    uint vplIdx0 = pixel.x, vplIdx1 = pixel.y;
    int retry = 2;
    if (kRandomSampleVPL || (vplIdx1 >= vplIdx0 && VPLCount > frameDim.x))
    {
        // We don't want duplicated point pair,
        // map the lower triangle pair to random pairs
        vplIdx0 = uint(sampleNext1D(sg) * VPLCount);
        do
        {
            vplIdx1 = uint(sampleNext1D(sg) * VPLCount);
            retry--;
        } while (vplIdx0 == vplIdx1 && retry >= 0);
    }
    vplIdx0 = min(vplIdx0, VPLCount - 1u);
    vplIdx1 = min(vplIdx1, VPLCount - 1u);

    VPLData vpl0 = gVPL[vplIdx0];
    VPLData vpl1 = gVPL[vplIdx1];

    float3 origin = vpl0.position.xyz;
    float3 target = vpl1.position.xyz;
    float3 dir = target - origin;
    float len = length(dir);
    float hitT;

    bool vis = traceShadowRay(origin, dir / len, len, hitT);

    gVisibility[pixel] = vis ? 1.0: 0.0;
    gPosW0[pixel] = float4(origin, hitT);
    gPosW1[pixel] = float4(target, 1.0);
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId: SV_DispatchThreadID)
{
    uint2 pixel = dispatchThreadId.xy;
    execute(pixel);
}

