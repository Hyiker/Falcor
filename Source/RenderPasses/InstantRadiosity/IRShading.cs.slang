#include "Scene/SceneDefines.slangh"
#include "Utils/Math/MathConstants.slangh"

import Rendering.Lights.EmissiveLightSamplerHelpers;
import Scene.Scene;
import Scene.RaytracingInline;
import Scene.Intersection;
import Utils.Geometry.GeometryHelpers;
import Utils.Math.MathHelpers;
import Utils.Sampling.SampleGenerator;
import VPLData;

// Inputs
StructuredBuffer<VPLData> gVPL;
ByteAddressBuffer gVPLCounter;
Texture2D<float4> gView;
Texture2D<float4> gPosW;
Texture2D<float4> gNormW;
Texture2D<float4> gEmissive;
Texture2D<float4> gDiffOpacity;
Texture2D<float4> gSpecRough;

// Outputs
RWTexture2D<float4> gColor;

// Static configurations
static const float kClamping = CLAMPING;
static const bool kVisualizeVPL = VISUALIZE_VPL;
static const uint kVPLSamples = VPL_SAMPLES;
static const bool kUseAlphaTest = true;

cbuffer CB
{
    uint2 frameDim = { 0, 0 };
    uint frameCount;
}

bool traceShadowRay(float3 origin, float3 dir, float distance)
{
    SceneRayQuery<kUseAlphaTest> sceneRayQuery;
    HitInfo hit;
    float hitT;

    Ray ray = Ray(origin, dir, 1e-4f, distance - 1e-4f);

    return !sceneRayQuery.traceRay(ray, hit, hitT, RAY_FLAG_NONE, 0xff);
}

void execute(uint2 pixel)
{
    if (any(pixel > frameDim))
        return;

    SampleGenerator sg = SampleGenerator(pixel, frameCount);

    float3 color = 0.f;
    float3 emissive = gEmissive[pixel].rgb;

    float3 view = gView[pixel].xyz;
    float3 posW = gPosW[pixel].xyz;
    float3 normW = gNormW[pixel].xyz;
    float3 diffuse = gDiffOpacity[pixel].rgb;
    float4 specRough = gSpecRough[pixel];

    float3 F0 = lerp(float3(0.04f, 0.04f, 0.04f), specRough.rgb, specRough.a);
    float roughness = specRough.w;

    const uint VPLCount = gVPLCounter.Load(0);
    const bool sampleAll = kVPLSamples == 0 || kVisualizeVPL;
    const uint sampleCount = sampleAll ? VPLCount : kVPLSamples;

    for (int i = 0; i < sampleCount; i++)
    {
        int idx = sampleAll ? i : min(uint(sampleNext1D(sg) * float(VPLCount)), VPLCount);
        VPLData vpl = gVPL[idx];

        float3 toLight = vpl.position.xyz - posW;
        float dist2 = dot(toLight, toLight);
        float dist = sqrt(dist2);

        float3 L = toLight / dist;
        float cosThetaI = dot(L, normW);

        if (kVisualizeVPL)
        {
            color += dist < 0.05 ? vpl.intensity.rgb : float3(0.0);
        }
        else
        {
            if (cosThetaI > 0.f)
            {
                // VPL factor
                float vplFac = max(0.0f, dot(vpl.normal.xyz, -L));

                float vis = traceShadowRay(posW, L, dist) ? 1.0 : 0.0;

                // Lambertian BRDF
                float3 diffuseContrib = diffuse * M_1_PI;

                float3 H = normalize(view + L);
                float cosThetaH = max(0.0f, dot(normW, H));

                float3 F = F0 + (1.0f - F0) * pow(1.0f - max(dot(H, view), 0.0f), 5.0f);

                float alpha = roughness * roughness;
                float alpha2 = alpha * alpha;
                float NDF = alpha2 / (M_PI * pow((cosThetaH * cosThetaH * (alpha2 - 1.0) + 1.0), 2.0));

                float k = (alpha + 1.0f) * (alpha + 1.0f) / 8.0f;
                float G_V = cosThetaI / (cosThetaI * (1.0f - k) + k);
                float G_L = dot(normW, view) / (dot(normW, view) * (1.0f - k) + k);
                float G = G_V * G_L;

                float3 specularContrib = (F * NDF * G) / (4.0f * max(dot(view, normW), 0.0f) * cosThetaI + 0.001f);

                float3 f = diffuseContrib + specularContrib;
                float3 Li = vplFac * vpl.intensity.rgb / max(dist2, kClamping);
                color += f * vis * Li * cosThetaI * vpl.intensity.a;
            }
        }
    }

    if (kVisualizeVPL)
    {
        gColor[pixel] = float4(color + diffuse * 0.03f, 1.0f);
    }
    else
    {
        color /= float(sampleCount);
        gColor[pixel] = float4(color + emissive, 1.0f);
    }
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId: SV_DispatchThreadID)
{
    uint2 pixel = dispatchThreadId.xy;
    execute(pixel);
}

