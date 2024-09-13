#include "Scene/SceneDefines.slangh"
#include "Utils/Math/MathConstants.slangh"

import Rendering.Lights.EmissiveLightSamplerHelpers;
import Scene.Scene;
import Scene.RaytracingInline;
import Scene.Intersection;
import Utils.Geometry.GeometryHelpers;
import Utils.Math.MathHelpers;
import VPLData;

// Inputs
RWStructuredBuffer<VPLData> gVPL;
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
static const bool kUseAlphaTest = true;

bool traceShadowRay(float3 origin, float3 dir, float distance)
{
    SceneRayQuery<kUseAlphaTest> sceneRayQuery;
    HitInfo hit;
    float hitT;

    Ray ray = Ray(origin, dir, 1e-4f, distance);

    return !sceneRayQuery.traceRay(ray, hit, hitT, RAY_FLAG_NONE, 0xff);
}

void execute(uint2 pixel)
{
    float3 color = 0.f;
    float3 emissive = gEmissive[pixel].rgb;

    float3 view = gView[pixel].xyz;
    float3 posW = gPosW[pixel].xyz;
    float3 normW = gNormW[pixel].xyz;
    float3 diffuse = gDiffOpacity[pixel].rgb;
    float4 specRough = gSpecRough[pixel];

    float3 F0 = lerp(float3(0.04f, 0.04f, 0.04f), specRough.rgb, specRough.a);
    float roughness = specRough.w;

    const uint VPLCount = 200;

    [[unroll]]
    for (int i = 0; i < VPLCount; i++)
    {
        VPLData vpl = gVPL[i];

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

                float vis = traceShadowRay(computeRayOrigin(posW, normW), L, dist - 1e-4) ? 1.0 : 0.0;

                // Lambertian BRDF
                float3 diffuseContrib = diffuse * cosThetaI * vplFac * vpl.intensity.rgb / dist2;

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

                color += (diffuseContrib + specularContrib) * vis;
            }
        }
    }

    if (kVisualizeVPL)
    {
        gColor[pixel] = float4(color + diffuse * 0.03f, 1.0f);
    }
    else
    {
        color /= float(VPLCount);
        gColor[pixel] = float4(color + emissive, 1.0f);
    }
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId: SV_DispatchThreadID)
{
    uint2 pixel = dispatchThreadId.xy;
    execute(pixel);
}

