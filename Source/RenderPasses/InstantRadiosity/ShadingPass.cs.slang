

#include "Scene/SceneDefines.slangh"
#include "Utils/Math/MathConstants.slangh"

import Scene.RaytracingInline;
import Scene.Intersection;
import Utils.Math.MathHelpers;
import Utils.Geometry.GeometryHelpers;
import Utils.Sampling.SampleGenerator;
import Rendering.Lights.LightHelpers;
import VPLData;

#define is_valid(name) (is_valid_##name != 0)

// Inputs
struct InstantRadiosityShader
{
    static const int kUseAlphaTest = 1;
    uint PRNGDimension; // First available PRNG dimension.
    uint2 frameDim;

    Texture2D<PackedHitInfo> vbuffer;
    Texture2D<float4> viewW;                // Optional view direction texture.
    RWStructuredBuffer<VPLData> photonData; // VPL data buffer.
    uint photonCount;

    // Outputs
    RWTexture2D<float4> output;

    float3 getPrimaryRayDir(uint2 launchIndex, uint2 launchDim, const Camera camera)
    {
        if (is_valid(viewW))
        {
            // If we have the view vector bound as a buffer, just fetch it. No need to compute anything.
            return -viewW[launchIndex].xyz;
        }
        else
        {
            // Compute the view vector. This must exactly match what the G-buffer pass is doing (jitter etc.).
            // Note that we do not take depth-of-field into account as it would require exactly matching the
            // sample generator between the passes, which is error prone. The host side will issue a warning instead.
            return camera.computeRayPinhole(launchIndex, launchDim).dir;
        }
    }

    bool traceShadowRay(float3 origin, float3 dir, float distance)
    {
        SceneRayQuery<kUseAlphaTest> sceneRayQuery;
        HitInfo hit;
        float hitT;

        Ray ray;
        ray.origin = origin;
        ray.dir = dir;
        ray.tMin = 0.001f;
        ray.tMax = distance - 0.001f;

        return sceneRayQuery.traceVisibilityRay(ray, RAY_FLAG_NONE, 0xff);
    }

    float3 evalDirectPhoton(const ShadingData sd, const IMaterialInstance mi, const VPLData vpl, inout SampleGenerator sg)
    {
        float3 L = vpl.getPosition() - sd.posW;
        float dist = length(L);
        L = normalize(L);
        float NdotL = dot(sd.getOrientedFaceNormal(), L);
        if (NdotL <= kMinCosTheta)
            return float3(0.f);

        // Get origin with offset applied in direction of the geometry normal to avoid self-intersection.
        const float3 origin = computeRayOrigin(sd.posW, dot(sd.faceN, L) >= 0.f ? sd.faceN : -sd.faceN);

        // Test visibility by tracing a shadow ray.
        bool V = traceShadowRay(origin, L, dist);
        if (!V)
            return float3(0.f);

        float3 irradiance = vpl.getRadiant() * rcp(dist * dist) * M_1_4PI;
        float3 cosThetaR = max(0.f, dot(vpl.getNormal(), -L));
        // Evaluate contribution.
        return mi.eval(sd, L, sg) * irradiance * cosThetaR;
    }

    void execute(uint2 pixel)
    {
        if (any(pixel >= frameDim))
            return;
        float3 outColor = float3(0.0);

        const HitInfo hit = HitInfo(vbuffer[pixel]);
        const float3 dir = getPrimaryRayDir(pixel, frameDim, gScene.camera);

        if (hit.isValid())
        {
            uint materialID = 0xffffffff;
            VertexData v = {};

            // Load hit data.
            switch (hit.getType())
            {
            case HitType::Triangle:
                const TriangleHit triangleHit = hit.getTriangleHit();
                materialID = gScene.getMaterialID(triangleHit.instanceID);
                v = gScene.getVertexData(triangleHit);
                break;
            }

            let lod = ExplicitLodTextureSampler(0.f);
            const ShadingData sd = gScene.materials.prepareShadingData(v, materialID, -dir, lod);
            let mi = gScene.materials.getMaterialInstance(sd, lod);
            outColor += mi.getProperties(sd).emission;

            SampleGenerator sg = SampleGenerator(pixel, 0);
            for (uint i = 0; i < photonCount; i++)
            {
                outColor += evalDirectPhoton(sd, mi, photonData[i], sg);
            }
        }
        output[pixel] = float4(outColor, 1.0f);
    }
};

cbuffer CB
{
    InstantRadiosityShader gIRShader;
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId: SV_DispatchThreadID)
{
    gIRShader.execute(dispatchThreadId.xy);
}
