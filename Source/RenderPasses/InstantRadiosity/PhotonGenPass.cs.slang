#include "Scene/SceneDefines.slangh"
#include "Utils/Math/MathConstants.slangh"

import Scene.RaytracingInline;
import Scene.Intersection;
import VPLData;
import SharedTypes;
import Utils.Math.MathHelpers;
import Utils.Sampling.SampleGenerator;

struct PhotonGenerator
{
    static const int kUseAlphaTest = 1;
    RWStructuredBuffer<VPLData> photonData;
    RWByteAddressBuffer totalPhotonCounter;
    RadiosityParams params;

    bool addPhoton(const VPLData vpl)
    {
        uint index = 0;
        totalPhotonCounter.InterlockedAdd(0, 1, index);
        if (index < params.maxVPLCount)
        {
            photonData[index] = vpl;
            return true;
        }
        else
        {
            totalPhotonCounter.InterlockedAdd(0, -1, index);
            return false;
        }
    }

    /**
     * Run photon generator for a given instance.
     */
    void execute(const uint2 instanceID)
    {
        uint primitiveIndex = instanceID.x;
        uint sampleID = instanceID.y;

        if (instanceID.x >= gScene.lightCollection.triangleCount || sampleID > params.numVPLSamples)
            return;

        SampleGenerator sg = SampleGenerator(uint2(primitiveIndex, sampleID), 0);
        let lod = ExplicitLodTextureSampler(0.f);

        // Sample the emissive triangle
        const EmissiveTriangle tri = gScene.lightCollection.getTriangle(primitiveIndex);
        bool validSf;
        ShadingFrame sf = ShadingFrame.createSafe(tri.normal, float4(0.f, 0.f, 1.f, 1.f), validSf);

        float2 u = sampleNext2D(sg);
        float3 x = tri.getPosition(float3(u.xy, 1.f - u.x - u.y));
        float3 n = tri.normal;
        float pdf = 1.f / tri.area;
        float pdfW = 1.f;
        float3 radiant = gScene.lightCollection.getAverageRadiance(primitiveIndex);
        float3 kd = float3(1.0);

        for (uint i = 0; i < params.maxPathLength; i++)
        {
            // Cosine weighted sampling
            if (!addPhoton(VPLData(x, radiant / pdf, n)))
            {
                break;
            }

            float3 woLocal = sample_cosine_hemisphere_concentric(sampleNext2D(sg), pdfW);
            float3 wo = sf.fromLocal(woLocal);
            radiant *= kd * abs(dot(wo, n)) * M_1_PI / pdfW;

            SceneRayQuery<kUseAlphaTest> sceneRayQuery;
            HitInfo hit;
            float hitT;
            Ray ray = Ray(x, wo);
            if (sceneRayQuery.traceRay(ray, hit, hitT, RAY_FLAG_NONE, 0xff) && hit.getType() == HitType::Triangle)
            {
                const TriangleHit triangleHit = hit.getTriangleHit();
                uint materialID = gScene.getMaterialID(triangleHit.instanceID);
                VertexData v = gScene.getVertexData(triangleHit);
                x = v.posW;
                n = v.normalW;

                // Evaluate kd
                const ShadingData sd = gScene.materials.prepareShadingData(v, materialID, -ray.dir, lod);
                let mi = gScene.materials.getMaterialInstance(sd, lod);
                BSDFProperties props = mi.getProperties(sd);
                kd = props.diffuseReflectionAlbedo;
            }
            else
                break;
        }
    }
};

cbuffer CB
{
    PhotonGenerator gPhotonGenerator;
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId: SV_DispatchThreadID)
{
    gPhotonGenerator.execute(dispatchThreadId.xy);
}
