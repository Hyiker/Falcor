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
Texture2D<PackedHitInfo> gVBuffer;
Texture2D<float4> gViewW;

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
/**
 * Setup ShadingData based on loaded vertex/material attributes for a hit point.
 * @param[in] hit Hit information.
 * @param[in] rayOrigin Ray origin.
 * @param[in] rayDir Normalized ray direction.
 * @param[in] lod Method for computing texture level-of-detail.
 * @return ShadingData struct.
 */
ShadingData loadShadingData(const HitInfo hit, const float3 rayOrigin, const float3 rayDir, const ITextureSampler lod)
{
    VertexData v = {};
    uint materialID = {};

#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_TRIANGLE_MESH)
    if (hit.getType() == HitType::Triangle)
    {
        const TriangleHit triangleHit = hit.getTriangleHit();
        v = gScene.getVertexData(triangleHit);
        materialID = gScene.getMaterialID(triangleHit.instanceID);
    }
#endif
#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_DISPLACED_TRIANGLE_MESH)
    if (hit.getType() == HitType::DisplacedTriangle)
    {
        const DisplacedTriangleHit displacedTriangleHit = hit.getDisplacedTriangleHit();
        v = gScene.getVertexData(displacedTriangleHit, -rayDir);
        materialID = gScene.getMaterialID(displacedTriangleHit.instanceID);
    }
#endif
#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_CURVE)
    if (hit.getType() == HitType::Curve)
    {
        const CurveHit curveHit = hit.getCurveHit();
        v = gScene.getVertexDataFromCurve(curveHit);
        materialID = gScene.getMaterialID(curveHit.instanceID);
    }
#endif
#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_SDF_GRID)
    if (hit.getType() == HitType::SDFGrid)
    {
        const SDFGridHit sdfGridHit = hit.getSDFGridHit();
        v = gScene.getVertexDataFromSDFGrid(sdfGridHit, rayOrigin, rayDir);
        materialID = gScene.getMaterialID(sdfGridHit.instanceID);
    }
#endif

    ShadingData sd = gScene.materials.prepareShadingData(v, materialID, -rayDir, lod);

    return sd;
}

float3 evalDirectAnalytic(const ShadingData sd, const IMaterialInstance mi, inout SampleGenerator sg)
{
    const uint lightCount = gScene.getLightCount();
    if (lightCount == 0)
        return float3(0.f);

    // Pick one of the analytic light sources randomly with equal probability.
    const uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
    float invPdf = lightCount; // Light selection pdf = 1.0 / lightCount.

    // Sample local light source.
    AnalyticLightSample ls;
    if (!sampleLight(sd.posW, gScene.getLight(lightIndex), sg, ls))
        return float3(0.f);

    // Reject sample if not in the hemisphere of a BSDF lobe.
    const uint lobeTypes = mi.getLobeTypes(sd);
    const bool hasReflection = lobeTypes & uint(LobeType::Reflection);
    const bool hasTransmission = lobeTypes & uint(LobeType::Transmission);
    float NdotL = dot(sd.getOrientedFaceNormal(), ls.dir);
    if ((NdotL <= kMinCosTheta && !hasTransmission) || (NdotL >= -kMinCosTheta && !hasReflection))
        return float3(0.f);

    // Get origin with offset applied in direction of the geometry normal to avoid self-intersection.
    const float3 origin = computeRayOrigin(sd.posW, dot(sd.faceN, ls.dir) >= 0.f ? sd.faceN : -sd.faceN);

    // Test visibility by tracing a shadow ray.
    bool V = traceShadowRay(origin, ls.dir, ls.distance);
    if (!V)
        return float3(0.f);

    // Evaluate contribution.
    return mi.eval(sd, ls.dir, sg) * ls.Li * invPdf;
}

void execute(uint2 pixel)
{
    if (any(pixel > frameDim))
        return;

    SampleGenerator sg = SampleGenerator(pixel, frameCount);

    float3 viewW = -gViewW[pixel].xyz;
    const HitInfo hit = HitInfo(gVBuffer[pixel]);

    if (!hit.isValid())
    {
        gColor[pixel] = float4(gScene.envMap.eval(viewW), 1.0f);
        return;
    }

    float3 color = 0.f;

    const uint VPLCount = gVPLCounter.Load(0);
    const bool sampleAll = kVPLSamples == 0 || kVisualizeVPL;
    const uint sampleCount = sampleAll ? VPLCount : kVPLSamples;

    // Shading data
    const float3 primaryRayOrigin = gScene.camera.getPosition();
    const float3 primaryRayDir = viewW;

    let lod = ExplicitLodTextureSampler(0.f);

    // Load shading data.
    ShadingData sd = loadShadingData(hit, primaryRayOrigin, primaryRayDir, lod);

    // Create BSDF instance and query its properties.
    const IMaterialInstance mi = gScene.materials.getMaterialInstance(sd, lod);

    BSDFProperties bsdfProperties = mi.getProperties(sd);

    for (int i = 0; i < sampleCount; i++)
    {
        int idx = sampleAll ? i : min(uint(sampleNext1D(sg) * float(VPLCount)), VPLCount);

        // VPL illumination configs
        VPLData vpl = gVPL[idx];
        float3 toLight = vpl.position.xyz - sd.posW;
        float dist2 = dot(toLight, toLight);
        float dist = sqrt(dist2);

        float3 L = toLight / dist;
        float cosThetaI = dot(L, sd.getOrientedFaceNormal());

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

                float vis = traceShadowRay(sd.posW, L, dist) ? 1.0 : 0.0;

                float3 Li = vplFac * vpl.intensity.rgb / max(dist2, kClamping);

                float3 f = mi.eval(sd, L, sg);

                color += f * vis * Li * vpl.intensity.a;
            }
        }
    }

    if (kVisualizeVPL)
    {
        gColor[pixel] = float4(color + bsdfProperties.diffuseReflectionAlbedo * 0.03f, 1.0f);
    }
    else
    {
        float3 Lr = evalDirectAnalytic(sd, mi, sg);

        color /= float(sampleCount);
        gColor[pixel] = float4(color + bsdfProperties.emission + Lr, 1.0f);
    }
}

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId: SV_DispatchThreadID)
{
    uint2 pixel = dispatchThreadId.xy;
    execute(pixel);
}

