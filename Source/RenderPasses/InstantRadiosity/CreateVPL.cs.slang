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

// Outputs
RWStructuredBuffer<VPLData> gVPL;
RWTexture2D<float4> gOutput;
RWByteAddressBuffer gVPLCounter;

// Static configurations
static const uint kMaxPathDepth = MAX_PATH_DEPTH;
static const uint kMaxVPLCount = MAX_VPL_COUNT;
static const int kUseAlphaTest = 1;

struct LightSample
{
    float3 Li;    ///< Incident radiance, not divided by pdf.
    float3 posW;  ///< World space position.
    float3 normW; ///< World space normal.
};

bool sampleTriangle(const uint triangleIndex, const float2 u, out LightSample ls, out ShadingData sd)
{
    ls = {};

    const EmissiveTriangle tri = gScene.lightCollection.getTriangle(triangleIndex);

    // Prepare light sampler data
    // Sample the triangle uniformly.
    const float3 barycentrics = sample_triangle(u);
    ls.posW = tri.getPosition(barycentrics);

    // Get triangle normal.
    ls.normW = tri.normal;

    // Evaluate emitted radiance.
    float2 uv = tri.getTexCoord(barycentrics);
    float3 Li = gScene.materials.evalEmissive(tri.materialID, uv);

    float pdfInv = max(FLT_MIN, tri.area);

    ls.Li = Li * pdfInv;

    // Prepare shading data
    VertexData v = {};
    v.posW = ls.posW;
    v.normalW = ls.normW;
    // Here we compute a random direction
    v.tangentW = float4(0.0);
    v.texC = uv;
    v.faceNormalW = ls.normW;

    sd = gScene.materials.prepareShadingData(v, tri.materialID, ls.normW, ExplicitLodTextureSampler(0.f));

    return true;
}

bool sampleLight(inout SampleGenerator sg, out LightSample ls, out ShadingData sd)
{
    ls = {};
    sd = {};

    if (gScene.lightCollection.getActiveTriangleCount() == 0)
        return false;

    // Randomly pick a triangle out of the global list of active triangles with uniform probability.
    float uLight = sampleNext1D(sg);
    uint triangleCount = gScene.lightCollection.getActiveTriangleCount();
    uint idx = min((uint)(uLight * triangleCount), triangleCount - 1); // Safety precaution as the result of the multiplication may be
                                                                       // rounded to triangleCount even if uLight < 1.0 when triangleCount
                                                                       // is large.
    uint triangleIndex = gScene.lightCollection.activeTriangles[idx];

    // Sample the triangle uniformly.
    float2 u = sampleNext2D(sg);
    if (!sampleTriangle(triangleIndex, u, ls, sd))
        return false;

    ls.Li *= (float)triangleCount;
    return true;
}

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

bool addVPL(float3 posW, float3 normW, float3 Li)
{
    uint idx = 0;
    gVPLCounter.InterlockedAdd(0, 1, idx);
    if (idx >= kMaxVPLCount)
    {
        gVPLCounter.InterlockedAdd(0, -1, idx);
        return false;
    }

    VPLData vpl = {};
    // Point light illumination = intensity * NdotL / (r * r)
    // Area light sampled illumination:
    // Li = Le / pdf, pdf = (r * r) / (NdotL * A)
    vpl.intensity = float4(Li, float(kMaxPathDepth));
    vpl.normal.xyz = normW;
    vpl.position.xyz = posW;

    gVPL[idx] = vpl;
    return true;
}

/**
 * Payload for scatter ray.
 */
struct ScatterRayData
{
    float3 radiance;  ///< Accumulated outgoing radiance from path.
    bool terminated;  ///< Set to true when path is terminated.
    float3 thp;       ///< Current path throughput. This is updated at each path vertex.
    uint pathLength;  ///< Path length in number of path segments (0 at origin, 1 at first secondary hit, etc.). Max 2^31.
    float3 origin;    ///< Next path segment origin.
    float3 direction; ///< Next path segment direction.

    SampleGenerator sg; ///< Per-ray state for the sample generator (up to 16B).

    /**
     * Initializes ray payload with default parameters.
     */
    __init(SampleGenerator sg)
    {
        this.terminated = false;
        this.pathLength = 0;
        this.radiance = float3(0, 0, 0);
        this.thp = float3(1, 1, 1);
        this.origin = float3(0, 0, 0);
        this.direction = float3(0, 0, 0);
        this.sg = sg;
    }
};

bool generateScatterRay(const ShadingData sd, const IMaterialInstance mi, inout SampleGenerator sg, out Ray ray, out float3 bsdfWeight)
{
    ray = {};
    bsdfWeight = 0.0;

    // Sample material.
    BSDFSample bsdfSample = {};
    if (mi.sample(sd, sg, bsdfSample, true))
    {
        // Offset ray origin to avoid self-intersection
        ray = Ray(sd.computeRayOrigin(), bsdfSample.wo);
        bsdfWeight = bsdfSample.weight / abs(dot(bsdfSample.wo, sd.faceN));

        return true;
    }

    return false;
}

// Generate light source ray by cosine weight sampling
float generateEmissiveRay(const ShadingData sd, inout SampleGenerator sg, out Ray ray)
{
    float pdf;

    float3 woLocal = sample_cosine_hemisphere_concentric(sampleNext2D(sg), pdf);
    ray = Ray(sd.computeRayOrigin(), sd.frame.fromLocal(woLocal));

    return pdf;
}

void handleHit(const HitInfo hit, const Ray ray, inout LightSample ls, out ShadingData sd)
{
    // Load shading data.
    sd = loadShadingData(hit, ls.posW, ray.dir, ExplicitLodTextureSampler(0.f));

    ls.posW = sd.posW;
    ls.normW = sd.getOrientedFaceNormal();
}

float3 approxSurfaceAlbedo(const ShadingData sd)
{
    const MaterialSystem ms = gScene.materials;

    SamplerState s = ms.getTextureSampler(sd.mtl.getDefaultTextureSamplerID());

    // Create material instance.
    let d = ms.getBasicMaterialData(sd.materialID);

    const float3 albedo = ms.sampleTexture(d.texBaseColor, s, sd.uv, d.baseColor, ExplicitLodTextureSampler(0.f)).rgb;

    // weight = albedo / PI * dot(wi, N)
    return albedo * M_1_PI;
}

void execute(uint id)
{
    SampleGenerator sg = SampleGenerator(uint2(id, 0), 0);

    // Actual VPL count = depth + 2(light source + ending point)
    uint depth = 1;
    int retry = 50;

    // Do very first light sampling
    LightSample ls;
    ShadingData sd;
    if (!sampleLight(sg, ls, sd))
        return;

    // Add luminant VPL
    if (!addVPL(ls.posW, ls.normW, ls.Li))
        return;

    Ray ray = {};

    generateEmissiveRay(sd, sg, ray);

    while (depth <= kMaxPathDepth && retry > 0)
    {
        // BSDF sample weight
        float3 bsdfWeight = 0.0f;

        // Trace ray
        SceneRayQuery<kUseAlphaTest> sceneRayQuery;
        HitInfo hit;
        float hitT;
        if (!sceneRayQuery.traceRay(ray, hit, hitT, RAY_FLAG_NONE, 0xff))
        {
            if (retry-- <= 0)
                break;
            continue;
        }

        handleHit(hit, ray, ls, sd);

        float cosThetaI = abs(dot(sd.faceN, ray.dir));
        ls.Li *= cosThetaI;

        // Approx surface albedo weight:
        // albedo / PI
        // Where pdf_L is previous light sampling pdf
        if (all(ls.Li <= 0.0) || !addVPL(ls.posW, ls.normW, ls.Li * approxSurfaceAlbedo(sd)))
            break;

        // Generate scattering ray
        if (!generateScatterRay(sd, gScene.materials.getMaterialInstance(sd, ExplicitLodTextureSampler(0.f)), sg, ray, bsdfWeight))
        {
            if (retry-- <= 0)
                break;
            continue;
        }

        ls.Li *= bsdfWeight;

        depth++;
    }
}

[numthreads(4, 1, 1)]
void main(uint3 dispatchThreadId: SV_DispatchThreadID)
{
    execute(dispatchThreadId.x);
}
