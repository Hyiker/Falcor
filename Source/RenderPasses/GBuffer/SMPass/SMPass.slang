#include "Scene/VertexAttrib.slangh"

import Scene.Raster;
import Utils.Math.MathHelpers;
import RenderPasses.GBuffer.GBuffer.GBufferHelpers;
import Rendering.Materials.TexLODHelpers;

cbuffer PerFrameCB
{
    uint2 gFrameDim;
};
cbuffer LightVP
{
    float4x4 gLightVP;
    float3 gLightPos;
    // int geoID;
    float3 gLightNormal;
}
cbuffer CubeMapParam
{
    uint gCubeFaceN;
    uint gLightN;
}
// cbuffer Accel_CM
// {
//     uint gDrawCMD; //0: 不渲染；1. 渲染一半；2. 渲染全部; 3. 默认值
// }

struct VSOutSM
{
    float3 normalW    : NORMAL;      ///< Shading normal in world space (not normalized!).
    float4 tangentW   : TANGENT;     ///< Shading tangent in world space (not normalized!).
    float2 texC       : TEXCRD;      ///< Texture coordinate.
    float3 posW       : POSW;        ///< Position in world space.
    // float4 prevPosH   : PREVPOSH;    ///< Position in clip space for the previous frame.

    // Per-triangle data
    GeometryInstanceID instanceID   : INSTANCE_ID;  ///< Geometry instance ID.
    uint materialID                 : MATERIAL_ID;  ///< Material ID.

    float4 posH : SV_POSITION; // same as glPOSITION, posH here is from Light
};

VSOutSM vsMain(VSIn vIn)
{
    VSOutSM vOut;
    const GeometryInstanceID instanceID = { vIn.instanceID };

    float4x4 worldMat = gScene.getWorldMatrix(instanceID);
    float3 posW = mul(worldMat, float4(vIn.pos, 1.f)).xyz;
    vOut.instanceID = instanceID;
    vOut.materialID = gScene.getMaterialID(instanceID);
    if(gScene.materials.isEmissiveTextured(vOut.materialID)==true){ // FIXME: all meshes with emissive material are filtered out. But we only need to filter the one which generate shdow map.
        posW = posW + float3(100000000,100000000,100000000);
    }
    vOut.posW = posW;

    vOut.texC = vIn.texC;
    vOut.normalW = mul(gScene.getInverseTransposeWorldMatrix(instanceID), vIn.unpack().normal);
    float4 tangent = vIn.unpack().tangent;
    vOut.tangentW = float4(mul((float3x3)gScene.getWorldMatrix(instanceID), tangent.xyz), tangent.w);

    // Compute the vertex position in the previous frame.
    // float3 prevPos = vIn.pos;
    // GeometryInstanceData instance = gScene.getGeometryInstance(instanceID);
    // if (instance.isDynamic())
    // {
    //     uint prevVertexIndex = gScene.meshes[instance.geometryIndex].prevVbOffset + vIn.vertexID;
    //     prevPos = gScene.prevVertices[prevVertexIndex].position;
    // }
    // float3 prevPosW = mul(gScene.getPrevWorldMatrix(instanceID), float4(prevPos, 1.f)).xyz;
    // vOut.prevPosH = mul(gScene.camera.data.prevViewProjMatNoJitter, float4(prevPosW, 1.f));

    // Compute pos viewed from light.
    vOut.posH = mul(gLightVP,float4(posW, 1.f));
    return vOut;
}

VertexData prepareVertexData2(VSOutSM vsOut, float3 faceNormalW)
{
    VertexData v;
    v.posW = vsOut.posW;
    v.texC = vsOut.texC;
    v.normalW = normalize(vsOut.normalW);
    v.faceNormalW = faceNormalW;
    v.tangentW.xyz = normalize(vsOut.tangentW.xyz);
    v.tangentW.w = sign(vsOut.tangentW.w); // Preserve zero to indicate invalid tangent.
    return v;
}

RWTexture2DArray<float> gCubeMap;

RWTexture2DArray<float4> gRSMposition;
RWTexture2DArray<float4> gRSMnormal;
RWTexture2DArray<float4> gRSMflux;

// Visualization buffer
RWTexture2D<float4> gRSMpositionVis;
RWTexture2D<float4> gRSMnormalVis;
RWTexture2D<float4> gRSMfluxVis;

#define is_valid(name) (is_valid_##name != 0)

[earlydepthstencil]
void psMain(VSOutSM vsOut, uint triangleIndex : SV_PrimitiveID, float3 barycentrics : SV_Barycentrics)
{
    // Using vOut.posH.xy as pixel coordinate since it has the SV_Position semantic.
    int2 ipos = int2(vsOut.posH.xy);

    float3 light_vec = gLightPos - vsOut.posW.xyz;
    float3 light_dir = normalize(light_vec);
    float depth = length(light_vec);
    if (depth < 0.01 || dot(-light_dir, gLightNormal) <= 0)
        discard;

    // CubeMap Draw Ignore
    // hack here. minus one to obtain smaller shadow range.
    // TODO: use cosine to restrict angle
    // if(gDrawCMD==0||(gDrawCMD==1&&ipos.y<0.5*gFrameDim.y+1)) // TODEBUG
    // {
        // gDirectShading[ipos] = float4(-1); // valid mask
        // discard;
    // }

    float3 faceNormal = gScene.getFaceNormalW(vsOut.instanceID, triangleIndex);
    VertexData v = prepareVertexData2(vsOut, faceNormal);
    let lod = ImplicitLodTextureSampler();

#if USE_ALPHA_TEST
    if (gScene.materials.alphaTest(v, vsOut.materialID, lod)) discard;
#endif
    const float3 viewDir = normalize(gScene.camera.getPosition() - v.posW);
    ShadingData sd = gScene.materials.prepareShadingData(v, vsOut.materialID, viewDir, lod);

#if ADJUST_SHADING_NORMALS

    adjustShadingNormal(sd, sd.frame);
#endif

    // Create material instance and query its properties.
    let mi = gScene.materials.getMaterialInstance(sd, lod);
    let bsdfProperties = mi.getProperties(sd);

    const GBufferData gbuf = prepareGBufferData(sd, v, mi, bsdfProperties);

    uint3 coord_depth = uint3(ipos.x,ipos.y,gCubeFaceN + 6 * gLightN);

    gCubeMap[coord_depth] = 1.0;

    // if (ipos.x % 8 == 4 && ipos.y % 8 == 4) { // Sample in center
    //     ipos -= 4;
    //     ipos /= 8;
    if (ipos.x % 16 == 8 && ipos.y % 16 == 8) { // Sample in center
        ipos -= 8;
        ipos /= 16;

        // For transfering to lightformer pass
        uint3 coord = uint3(ipos, gCubeFaceN + 6 * gLightN);
        gRSMposition[coord]=gbuf.posW;
        gRSMnormal[coord]=gbuf.normW;
        gRSMflux[coord]=float4(gbuf.diffuseOpacity.xyz,1);

        // Visualization buffer
        gRSMpositionVis[ipos]=gbuf.posW;
        gRSMnormalVis[ipos]=gbuf.normW;
        gRSMfluxVis[ipos]=float4(gbuf.diffuseOpacity.xyz,1);
        // gRSMfluxVis[ipos]=float4(float3(dot(-light_dir, gLightNormal)),1);
        // gRSMfluxVis[ipos]=float4(-light_dir,1);
    }

    // float dist = max(length(gLightPos-vsOut.posW), 1e-3);
    // float3 directColor = float3(-1);
    // if (dot(-light_dir, gLightNormal) > 0)
    // float3 directColor = gIntensity * gbuf.diffuseOpacity.xyz * max(dot(faceNormal, light_dir), 0) / (dist*dist);
    // float3 directColor = gbuf.diffuseOpacity.xyz * max(dot(faceNormal, light_dir), 0) / (dist*dist);
    // gDirectShading[ipos]=float4(directColor,1);
}
