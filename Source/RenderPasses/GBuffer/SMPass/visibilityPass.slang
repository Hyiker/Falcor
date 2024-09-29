#include "Scene/VertexAttrib.slangh"

import Scene.Raster;
import Utils.Math.MathHelpers;
import RenderPasses.GBuffer.GBuffer.GBufferHelpers;
import Rendering.Materials.TexLODHelpers;

cbuffer PerFrameCB
{
    uint2 gFrameDim;
};

cbuffer LightPos
{
    float3 gLightPos[8]; // 支持多光源
}
cbuffer smCtrParam
{
    uint width;
    bool ENABLE_PCF;  //
    int FILTER_PCF_S; // 1, width = 2*n+1
    float BIASMAX;    // 0.5
    float BIASMIN;    // 0.005
    uint nLight;
}

// GBufferRaster Channels
//  RWTexture2D<float2> gLinearZAndDeriv;
RWTexture2DArray<float> gCubeMap;
// RWTexture2D<float3> gVisibility;
RWTexture2D<PackedHitInfo> gVBuffer;

// Shadow clues output
RWTexture2D<float2> gToLight[8];
RWTexture2D<float4> gToLightVec[8];
RWTexture2D<float4> gHalfVec[8];

#define is_valid(name) (is_valid_##name != 0)

float3 sampleCubeIdx(const float3 _dirL2W, const uint itL)
{
    float3 dirL2W = _dirL2W;
    float2 coord;

    float3 dir = dirL2W;
    uint32_t index = 0;
    float3 samplePos;
    float absx = abs(dir.x), absy = abs(dir.y), absz = abs(dir.z);
    if (absx > absy && absx > absz)
    {
        if (dir.x > 0)
        {
            index = 5;
            samplePos = float3((dir.z / absx + 1) / 2.0, (-dir.y / absx + 1) / 2.0, index + itL * 6);
        }
        else
        {
            index = 1;
            samplePos = float3((-dir.z / absx + 1) / 2.0, (-dir.y / absx + 1) / 2.0, index + itL * 6);
        }
    }
    else if (abs(dir.y) > abs(dir.z))
    {
        if (dir.y > 0)
        {
            index = 2;
            samplePos = float3((dir.x / absy + 1) / 2.0, (-dir.z / absy + 1) / 2.0, index + itL * 6);
        }
        else
        {
            index = 3;
            samplePos = float3((dir.x / absy + 1) / 2.0, (dir.z / absy + 1) / 2.0, index + itL * 6);
        }
    }
    else
    {
        if (dir.z > 0)
        {
            index = 4;
            samplePos = float3((-dir.x / absz + 1) / 2.0, (-dir.y / absz + 1) / 2.0, index + itL * 6);
        }
        else
        {
            index = 0;
            samplePos = float3((dir.x / absz + 1) / 2.0, (-dir.y / absz + 1) / 2.0, index + itL * 6);
        }
    }
    samplePos.x = samplePos.x * width;
    samplePos.y = samplePos.y * width;
    return samplePos;
}

float3 SamplerPCF(int32_t3 sampleIndex, float dir_len, float bias)
{
    float vis_avr = 0;
    uint count = 0;
    for (int x = -FILTER_PCF_S; x <= FILTER_PCF_S; x++)
    {
        for (int y = -FILTER_PCF_S; y <= FILTER_PCF_S; y++)
        {
            int32_t3 newSample = sampleIndex;
            newSample.x += x;
            newSample.y += y;
            if (newSample.x < 0 || newSample.y < 0 || newSample.x > width || newSample.y > width)
            {
                // TODEBUG
                if (newSample.x < 0)
                    newSample.x = abs(newSample.x);
                if (newSample.y < 0)
                    newSample.y = abs(newSample.y);
                if (newSample.x > width)
                    newSample.x = 2 * width - newSample.x;
                if (newSample.y > width)
                    newSample.y = 2 * width - newSample.y;
                // continue;
            }
            float depth = gCubeMap[newSample];
            if (dir_len < depth + bias)
            {
                vis_avr++;
            }
            count++;
        }
    }
    return float3(vis_avr / (1.0 * count));
}

float SamplerPCF4Depth(int32_t3 sampleIndex)
{
    float d_avr = 0;
    uint count = 0;
    for (int x = -FILTER_PCF_S; x <= FILTER_PCF_S; x++)
    {
        for (int y = -FILTER_PCF_S; y <= FILTER_PCF_S; y++)
        {
            int32_t3 newSample = sampleIndex;
            newSample.x += x;
            newSample.y += y;
            if (newSample.x < 0 || newSample.y < 0 || newSample.x >= width || newSample.y >= width)
            {
                // TODEBUG
                if (newSample.x < 0)
                    newSample.x = abs(newSample.x);
                if (newSample.y < 0)
                    newSample.y = abs(newSample.y);
                if (newSample.x >= width)
                    newSample.x = 2 * width - newSample.x - 2;
                if (newSample.y >= width)
                    newSample.y = 2 * width - newSample.y - 2;
                // continue;
            }
            float depth = gCubeMap[newSample];
            d_avr += depth;
            count++;
        }
    }
    return float(d_avr / (1.0 * count));
}

VSOut vsMain(VSIn vsIn)
{
    return defaultVS(vsIn);
}

float2 computeMotionVector(const VSOut vsOut, const int2 ipos)
{
    float2 pixelPos = ipos + float2(0.5, 0.5); // Current sample in pixel coords.
    float4 prevPosH = vsOut.prevPosH;          // Sample in previous frame in clip space coords, no jittering applied.
    return calcMotionVector(pixelPos, prevPosH, gFrameDim) + float2(gScene.camera.data.jitterX, -gScene.camera.data.jitterY); // Remove
                                                                                                                              // camera
                                                                                                                              // jitter from
                                                                                                                              // motion
                                                                                                                              // vector
}

struct GBufferPSOut
{
    float4 posW : SV_TARGET0; // 多目标渲染
    float4 normW : SV_TARGET1;
    float4 emissive : SV_TARGET2;
    float4 viewW : SV_TARGET3;
    float4 diffuseOpacity : SV_TARGET4;
    float4 specRough : SV_TARGET5;
    float2 texC : SV_TARGET6;
    float2 mvec : SV_TARGET7;
};

[earlydepthstencil] // visibility pass也需要一个depthStencil
GBufferPSOut psMain(VSOut vsOut, uint triangleIndex: SV_PrimitiveID, float3 barycentrics: SV_Barycentrics)
{
    // Using vOut.posH.xy as pixel coordinate since it has the SV_Position semantic.
    int2 ipos = int2(vsOut.posH.xy);

    float3 faceNormal = gScene.getFaceNormalW(vsOut.instanceID, triangleIndex);
    VertexData v = prepareVertexData(vsOut, faceNormal);
    let lod = ImplicitLodTextureSampler();

#if USE_ALPHA_TEST
    if (gScene.materials.alphaTest(v, vsOut.materialID, lod))
        discard;
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

    const float4 texGrads = float4(ddx(sd.uv), ddy(sd.uv));
    const float2 mvec = computeMotionVector(vsOut, ipos);

    GBufferPSOut psOut = {};

    // Store render target outputs.
    if (is_valid(gPosW))
        psOut.posW = gbuf.posW;
    if (is_valid(gNormW))
        psOut.normW = gbuf.normW;
    // if (is_valid(gEmissive))        psOut.emissive     = gbuf.emissive;
    // if (is_valid(gViewW))           psOut.viewW       = float4(viewDir, 1.0);
    // if (is_valid(gDiffOpacity))     psOut.diffuseOpacity  = gbuf.diffuseOpacity;
    // if (is_valid(gSpecRough))       psOut.specRough   = gbuf.specRough;
    if (is_valid(gTexC))
        psOut.texC = gbuf.texC;
    if (is_valid(gMotionVector))
        psOut.mvec = mvec;

    // Linear z and its derivative
    // if (is_valid(gLinearZAndDeriv))
    // {
    //     const float linearZ = vsOut.posH.z * vsOut.posH.w; // posH是posW变换到相机坐标系下的结果，因此这里posH的z可能就是相机到物体的距离
    //     gLinearZAndDeriv[ipos] = float2(linearZ, max(abs(ddx(linearZ)), abs(ddy(linearZ))));
    // }

    for (uint lightID = 0; lightID < nLight; lightID++)
    {
        // gVisibilityAll[lightID][ipos] = float3(0.0);
        float3 light_dir = gLightPos[lightID] - vsOut.posW;
        float3 view_dir = gScene.camera.getPosition() - vsOut.posW;
        float light_dir_len = length(light_dir);
        int32_t3 sampleIndex = sampleCubeIdx(-light_dir, lightID); // Conversion to int implicitly?? Round maybe better than clamp!

        // float bias = max(BIASMAX / abs(dot(vsOut.normalW, -dirL2W)), 0.005);
        // float3 v=0.0;
        // if(ENABLE_PCF){
        //     v = SamplerPCF(sampleIndex,dir_len,bias);
        //     gVisibilityAll[lightID][ipos]=v;
        // }else{
        //     if(dir_len<gCubeMap[sampleIndex] + bias){
        //         v = float3(1.0);
        //         gVisibilityAll[lightID][ipos]=v;
        //     }
        // }

        float occlusionToLight = 0;
        if (ENABLE_PCF)
        {
            occlusionToLight = SamplerPCF4Depth(sampleIndex);
        }
        else
        {
            occlusionToLight = gCubeMap[sampleIndex];
        }
        if (light_dir_len < 0.01)
            occlusionToLight = 0;
        gToLight[lightID][ipos] = float2(light_dir_len, occlusionToLight);
        gToLightVec[lightID][ipos] = float4(normalize(light_dir), 1);
        gHalfVec[lightID][ipos] = float4(normalize((normalize(light_dir) + normalize(view_dir)) / 2), 1);
    }
    return psOut;
}

