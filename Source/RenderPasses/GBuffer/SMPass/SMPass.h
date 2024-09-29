#pragma once
#include "Falcor.h"
#include "../GBuffer/GBuffer.h"
#include "RenderGraph/RenderGraph.h"
#include "RenderGraph/RenderPass.h"
#include "Core/Pass/FullScreenPass.h"

// ENABLE_EULAR_ROT 在RenderPass.h中定义
#define DISABLE_DEBUG // 不渲染6个gSM

using namespace Falcor;

class SMPass : public GBuffer
{
public:
    FALCOR_PLUGIN_CLASS(SMPass, "SMPass", "Output Indirect VPLs buffers independently. For dataset generation.");

    /** Create a new render pass object.
        \param[in] pDevice GPU device.
        \param[in] dict Dictionary of serialized parameters.
        \return A new object, or an exception is thrown if creation failed.
    */
    static ref<SMPass> create(ref<Device> pDevice, const Properties& props) { return make_ref<SMPass>(pDevice, props); }

    SMPass(ref<Device> pDevice, const Properties& dict);

    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pRenderContext, const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override;
    virtual void onSceneUpdates(RenderContext* pRenderContext, Scene::UpdateFlags sceneUpdates) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual Properties getProperties() const override;
    void averageLightSamples(RenderContext* pRenderContext, const RenderData& renderData);

protected:
    // Internal state
    ref<Fbo> mpFbo;

    std::vector<float3> CubeMapLookAtDir_local = {
        float3(0.0f, 0.0f, -1.0f),
        float3(-1.0f, 0.0f, 0.0f),
        float3(0.0f, 1.0f, 0.0f),
        float3(0.0f, -1.0f, 0.0f),
        float3(0.0f, 0.0f, 1.0f),
        float3(1.0f, 0.0f, 0.0f)};
    std::vector<float3> CubeMapLookUpDir_local = {
        float3(0.0f, 1.0f, 0.0f),
        float3(0.0f, 1.0f, 0.0f),
        float3(0.0f, 0.0f, 1.0f),
        float3(0.0f, 0.0f, -1.0f),
        float3(0.0f, 1.0f, 0.0f),
        float3(0.0f, 1.0f, 0.0f)};
    float4x4 shadowProjMatrices = math::perspective(math::radians(90.0f), 1.0f, 0.01f, 1000.0f); // fov, aspect, near, far
    std::vector<float3> lightPosList;
    std::vector<float3> lightNormList;
    uint32_t lightCount;

    // Rasterization resources
    struct
    {
        ref<GraphicsState> pState;
        ref<Program> pProgram;
        ref<ProgramVars> pVars;
    } mDepthPass, mSMPass;

    // uint2 mMapSize = uint2(512, 512);
    uint2 mMapSize = uint2(1024, 1024); // Chess
    // int rsm_down_sampling_scale = 8;
    int rsm_down_sampling_scale = 16;
};
