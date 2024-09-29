#pragma once
#include "Falcor.h"
#include "../GBuffer/GBuffer.h"
#include "RenderGraph/RenderGraph.h"
#include "RenderGraph/RenderPass.h"
#include "Core/Pass/FullScreenPass.h"

using namespace Falcor;

class VisPass : public GBuffer
{
public:
    FALCOR_PLUGIN_CLASS(VisPass, "VisPass", "Shadow map visualization.");


    /** Create a new render pass object.
        \param[in] pDevice GPU device.
        \param[in] dict Dictionary of serialized parameters.
        \return A new object, or an exception is thrown if creation failed.
    */
    static ref<VisPass> create(ref<Device> pDevice, const Properties& props) { return make_ref<VisPass>(pDevice, props); }

    VisPass(ref<Device> pDevice, const Properties& dict);

    RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pRenderContext, const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override;
    virtual void onSceneUpdates(RenderContext* pRenderContext, Scene::UpdateFlags sceneUpdates) override;
    // virtual void renderUI(Gui::Widgets& widget);
    virtual Properties getProperties() const override;

    void setPCFSize(int n)
    {
        mFILTER_PCF_S = n;
        mDirty = true;
    }

    float getPCFSize() const { return mFILTER_PCF_S; }

     void setBiasMax(float n)
    {
        mBIASMAX = n;
        mDirty = true;
    }

    float getBiasMax() const { return mBIASMAX; }

     void setBiasMin(float n)
    {
        mBIASMIN = n;
        mDirty = true;
    }

    float getBiasMin() const { return mBIASMIN; }



private:
    // Internal state
    ref<Fbo> mpFbo;
    std::vector<std::shared_ptr<Texture>> pTexCube_arr;
    mutable bool mDirty = true;
    // Rasterization resources
    struct
    {
        ref<GraphicsState> pState;
        ref<Program> pProgram;
        ref<ProgramVars> pVars;
    } mDepthPassVis, mVisibilityPass;

    //struct
    //{
    //    GraphicsState::SharedPtr pState;
    //    GraphicsProgram::SharedPtr pProgram;
    //    GraphicsVars::SharedPtr pVars;
    //} ;

    // Shadow Map Configuration
    uint nVisOutMax = 8;
    bool mENABLEPCF = false;                // 是否开启pcf滤波
    int mFILTER_PCF_S = 1;                  // 卷积核宽度，width = 2*mFILTER_PCF_S+1
    float mBIASMAX = 1.0f;                   // 自适应bias
    float mBIASMIN = 0.005f;
    bool mOptionsChanged = false;
    uint32_t lightCount;
    uint32_t multisample = 1;

    // uint2 mMapSize = uint2(512, 512);
    uint2 mMapSize = uint2(1024, 1024); // Chess scene
};
