#pragma once
#include "Falcor.h"
#include "RenderGraph/RenderPass.h"

using namespace Falcor;

class IRShadingPass : public RenderPass
{
public:
    FALCOR_PLUGIN_CLASS(IRShadingPass, "IRShadingPass", "Instant radiosity shading using VPLs.");

    static ref<IRShadingPass> create(ref<Device> pDevice, const Properties& props) { return make_ref<IRShadingPass>(pDevice, props); }

    IRShadingPass(ref<Device> pDevice, const Properties& props);

    virtual Properties getProperties() const override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override;

private:
    struct Params
    {
        float clamping = 0.1f;     ///< VPL shading clamping factor to avoid artifact.
        bool visualizeVPL = true; ///< Enable VPL position and intensity visualization.
    };

    void parseProperties(const Properties& props);

    void executeShadingPass(RenderContext* pRenderContext, const RenderData& renderData);

    // Configuration
    Params mParams;

    // Internal state
    ref<Scene> mpScene; ///< Current scene

    ref<ComputePass> mpShadingPass;

    bool mOptionsChanged = false;
};
