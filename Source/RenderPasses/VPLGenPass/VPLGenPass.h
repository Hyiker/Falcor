#pragma once

#include "Falcor.h"
#include "RenderGraph/RenderPass.h"

using namespace Falcor;

/**
 * Virtual Point Light(VPL) source generate pass.
 */
class VPLGenPass : public RenderPass
{
public:
    FALCOR_PLUGIN_CLASS(VPLGenPass, "VPLGenPass", {"Standalone pass for VPL generation."})

    static ref<VPLGenPass> create(ref<Device> pDevice, const Properties& props) { return make_ref<VPLGenPass>(pDevice, props); }

    VPLGenPass(ref<Device> pDevice, const Properties& props);

    virtual Properties getProperties() const override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override;

private:
    struct Params
    {
        uint32_t pathsPerLight = 1; ///< Number of Paths generated per light.
        uint32_t maxPathLength = 1; ///< Maximum path length for each path(Some may be early-terminated).
    };

    void parseProperties(const Properties& props);
    void recreatePrograms();

    void executeCreatePass(RenderContext* pRenderContext, const RenderData& renderData);

    // Configuration
    Params mParams; ///< VPL generate params.

    ref<Scene> mpScene;                     ///< Current scene
    ref<SampleGenerator> mpSampleGenerator; ///< GPU pseudo-random sample generator.

    ref<ComputePass> mpCreateVPLPass;

    bool mOptionsChanged = false;
};
