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
        uint32_t maxVPLCount = 100u; ///< Maximum VPL count in total.
        uint32_t maxPathDepth = 5u;  ///< Maximum path depth for each path(Some may be early-terminated).
    };

    void parseProperties(const Properties& props);
    void recreatePrograms();

    void executeCreatePass(RenderContext* pRenderContext, const RenderData& renderData);

    // Configuration
    Params mParams; ///< VPL generate params.

    // Internal state
    ref<Scene> mpScene;                     ///< Current scene
    ref<SampleGenerator> mpSampleGenerator; ///< GPU pseudo-random sample generator.

    // GPU buffer
    ref<Buffer> mpCounterStagingBuffer;

    ref<ComputePass> mpCreateVPLPass;

    // CPU statistics
    uint32_t mVPLCount;

    bool mOptionsChanged = false;
};
