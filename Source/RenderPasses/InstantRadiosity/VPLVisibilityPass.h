#pragma once

#include "Falcor.h"
#include "RenderGraph/RenderPass.h"
#include "VPLData.slang"

using namespace Falcor;

/**
 * Virtual Point Light(VPL) visibility data generate pass.
 */
class VPLVisibilityPass : public RenderPass
{
public:
    FALCOR_PLUGIN_CLASS(VPLVisibilityPass, "VPLVisibilityPass", {"Standalone pass for VPL pair visibility test."})

    static ref<VPLVisibilityPass> create(ref<Device> pDevice, const Properties& props)
    {
        return make_ref<VPLVisibilityPass>(pDevice, props);
    }

    VPLVisibilityPass(ref<Device> pDevice, const Properties& props);

    virtual Properties getProperties() const override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override;

    uint32_t getVPLSampleSeed() const;
    void setVPLSampleSeed(uint32_t value);

    static void registerBindings(pybind11::module& m);

private:
    struct Params
    {
        uint2 visMapRes = uint2(512, 512); ///< Visibility map resolution, each pixel records the p0, p1 and V(p0 <-> p1)
        bool randomSampleVPL = true;       ///< Whether random sample VPL for visibility test, if false, use vpls[pixel.x], vpls[pixel.y]
        uint32_t vplSampleSeed = 0;        ///< Seed for VPL sampling if randomSampleVPL is enabled
    };

    void parseProperties(const Properties& props);

    // Configuration
    Params mParams;

    // Internal state
    ref<Scene> mpScene;
    ref<SampleGenerator> mpSampleGenerator; ///< GPU pseudo-random sample generator.

    ref<ComputePass> mpVisibilityPass;

    bool mOptionsChanged = false;
};
