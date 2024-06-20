#pragma once
#include "Core/Pass/ComputePass.h"
#include "Falcor.h"
#include "RenderGraph/RenderPass.h"
#include "Utils/Sampling/SampleGenerator.h"
#include "SharedTypes.slang"

using namespace Falcor;

/**
 * Instant radiosity renderer.
 *
 */
class InstantRadiosity : public RenderPass
{
public:
    FALCOR_PLUGIN_CLASS(InstantRadiosity, "InstantRadiosity", "Instant radiosity(VPL) method.");

    static ref<InstantRadiosity> create(ref<Device> pDevice, const Properties& props) { return make_ref<InstantRadiosity>(pDevice, props); }

    InstantRadiosity(ref<Device> pDevice, const Properties& props);

    Properties getProperties() const override;
    RenderPassReflection reflect(const CompileData& compileData) override;
    void buildVPLs(RenderContext* pRenderContext, const ref<LightCollection>& lightData);
    void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    void renderUI(Gui::Widgets& widget) override;
    void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override;
    bool onMouseEvent(const MouseEvent& mouseEvent) override { return false; }
    bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

private:
    void parseProperties(const Properties& props);

    // Internal state

    /// Current scene.
    ref<Scene> mpScene;
    /// GPU sample generator.
    ref<SampleGenerator> mpSampleGenerator;

    // Configuration
    RadiosityParams mParams;

    // Runtime data
    uint32_t mVPLCount = 0;
    bool mOptionsChanged = false;
    bool mRebuildVPLs = true;

    // VPL generation pass
    ref<ComputePass> mpPhotonGenPass;
    // VPL accumulation pass
    ref<ComputePass> mpShadingPass;

    // Scratch data for VPL generation
    ref<Buffer> mpPhotonData;           ///< VPL buffer.
    ref<Buffer> mpPhotonCounter;        ///< VPL count buffer.
    ref<Buffer> mpPhotonCounterStaging; ///< Staging buffer for VPL count.
    ref<Fence> mpReadbackFence;         ///< Fence for VPL generation.
};
