#include "IRShadingPass.h"
#include "InstantRadiosity.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"

using namespace Falcor;

namespace
{
const std::string kIRShadingPassFilename = "RenderPasses/InstantRadiosity/IRShading.cs.slang";

// Scripting options.
const std::string kClamping = "clamping";
const std::string kVisualizeVPL = "visualizeVPL";
const std::string kVPLSamples = "vplSamples";

// Inputs
const std::string kVPLBuffer = "gVPL";
const std::string kVPLCounterBuffer = "gVPLCounter";
const Falcor::ChannelList kInputChannels = {
    {"vbuffer", "gVBuffer", "Visibility buffer in packed format", false},
    {"viewW", "gViewW", "World-space view direction (xyz float format)", false, ResourceFormat::RGBA32Float},
};

// Outputs
const Falcor::ChannelList kOutputChannels = {
    {"color", "gColor", "Output color (linear)", true /* optional */, ResourceFormat::RGBA32Float},
};

} // namespace

IRShadingPass::IRShadingPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);

    // Create sample generator.
    mpSampleGenerator = SampleGenerator::create(mpDevice, SAMPLE_GENERATOR_DEFAULT);
}

void IRShadingPass::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        if (key == kClamping)
            mParams.clamping = value;
        else if (key == kVisualizeVPL)
            mParams.visualizeVPL = value;
        else if (key == kVPLSamples)
            mParams.vplSamples = value;
        else
            logWarning("Unknown property '{}' in IRShadingPass properties.", key);
    }
}

Properties IRShadingPass::getProperties() const
{
    Properties props;

    props[kClamping] = mParams.clamping;
    props[kVisualizeVPL] = mParams.visualizeVPL;
    props[kVPLSamples] = mParams.vplSamples;
    return props;
}

RenderPassReflection IRShadingPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    addRenderPassInputs(reflector, kInputChannels);
    // Max sample per path = max path bounces + 2(starting point and ending point (if exists))
    reflector.addInput("vpl", "Virtual point light(VPL) data").rawBuffer(kMaxVPLCountLimit * sizeof(VPLData));
    reflector.addInput("vplCounter", "Virtual point light(VPL) data").rawBuffer(sizeof(uint32_t));

    addRenderPassOutputs(reflector, kOutputChannels);

    return reflector;
}

void IRShadingPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Clear outputs if no scene is loaded.
    if (!mpScene)
    {
        clearRenderPassChannels(pRenderContext, kOutputChannels, renderData);
        return;
    }

    auto& dict = renderData.getDictionary();

    // Update refresh flag if changes that affect the output have occured.
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, Falcor::RenderPassRefreshFlags::None);
        flags |= Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        dict[Falcor::kRenderPassRefreshFlags] = flags;

        mpShadingPass = nullptr;

        mOptionsChanged = false;
    }

    executeShadingPass(pRenderContext, renderData);
}

void IRShadingPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    mpScene = pScene;

    mpShadingPass = nullptr;

    if (mpScene)
    {
        if (pScene->hasProceduralGeometry())
        {
            logWarning("IRShadingPass: This pass only supports triangles. Other types of geometry will be ignored.");
        }
    }
}

void IRShadingPass::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    dirty |= widget.var("Clamping", mParams.clamping, 0.0f, 1.0f);

    dirty |= widget.var("VPL samples", mParams.vplSamples, 0u, kMaxVPLCountLimit);

    dirty |= widget.checkbox("Visualize VPL", mParams.visualizeVPL);

    if (dirty)
    {
        mOptionsChanged = true;
    }
}

void IRShadingPass::executeShadingPass(RenderContext* pRenderContext, const RenderData& renderData)
{
    FALCOR_ASSERT(mpScene);

    FALCOR_PROFILE(pRenderContext, "IR shading Pass");

    if (!mpShadingPass)
    {
        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add(mpSampleGenerator->getDefines());

        defines.add("CLAMPING", std::to_string(mParams.clamping));
        defines.add("VISUALIZE_VPL", mParams.visualizeVPL ? "1" : "0");
        defines.add("VPL_SAMPLES", std::to_string(mParams.vplSamples));

        ProgramDesc baseDesc;
        baseDesc.addShaderModules(mpScene->getShaderModules());
        baseDesc.addTypeConformances(mpScene->getTypeConformances());

        ProgramDesc desc = baseDesc;
        desc.addShaderLibrary(kIRShadingPassFilename).csEntry("main");
        mpShadingPass = ComputePass::create(mpDevice, desc, defines, true);

        // Bind static resources.
        auto var = mpShadingPass->getRootVar();
        mpScene->setRaytracingShaderData(pRenderContext, var);
        mpSampleGenerator->bindShaderData(var);
        mFrameCount = 0;
    }

    ShaderVar var = mpShadingPass->getRootVar();

    // Bind I/O buffers. These needs to be done per-frame as the buffers may change anytime.
    auto bind = [&](const ChannelDesc& desc)
    {
        if (!desc.texname.empty())
        {
            var[desc.texname] = renderData.getTexture(desc.name);
        }
    };

    for (auto channel : kInputChannels)
        bind(channel);
    for (auto channel : kOutputChannels)
        bind(channel);
    var[kVPLBuffer] = static_ref_cast<Buffer>(renderData.getResource("vpl"));
    var[kVPLCounterBuffer] = static_ref_cast<Buffer>(renderData.getResource("vplCounter"));
    const uint2 targetDim = renderData.getDefaultTextureDims();
    var["CB"]["frameDim"] = targetDim;
    var["CB"]["frameCount"] = mFrameCount++;

    // Compute dispatch
    mpShadingPass->execute(pRenderContext, uint3(targetDim, 1));
}
