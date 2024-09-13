#include "IRShadingPass.h"
#include "IRShadingPass.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"
#include "VPLData.slang"

using namespace Falcor;

namespace
{
const std::string kIRShadingPassFilename = "RenderPasses/InstantRadiosity/IRShading.cs.slang";

// Scripting options.
const std::string kClamping = "clamping";
const std::string kVisualizeVPL = "visualizeVPL";

// Inputs
const std::string kVPLBufferDesc = "VPL data buffer";
const Falcor::ChannelList kInputChannels = {
    {"viewW", "gView", "World-space view direction (xyz float format)", false, ResourceFormat::RGBA32Float},
    {"posW", "gPosW", "World-space position (xyz float format)", false, ResourceFormat::RGBA32Float},
    {"normW", "gNormW", "Shading normal in world space", false, ResourceFormat::RGBA32Float},
    {"emissive", "gEmissive", "Emissive color", false, ResourceFormat::RGBA32Float},
    {"diffuseOpacity", "gDiffOpacity", "Diffuse reflection albedo and opacity", false, ResourceFormat::RGBA32Float},
    {"specRough", "gSpecRough", "Specular reflectance and roughness", false, ResourceFormat::RGBA32Float},
};

// Outputs
const Falcor::ChannelList kOutputChannels = {
    {"color", "gColor", "Output color (linear)", true /* optional */, ResourceFormat::RGBA32Float},
};

} // namespace

IRShadingPass::IRShadingPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);
}

void IRShadingPass::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        if (key == kClamping)
            mParams.clamping = value;
        else if (key == kVisualizeVPL)
            mParams.visualizeVPL = value;
        else
            logWarning("Unknown property '{}' in IRShadingPass properties.", key);
    }
}

Properties IRShadingPass::getProperties() const
{
    Properties props;

    props[kClamping] = mParams.clamping;
    return props;
}

RenderPassReflection IRShadingPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    addRenderPassInputs(reflector, kInputChannels);
    // Max sample per path = max path bounces + 2(starting point and ending point (if exists))
    reflector.addInput("vpl", "Virtual point light(VPL) data").rawBuffer(1000u * sizeof(VPLData));

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

        defines.add("CLAMPING", std::to_string(mParams.clamping));
        defines.add("VISUALIZE_VPL", mParams.visualizeVPL ? "1" : "0");

        ProgramDesc baseDesc;
        baseDesc.addShaderModules(mpScene->getShaderModules());
        baseDesc.addTypeConformances(mpScene->getTypeConformances());

        ProgramDesc desc = baseDesc;
        desc.addShaderLibrary(kIRShadingPassFilename).csEntry("main");
        mpShadingPass = ComputePass::create(mpDevice, desc, defines, true);

        // Bind static resources.
        mpScene->setRaytracingShaderData(pRenderContext, mpShadingPass->getRootVar());
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
    var["gVPL"] = static_ref_cast<Buffer>(renderData.getResource("vpl"));

    // Compute dispatch
    const uint2 targetDim = renderData.getDefaultTextureDims();
    mpShadingPass->execute(pRenderContext, uint3(targetDim, 1));
}
