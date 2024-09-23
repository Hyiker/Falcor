#include "VPLVisibilityPass.h"
#include "InstantRadiosity.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"

using namespace Falcor;

namespace
{

const std::string kVPLVisibilityPassFilename = "RenderPasses/InstantRadiosity/VPLVisibility.cs.slang";

// Scripting options
const std::string kRandomSampleVPL = "randomSampleVPL";
const std::string kVisMapRes = "visMapRes";
const std::string kVPLSampleSeed = "vplSampleSeed";

// Inputs
const std::string kVPLBuffer = "gVPL";
const std::string kVPLCounterBuffer = "gVPLCounter";

// Outputs
const Falcor::ChannelList kOutputChannels = {
    {"visibility", "gVisibility", "Point to point visibility", false, ResourceFormat::R16Float},
    {"posW0", "gPosW0", "Point 0", false, ResourceFormat::RGBA32Float},
    {"posW1", "gPosW1", "Point 1", false, ResourceFormat::RGBA32Float}};

} // namespace

VPLVisibilityPass::VPLVisibilityPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);

    // Create sample generator.
    mpSampleGenerator = SampleGenerator::create(mpDevice, SAMPLE_GENERATOR_DEFAULT);
}

void VPLVisibilityPass::registerBindings(pybind11::module& m)
{
    // VPL generate pass
    pybind11::class_<VPLVisibilityPass, RenderPass, ref<VPLVisibilityPass>> pass(m, "VPLVisibilityPass");
    // TODO
    pass.def_property("vplSampleSeed", &VPLVisibilityPass::getVPLSampleSeed, &VPLVisibilityPass::setVPLSampleSeed);
}

uint32_t VPLVisibilityPass::getVPLSampleSeed() const
{
    return mParams.vplSampleSeed;
}
void VPLVisibilityPass::setVPLSampleSeed(uint32_t value)
{
    mParams.vplSampleSeed = value;
}

void VPLVisibilityPass::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        if (key == kRandomSampleVPL)
            mParams.randomSampleVPL = value;
        else if (key == kVisMapRes)
            mParams.visMapRes = value;
        else if (key == kVPLSampleSeed)
            mParams.vplSampleSeed = value;
        else
            logWarning("Unknown property '{}' in VPLVisibilityPass properties.", key);
    }
}

Properties VPLVisibilityPass::getProperties() const
{
    Properties props;

    props[kRandomSampleVPL] = mParams.randomSampleVPL;
    props[kVPLSampleSeed] = mParams.vplSampleSeed;
    props[kVisMapRes] = mParams.visMapRes;

    return props;
}

RenderPassReflection VPLVisibilityPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    reflector.addInput("vpl", "Virtual point light(VPL) data").rawBuffer(kMaxVPLCountLimit * sizeof(VPLData));
    reflector.addInput("vplCounter", "Virtual point light(VPL) data").rawBuffer(sizeof(uint32_t));

    // Custom resolution VPL visibility matrix
    for (const auto& it : kOutputChannels)
    {
        auto& tex = reflector.addOutput(it.name, it.desc).texture2D(mParams.visMapRes.x, mParams.visMapRes.y);
        tex.bindFlags(ResourceBindFlags::UnorderedAccess);
        if (it.format != ResourceFormat::Unknown)
            tex.format(it.format);
        if (it.optional)
            tex.flags(RenderPassReflection::Field::Flags::Optional);
    }

    return reflector;
}

void VPLVisibilityPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
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

        mpVisibilityPass = nullptr;

        mOptionsChanged = false;
    }

    if (!mpVisibilityPass)
    {
        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add(mpSampleGenerator->getDefines());

        defines.add("RANDOM_SAMPLE_VPL", mParams.randomSampleVPL ? "1" : "0");

        ProgramDesc baseDesc;
        baseDesc.addShaderModules(mpScene->getShaderModules());
        baseDesc.addTypeConformances(mpScene->getTypeConformances());

        ProgramDesc desc = baseDesc;
        desc.addShaderLibrary(kVPLVisibilityPassFilename).csEntry("main");
        mpVisibilityPass = ComputePass::create(mpDevice, desc, defines, true);

        // Bind static resources.
        auto var = mpVisibilityPass->getRootVar();
        mpScene->setRaytracingShaderData(pRenderContext, var);
        mpSampleGenerator->bindShaderData(var);
    }

    ShaderVar var = mpVisibilityPass->getRootVar();

    // Bind I/O buffers. These needs to be done per-frame as the buffers may change anytime.
    auto bind = [&](const ChannelDesc& desc)
    {
        if (!desc.texname.empty())
        {
            var[desc.texname] = renderData.getTexture(desc.name);
        }
    };

    for (auto channel : kOutputChannels)
        bind(channel);

    var[kVPLBuffer] = static_ref_cast<Buffer>(renderData.getResource("vpl"));
    var[kVPLCounterBuffer] = static_ref_cast<Buffer>(renderData.getResource("vplCounter"));
    var["CB"]["frameDim"] = mParams.visMapRes;
    var["CB"]["vplSampleSeed"] = mParams.vplSampleSeed;

    // Compute dispatch
    mpVisibilityPass->execute(pRenderContext, uint3(mParams.visMapRes, 1));
}

void VPLVisibilityPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    mpScene = pScene;

    mpVisibilityPass = nullptr;

    if (mpScene)
    {
        if (pScene->hasProceduralGeometry())
        {
            logWarning("VPLVisibilityPass: This pass only supports triangles. Other types of geometry will be ignored.");
        }
    }
}

void VPLVisibilityPass::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    if (widget.var("Output resolution", mParams.visMapRes, 16u, 8192u))
    {
        requestRecompile();
    }

    dirty |= widget.checkbox("Random sample VPL", mParams.randomSampleVPL);

    if (dirty)
        mOptionsChanged = true;

    // Uniform data
    widget.var("Sample seed", mParams.vplSampleSeed, 0u, std::numeric_limits<uint32_t>::max());
}
