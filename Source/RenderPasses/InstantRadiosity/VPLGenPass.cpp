#include "VPLGenPass.h"
#include "InstantRadiosity.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"

using namespace Falcor;

namespace
{
const std::string kCreateVPLPassFilename = "RenderPasses/InstantRadiosity/CreateVPL.cs.slang";

// Scripting options.
const std::string kMaxVPLCount = "maxVPLCount";
const std::string kMaxPathDepth = "maxPathDepth";

// VPL configuration limits.
static const uint32_t kMaxPathDepthLimit = 12u;

// Outputs
const std::string kVPLBuffer = "gVPL";
const std::string kVPLCounterBuffer = "gVPLCounter";
const ChannelDesc kOutputChannel{"output", "gOutput", "An output to ensure VPLGenPass being executed", false, ResourceFormat::RGBA32Float};
} // namespace

void VPLGenPass::registerBindings(pybind11::module& m)
{
    // VPL data
    pybind11::class_<VPLData> vplData(m, "VPLData");
    vplData.def_readwrite("position", &VPLData::position);
    vplData.def_readwrite("normal", &VPLData::normal);
    vplData.def_readwrite("intensity", &VPLData::intensity);

    // VPL generate pass
    pybind11::class_<VPLGenPass, RenderPass, ref<VPLGenPass>> pass(m, "VPLGenPass");
    pass.def_property_readonly("vpls", &VPLGenPass::getVPLData);
    pass.def_property("maxVplCount", &VPLGenPass::getMaxVPLCount, &VPLGenPass::setMaxVPLCount);
}

VPLGenPass::VPLGenPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);

    // Create sample generator.
    mpSampleGenerator = SampleGenerator::create(mpDevice, SAMPLE_GENERATOR_DEFAULT);

    mpCounterStagingBuffer = mpDevice->createBuffer(sizeof(uint32_t), ResourceBindFlags::None, MemoryType::ReadBack, nullptr);
    mpVPLStagingBuffer =
        mpDevice->createBuffer(sizeof(VPLData) * kMaxVPLCountLimit, ResourceBindFlags::None, MemoryType::ReadBack, nullptr);
    mpFence = mpDevice->createFence();
}

void VPLGenPass::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        if (key == kMaxVPLCount)
            mParams.maxVPLCount = value;
        else if (key == kMaxPathDepth)
            mParams.maxPathDepth = value;
        else
            logWarning("Unknown property '{}' in VPLGenPass properties.", key);
    }
}

Properties VPLGenPass::getProperties() const
{
    Properties props;

    props[kMaxPathDepth] = mParams.maxPathDepth;
    props[kMaxVPLCount] = mParams.maxVPLCount;

    return props;
}

RenderPassReflection VPLGenPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    // Max sample per path = max path bounces + 2(starting point and ending point (if exists))
    reflector.addOutput("vpl", "Virtual point light(VPL) data")
        .rawBuffer(kMaxVPLCountLimit * sizeof(VPLData))
        .flags(RenderPassReflection::Field::Flags::Persistent);
    reflector.addOutput("vplCounter", "VPL counter buffer").rawBuffer(sizeof(uint32_t));

    addRenderPassOutputs(reflector, {kOutputChannel});

    return reflector;
}

void VPLGenPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Clear outputs if no scene is loaded.
    if (!mpScene)
    {
        clearRenderPassChannels(pRenderContext, {kOutputChannel}, renderData);
        return;
    }

    bool lightChanged = is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightsMoved) ||
                        is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightIntensityChanged) ||
                        is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightPropertiesChanged) ||
                        is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightCountChanged);
    bool envmapChanged = is_set(mpScene->getUpdates(), Scene::UpdateFlags::EnvMapChanged) ||
                         is_set(mpScene->getUpdates(), Scene::UpdateFlags::EnvMapPropertiesChanged);
    bool meshLightChanged = is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightCollectionChanged);

    bool sceneChanged = is_set(mpScene->getUpdates(), Scene::UpdateFlags::GeometryChanged);
    // Check for scene changes that require shader recompilation.
    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RecompileNeeded) || lightChanged || envmapChanged || meshLightChanged ||
        sceneChanged)
    {
        recreatePrograms();
        mRegenerate = true;
    }

    if (mpScene->getRenderSettings().useEmissiveLights)
    {
        mpScene->getLightCollection(pRenderContext);
    }

    auto& dict = renderData.getDictionary();

    // Update refresh flag if changes that affect the output have occured.
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, Falcor::RenderPassRefreshFlags::None);
        flags |= Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        dict[Falcor::kRenderPassRefreshFlags] = flags;
        recreatePrograms();
        mOptionsChanged = false;
        mRegenerate = true;
    }

    executeCreatePass(pRenderContext, renderData);
}

void VPLGenPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    mpScene = pScene;

    recreatePrograms();

    if (mpScene)
    {
        if (pScene->hasProceduralGeometry())
        {
            logWarning("VPLGenPass: This pass only supports triangles. Other types of geometry will be ignored.");
        }
    }
}

const std::vector<VPLData>& VPLGenPass::getVPLData() const
{
    return mVPLs;
}

void VPLGenPass::setMaxVPLCount(uint32_t value)
{
    mParams.maxVPLCount = value;
    mOptionsChanged = true;
}

uint32_t VPLGenPass::getMaxVPLCount() const
{
    return mParams.maxVPLCount;
}

void VPLGenPass::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    dirty |= widget.var("Max VPL count", mParams.maxVPLCount, 1u, kMaxVPLCountLimit);

    dirty |= widget.var("Max path depth", mParams.maxPathDepth, 1u, 16u);

    widget.text(fmt::format("VPL Count = {}", mVPLCount));

    if (dirty)
    {
        mOptionsChanged = true;
    }
}

void VPLGenPass::recreatePrograms()
{
    mpCreateVPLPass = nullptr;
    mRegenerate = true;
}

void VPLGenPass::executeCreatePass(RenderContext* pRenderContext, const RenderData& renderData)
{
    FALCOR_ASSERT(mpScene);

    FALCOR_PROFILE(pRenderContext, "VPL Create Pass");

    if (!mRegenerate)
        return;

    if (!mpCreateVPLPass)
    {
        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add(mpSampleGenerator->getDefines());

        defines.add("MAX_VPL_COUNT", std::to_string(mParams.maxVPLCount));
        defines.add("MAX_PATH_DEPTH", std::to_string(mParams.maxPathDepth));

        ProgramDesc baseDesc;
        baseDesc.addShaderModules(mpScene->getShaderModules());
        baseDesc.addTypeConformances(mpScene->getTypeConformances());

        ProgramDesc desc = baseDesc;
        desc.addShaderLibrary(kCreateVPLPassFilename).csEntry("main");
        mpCreateVPLPass = ComputePass::create(mpDevice, desc, defines, true);

        // Bind static resources.
        auto var = mpCreateVPLPass->getRootVar();
        if (mpScene)
            mpScene->bindShaderDataForRaytracing(pRenderContext, var["gScene"]);
        mpSampleGenerator->bindShaderData(var);
    }
    auto counterBuffer = static_ref_cast<Buffer>(renderData.getResource("vplCounter"));
    auto vplBuffer = static_ref_cast<Buffer>(renderData.getResource("vpl"));

    pRenderContext->clearUAV(counterBuffer->getUAV().get(), uint4(0));

    ShaderVar var = mpCreateVPLPass->getRootVar();

    var[kOutputChannel.texname] = renderData.getTexture(kOutputChannel.name);
    var[kVPLBuffer] = vplBuffer;
    var[kVPLCounterBuffer] = counterBuffer;

    // Compute dispatch
    uint32_t dispatchDim = mParams.maxVPLCount / (mParams.maxPathDepth);
    do
    {
        mpCreateVPLPass->execute(pRenderContext, uint3(dispatchDim, 1, 1));

        pRenderContext->copyResource(mpCounterStagingBuffer.get(), counterBuffer.get());
        pRenderContext->submit(true);

        mVPLCount = *reinterpret_cast<const uint32_t*>(mpCounterStagingBuffer->map());
        mpCounterStagingBuffer->unmap();
        FALCOR_ASSERT(mVPLCount <= mParams.maxVPLCount);
    } while (mVPLCount < mParams.maxVPLCount);

    // VPL read back
    {
        pRenderContext->copyResource(mpVPLStagingBuffer.get(), vplBuffer.get());
        mVPLs.resize(mVPLCount);
        pRenderContext->submit(true);

        const auto* data = reinterpret_cast<const VPLData*>(mpVPLStagingBuffer->map());
        std::memcpy(mVPLs.data(), data, sizeof(VPLData) * mVPLCount);
        mpVPLStagingBuffer->unmap();
    }
    mRegenerate = false;
}
