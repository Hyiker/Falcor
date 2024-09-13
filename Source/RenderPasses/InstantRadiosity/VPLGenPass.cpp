#include "VPLGenPass.h"
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
static const uint32_t kMaxVPLCountLimit = 1000u;
static const uint32_t kMaxPathDepthLimit = 12u;

// Outputs
const std::string kVPLBuffer = "gVPL";
const std::string kVPLBufferDesc = "VPL data buffer";
const ChannelDesc kOutputChannel{"output", "gOutput", "An output to ensure VPLGenPass being executed", false, ResourceFormat::RGBA32Float};
} // namespace

VPLGenPass::VPLGenPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);

    // Create sample generator.
    mpSampleGenerator = SampleGenerator::create(mpDevice, SAMPLE_GENERATOR_DEFAULT);

    mpCounterBuffer = mpDevice->createBuffer(sizeof(uint32_t));
    mpCounterStagingBuffer = mpDevice->createBuffer(sizeof(uint32_t), ResourceBindFlags::None, MemoryType::ReadBack, nullptr);
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
    reflector.addOutput("vpl", "Virtual point light(VPL) data").rawBuffer(kMaxVPLCountLimit * sizeof(VPLData));

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
    // Check for scene changes that require shader recompilation.
    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RecompileNeeded) || lightChanged || envmapChanged || meshLightChanged)
    {
        recreatePrograms();
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

void VPLGenPass::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    dirty |= widget.var("Max VPL count", mParams.maxVPLCount, 1u, 1024u);

    dirty |= widget.var("Max path depth", mParams.maxPathDepth, 1u, 8u);

    widget.text(fmt::format("VPL Count = {}", mVPLCount));

    if (dirty)
    {
        mOptionsChanged = true;
    }
}

void VPLGenPass::recreatePrograms()
{
    mpCreateVPLPass = nullptr;
}

void VPLGenPass::executeCreatePass(RenderContext* pRenderContext, const RenderData& renderData)
{
    FALCOR_ASSERT(mpScene);

    FALCOR_PROFILE(pRenderContext, "VPL Create Pass");

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
        mpScene->setRaytracingShaderData(pRenderContext, var);
        mpSampleGenerator->bindShaderData(var);
    }

    pRenderContext->clearUAV(mpCounterBuffer->getUAV().get(), uint4(0));

    ShaderVar var = mpCreateVPLPass->getRootVar();

    var[kOutputChannel.texname] = renderData.getTexture(kOutputChannel.name);
    var[kVPLBuffer] = static_ref_cast<Buffer>(renderData.getResource("vpl"));
    var["counter"] = mpCounterBuffer;

    // Compute dispatch
    uint32_t dispatchDim = mParams.maxVPLCount / (mParams.maxPathDepth);
    do
    {
        mpCreateVPLPass->execute(pRenderContext, uint3(dispatchDim, 1, 1));

        pRenderContext->submit(true);

        mVPLCount = *reinterpret_cast<const uint32_t*>(mpCounterStagingBuffer->map());
        mpCounterStagingBuffer->unmap();
        FALCOR_ASSERT(mVPLCount <= mParams.maxVPLCount);

        pRenderContext->copyResource(mpCounterStagingBuffer.get(), mpCounterBuffer.get());
    } while (mVPLCount < mParams.maxVPLCount);
}
