#include "InstantRadiosity.h"
#include "Core/Pass/ComputePass.h"
#include "Core/Program/DefineList.h"
#include "Core/Program/Program.h"
#include "Core/Program/ShaderVar.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, InstantRadiosity>();
}

namespace
{
const char kPhotonGenPassFile[] = "RenderPasses/InstantRadiosity/PhotonGenPass.cs.slang";
const char kShadingPassFile[] = "RenderPasses/InstantRadiosity/ShadingPass.cs.slang";

const ChannelList kInputChannels = {
    // clang-format off
    { "vbuffer",        "vbuffer",     "Visibility buffer in packed format" },
    { "viewW",    "viewW",       "World-space view direction (xyz float format)", true /* optional */ },
    // clang-format on
};

const std::string kOutput = "output";

const char kMaxVPLCount[] = "maxVPLCount";
const char kMaxPathLength[] = "maxPathLength";
const char kNumVPLSamples[] = "numVPLSamples";
} // namespace

InstantRadiosity::InstantRadiosity(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);

    // Create a sample generator.
    mpSampleGenerator = SampleGenerator::create(mpDevice, SAMPLE_GENERATOR_UNIFORM);
    FALCOR_ASSERT(mpSampleGenerator);
}

void InstantRadiosity::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        if (key == kMaxVPLCount)
            mParams.maxVPLCount = value;
        else if (key == kMaxPathLength)
            mParams.maxPathLength = value;
        else if(key == kNumVPLSamples)
            mParams.numVPLSamples = value;
        else
            logWarning("Unknown property '{}' in InstantRadiosity properties.", key);
    }
}

Properties InstantRadiosity::getProperties() const
{
    Properties props;
    props[kMaxVPLCount] = mParams.maxVPLCount;
    props[kMaxPathLength] = mParams.maxPathLength;
    return props;
}

RenderPassReflection InstantRadiosity::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    addRenderPassInputs(reflector, kInputChannels);
    reflector.addOutput(kOutput, "Scene debugger output").bindFlags(ResourceBindFlags::UnorderedAccess).format(ResourceFormat::RGBA32Float);

    return reflector;
}

void InstantRadiosity::buildVPLs(RenderContext* pRenderContext, const ref<LightCollection>& lightData)
{
    // Clear VPL buffer.
    uint32_t zero = 0;
    mpPhotonCounter->setBlob(&zero, 0, sizeof(uint32_t));
    // Distribute lights to cs threads.
    if (!mpPhotonGenPass)
    {
        logError("InstantRadiosity: VPL generation pass not initialized.");
        return;
    }
    const auto& meshTriangles = lightData->getMeshLightTriangles(pRenderContext);
    mpScene->setRaytracingShaderData(pRenderContext, mpPhotonGenPass->getRootVar());

    ShaderVar var = mpPhotonGenPass->getRootVar()["CB"]["gPhotonGenerator"];
    var["params"].setBlob(mParams);

    mpPhotonGenPass->execute(pRenderContext, uint3(meshTriangles.size(), mParams.numVPLSamples, 1));
    mRebuildVPLs = false;
}

void InstantRadiosity::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Update refresh flag if options that affect the output have changed.
    auto& dict = renderData.getDictionary();
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, RenderPassRefreshFlags::None);
        dict[Falcor::kRenderPassRefreshFlags] = flags | Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        mOptionsChanged = false;
        mRebuildVPLs = true;
    }

    const auto& pOutput = renderData.getTexture(kOutput);
    // If we have no scene, just clear the outputs and return.
    if (!mpScene)
    {
        pRenderContext->clearUAV(pOutput->getUAV().get(), float4(0.f));
        return;
    }

    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RecompileNeeded) ||
        is_set(mpScene->getUpdates(), Scene::UpdateFlags::GeometryChanged))
    {
        FALCOR_THROW("This render pass does not support scene changes that require shader recompilation.");
    }

    mRebuildVPLs = mRebuildVPLs || is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightPropertiesChanged) ||
                   is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightCollectionChanged) ||
                   is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightCountChanged) ||
                   is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightIntensityChanged) ||
                   is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightsMoved);
    // Rebuild VPLs pass execution on light sources or configs change.
    if (mRebuildVPLs)
    {
        buildVPLs(pRenderContext, mpScene->getLightCollection(pRenderContext));
    }

    // Read back the number of photons generated.
    {
        pRenderContext->copyResource(mpPhotonCounterStaging.get(), mpPhotonCounter.get());
        pRenderContext->submit(false);
        pRenderContext->signal(mpReadbackFence.get());

        mpReadbackFence->wait();
        const uint32_t* pPhotonCount = reinterpret_cast<const uint32_t*>(mpPhotonCounterStaging->map());
        std::memcpy(&mVPLCount, pPhotonCount, sizeof(uint32_t));
        mpPhotonCounterStaging->unmap();
    }

    // Create Shading pass.
    if (!mpShadingPass)
    {
        ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
        desc.addShaderLibrary(kShadingPassFile).csEntry("main");
        desc.addTypeConformances(mpScene->getTypeConformances());

        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add(mpSampleGenerator->getDefines());
        defines.add(getValidResourceDefines(kInputChannels, renderData));

        mpShadingPass = ComputePass::create(mpDevice, desc, defines, true);

        // Bind Static resources.
        auto var = mpShadingPass->getRootVar();
        mpScene->setRaytracingShaderData(pRenderContext, var);
        mpSampleGenerator->bindShaderData(var);
    }

    // Bind dynamic resources.
    ShaderVar var = mpShadingPass->getRootVar()["CB"]["gIRShader"];
    var["PRNGDimension"] = dict.keyExists(kRenderPassPRNGDimension) ? dict[kRenderPassPRNGDimension] : 0u;
    var["frameDim"] = renderData.getDefaultTextureDims();
    var["photonData"] = mpPhotonData;
    var["photonCount"].set(mVPLCount);
    var["output"] = pOutput;

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

    // Get dimensions of ray dispatch.
    const uint2 targetDim = renderData.getDefaultTextureDims();
    FALCOR_ASSERT(targetDim.x > 0 && targetDim.y > 0);
    mpShadingPass->execute(pRenderContext, uint3(targetDim, 1));
}

void InstantRadiosity::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    dirty |= widget.var("Max VPL count", mParams.maxVPLCount, 0u, 1u << 16);
    dirty |= widget.var("Max path length", mParams.maxPathLength, 0u, 1024u);
    dirty |= widget.var("Num VPL samples", mParams.numVPLSamples, 1u, 1024u);

    widget.text("VPL count: " + std::to_string(mVPLCount));
    // If rendering options that modify the output have changed, set flag to indicate that.
    // In execute() we will pass the flag to other passes for reset of temporal data etc.
    if (dirty)
    {
        mOptionsChanged = true;
    }
}

void InstantRadiosity::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    // Clear data for previous scene.
    // After changing scene, the programs should to be recreated.

    mpPhotonGenPass = nullptr;
    mpPhotonData = nullptr;
    mpPhotonCounter = nullptr;
    mpPhotonCounterStaging = nullptr;
    mpReadbackFence = nullptr;
    mpShadingPass = nullptr;
    mRebuildVPLs = true;

    // Set new scene.
    mpScene = pScene;

    if (mpScene)
    {
        if (pScene->hasGeometryType(Scene::GeometryType::Custom))
        {
            logWarning("InstantRadiosity: This render pass does not support custom primitives.");
        }

        // Create photon(VPL) generation pass program.
        ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
        desc.addShaderLibrary(kPhotonGenPassFile).csEntry("main");
        desc.addTypeConformances(mpScene->getTypeConformances());
        DefineList defines;
        defines.add(mpSampleGenerator->getDefines());
        defines.add(mpScene->getSceneDefines());
        mpPhotonGenPass = ComputePass::create(mpDevice, desc, defines);

        // Bind VPL buffer.
        auto var = mpPhotonGenPass->getRootVar()["CB"]["gPhotonGenerator"];

        if (!mpPhotonData)
        {
            mpPhotonData = mpDevice->createStructuredBuffer(
                var["photonData"],
                mParams.maxVPLCount,
                ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
                MemoryType::DeviceLocal,
                nullptr,
                false
            );
        }
        var["photonData"] = mpPhotonData;

        if (!mpPhotonCounter)
        {
            mpReadbackFence = mpDevice->createFence();

            static const uint32_t zero = 0;
            mpPhotonCounter = mpDevice->createBuffer(sizeof(uint32_t), ResourceBindFlags::UnorderedAccess, MemoryType::DeviceLocal, &zero);
            mpPhotonCounterStaging = mpDevice->createBuffer(sizeof(uint32_t), ResourceBindFlags::None, MemoryType::ReadBack);
        }
        var["totalPhotonCounter"] = mpPhotonCounter;
    }
}
