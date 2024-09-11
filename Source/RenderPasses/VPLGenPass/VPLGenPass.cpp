#include "VPLGenPass.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"
#include "VPLData.slang"

using namespace Falcor;

namespace
{

static_assert(sizeof(VPLData) % 16 == 0, "VPLData struct should be 16-byte aligned.");

const std::string kCreateVPLPassFilename = "RenderPasses/VPLGenPass/CreateVPL.cs.slang";

// Scripting options.
const std::string kPathsPerLight = "pathsPerLight";
const std::string kMaxPathLength = "maxPathLength";

// VPL configuration limits.
static const uint32_t kMaxPathPerLight = 1024u;
static const uint32_t kMaxPathBounces = 8u;

// Outputs
const std::string kVPLBuffer = "gVPL";
const std::string kVPLBufferDesc = "VPL data buffer";
const ChannelDesc kOutputChannel{"output", "gOutput", "An output to ensure VPLGenPass being executed", false, ResourceFormat::RGBA32Float};
} // namespace

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, VPLGenPass>();
}

VPLGenPass::VPLGenPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);

    // Create sample generator.
    mpSampleGenerator = SampleGenerator::create(mpDevice, SAMPLE_GENERATOR_DEFAULT);
}

void VPLGenPass::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        if (key == kPathsPerLight)
            mParams.pathsPerLight = value;
        else if (key == kMaxPathLength)
            mParams.maxPathLength = value;
        else
            logWarning("Unknown property '{}' in VPLGenPass properties.", key);
    }
}

Properties VPLGenPass::getProperties() const
{
    Properties props;

    props[kPathsPerLight] = mParams.pathsPerLight;
    props[kMaxPathLength] = mParams.maxPathLength;

    return props;
}

RenderPassReflection VPLGenPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    // Max sample per path = max path bounces + 2(starting point and ending point (if exists))
    uint32_t maxSamples = kMaxPathPerLight * (kMaxPathBounces + 2u);
    reflector.addOutput("gVPL", "Virtual point light(VPL) data").rawBuffer(maxSamples * sizeof(VPLData));

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

    auto& dict = renderData.getDictionary();

    // Update refresh flag if changes that affect the output have occured.
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, Falcor::RenderPassRefreshFlags::None);
        flags |= Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        dict[Falcor::kRenderPassRefreshFlags] = flags;
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

    dirty |= widget.var("Paths/light", mParams.pathsPerLight, 1u, 1024u);

    dirty |= widget.var("Max path length", mParams.maxPathLength, 1u, 8u);

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
    FALCOR_PROFILE(pRenderContext, "VPL Create Pass");

    FALCOR_ASSERT(mpScene);

    if (!mpCreateVPLPass)
    {
        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add(mpSampleGenerator->getDefines());

        defines.add("PATHS_PER_LIGHT", std::to_string(mParams.pathsPerLight));
        defines.add("MAX_PATH_LENGTH", std::to_string(mParams.maxPathLength));

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

    ShaderVar var = mpCreateVPLPass->getRootVar();

    var[kOutputChannel.texname] = renderData.getTexture(kOutputChannel.desc);
    var[kVPLBuffer] = static_ref_cast<Buffer>(renderData.getResource(kVPLBufferDesc));

    // Compute dispatch
    mpCreateVPLPass->execute(pRenderContext, uint3(1, 1, 1));
}
