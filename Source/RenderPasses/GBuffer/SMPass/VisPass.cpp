#include "VisPass.h"
#include "Falcor.h"
#include "RenderGraph/RenderPassStandardFlags.h"

namespace
{
std::string kLightCount = "lightCount";
const std::string kDepthPassProgramFile = "RenderPasses/GBuffer/GBuffer/DepthPass.3d.slang"; // TODEBUG
const std::string kVisPassProgramFile = "RenderPasses/GBuffer/SMPass/visibilityPass.slang";

const RasterizerState::CullMode kDefaultCullMode = RasterizerState::CullMode::None;

const std::string kVBufferName = "vbuffer";
const ChannelList kSMExtraChannels = {
    {kVBufferName, "gVBuffer", "Visibility buffer", true /* optional */, ResourceFormat::Unknown /* set at runtime */},
};

const std::string kDepthName = "depth";
} // namespace

VisPass::VisPass(ref<Device> pDevice, const Properties& dict) : GBuffer(pDevice)
{
    // Check for required features.
    if (!mpDevice->isFeatureSupported(Device::SupportedFeatures::Barycentrics))
    {
        throw RuntimeError("GBufferRaster: Pixel shader barycentrics are not supported by the current device");
    }

    // parseDictionary(dict);
    // For Visibility
    mDepthPassVis.pState = GraphicsState::create(mpDevice);
    mVisibilityPass.pState = GraphicsState::create(mpDevice);

    // For visibility
    DepthStencilState::Desc dsDesc_vis;
    dsDesc_vis.setDepthFunc(ComparisonFunc::Equal).setDepthWriteMask(false);
    ref<DepthStencilState> pDsState_vis = DepthStencilState::create(dsDesc_vis);
    mVisibilityPass.pState->setDepthStencilState(pDsState_vis);

    mpFbo = Fbo::create(mpDevice);
    for (const auto& [key, value] : dict)
    {
        if (key == kLightCount)
            lightCount = (uint32_t)value;
    }
}

inline void addRenderPassOutputsMSAA(
    RenderPassReflection& reflector,
    const ChannelList& channels,
    ResourceBindFlags bindFlags = ResourceBindFlags::UnorderedAccess,
    const uint2 dim = {},
    uint32_t sampleCount = 1
)
{
    for (const auto& it : channels)
    {
        auto& tex = reflector.addOutput(it.name, it.desc).texture2D(dim.x, dim.y, sampleCount);
        tex.bindFlags(bindFlags);
        if (it.format != ResourceFormat::Unknown)
            tex.format(it.format);
        if (it.optional)
            tex.flags(RenderPassReflection::Field::Flags::Optional);
    }
}
RenderPassReflection VisPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    const uint2 sz = RenderPassHelpers::calculateIOSize(mOutputSizeSelection, mFixedOutputSize, compileData.defaultTexDims);
    reflector.addOutput(kDepthName, "Depth buffer")
        .format(ResourceFormat::D32Float)
        .bindFlags(ResourceBindFlags::DepthStencil)
        .texture2D(sz.x, sz.y, multisample);

    reflector.addInput("cubeMap", "cube Map")
        .format(ResourceFormat::R32Float)
        .bindFlags(ResourceBindFlags::UnorderedAccess)
        .texture2D(mMapSize.x, mMapSize.y, 1, 1, 6 * lightCount);

    /*reflector.addOutput(name, "visibility")
        .format(ResourceFormat::RGBA32Float)
        .bindFlags(ResourceBindFlags::UnorderedAccess)
        .texture2D(sz.x, sz.y);*/

    for (uint i = 0; i < lightCount; i++)
    {
        // visibility
        // reflector.addOutput(name + std::to_string(i), "vis")
        //     .format(ResourceFormat::RGBA32Float)
        //     .bindFlags(ResourceBindFlags::UnorderedAccess)
        //     .texture2D(sz.x, sz.y);

        reflector.addOutput("gToLight" + std::to_string(i), "gToLight")
            .format(ResourceFormat::RG32Float)
            .bindFlags(ResourceBindFlags::UnorderedAccess)
            .texture2D(sz.x, sz.y);
        // TODO: Use RG32Float instead?
        reflector.addOutput("gToLightVec" + std::to_string(i), "gToLightVec")
            .format(ResourceFormat::RGBA32Float)
            .bindFlags(ResourceBindFlags::UnorderedAccess)
            .texture2D(sz.x, sz.y);
        reflector.addOutput("gHalfVec" + std::to_string(i), "gHalfVec")
            .format(ResourceFormat::RGBA32Float)
            .bindFlags(ResourceBindFlags::UnorderedAccess)
            .texture2D(sz.x, sz.y);
    }
    addRenderPassOutputsMSAA(reflector, kGBufferChannels, ResourceBindFlags::RenderTarget, sz, multisample);
    addRenderPassOutputs(reflector, kSMExtraChannels, ResourceBindFlags::UnorderedAccess, sz);

    reflector.getField(kVBufferName)->format(mVBufferFormat);
    return reflector;
}

void VisPass::compile(RenderContext* pRenderContext, const CompileData& compileData)
{
    GBuffer::compile(pRenderContext, compileData);
}

void VisPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    GBuffer::setScene(pRenderContext, pScene);

    // For visibility
    mDepthPassVis.pProgram = nullptr;
    mDepthPassVis.pVars = nullptr;

    mVisibilityPass.pProgram = nullptr;
    mVisibilityPass.pVars = nullptr;

    if (pScene)
    {
        if (pScene->getMeshVao() && pScene->getMeshVao()->getPrimitiveTopology() != Vao::Topology::TriangleList)
        {
            FALCOR_THROW("GBufferRaster: Requires triangle list geometry due to usage of SV_Barycentrics.");
        }
    }
}

void VisPass::onSceneUpdates(RenderContext* pRenderContext, Scene::UpdateFlags sceneUpdates) {}

Properties VisPass::getProperties() const
{
    return {};
}

void UAVclear(ref<Texture> pTex, RenderContext* pRenderContext)
{
    if (pTex)
    {
        if (isIntegerFormat(pTex->getFormat()))
        {
            pRenderContext->clearUAV(pTex->getUAV().get(), uint4(0));
        }
        else
        {
            pRenderContext->clearUAV(pTex->getUAV().get(), float4(0.f));
        }
    }
}

void VisPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // If there is no scene, clear depth buffer and return.
    if (mpScene == nullptr)
        return;

    // Check for scene changes.
    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RecompileNeeded))
    {
        mDepthPassVis.pProgram = nullptr;
        mDepthPassVis.pVars = nullptr;
        mVisibilityPass.pProgram = nullptr;
        mVisibilityPass.pVars = nullptr;
    }

    auto& dict = renderData.getDictionary();
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, RenderPassRefreshFlags::None);
        dict[Falcor::kRenderPassRefreshFlags] = flags | Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        mOptionsChanged = false;
    }

    // 初始化
    GBuffer::execute(pRenderContext, renderData);
    RasterizerState::CullMode cullMode = mForceCullMode ? mCullMode : kDefaultCullMode; // 过滤掉背向的

    // Update frame dimension based on render pass output.
    auto pDepth = renderData.getTexture(kDepthName);
    FALCOR_ASSERT(pDepth);
    updateFrameDim(uint2(pDepth->getWidth(), pDepth->getHeight()));

    // Clear depth buffer.
    pRenderContext->clearDsv(pDepth->getDSV().get(), 1.f, 0);

    // Bind primary channels as render targets and clear them.
    for (size_t i = 0; i < kGBufferChannels.size(); ++i)
    {
        ref<Texture> pTex0 = getOutput(renderData, kGBufferChannels[i].name);
        mpFbo->attachColorTarget(pTex0, uint32_t(i));
    }

    pRenderContext->clearFbo(mpFbo.get(), float4(0), 1.f, 0, FboAttachmentType::Color);

    // Clear extra output buffers.
    // Clear UAV buffers
    clearRenderPassChannels(pRenderContext, kSMExtraChannels, renderData);

    // auto pTex = renderData.getTexture("visibility");
    // UAVclear(pTex, pRenderContext);
    for (uint i = 0; i < lightCount; i++)
    {
        // auto _pTex = renderData.getTexture("visibility" + std::to_string(i));
        // UAVclear(_pTex, pRenderContext);
        auto _pTexL = renderData.getTexture("gToLight" + std::to_string(i));
        UAVclear(_pTexL, pRenderContext);
        auto _pTexV = renderData.getTexture("gToLightVec" + std::to_string(i));
        UAVclear(_pTexV, pRenderContext);
        auto _pTexH = renderData.getTexture("gHalfVec" + std::to_string(i));
        UAVclear(_pTexH, pRenderContext);
    }

    // Create programs
    {
        // For Visibility
        // Depth
        if (!mDepthPassVis.pProgram)
        {
            ProgramDesc desc;
            desc.addShaderModules(mpScene->getShaderModules());
            desc.addShaderLibrary(kDepthPassProgramFile).vsEntry("vsMain").psEntry("psMain");
            desc.addTypeConformances(mpScene->getTypeConformances());
            desc.setShaderModel(ShaderModel::SM6_2);

            mDepthPassVis.pProgram = Program::create(mpDevice, desc, mpScene->getSceneDefines());
            mDepthPassVis.pState->setProgram(mDepthPassVis.pProgram);
        }

        // Visibility
        if (!mVisibilityPass.pProgram)
        {
            ProgramDesc desc;
            desc.addShaderModules(mpScene->getShaderModules());
            desc.addShaderLibrary(kVisPassProgramFile).vsEntry("vsMain").psEntry("psMain");
            desc.addTypeConformances(mpScene->getTypeConformances()); // type一致性
            desc.setShaderModel(ShaderModel::SM6_2);                  // slang版本号？

            mVisibilityPass.pProgram = Program::create(mpDevice, desc, mpScene->getSceneDefines());
            mVisibilityPass.pState->setProgram(mVisibilityPass.pProgram);
        }
    }

    // Depth pass.
    {
        // Set program defines.
        mDepthPassVis.pState->getProgram()->addDefine("USE_ALPHA_TEST", mUseAlphaTest ? "1" : "0");

        // Create program vars.
        if (!mDepthPassVis.pVars)
            mDepthPassVis.pVars = ProgramVars::create(mpDevice, mDepthPassVis.pProgram.get());

        mpFbo->attachDepthStencilTarget(pDepth);
        mDepthPassVis.pState->setFbo(mpFbo);
        mpScene->rasterize(pRenderContext, mDepthPassVis.pState.get(), mDepthPassVis.pVars.get(), cullMode);
    }

    // vis pass
    {
        mVisibilityPass.pState->getProgram()->addDefine("USE_ALPHA_TEST", mUseAlphaTest ? "1" : "0");
        mVisibilityPass.pProgram->addDefine("ADJUST_SHADING_NORMALS", mAdjustShadingNormals ? "1" : "0");
        mVisibilityPass.pProgram->addDefines(getValidResourceDefines(kGBufferChannels, renderData));
        mVisibilityPass.pProgram->addDefines(getValidResourceDefines(kSMExtraChannels, renderData));

        if (!mVisibilityPass.pVars)
            mVisibilityPass.pVars = ProgramVars::create(mpDevice, mVisibilityPass.pProgram.get());

        auto visPassVar = mVisibilityPass.pVars->getRootVar();
        for (const auto& channel : kSMExtraChannels)
        {
            ref<Texture> pTex0 = getOutput(renderData, channel.name);
            visPassVar[channel.texname] = pTex0;
            std::string dd = "is_valid_" + channel.texname;
            mVisibilityPass.pProgram->addDefine(dd, "1");
        }

        visPassVar["PerFrameCB"]["gFrameDim"] = mFrameDim;

        auto tex = renderData.getTexture("cubeMap");
        visPassVar["gCubeMap"] = tex;
        visPassVar["smCtrParam"]["width"] = tex->getWidth();
        visPassVar["smCtrParam"]["ENABLE_PCF"] = mENABLEPCF;
        visPassVar["smCtrParam"]["FILTER_PCF_S"] = mFILTER_PCF_S; // int, width = 2*FILTER_PCF_S+1
        visPassVar["smCtrParam"]["BIASMAX"] = mBIASMAX;
        visPassVar["smCtrParam"]["BIASMIN"] = mBIASMIN;

        visPassVar["smCtrParam"]["nLight"] = lightCount;

        for (uint32_t i = 0; i < lightCount; i++)
        {
            float3 pos = dict["lightPos" + std::to_string(i)];
            visPassVar["LightPos"]["gLightPos"][i] = pos;

            // mVisibilityPass.pVars["gVisibility"] = renderData.getTexture("visibility");
            auto texL = renderData.getTexture("gToLight" + std::to_string(i));
            visPassVar["gToLight"][i] = texL;

            auto texV = renderData.getTexture("gToLightVec" + std::to_string(i));
            visPassVar["gToLightVec"][i] = texV;

            auto texH = renderData.getTexture("gHalfVec" + std::to_string(i));
            visPassVar["gHalfVec"][i] = texH;
        }

        mVisibilityPass.pState->setFbo(mpFbo); // Sets the viewport

        // Rasterize the scene.
        mpScene->rasterize(pRenderContext, mVisibilityPass.pState.get(), mVisibilityPass.pVars.get(), cullMode);
    }
}
