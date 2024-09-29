#include "SMPass.h"
#include "Falcor.h"
#include "RenderGraph/RenderPassStandardFlags.h"

namespace
{
const std::string kDepthPassProgramFile = "RenderPasses/GBuffer/SMPass/DepthPass.3d.slang";
const std::string kSMPassProgramFile = "RenderPasses/GBuffer/SMPass/SMPass.slang";
std::string kLightCount = "lightCount";
const RasterizerState::CullMode kDefaultCullMode = RasterizerState::CullMode::Back;
const std::string kVBufferName = "vbuffer";
// 这里输出有六个面，对应一个cube，同时还需要指定朝向
const std::string kDepthName = "depth";
const ChannelList kLightSampleChannels = {
    {"position", "gPosition", "position at sample", true /* optional */, ResourceFormat::RGBA32Float /* set at runtime */},
    {"normal", "gNormal", "normal at sample", true /* optional */, ResourceFormat::RGBA32Float}};
// {"intensity", "gIntensity", "intensity at sample", true /* optional */, ResourceFormat::RGBA32Float}};
const ChannelList kInternalChannels = {{"mean", "mean", "mean", true, ResourceFormat::RGBA32Float}};
const ChannelList kRSMChannels = {
    {"RSMposition", "gRSMposition", "Position in shadow map", true, ResourceFormat::RGBA32Float},
    {"RSMnormal", "gRSMnormal", "Normal in shadow map", true, ResourceFormat::RGBA32Float},
    {"RSMflux", "gRSMflux", "Flux in shadow map", true, ResourceFormat::RGBA32Float}};
} // namespace

SMPass::SMPass(ref<Device> pDevice, const Properties& dict) : GBuffer(std::move(pDevice))
{
    // Check for required features.
    if (!mpDevice->isFeatureSupported(Device::SupportedFeatures::Barycentrics))
    {
        throw RuntimeError("GBufferRaster: Pixel shader barycentrics are not supported by the current device");
    }
    parseProperties(dict);
    // Initialize graphics state
    mDepthPass.pState = GraphicsState::create(mpDevice);
    // Initialize graphics state
    mSMPass.pState = GraphicsState::create(mpDevice);

    // Set depth function
    DepthStencilState::Desc dsDesc;
    dsDesc.setDepthFunc(ComparisonFunc::Equal).setDepthWriteMask(false);
    ref<DepthStencilState> pDsState = DepthStencilState::create(dsDesc);
    mSMPass.pState->setDepthStencilState(pDsState);

    mpFbo = Fbo::create(mpDevice);
    for (const auto& [key, value] : dict)
    {
        if (key == kLightCount)
            lightCount = (uint32_t)value;
    }
}

void SMPass::averageLightSamples(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Sample direct vpls each frame is time-waste.. also, the consistency is guaranteed?
    // Only update modified light?
    // Copy from GPU to Memory?? and then accumulate then push to GPU again??
    // Direct operate them in parallel in GPU.

    std::vector<float> vecPosW =
        CopyContext::ReadTextureTask::create(pRenderContext, renderData.getTexture("position").get(), 0)->getData<float>();
    std::vector<float> vecNorm =
        CopyContext::ReadTextureTask::create(pRenderContext, renderData.getTexture("normal").get(), 0)->getData<float>();
    // std::vector<float> vecIntensity  = CopyContext::ReadTextureTask::create(pRenderContext,
    // renderData.getTexture("intensity").get(),0)->getFloatData();
    uint32_t index = 0;
    for (uint32_t i = 0; i < lightCount; i++)
    {
        float3 pos(0.f);
        float3 norm(0.f);
        // float3 intensity;
        for (int j = 0; j < 500; j++)
        {
            // calculate average posW for each light
            pos.x += vecPosW[index];
            pos.y += vecPosW[index + 1];
            pos.z += vecPosW[index + 2];
            // calculate average norm for each light
            norm.x += vecNorm[index];
            norm.y += vecNorm[index + 1];
            norm.z += vecNorm[index + 2];
            // intensity += float3(vecIntensity[index], vecIntensity[index + 1], vecIntensity[index + 2]);
            index += 4; // x y z w
        }
        pos *= 0.002f;
        norm *= 0.002f;
        // intensity *= 0.002;

        lightNormList.push_back(math::normalize(norm));
        lightPosList.push_back(pos);
    }

    auto& dict = renderData.getDictionary();
    for (uint32_t i = 0; i < lightCount; i++)
    {
        dict["lightPos" + std::to_string(i)] = lightPosList[i];
    }
}

RenderPassReflection SMPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    uint2 dim(500, lightCount);
    addRenderPassInputs(reflector, kLightSampleChannels, ResourceBindFlags::UnorderedAccess, dim);
    reflector.addOutput(kDepthName, "Depth buffer")
        .format(ResourceFormat::D32Float)
        .bindFlags(ResourceBindFlags::DepthStencil)
        .texture2D(mMapSize.x, mMapSize.y);
    reflector.addOutput("cubeMap", "cube Map")
        .format(ResourceFormat::R32Float)
        .bindFlags(ResourceBindFlags::UnorderedAccess)
        .texture2D(mMapSize.x, mMapSize.y, 1, 1, 6 * lightCount);

    for (const auto& it : kRSMChannels)
    {
        auto& tex = reflector.addOutput(it.name, it.desc)
                        .format(ResourceFormat::RGBA32Float)
                        .bindFlags(ResourceBindFlags::UnorderedAccess)
                        .texture2D(mMapSize.x / rsm_down_sampling_scale, mMapSize.y / rsm_down_sampling_scale, 1, 1, 6 * lightCount);
        // if (it.format != ResourceFormat::Unknown) tex.format(it.format);
        // if (it.optional) tex.flags(RenderPassReflection::Field::Flags::Optional);

        // Visualize texture arrays
        for (uint32_t j = 0; j < lightCount; j++)
        {
            for (uint32_t faceID = 0; faceID < 6; faceID++)
            {
                std::string texName = it.name + "Vis_" + std::to_string(j) + "_" + std::to_string(faceID);
                reflector.addOutput(texName, texName)
                    .format(ResourceFormat::RGBA32Float)
                    .bindFlags(ResourceBindFlags::UnorderedAccess)
                    .texture2D(mMapSize.x / rsm_down_sampling_scale, mMapSize.y / rsm_down_sampling_scale);
            }
        }
    }

    return reflector;
}

void SMPass::compile(RenderContext* pRenderContext, const CompileData& compileData)
{
    GBuffer::compile(pRenderContext, compileData);
}

void SMPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    GBuffer::setScene(pRenderContext, pScene);

    mDepthPass.pProgram = nullptr;
    mDepthPass.pVars = nullptr;

    mSMPass.pProgram = nullptr;
    mSMPass.pVars = nullptr;
    if (pScene)
    {
        if (pScene->getMeshVao() && pScene->getMeshVao()->getPrimitiveTopology() != Vao::Topology::TriangleList)
        {
            FALCOR_THROW("GBufferRaster: Requires triangle list geometry due to usage of SV_Barycentrics.");
        }
    }
}

void SMPass::onSceneUpdates(RenderContext* pRenderContext, Scene::UpdateFlags sceneUpdates) {}

void SMPass::renderUI(Gui::Widgets& widget) {}

Properties SMPass::getProperties() const
{
    return {};
}
inline void clearRenderPassChannels(
    RenderContext* pRenderContext,
    const ChannelList& channels,
    const RenderData& renderData,
    bool invalid_flag
)
{
    for (const auto& channel : channels)
    {
        auto pTex = renderData.getTexture(channel.name);
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
}
void SMPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // If there is no scene, clear depth buffer and return.
    if (mpScene == nullptr)
        return;

    // Check for scene changes.
    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RecompileNeeded))
    {
        mDepthPass.pProgram = nullptr;
        mDepthPass.pVars = nullptr;
        mSMPass.pProgram = nullptr;
        mSMPass.pVars = nullptr;
    }

    // Get light params
    lightPosList.clear();
    lightNormList.clear();
    averageLightSamples(pRenderContext, renderData);

    // Falcor::InternalDictionary& iDict = renderData.getDictionary();
    // iDict["width"] = mMapSize.x;

    auto& dict = renderData.getDictionary();
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, RenderPassRefreshFlags::None);
        dict[Falcor::kRenderPassRefreshFlags] = flags | Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        mOptionsChanged = false;
    }

    RasterizerState::CullMode cullMode = mForceCullMode ? mCullMode : kDefaultCullMode; // 过滤掉背向的

    // Update frame dimension based on render pass output.
    auto pDepth = renderData.getTexture(kDepthName);
    FALCOR_ASSERT(pDepth);
    updateFrameDim(uint2(pDepth->getWidth(), pDepth->getHeight()));

    // Clear UAV buffers
    auto pTex = renderData.getTexture("cubeMap");
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

    clearRenderPassChannels(pRenderContext, kRSMChannels, renderData, true);
    for (const auto& channel : kRSMChannels)
    {
        for (uint32_t j = 0; j < lightCount; j++)
        {
            for (uint32_t faceID = 0; faceID < 6; faceID++)
            {
                std::string texName = channel.name + "Vis_" + std::to_string(j) + "_" + std::to_string(faceID);
                auto pFaceTex = renderData.getTexture(texName);
                if (pFaceTex)
                {
                    if (isIntegerFormat(pFaceTex->getFormat()))
                    {
                        pRenderContext->clearUAV(pFaceTex->getUAV().get(), uint4(0));
                    }
                    else
                    {
                        pRenderContext->clearUAV(pFaceTex->getUAV().get(), float4(-1.f));
                    }
                }
            }
        }
    }

    // Create programs
    {
        // Create depth pass program.
        if (!mDepthPass.pProgram)
        {
            ProgramDesc desc;
            desc.addShaderModules(mpScene->getShaderModules());
            desc.addShaderLibrary(kDepthPassProgramFile).vsEntry("vsMain").psEntry("psMain");
            desc.addTypeConformances(mpScene->getTypeConformances());
            desc.setShaderModel(ShaderModel::SM6_2);

            mDepthPass.pProgram = Program::create(mpDevice, desc, mpScene->getSceneDefines());
            mDepthPass.pState->setProgram(mDepthPass.pProgram);
        }

        // Create Shadow Map Pass
        if (!mSMPass.pProgram)
        {
            ProgramDesc desc;
            desc.addShaderModules(mpScene->getShaderModules());
            desc.addShaderLibrary(kSMPassProgramFile).vsEntry("vsMain").psEntry("psMain");
            desc.addTypeConformances(mpScene->getTypeConformances());
            desc.setShaderModel(ShaderModel::SM6_2);

            mSMPass.pProgram = Program::create(mpDevice, desc, mpScene->getSceneDefines());
            mSMPass.pState->setProgram(mSMPass.pProgram);
        }
    }

    // Start render shadow map for each light
    for (uint lightID = 0; lightID < lightPosList.size(); lightID++)
    {
        for (int faceID = 0; faceID < 6; faceID++)
        {
            // Light VP Preparation
            math::float4x4 shadowViewMatrix = math::matrixFromLookAt(
                lightPosList[lightID], lightPosList[lightID] + CubeMapLookAtDir_local[faceID], CubeMapLookUpDir_local[faceID]
            );
            math::float4x4 shadowVP = math::mul(shadowProjMatrices, shadowViewMatrix);

            // 绑定gbuffer的channel，Bind primary channels as render targets and clear them.
            pRenderContext->clearDsv(pDepth->getDSV().get(), 1.f, 0); // clear depth stencil view
            pRenderContext->clearFbo(mpFbo.get(), float4(0), 1.f, 0, FboAttachmentType::Color);

            // Depth pass.
            {
                // Set program defines.
                mDepthPass.pState->getProgram()->addDefine("USE_ALPHA_TEST", mUseAlphaTest ? "1" : "0");

                // Create program vars.
                if (!mDepthPass.pVars)
                    mDepthPass.pVars = ProgramVars::create(mpDevice, mDepthPass.pProgram.get());

                mpFbo->attachDepthStencilTarget(pDepth);
                mDepthPass.pState->setFbo(mpFbo);
                mDepthPass.pVars->getRootVar()["LightVP"]["gLightVP"] = shadowVP;
                mpScene->rasterize(pRenderContext, mDepthPass.pState.get(), mDepthPass.pVars.get(), cullMode);
            }

            // SM pass
            {
                // Set program defines.
                mSMPass.pProgram->addDefine("ADJUST_SHADING_NORMALS", mAdjustShadingNormals ? "1" : "0");
                mSMPass.pProgram->addDefine("USE_ALPHA_TEST", mUseAlphaTest ? "1" : "0");

                // Create program vars.
                if (!mSMPass.pVars)
                    mSMPass.pVars = ProgramVars::create(mpDevice, mSMPass.pProgram.get());

                // Bind extra channels as UAV buffers.Unordered Access View,允许在着色器中对缓冲区进行读写操作，而不需要按照特定顺序进行访问
                mSMPass.pVars->getRootVar()["gCubeMap"] = renderData.getTexture("cubeMap");

                // TODO: move some operations out of per-face loop to accelerate
                auto smPassVar = mSMPass.pVars->getRootVar();
                // Bind extra channels as UAV buffers.
                for (const auto& channel : kRSMChannels)
                {
                    auto pRSMTex = renderData.getTexture(channel.name);
                    smPassVar[channel.texname] = pRSMTex; // Write to texture
                }

                // 传入数据
                smPassVar["PerFrameCB"]["gFrameDim"] = mFrameDim;
                // 六个sm cubemap mvp矩阵
                smPassVar["LightVP"]["gLightVP"] = shadowVP;
                smPassVar["LightVP"]["gLightPos"] = lightPosList[lightID];
                // mSMPass.pVars["LightVP"]["geoID"] = emissiveID[itL];
                smPassVar["LightVP"]["gLightNormal"] = lightNormList[lightID];

                smPassVar["CubeMapParam"]["gCubeFaceN"] = uint(faceID);
                smPassVar["CubeMapParam"]["gLightN"] = uint(lightID);

                for (const auto& channel : kRSMChannels)
                {
                    // Visualize texture arrays
                    std::string texName = channel.name + "Vis_" + std::to_string(lightID) + "_" + std::to_string(faceID);
                    auto pRSMTex = renderData.getTexture(texName);
                    smPassVar[channel.texname + "Vis"] = pRSMTex;
                }

                // CubeMap 绘制加速
                mSMPass.pState->setFbo(mpFbo); // Sets the viewport
                // Rasterize the scene.
                mpScene->rasterize(pRenderContext, mSMPass.pState.get(), mSMPass.pVars.get(), cullMode);
            }
        }
    }
}
