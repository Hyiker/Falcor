/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#include "RenderGraph/RenderPassStandardFlags.h"
#include "Utils/SampleGenerators/DxSamplePattern.h"
#include "Utils/SampleGenerators/HaltonSamplePattern.h"
#include "Utils/SampleGenerators/StratifiedSamplePattern.h"
#include "RenderGraph/RenderGraph.h"
#include "RenderGraph/RenderPass.h"
#include "LightSample.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"
#include "Rendering/Lights/EmissiveUniformSampler.h"
#include "Rendering/Lights/EnvMapSampler.h"

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, LightSample>();
}

namespace
{
std::string kLightCount = "lightCount";
const std::string kProgramComputeFile = "RenderPasses/LightSample/LightSample.cs.slang";
const ChannelList kLightSampleChannels = {
    {"position", "gPosition", "position at sample", true /* optional */, ResourceFormat::RGBA32Float /* set at runtime */},
    {"normal", "gNormal", "normal at sample", true /* optional */, ResourceFormat::RGBA32Float},
    {"intensity", "gIntensity", "intensity at sample", true /* optional */, ResourceFormat::RGBA32Float}};
} // namespace

LightSample::LightSample(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    // Parse dictionary.
    for (const auto& [key, value] : props)
    {
        if (key == kLightCount)
            lightCount = (uint32_t)value;
    }
}

RenderPassReflection LightSample::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    uint2 dim(sample_per_light, lightCount);
    // Add all outputs as UAVs. These are all optional.
    addRenderPassOutputs(reflector, kLightSampleChannels, ResourceBindFlags::UnorderedAccess, dim);

    return reflector;
}

void LightSample::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    mpScene = pScene;
    const ref<LightCollection>& emissiveLight = pScene->getLightCollection(pRenderContext);
    emissive_lightCount = emissiveLight->getMeshLights().size();
    // std::cout << "Light Size : " << emissiveLight->getMeshLights().size() << std::endl;
    // for (int i = 0; i < emissiveLight->getMeshLights().size(); i++)
    // {
    //     MeshLightData data = emissiveLight->getMeshLights()[i];
    //     // std::cout << "emissive: " << i << " have material id-> " << data.materialID << std::endl;
    // }
}

void LightSample::renderUI(Gui::Widgets& widget)
{
    widget.text("Stats:");
    widget.tooltip("The number of LightSamples.\n");

    std::ostringstream oss;
    oss << "Number of lights: " << lightCount << "\n"
        << "EmissiveLight: " << emissive_lightCount << "\n"
        << "Total LightSamples: " << sample_per_light * lightCount << "\n";
    if (mpEnvMapSampler)
        oss << "EnvMap exist !"
            << "\n";
    // oss << "Path length (avg): " << std::fixed << std::setprecision(3) << mStats.avgPathLength << "\n"
    //     << "Path vertices (avg): " << std::fixed << std::setprecision(3) << mStats.avgPathVertices << "\n"
    //     << "Total rays (avg): " << std::fixed << std::setprecision(3) << mStats.avgTotalRays << "\n"
    //     << "Visibility rays (avg): " << std::fixed << std::setprecision(3) << mStats.avgVisibilityRays << "\n"
    //     << "ClosestHit rays (avg): " << std::fixed << std::setprecision(3) << mStats.avgClosestHitRays << "\n"
    //     << "Path vertices: " << mStats.pathVertices << "\n"
    //     << "Total rays: " << mStats.totalRays << "\n"
    //     << "Visibility rays: " << mStats.visibilityRays << "\n"
    //     << "ClosestHit rays: " << mStats.closestHitRays << "\n"
    //     << "Volume lookups: " << mStats.volumeLookups << "\n"
    //     << "Volume lookups (avg): " << mStats.avgVolumeLookups << "\n";

    widget.text(oss.str());
}

void LightSample::executeCompute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (!mpComputePass)
    {
        ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
        desc.addShaderLibrary(kProgramComputeFile).csEntry("main");
        desc.addTypeConformances(mpScene->getTypeConformances());

        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add("USE_ENV_LIGHT", (mpScene && mpScene->useEnvLight()) ? "1" : "0");
        mpComputePass = ComputePass::create(mpDevice, desc, defines, true);
        uint3 sz = mpComputePass->getThreadGroupSize();
        // std::cout << to_string(sz) << std::endl;
        // Bind static resources
        mpScene->setRaytracingShaderData(pRenderContext, mpComputePass->getRootVar());
    }
    if (mpScene->getEnvMap() != nullptr)
    {
        if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::EnvMapChanged))
        {
            mpEnvMapSampler = nullptr;
        }
        if (!mpEnvMapSampler)
        {
            mpEnvMapSampler = std::make_unique<EnvMapSampler>(mpDevice, mpScene->getEnvMap());
            ShaderVar vars = mpComputePass->getRootVar();
            mpEnvMapSampler->bindShaderData(vars["envS"]);
        }
    }

    ShaderVar vars = mpComputePass->getRootVar();
    vars["gPosition"] = renderData.getTexture("position");
    vars["gNormal"] = renderData.getTexture("normal");
    vars["gIntensity"] = renderData.getTexture("intensity");
    mpComputePass->execute(pRenderContext, uint3(sample_per_light, 1, 1));
}

void LightSample::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    ref<Texture> pOutput;
    auto findOutput = [&](const std::string& name)
    {
        auto pTex = renderData.getTexture(name);
        if (pTex && !pOutput)
            pOutput = pTex;
    };
    for (const auto& channel : kLightSampleChannels)
        findOutput(channel.name);

    if (!pOutput)
    {
        logWarning("LightSample::execute() - Render pass has no connected outputs. Is this intended?");
        return;
    }
    FALCOR_ASSERT(pOutput);

    // If there is no scene, clear the output and return.
    if (mpScene == nullptr)
    {
        clearRenderPassChannels(pRenderContext, kLightSampleChannels, renderData);
        return;
    }
    // Configure depth-of-field.
    // When DOF is enabled, two PRNG dimensions are used. Pass this info to subsequent passes via the dictionary.

    executeCompute(pRenderContext, renderData);
}
