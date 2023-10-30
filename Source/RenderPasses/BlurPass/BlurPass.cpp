/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
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
#include "BlurPass.h"

namespace
{
const char kSrc[] = "src";
const char kDst[] = "dst";

const char kKernelSize[] = "kernelSize";

const char kShaderFilename[] = "RenderPasses/BlurPass/BlurPass.ps.slang";
} // namespace

void BlurPass::registerBindings(pybind11::module& m)
{
    pybind11::class_<BlurPass, RenderPass, ref<BlurPass>> pass(m, "BlurPass");
    pass.def_property(kKernelSize, &BlurPass::getKernelSize, &BlurPass::setKernelSize);
}

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, BlurPass>();
}

BlurPass::BlurPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);
}

void BlurPass::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        if (key == kKernelSize)
        {
            setKernelSize(value);
        }
        else
        {
            logWarning("Unknown field `{}` in a BlurPass dictionary", key);
        }
    }
}

Properties BlurPass::getProperties() const
{
    Properties props;
    props[kKernelSize] = mKernelSize;

    return props;
}

RenderPassReflection BlurPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput("src", "Input texture");
    reflector.addOutput("dst", "Output texture");
    return reflector;
}
void BlurPass::compile(RenderContext* pRenderContext, const CompileData& compileData)
{
    DefineList defines;
    switch (mKernelSize)
    {
    case KernelSize::Gaussian5x5:
        defines.add("_GAUSSIAN_BLUR_KERNEL5x5");
        break;
    case KernelSize::Gaussian7x7:
        defines.add("_GAUSSIAN_BLUR_KERNEL7x7");
        break;
    case KernelSize::Gaussian9x9:
        defines.add("_GAUSSIAN_BLUR_KERNEL9x9");
        break;
    }
    defines.add("_HORIZONTAL_BLUR");
    mpHorizontalBlur = FullScreenPass::create(mpDevice, kShaderFilename, defines, 0);
    defines.remove("_HORIZONTAL_BLUR");
    defines.add("_VERTICAL_BLUR");
    mpVerticalBlur = FullScreenPass::create(mpDevice, kShaderFilename, defines, 0);

    // Make the programs share the vars
    mpVerticalBlur->setVars(mpHorizontalBlur->getVars());
}

void BlurPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData.getTexture("src");
    const auto& pInputTex = renderData.getTexture(kSrc);
    const auto& pOutputTex = renderData.getTexture(kDst);
    FALCOR_ASSERT(pInputTex && pOutputTex);
    if (pInputTex->getWidth() != pOutputTex->getWidth() || pInputTex->getHeight() != pOutputTex->getHeight())
    {
        logWarning("Blur pass I/O has different dimensions. The image will be resampled.");
    }

    ref<Fbo> pFbo = Fbo::create(mpDevice);
    pFbo->attachColorTarget(pOutputTex, 0);
    createTmpFbo(pInputTex.get());

    // horizontal
    {
        auto var = mpHorizontalBlur->getRootVar();
        var["gColorTex"] = pInputTex;
        mpHorizontalBlur->execute(pRenderContext, mpTmpFbo);
    }

    // vertical
    {
        auto var = mpVerticalBlur->getRootVar();
        var["gColorTex"] = mpTmpFbo->getColorTexture(0);
        mpVerticalBlur->execute(pRenderContext, pFbo);
    }
}

void BlurPass::createTmpFbo(const Texture* pSrc)
{
    bool createFbo = mpTmpFbo == nullptr;
    ResourceFormat srcFormat = pSrc->getFormat();

    if (createFbo == false)
    {
        createFbo = (pSrc->getWidth() != mpTmpFbo->getWidth()) || (pSrc->getHeight() != mpTmpFbo->getHeight()) ||
                    (srcFormat != mpTmpFbo->getColorTexture(0)->getFormat()) ||
                    pSrc->getArraySize() != mpTmpFbo->getColorTexture(0)->getArraySize();
    }

    if (createFbo)
    {
        Fbo::Desc fboDesc;
        fboDesc.setColorTarget(0, srcFormat);
        mpTmpFbo = Fbo::create2D(mpDevice, pSrc->getWidth(), pSrc->getHeight(), fboDesc, pSrc->getArraySize());
    }
}

void BlurPass::setKernelSize(KernelSize kernelSize)
{
    mKernelSize = kernelSize;
    requestRecompile();
}

void BlurPass::renderUI(Gui::Widgets& widget)
{
    if (widget.dropdown("Kernel Size", kKernelSizeDropDown, reinterpret_cast<uint32_t&>(mKernelSize)))
    {
        setKernelSize(mKernelSize);
    }
}
