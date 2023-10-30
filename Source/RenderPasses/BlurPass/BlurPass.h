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
#pragma once
#include "Falcor.h"
#include "RenderGraph/RenderPass.h"

using namespace Falcor;
#include "Core/Pass/FullScreenPass.h"
class BlurPass : public RenderPass
{
public:
    FALCOR_PLUGIN_CLASS(BlurPass, "BlurPass", "Gaussian blur post-processing pass.");

    enum class KernelSize
    {
        Gaussian5x5,
        Gaussian7x7,
        Gaussian9x9
    };

    FALCOR_ENUM_INFO(
        KernelSize,
        {{KernelSize::Gaussian5x5, "Gaussian5x5"}, {KernelSize::Gaussian7x7, "Gaussian7x7"}, {KernelSize::Gaussian9x9, "Gaussian9x9"}}
    );
    const Gui::DropdownList kKernelSizeDropDown = {
        {(uint32_t)KernelSize::Gaussian5x5, "Gaussian5x5"},
        {(uint32_t)KernelSize::Gaussian7x7, "Gaussian7x7"},
        {(uint32_t)KernelSize::Gaussian9x9, "Gaussian9x9"}};

    static ref<BlurPass> create(ref<Device> pDevice, const Properties& props) { return make_ref<BlurPass>(pDevice, props); }

    BlurPass(ref<Device> pDevice, const Properties& props);

    void parseProperties(const Properties& props);

    virtual Properties getProperties() const override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pRenderContext, const CompileData& compileData) override;
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene) override {}
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return false; }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

    void setKernelSize(KernelSize kernelSize);
    auto getKernelSize() const { return mKernelSize; }

    static void registerBindings(pybind11::module& m);

private:
    void createTmpFbo(const Texture* pSrc);

    ref<FullScreenPass> mpHorizontalBlur;
    ref<FullScreenPass> mpVerticalBlur;
    ref<Fbo> mpTmpFbo;
    KernelSize mKernelSize = KernelSize::Gaussian5x5;
};

FALCOR_ENUM_REGISTER(BlurPass::KernelSize);
