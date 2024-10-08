/***************************************************************************
 # Copyright (c) 2015-24, NVIDIA CORPORATION. All rights reserved.
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
#include "Testing/UnitTest.h"
#include "Utils/HostDeviceShared.slangh"
#include <random>

namespace Falcor
{
namespace
{
std::vector<ShaderModel> kShaderModels = {
    {ShaderModel::SM6_2},
    {ShaderModel::SM6_3},
};

const uint32_t kNumElems = 256;
std::mt19937 r;
std::uniform_real_distribution u;

void test(GPUUnitTestContext& ctx, ShaderModel shaderModel, bool useUav, bool useStructured)
{
    ref<Device> pDevice = ctx.getDevice();

    DefineList defines = {{"USE_UAV", useUav ? "1" : "0"}};

    ctx.createProgram("Tests/Slang/Float16Tests.cs.slang", "testFloat16", defines, SlangCompilerFlags::None, shaderModel);
    ctx.allocateStructuredBuffer("result", kNumElems);

    std::vector<uint16_t> elems(kNumElems);
    for (auto& v : elems)
        v = f32tof16(float(u(r)));
    auto var = ctx.vars().getRootVar();

    if (useStructured)
    {
        auto buf = pDevice->createStructuredBuffer(
            var["data"],
            kNumElems,
            ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
            MemoryType::DeviceLocal,
            elems.data()
        );
        ASSERT_EQ(buf->getStructSize(), sizeof(float16_t));
        ASSERT_EQ(buf->getElementCount(), kNumElems);
        var["data"] = buf;
    }
    else
    {
        auto buf = pDevice->createBuffer(
            kNumElems * sizeof(float16_t),
            ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess,
            MemoryType::DeviceLocal,
            elems.data()
        );
        ASSERT_EQ(buf->getSize(), kNumElems * sizeof(float16_t));
        var["data"] = buf;
    }

    ctx.runProgram(kNumElems, 1, 1);

    // Verify results.
    std::vector<uint16_t> result = ctx.readBuffer<uint16_t>("result");
    for (uint32_t i = 0; i < kNumElems; i++)
    {
        EXPECT_EQ(result[i], elems[i]) << "i = " << i << " shaderModel=" << enumToString(shaderModel);
    }
}
} // namespace

GPU_TEST(StructuredBuffer_LoadFloat16_Structured)
{
    for (auto sm : kShaderModels)
        test(ctx, sm, false, true);
}

GPU_TEST(StructuredBuffer_LoadFloat16_Raw)
{
    for (auto sm : kShaderModels)
        test(ctx, sm, false, false);
}

GPU_TEST(RWStructuredBuffer_LoadFloat16_Structured)
{
    for (auto sm : kShaderModels)
        test(ctx, sm, true, true);
}

GPU_TEST(RWStructuredBuffer_LoadFloat16_Raw)
{
    for (auto sm : kShaderModels)
        test(ctx, sm, true, false);
}

} // namespace Falcor
