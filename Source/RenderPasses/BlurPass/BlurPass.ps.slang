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

Texture2D gColorTex;
SamplerState gSampler;

static const float kKernel5x5[5] = { 0.0062, 0.1961, 0.5954, 0.1961, 0.0062 };
static const float kKernel7x7[7] = { 0.0027, 0.0451, 0.2415, 0.4215, 0.2415, 0.0451, 0.0027 };
static const float kKernel9x9[9] = { 0.0017, 0.0168, 0.0871, 0.2329, 0.3231, 0.2329, 0.0871, 0.0168, 0.0017 };

#if defined(_GAUSSIAN_BLUR_KERNEL5x5)
#define kKernel kKernel5x5
#define _KERNEL_SIZE 5
#elif defined(_GAUSSIAN_BLUR_KERNEL7x7)
#define kKernel kKernel7x7
#define _KERNEL_SIZE 7
#elif defined(_GAUSSIAN_BLUR_KERNEL9x9)
#define kKernel kKernel9x9
#define _KERNEL_SIZE 9
#else
#error Please define a kernel size
#endif

float4 gaussianBlur(Texture2D tex, SamplerState samp, float2 texC)
{
    float3 color = 0.f;
#ifdef _HORIZONTAL_BLUR
    const int2 direction = int2(1, 0);
#else
    const int2 direction = int2(0, 1);
#endif

    int2 offset = -(_KERNEL_SIZE / 2) * int2(direction);
    [ForceUnroll]
    for (uint i = 0; i < _KERNEL_SIZE; ++i)
    {
        float3 c = tex.SampleLevel(samp, texC, 0, offset + i * int2(direction)).rgb;
        color += c * kKernel[i];
    }

    return float4(color, 1.f);
}

float4 main(float2 texC: TEXCOORD) : SV_TARGET0
{
    float4 color = gaussianBlur(gColorTex, gSampler, texC);

    return color;
}
