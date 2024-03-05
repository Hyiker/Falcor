#pragma once
#include "Core/Macros.h"
namespace Falcor
{
// Very fast radix sort from Unreal Engine
// https://github.com/EpicGames/UnrealEngine/blob/release/Engine/Source/Runtime/Core/Public/Templates/Sorting.h#L541

/**
 * Very fast 32bit radix sort.
 * SortKeyClass defines operator() that takes ValueType and returns a uint32_t. Sorting based on key.
 * No comparisons. Is stable.
 * Use a smaller CountType for smaller histograms.
 */
template<typename ValueType, typename CountType, class SortKeyClass>
void radixSort32(ValueType* dst, ValueType* src, CountType num, SortKeyClass sortKey)
{
    CountType histograms[1024 + 2048 + 2048];
    CountType* histogram0 = histograms + 0;
    CountType* histogram1 = histogram0 + 1024;
    CountType* histogram2 = histogram1 + 2048;

    std::memset(histograms, 0, sizeof(histograms));

    {
        // Parallel histogram generation pass
        const ValueType* s = (const ValueType*)src;
        for (CountType i = 0; i < num; i++)
        {
            uint32_t Key = sortKey(s[i]);
            histogram0[(Key >> 0) & 1023]++;
            histogram1[(Key >> 10) & 2047]++;
            histogram2[(Key >> 21) & 2047]++;
        }
    }
    {
        // Prefix sum
        // Set each histogram entry to the sum of entries preceding it
        CountType sum0 = 0;
        CountType sum1 = 0;
        CountType sum2 = 0;
        for (CountType i = 0; i < 1024; i++)
        {
            CountType t;
            t = histogram0[i] + sum0;
            histogram0[i] = sum0 - 1;
            sum0 = t;
            t = histogram1[i] + sum1;
            histogram1[i] = sum1 - 1;
            sum1 = t;
            t = histogram2[i] + sum2;
            histogram2[i] = sum2 - 1;
            sum2 = t;
        }
        for (CountType i = 1024; i < 2048; i++)
        {
            CountType t;
            t = histogram1[i] + sum1;
            histogram1[i] = sum1 - 1;
            sum1 = t;
            t = histogram2[i] + sum2;
            histogram2[i] = sum2 - 1;
            sum2 = t;
        }
    }
    {
        // Sort pass 1
        const ValueType* s = (const ValueType*)src;
        ValueType* d = dst;
        for (CountType i = 0; i < num; i++)
        {
            ValueType Value = s[i];
            uint32_t Key = sortKey(Value);
            d[++histogram0[((Key >> 0) & 1023)]] = Value;
        }
    }
    {
        // Sort pass 2
        const ValueType* s = (const ValueType*)dst;
        ValueType* d = src;
        for (CountType i = 0; i < num; i++)
        {
            ValueType Value = s[i];
            uint32_t Key = sortKey(Value);
            d[++histogram1[((Key >> 10) & 2047)]] = Value;
        }
    }
    {
        // Sort pass 3
        const ValueType* s = (const ValueType*)src;
        ValueType* d = dst;
        for (CountType i = 0; i < num; i++)
        {
            ValueType Value = s[i];
            uint32_t Key = sortKey(Value);
            d[++histogram2[((Key >> 21) & 2047)]] = Value;
        }
    }
}

template<typename T>
struct RadixSortKeyCastUint32
{
    FALCOR_FORCEINLINE uint32_t operator()(const T& value) const { return (uint32_t)value; }
};

template<typename ValueType, typename CountType>
void radixSort32(ValueType* dst, ValueType* src, CountType num)
{
    RadixSort32(dst, src, num, RadixSortKeyCastUint32<ValueType>());
}

// float cast to uint32_t which maintains sorted order
// http://codercorner.com/RadixSortRevisited.htm
struct RadixSortKeyFloat
{
    FALCOR_FORCEINLINE uint32_t operator()(float value) const
    {
        union
        {
            float f;
            uint32_t i;
        } v;
        v.f = value;

        uint32_t mask = -int(v.i >> 31) | 0x80000000;
        return v.i ^ mask;
    }
};

template<typename CountType>
void radixSort32(float* dst, float* src, CountType num)
{
    radixSort32(dst, src, num, RadixSortKeyFloat());
}
} // namespace Falcor
