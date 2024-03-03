#pragma once

#include <cinttypes>

#include "Core/Macros.h"

namespace Falcor
{

FALCOR_FORCEINLINE uint32_t murmurFinalize32(uint32_t hash)
{
    hash ^= hash >> 16;
    hash *= 0x85ebca6b;
    hash ^= hash >> 13;
    hash *= 0xc2b2ae35;
    hash ^= hash >> 16;
    return hash;
}

/**
 * @brief Murmur32 fast hash function.
 *
 * @param initList The list of elements to hash.
 * @return 32-bit hash value.
 */
FALCOR_FORCEINLINE uint32_t murmur32(std::initializer_list<uint32_t> initList)
{
    uint32_t hash = 0;
    for (auto element : initList)
    {
        element *= 0xcc9e2d51;
        element = (element << 15) | (element >> (32 - 15));
        element *= 0x1b873593;

        hash ^= element;
        hash = (hash << 13) | (hash >> (32 - 13));
        hash = hash * 5 + 0xe6546b64;
    }

    return murmurFinalize32(hash);
}

} // namespace Falcor
