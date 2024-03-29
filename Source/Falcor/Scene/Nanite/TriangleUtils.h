#pragma once
#include <cinttypes>
#include <tbb/concurrent_unordered_map.h>

#include "HashUtils.h"
#include "Core/Macros.h"
#include "Utils/Math/VectorTypes.h"

namespace Falcor
{
FALCOR_FORCEINLINE uint32_t hashPosition(const float3 position)
{
    union
    {
        float f;
        uint32_t i;
    } x;
    union
    {
        float f;
        uint32_t i;
    } y;
    union
    {
        float f;
        uint32_t i;
    } z;

    x.f = position.x;
    y.f = position.y;
    z.f = position.z;

    return murmur32({position.x == 0.0f ? 0u : x.i, position.y == 0.0f ? 0u : y.i, position.z == 0.0f ? 0u : z.i});
}

/**
 * @brief Compute the next triangle index in the size-3 cycle.
 *
 * @param value The current triangle index.
 * @return The next triangle index in the size-3 cycle.
 */
FALCOR_FORCEINLINE uint32_t triangleIndexCycle(uint32_t value)
{
    uint32_t valueMod3 = value % 3;
    uint32_t value1Mod3 = (1 << valueMod3) & 3;
    return value - valueMod3 + value1Mod3;
}

/**
 * @brief Compute the next triangle index in the size-3 cycle.
 *
 * @param value The current triangle index.
 * @param offset The offset to add to the current triangle index.
 * @return The next triangle index in the size-3 cycle.
 */
FALCOR_FORCEINLINE uint32_t triangleIndexCycle(uint32_t value, uint32_t offset)
{
    return value - value % 3 + (value + offset) % 3;
}

template<bool PositionOnly>
struct PackedStaticVertexDataEqual
{
    FALCOR_FORCEINLINE bool operator()(const PackedStaticVertexData& lhs, const PackedStaticVertexData& rhs) const
    {
        if constexpr (PositionOnly)
        {
            return all(lhs.position == rhs.position);
        }
        else
        {
            return all(lhs.position == rhs.position) && all(lhs.packedNormalTangentCurveRadius == rhs.packedNormalTangentCurveRadius) &&
                   all(lhs.texCrd == rhs.texCrd);
        }
    }
};
template<bool PositionOnly>
struct PackedStaticVertexDataHash
{
    FALCOR_FORCEINLINE uint32_t operator()(const PackedStaticVertexData& vertex) const
    {
        if constexpr (PositionOnly)
        {
            return murmur32({hashPosition(vertex.position)});
        }
        else
        {
            return murmur32(
                {hashPosition(vertex.position),
                 hashPosition(vertex.packedNormalTangentCurveRadius),
                 hashPosition(float3(vertex.texCrd, 0.0f))}
            );
        }
    }
};

template<typename ValueType, bool PositionOnly = false>
using PackedStaticVertexHashMap =
    std::unordered_map<PackedStaticVertexData, ValueType, PackedStaticVertexDataHash<PositionOnly>, PackedStaticVertexDataEqual<PositionOnly>>;

/**
 * @brief Helper class to hash edges.
 * Maps from edge hash(vertex pos0, vertex pos1) to edge indices(may be redundant).
 */
class EdgeHash
{
public:
    EdgeHash() = default;

    EdgeHash(int size);

    /**
     * @brief add an edge to the hashmap.
     *
     * @param edgeIndex The index of the edge.
     * @param getPosition A function to get the position of a vertex by the edge index.
     */
    void addEdge(uint32_t edgeIndex, std::function<float3(uint32_t)> getPosition);

    /**
     * @brief Iterate through all edges with the same hash but the opposite direction.
     *
     * @param edgeIndex The index of the edge.
     * @param addEdge Whether to add the input edge into hashmap.
     * @param getPosition A function to get the position of a vertex by the edge index.
     * @param callback Callback function for handling the opposite direction edge, passing in the edgeIndex and otherEdgeIndex.
     */
    void forEachEdgeWithOppositeDirection(
        uint32_t edgeIndex,
        bool addEdge,
        std::function<float3(uint32_t)> getPosition,
        std::function<void(uint32_t, uint32_t)> callback
    );

private:
    tbb::concurrent_unordered_multimap<uint32_t, uint32_t> mEdgeHashmap;
};

struct EdgeAdjacency
{
    std::vector<int> direct;          // edge index -> opposite directed edge index
    std::multimap<int, int> extended; // edge index -> adjacent edge indices

    EdgeAdjacency(uint32_t size) : direct(size, -1) {}

    /**
     * @brief Link two edges.
     * If both edges direct < 0, mark them as direct, otherwise mark them as extended.
     * @param edgeIndex0
     * @param edgeIndex1
     */
    void link(int edgeIndex0, int edgeIndex1);

    /**
     * @brief Iterate over all edges connected to the given edge.
     *
     * @param edgeIndex The index of the edge.
     * @param callback Callback function for handling the adjacent edge, passing in the edgeIndex and adjIndex.
     */
    void forEachAdjacentEdges(uint32_t edgeIndex, std::function<void(uint32_t, int)> callback) const;
};

} // namespace Falcor
