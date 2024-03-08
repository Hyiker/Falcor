#include "TriangleUtils.h"
namespace Falcor
{
EdgeHash::EdgeHash(int size) : mEdgeHashmap(std::floor(std::log2(size))) {}

void EdgeHash::addEdge(uint32_t edgeIndex, std::function<float3(uint32_t)> getPosition)
{
    float3 pos0 = getPosition(edgeIndex);
    float3 pos1 = getPosition(triangleIndexCycle(edgeIndex));

    uint32_t hash0 = hashPosition(pos0);
    uint32_t hash1 = hashPosition(pos1);

    uint32_t hash = murmur32({hash0, hash1});
    mEdgeHashmap.emplace(hash, edgeIndex);
}

void EdgeHash::forEachEdgeWithOppositeDirection(
    uint32_t edgeIndex,
    std::function<float3(uint32_t)> getPosition,
    std::function<void(uint32_t, uint32_t)> callback
)
{
    float3 pos0 = getPosition(edgeIndex);
    float3 pos1 = getPosition(triangleIndexCycle(edgeIndex));

    uint32_t hash0 = hashPosition(pos0);
    uint32_t hash1 = hashPosition(pos1);

    uint32_t hash = murmur32({hash0, hash1});

    auto range = mEdgeHashmap.equal_range(hash);
    for (auto it = range.first; it != range.second; it++)
    {
        uint32_t otherEdgeIndex = it->second;
        float3 otherPos0 = getPosition(triangleIndexCycle(otherEdgeIndex));
        float3 otherPos1 = getPosition(otherEdgeIndex);

        if (all(pos0 == otherPos0) && all(pos1 == otherPos1))
        {
            callback(edgeIndex, otherEdgeIndex);
        }
    }
}

void EdgeAdjacency::link(int edgeIndex0, int edgeIndex1)
{
    if (direct[edgeIndex0] < 0 && direct[edgeIndex1] < 0)
    {
        direct[edgeIndex0] = edgeIndex1;
        direct[edgeIndex1] = edgeIndex0;
    }
    else
    {
        extended.emplace(edgeIndex0, edgeIndex1);
        extended.emplace(edgeIndex1, edgeIndex0);
    }
}

void EdgeAdjacency::forEachAdjacentEdges(uint32_t edgeIndex, std::function<void(uint32_t, int)> callback) const
{
    int directIndex = direct[edgeIndex];
    if (directIndex >= 0)
    {
        callback(edgeIndex, directIndex);
    }

    auto range = extended.equal_range(edgeIndex);
    for (auto it = range.first; it != range.second; it++)
    {
        callback(edgeIndex, it->second);
    }
}

} // namespace Falcor
