#include "Cluster.h"
namespace Falcor
{
Cluster::Cluster(
    const fstd::span<const PackedStaticVertexData>& vertices,
    const fstd::span<const uint32_t>& triangleIndices,
    const std::vector<bool>& boundaryEdges
)
{
    uint32_t numIndices = (uint32_t)triangleIndices.size();

    mVertices.reserve(numIndices / 3);
    mIndices.reserve(numIndices);
    mExternalEdges.reserve(numIndices);

    std::map<uint32_t, uint32_t> indexMap; // from global index to cluster local index

    mIndices.reserve(triangleIndices.size());
    for (auto globalIndex : triangleIndices)
    {
        auto it = indexMap.find(globalIndex);
        uint32_t localIndex = it == indexMap.end() ? (uint32_t)mVertices.size() : it->second;

        if (it == indexMap.end())
        {
            indexMap[globalIndex] = localIndex;
            mVertices.push_back(vertices[globalIndex]);
        }

        mIndices.push_back(localIndex);
    }

    // mBoundaryEdges = boundaryEdges;
    mVertices.shrink_to_fit();
    mIndices.shrink_to_fit();
    mExternalEdges.shrink_to_fit();
}
} // namespace Falcor
