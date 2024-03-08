#include "Cluster.h"
namespace Falcor
{
Cluster::Cluster(
    const fstd::span<const PackedStaticVertexData>& vertices,
    std::function<uint32_t(uint32_t)> convertIndex,
    const GraphPartitioner& partitioner,
    const EdgeAdjacency& adjacency,
    const GraphPartitioner::Range& triangleRange,
    std::function<void(uint32_t, uint64_t)> clusterCallback
)
{
    mGUID = (uint64_t(triangleRange.first) << 32) | triangleRange.second;
    uint32_t numTriangles = triangleRange.second - triangleRange.first;

    mVertices.reserve(numTriangles * 3);
    mIndices.reserve(numTriangles * 3);
    mExternalEdges.reserve(numTriangles * 3);

    std::map<uint32_t, uint32_t> indexMap; // from global index to cluster local index

    for (uint32_t i = triangleRange.first; i < triangleRange.second; i++)
    {
        uint32_t triangleIndex = partitioner.getIndex(i);
        clusterCallback(triangleIndex, mGUID);
        for (uint32_t j = 0; j < 3; j++)
        {
            uint32_t globalIndex = convertIndex(triangleIndex * 3 + j);
            auto it = indexMap.find(globalIndex);

            if (it == indexMap.end())
            {
                uint32_t localIndex = uint32_t(mVertices.size());
                indexMap[globalIndex] = localIndex;
                mVertices.push_back(vertices[globalIndex]);
                mIndices.push_back(localIndex);
            }
            else
            {
                mIndices.push_back(it->second);
            }

            uint32_t edgeIndex = triangleIndex * 3 + j;
            int adjNum = 0;
            // accumulate edge adjacency count
            adjacency.forEachAdjacentEdges(
                edgeIndex,
                [&](uint32_t edgeIndex0, int edgeIndex1)
                {
                    FALCOR_ASSERT(edgeIndex1 >= 0, "Invalid edge index");
                    uint32_t adjTriangle = edgeIndex1 / 3;
                    if (adjTriangle < triangleRange.first && adjTriangle >= triangleRange.second)
                    {
                        adjNum++;
                    }
                }
            );

            mExternalEdges.push_back(uint8_t(adjNum));
        }
    }

    // mBoundaryEdges = boundaryEdges;
    mVertices.shrink_to_fit();
    mIndices.shrink_to_fit();
    mExternalEdges.shrink_to_fit();
}
} // namespace Falcor
