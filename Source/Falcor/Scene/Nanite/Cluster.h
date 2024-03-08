#pragma once
#include <fstd/span.h>
#include <vector>

#include "GraphPartitioner.h"
#include "TriangleUtils.h"
#include "Scene/SceneTypes.slang"
#include "Utils/Math/AABB.h"

namespace Falcor
{
class Cluster
{
public:
    Cluster() = default;
    /**
     * @brief Construct a Cluster.
     *
     * @param vertices The global vertices buffer.
     * @param convertIndex The index conversion function for 16-bit index conversion.
     * @param partitioner The graph partitioner.
     * @param adjacency The edge adjacency.
     * @param triangleRange The range of the triangles.
     * @param clusterCallback The callback function for cluster creation, triggered for every triangle index.
     */
    Cluster(
        const fstd::span<const PackedStaticVertexData>& vertices,
        std::function<uint32_t(uint32_t)> convertIndex,
        const GraphPartitioner& partitioner,
        const EdgeAdjacency& adjacency,
        const GraphPartitioner::Range& triangleRange,
        std::function<void(uint32_t, uint64_t)> clusterCallback
    );

private:
    uint64_t mGUID; // global unique cluster id
    // Mesh data
    std::vector<PackedStaticVertexData> mVertices;
    std::vector<uint32_t> mIndices;
    std::vector<bool> mBoundaryEdges;
    std::vector<uint8_t> mExternalEdges;

    // Bounding box
    AABB mBoundingBox;
};
} // namespace Falcor
