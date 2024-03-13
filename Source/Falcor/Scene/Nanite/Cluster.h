#pragma once
#include <fstd/span.h>
#include <vector>

#include "GraphPartitioner.h"
#include "TriangleUtils.h"
#include "Scene/SceneTypes.slang"
#include "Utils/Math/AABB.h"
#include "Utils/Math/Sphere.h"

namespace Falcor
{
class Cluster
{
public:
    static constexpr uint32_t kSize = 128; ///< The number of triangles in a cluster.
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

    /**
     * @brief Construct a new Cluster by partitioning an existing cluster.
     *
     * @param srcCluster The source cluster.
     * @param partitioner The graph partitioner, storing the partition result.
     * @param adjacency The edge adjacency.
     * @param triangleRange The range of the triangles.
     */
    Cluster(
        const Cluster& srcCluster,
        const GraphPartitioner& partitioner,
        const EdgeAdjacency& adjacency,
        const GraphPartitioner::Range& triangleRange
    );

    /** Compute bounding box and spheres.
     */
    void computeBounds();

    /**
     * @brief Partition the cluster into smaller clusters.
     *
     * @param partitioner Graph partitioner.
     * @param adjacency The edge adjacency.
     */
    void split(GraphPartitioner& partitioner, const EdgeAdjacency& adjacency) const;

    /**
     * @brief Simply cluster with given desired triangle count.
     * The desired triangle count is not guaranteed to be met.
     * @param targetTriangleCount The target triangle count.
     * @return The error metric.
     */
    float simplify(uint32_t targetTriangleCount);

    static Cluster merge(fstd::span<const Cluster> clusters);

    static Cluster merge(fstd::span<const Cluster*> pClusters);

    EdgeAdjacency getEdgeAdjacency() const;

    auto getGUID() const { return mGUID; }

    auto getExternalEdgesCount() const { return mExternalEdgeCount; }

    auto getIndicesSize() const { return mIndices.size(); }

    const auto& getBounds() const { return mBoundingBox; }

    auto getVertex(uint32_t vertexIndex) const { return mVertices[mIndices[vertexIndex]]; }

private:
    uint64_t mGUID = 0ull; ///< Global unique cluster id
    // Mesh data
    std::vector<PackedStaticVertexData> mVertices;
    std::vector<uint32_t> mIndices;
    std::vector<bool> mBoundaryEdges;
    std::vector<uint8_t> mExternalEdges;

    std::map<uint32_t, uint32_t> mAdjacentClusters; ///< Mapping from clusterIndex -> shared edges count

    uint32_t mExternalEdgeCount = 0; ///< Count of non-zero external edges

    // Bounding box
    AABB mBoundingBox;

    uint32_t mGroupIndex = std::numeric_limits<uint32_t>().max();
    uint32_t mGroupPartIndex = std::numeric_limits<uint32_t>().max();

    friend class NaniteDataBuilder;
};

struct ClusterGroup
{
    static constexpr uint32_t kMinSize = 8;
    static constexpr uint32_t kMaxSize = 32;
    static constexpr uint32_t kMaxChildren = 128;

    Sphere bound;
    Sphere LODBound;

    std::vector<uint32_t> childrenIndices;
};
} // namespace Falcor
