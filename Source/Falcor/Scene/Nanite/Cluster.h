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
    static constexpr uint32_t kMaxIndexSize = 256; ///< The maximum number of indices in a cluster.
    static constexpr uint32_t kSize = 128;         ///< The number of triangles in a cluster. We seek to use 8-bit indices for
                                                   ///< this size, considering the worst case, floor(256 / 3) = 85 triangles(Due
                                                   ///< to the partition algorithm, we have to make it even smaller).
    Cluster() = default;
    /**
     * @brief Construct a Cluster.
     *
     * @param vertices The global vertices buffer.
     * @param convertIndex The index conversion function for 16-bit index conversion.
     * @param materialID The mesh materialID.
     * @param partitioner The graph partitioner.
     * @param adjacency The edge adjacency.
     * @param triangleRange The range of the triangles.
     */
    Cluster(
        const fstd::span<const PackedStaticVertexData>& vertices,
        std::function<uint32_t(uint32_t)> convertIndex,
        MaterialID materialID,
        const GraphPartitioner& partitioner,
        const EdgeAdjacency& adjacency,
        const GraphPartitioner::Range& triangleRange,
        bool isFrontFaceCW
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

    /** Finalize cluster, compute bounding data, areas etc.
     */
    void finalize();

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
     * If max error is set, the simplification will iterate until the target triangle count met or error exceed the threshold.
     * @param targetTriangleCount The target triangle count.
     * @param targetError The target error.
     * @param maxError The maximum error tolerance.
     * @return The error metric.
     */
    float simplify(uint32_t targetTriangleCount, float targetError = 0.f, float maxError = 0.f);

    static Cluster merge(fstd::span<const Cluster> clusters);

    static Cluster merge(fstd::span<const Cluster*> pClusters);

    EdgeAdjacency getEdgeAdjacency() const;

    auto getGUID() const { return mGUID; }

    auto getExternalEdgesCount() const { return mExternalEdgeCount; }

    uint32_t getIndexCount() const { return mIndices.size(); }

    uint32_t getVertexCount() const { return mVertices.size(); }

    const auto& getBounds() const { return mBoundingBox; }

    auto getVertex(uint32_t vertexIndex) const { return mVertices[mIndices[vertexIndex]]; }

    int getMipLevel() const { return mMipLevel; }

    float getError() const { return mError; }

    void saveToFile(const std::filesystem::path& p) const;

    MaterialID materialID;
    std::set<NodeID> instances;

    // Mesh flags
    bool isFrontFaceCW = false;
    std::vector<uint32_t> children; ///< Children cluster indices
private:
    uint64_t mGUID = 0ull; ///< Global unique cluster id

    // Mesh data
    std::vector<PackedStaticVertexData> mVertices;
    std::vector<uint32_t> mIndices;
    std::vector<bool> mBoundaryEdges;
    std::vector<uint8_t> mExternalEdges;

    float mSurfaceArea = 0.f;

    std::map<uint32_t, uint32_t> mAdjacentClusters; ///< Mapping from clusterIndex -> shared edges count

    uint32_t mExternalEdgeCount = 0; ///< Count of non-zero external edges

    // Bounding box
    AABB mBoundingBox;

    int mMipLevel = 0;  ///< Cluster mip level, parents have higher mip levels
    float mError = 0.f; ///< Cluster mip level error

    uint32_t mGroupIndex = std::numeric_limits<uint32_t>().max();
    uint32_t mGroupPartIndex = std::numeric_limits<uint32_t>().max();

    friend class NaniteDataBuilder;
    friend class SceneBuilder;
};

struct ClusterGroup
{
    static constexpr uint32_t kMinSize = 8;
    static constexpr uint32_t kMaxSize = 32;
    static constexpr uint32_t kMaxChildren = 128;

    int mipLevel = 0;

    Sphere bound;
    Sphere LODBound;

    std::vector<uint32_t> childrenIndices;
};
} // namespace Falcor
