#pragma once

#include <fstd/span.h>
#include "Core/Macros.h"
#include "Cluster.h"
#include "Scene/SceneBuilder.h"

namespace Falcor
{
class FALCOR_API NaniteDataBuilder
{
public:
    /**
     * @brief Build the Nanite data for the scene.
     * @param[in] vertices The vertices of the scene.
     * @param[in] triangleIndices The triangle indices of the scene.
     * @param[in] meshList The specifications of the meshes.
     */
    void buildNaniteData(
        fstd::span<const PackedStaticVertexData> vertices,
        fstd::span<const uint32_t> triangleIndices,
        fstd::span<const SceneBuilder::MeshSpec> meshList
    );

    auto getClusterGUIDs() const { return mClusterGUIDs; }

private:
    /**
     * @brief Cluster the triangles in a mesh.
     *
     * @param vertices The global vertices buffer.
     * @param triangleIndices The global triangle indices buffer.
     * @param meshList The mesh list.
     * @return The clustering result of the mesh.
     */
    std::vector<Cluster> clusterTriangles(
        fstd::span<const PackedStaticVertexData> vertices,
        fstd::span<const uint32_t> triangleIndices,
        const SceneBuilder::MeshSpec& meshSpec
    );

    /**
     * @brief Reduce the children collection to form a DAG parent.
     * Store data into the pre-allocated(for parallelism) mClusters and mClusterGroups.
     *
     * @param clusterCount Counter variable for real cluster count.
     * @param children The index collection of DAG children.
     * @param groupIndex The index of the parent group for storing the ClusterGroup.
     */
    void reduceDAG(std::atomic_uint32_t& clusterCount, fstd::span<const uint32_t> children, uint32_t groupIndex);

    /**
     * @brief Build DAG data for mesh simplification.
     * Resulting DAG data will be stored in mClusterGroups.
     * @param clusterRangeStart The start index of the clusters.
     * @param clusterRangeCount The count of the clusters.
     */
    void buildDAG(uint32_t clusterRangeStart, uint32_t clusterRangeCount);

    /**
     * @brief Build the coarse representation of original high precision meshes.
     *
     */
    void buildCoarseRepresentation();

    /**
     * @brief Encode the Nanite meshes.
     *
     */
    void encodeNaniteMeshes();

    std::vector<Cluster> mClusters;           ///< Original clusters data.
    std::vector<ClusterGroup> mClusterGroups; ///< Flattened hierarchical cluster groups.
    std::vector<uint64_t> mClusterGUIDs;      ///< Global unique cluster ids(stored for debug).
};
} // namespace Falcor
