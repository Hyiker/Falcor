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
     * @brief Build DAG data for mesh simplification.
     *
     */
    void buildDAG();

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

    std::vector<Cluster> mClusters;
    std::vector<uint64_t> mClusterGUIDs;
};
} // namespace Falcor
