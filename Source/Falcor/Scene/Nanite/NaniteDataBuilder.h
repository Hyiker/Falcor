#pragma once

#include <fstd/span.h>
#include "Core/Macros.h"
namespace Falcor
{
class FALCOR_API NaniteDataBuilder
{
public:
    /**
     * @brief Build the Nanite data for the scene.
     * @param[in] vertices The vertices of the scene.
     * @param[in] triangleIndices The triangle indices of the scene.
     * @param[in] meshDescs The mesh descriptions of the scene, containing the vb and ib data.
     */
    void buildNaniteData(
        fstd::span<const PackedStaticVertexData> vertices,
        fstd::span<const uint32_t> triangleIndices,
        fstd::span<const MeshDesc> meshDescs
    );

private:
    /**
     * @brief Cluster the triangles in a mesh.
     *
     * @param vertices
     * @param triangleIndices
     * @param meshDescs
     */
    void clusterTriangles(
        fstd::span<const PackedStaticVertexData> vertices,
        fstd::span<const uint32_t> triangleIndices,
        const MeshDesc& meshDescs
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
};
} // namespace Falcor
