#pragma once
#include <fstd/span.h>
#include <vector>

#include "Scene/SceneTypes.slang"
#include "Utils/Math/AABB.h"

namespace Falcor
{
class Cluster
{
public:
    Cluster() = default;
    /**
     * @brief Construct a Cluster from vertices, triangle indices and boundary edges.
     *
     * @param vertices
     * @param triangleIndices
     * @param boundaryEdges
     */
    Cluster(
        const fstd::span<const PackedStaticVertexData>& vertices,
        const fstd::span<const uint32_t>& triangleIndices,
        const std::vector<bool>& boundaryEdges
    );

private:
    // Mesh data
    std::vector<PackedStaticVertexData> mVertices;
    std::vector<uint32_t> mIndices;
    std::vector<bool> mBoundaryEdges;
    std::vector<bool> mExternalEdges;

    // Bounding box
    AABB mBoundingBox;
};
} // namespace Falcor
