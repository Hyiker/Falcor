#pragma once

#include <fstd/span.h>
#include <Scene/SceneTypes.slang>
namespace Falcor
{
/** Mesh optimizer.
 */
class MeshOptimizer
{
public:
    enum class VertexFlags : uint8_t
    {
        MergeMask = 3,            /// Merge position 0 or 1
        AdjTriMask = (1 << 2),    // Has been added to AdjTris
        Locked = (1 << 3),        // Vert is locked, disallowing position movement
        RemoveTriMask = (1 << 4), // Triangle will overlap another after merge and should be removed
    };
    MeshOptimizer(fstd::span<PackedStaticVertexData> vertices, fstd::span<uint32_t> indices);
    float simplify(uint32_t targetTriCount, float targetError);
    // void lockPosition(const float3& position);

    auto getSimplifiedIndicesCount() const { return mSimplifiedIndicesCount; }
    auto getSimplifiedVerticesCount() const { return mSimplifiedVerticesCount; }

private:
    void remapIndices();

    fstd::span<PackedStaticVertexData> mVertices;
    fstd::span<uint32_t> mIndices;

    std::vector<PackedStaticVertexData> mVerticesRemap;
    std::vector<uint32_t> mIndicesRemap;

    std::vector<uint32_t> mIndicesSimplified;

    uint32_t mSimplifiedIndicesCount = 0;
    uint32_t mSimplifiedVerticesCount = 0;
};
} // namespace Falcor
