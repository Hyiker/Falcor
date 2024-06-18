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

    /**
     * @brief Simplify the mesh.
     *
     * @param targetTriCount Desired triangle count.
     * @param targetError Desired error.
     * @return The actual error.
     */
    float simplify(uint32_t targetTriCount, float targetError);

    /**
     * @brief Iteratively simplify the mesh.
     *        Iteration stops when the error is below maxError or the target triangle count is reached.
     * @param targetTriCount Desired triangle count.
     * @param targetError Desired error.
     * @param maxError Maximum error.
     * @param maxIterations Maximum number of iterations.
     * @return The actual error.
     */
    float simplifyIterative(uint32_t targetTriCount, float targetError, float maxError, uint32_t maxIterations = 3);

    auto getSimplifiedIndicesCount() const { return mSimplifiedIndexCount; }

    auto getSimplifiedVerticesCount() const { return mSimplifiedVerticesCount; }

    void setLockBorder(bool lockBorder) { mLockBorder = lockBorder; }

private:
    void remapIndices();

    fstd::span<PackedStaticVertexData> mVertices;
    fstd::span<uint32_t> mIndices;

    std::vector<PackedStaticVertexData> mVerticesRemap;
    std::vector<uint32_t> mIndicesRemap;

    uint32_t mSimplifiedIndexCount = 0;
    uint32_t mSimplifiedVerticesCount = 0;

    bool mLockBorder = false;
};
} // namespace Falcor
