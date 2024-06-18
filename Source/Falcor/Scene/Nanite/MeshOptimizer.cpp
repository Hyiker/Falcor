#include <meshoptimizer.h>

#include "MeshOptimizer.h"

namespace Falcor
{
MeshOptimizer::MeshOptimizer(fstd::span<PackedStaticVertexData> vertices, fstd::span<uint32_t> indices)
    : mVertices(vertices), mIndices(indices)
{
    FALCOR_CHECK(mVertices.size() > 3, "Must have at least 3 vertices");
    FALCOR_CHECK(mIndices.size() > 3, "Must have at least 3 indices");
}

float MeshOptimizer::simplify(uint32_t targetTriCount, float targetError)
{
    uint32_t targetIndexCount = targetTriCount * 3;
    float resultError = 0.f;
    uint32_t option = meshopt_SimplifyLockBorder;
    mSimplifiedIndexCount = meshopt_simplify(
        mIndices.data(),
        mIndices.data(),
        mIndices.size(),
        (const float*)mVertices.data(),
        mVertices.size(),
        sizeof(PackedStaticVertexData),
        targetIndexCount,
        targetError,
        option,
        &resultError
    );

    // find unused vertices and remove them
    mSimplifiedVerticesCount = meshopt_optimizeVertexFetch(
        mVertices.data(), mIndices.data(), mSimplifiedIndexCount, mVertices.data(), mVertices.size(), sizeof(PackedStaticVertexData)
    );

    return resultError;
}

float MeshOptimizer::simplifyIterative(uint32_t targetTriCount, float targetError, float maxError, uint32_t maxIterations)
{
    FALCOR_CHECK(maxIterations > 0, "maxIterations must be greater than 0");
    FALCOR_CHECK(maxError > targetError, "maxError must be greater than targetError");
    // Make copies of the original vertices and indices
    std::vector<PackedStaticVertexData> originalVertices(mVertices.begin(), mVertices.end());
    std::vector<uint32_t> originalIndices(mIndices.begin(), mIndices.end());
    uint32_t targetIndexCount = targetTriCount * 3;
    float resultError = 0.f;
    uint32_t option = 0;
    constexpr float kErrorIterationFactor = 1.5f;
    option |= mLockBorder ? meshopt_SimplifyLockBorder : 0;

    while (true)
    {
        mSimplifiedIndexCount = meshopt_simplify(
            mIndices.data(),
            mIndices.data(),
            mIndices.size(),
            (const float*)mVertices.data(),
            mVertices.size(),
            sizeof(PackedStaticVertexData),
            targetIndexCount,
            targetError,
            option,
            &resultError
        );
        maxIterations--;
        if (mSimplifiedIndexCount <= targetIndexCount || maxIterations == 0 || targetError >= maxError)
        {
            // end iteration if
            // 1. target triangle count is reached or
            // 2. max iterations reached
            // 3. max error is reached
            break;
        }
        else
        {
            // otherwise, restore the original vertices and indices and try again
            std::copy(originalVertices.begin(), originalVertices.end(), mVertices.begin());
            std::copy(originalIndices.begin(), originalIndices.end(), mIndices.begin());
            targetError *= kErrorIterationFactor;
            targetError = std::min(targetError, maxError);
        }
    }

    // find unused vertices and remove them
    mSimplifiedVerticesCount = meshopt_optimizeVertexFetch(
        mVertices.data(), mIndices.data(), mSimplifiedIndexCount, mVertices.data(), mVertices.size(), sizeof(PackedStaticVertexData)
    );

    return resultError;
}

void MeshOptimizer::remapIndices()
{
    std::vector<uint32_t> remapTable(mVertices.size());
    size_t vertexCount = meshopt_generateVertexRemap(
        remapTable.data(), mIndices.data(), mIndices.size(), mVertices.data(), mVertices.size(), sizeof(PackedStaticVertexData)
    );

    mVerticesRemap.resize(vertexCount);
    mIndicesRemap.resize(mIndices.size());

    meshopt_remapIndexBuffer(mIndicesRemap.data(), mIndices.data(), mIndices.size(), remapTable.data());
    meshopt_remapVertexBuffer(mVerticesRemap.data(), mVertices.data(), mVertices.size(), sizeof(PackedStaticVertexData), remapTable.data());
}

} // namespace Falcor
