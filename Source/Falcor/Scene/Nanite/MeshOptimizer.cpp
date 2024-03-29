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
    mSimplifiedIndicesCount = meshopt_simplify(
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
        mVertices.data(), mIndices.data(), mSimplifiedIndicesCount, mVertices.data(), mVertices.size(), sizeof(PackedStaticVertexData)
    );

    return resultError;
}
// void MeshOptimizer::lockPosition(const float3& position) {}

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
