#pragma once

#include "Core/Macros.h"

namespace Falcor
{

using double3 = math::vector<double, 3>;

/**
 * @brief Quadric helper struct.
 * Storing necessary quadric matrix data.
 */
struct Quadric
{
    double nxx;
    double nyy;
    double nzz;

    double nxy;
    double nxz;
    double nyz;

    double3 dn;
    double d2;
    double a;

    Quadric() = default;

    Quadric(const float3& p0, const float3& p1, const float3& p2);

    void zero() { nxx = nyy = nzz = nxy = nxz = nyz = dn.x = dn.y = dn.z = d2 = a = 0.0; }
};

struct EdgeQuadric
{
    double nxx;
    double nyy;
    double nzz;

    double nxy;
    double nxz;
    double nyz;

    double3 n;

    double a;

    EdgeQuadric() = default;

    EdgeQuadric(const float3& p0, const float3& p1, float weight);

    void zero() { nxx = nyy = nzz = nxy = nxz = nyz = n.x = n.y = n.z = a = 0.0; }
};

class MeshSimplifier
{
public:
    enum class CornerFlags : uint8_t
    {
        MergeMask = 3,             // Merge position 0 or 1
        AdjTriMask = (1 << 2),     // Has been added to AdjTris
        LockedVertMask = (1 << 3), // Vert is locked, disallowing position movement
        RemoveTriMask = (1 << 4),  // Triangle will overlap another after merge and should be removed
    };
    MeshSimplifier(fstd::span<PackedStaticVertexData> vertices, fstd::span<uint32_t> indices);
    void setEdgeWeight(float weight) { mEdgeWeight = weight; }
    void setLimitErrorToSurfaceArea(bool value) { mLimitErrorToSurfaceArea = value; }

    /**
     * @brief Lock vertex position hint.
     *
     * @param position The position to lock.
     */
    void lockPosition(const float3& position);

    float simplify(
        uint32_t targetVertCount,
        uint32_t targetTriCount,
        float targetError,
        uint32_t limitVertCount,
        uint32_t limitTriCount,
        float limitError
    );
    void preserveSurfaceArea();
    void compact();

    auto getRemainingVertCount() const { return mRemainingVertCount; }
    auto getRemainingTriCount() const { return mRemainingTriCount; }

private:
    using Float3Pair = std::pair<float3, float3>;
    /**
     * @brief Fix up triangle (vertices) to prevent ill-formed triangles.
     *
     * @param triIndex The triangle index(local).
     */
    void fixUpTriangle(uint32_t triIndex);

    auto& getVertex(uint32_t localIndex) { return mVertices[mIndices[localIndex]]; }

    auto& getVertexByVIndex(uint32_t vertIndex) { return mVertices[vertIndex]; }

    void removeDuplicatedVertex(uint32_t localIndex);

    bool addUniquePair(Float3Pair& pair, uint32_t pairIndex);

    void setVertexIndex(uint32_t corner, uint32_t newIndex);

    void removeTriangle(uint32_t triIndex);

    void computeTriangleQuadrics(uint32_t triIndex);

    void computeEdgeQuadrics(uint32_t edgeIndex);

    float evalMerge(float3 position0, float3 position1, bool executeMerge);

    std::vector<Float3Pair> mPairs;
    std::unordered_multimap<uint32_t, uint32_t> mPairHash0;
    std::unordered_multimap<uint32_t, uint32_t> mPairHash1;
    // FBinaryHeap<float> PairHeap;

    std::vector<Quadric> mTriQuadrics;
    std::vector<EdgeQuadric> mEdgeQuadrics;

    fstd::span<PackedStaticVertexData> mVertices;
    fstd::span<uint32_t> mIndices;

    std::unordered_multimap<uint32_t, uint32_t> mVertHashmap; ///< positionHash -> vertexlocalIndex
    std::unordered_multimap<uint32_t, uint32_t> mCornerHashmap;

    std::vector<uint32_t> mVertRefCount; ///< Vertex reference count by corners.
    std::vector<CornerFlags> mCornerFlags;
    std::vector<bool> mTriangleRemoved;
    std::vector<bool> mEdgeQuadricsValid;

    uint32_t mTriCount; ///< The number of triangles in the mesh.

    uint32_t mRemainingVertCount;
    uint32_t mRemainingTriCount;

    float mEdgeWeight = 0.f;
    bool mLimitErrorToSurfaceArea = true;
};

FALCOR_ENUM_CLASS_OPERATORS(MeshSimplifier::CornerFlags);

} // namespace Falcor
