#include "MeshSimplifier.h"

#include "Core/Error.h"
#include "TriangleUtils.h"
namespace Falcor
{

Quadric::Quadric(const float3& p0, const float3& p1, const float3& p2)
{
    const double3 p01 = p1 - p0;
    const double3 p02 = p2 - p0;

    // Compute the wedge product, giving the normal direction scaled by
    // twice the triangle area.
    double3 n = cross(p02, p01);

    const double len = length(n);
    const double area = 0.5 * len;
    if (area < 1e-12)
    {
        return;
    }
    else
    {
        n.x /= len;
        n.y /= len;
        n.z /= len;
    }

    nxx = n.x * n.x;
    nyy = n.y * n.y;
    nzz = n.z * n.z;

    nxy = n.x * n.y;
    nxz = n.x * n.z;
    nyz = n.y * n.z;

    const double dist = -dot(n, double3(p0));

    dn = dist * n;
    d2 = dist * dist;

    a = 1.0;
}

EdgeQuadric::EdgeQuadric(const float3& p0, const float3& p1, float weight)
{
    n = p1 - p0;

    const double len = length(n);
    if (len < 1e-5)
    {
        zero();
        return;
    }
    else
    {
        n.x /= len;
        n.y /= len;
        n.z /= len;
    }

    a = weight * len;

    nxx = a - a * n.x * n.x;
    nyy = a - a * n.y * n.y;
    nzz = a - a * n.z * n.z;

    nxy = -a * n.x * n.y;
    nxz = -a * n.x * n.z;
    nyz = -a * n.y * n.z;
}

MeshSimplifier::MeshSimplifier(fstd::span<PackedStaticVertexData> vertices, fstd::span<uint32_t> indices)
    : mVertices(vertices)
    , mIndices(indices)
    , mCornerFlags(indices.size(), CornerFlags(0))
    , mTriangleRemoved(indices.size() / 3, false)
    , mEdgeQuadricsValid(indices.size(), false)
    , mTriCount(uint32_t(indices.size()) / 3)
    , mRemainingVertCount(uint32_t(vertices.size()))
    , mRemainingTriCount(mTriCount)
{
    for (uint32_t localIndex = 0; localIndex < mVertices.size(); localIndex++)
    {
        mVertHashmap.emplace(hashPosition(getVertex(localIndex).position), localIndex);
    }

    for (uint32_t corner = 0; corner < mIndices.size(); corner++)
    {
        uint32_t vertIndex = mIndices[corner];

        mVertRefCount[vertIndex]++;

        const auto& position = getVertex(corner).position;
        mCornerHashmap.emplace(hashPosition(position), corner);

        uint32_t otherCorner = triangleIndexCycle(corner);

        Float3Pair pair;
        pair.first = position;
        pair.second = getVertex(otherCorner).position;

        if (addUniquePair(pair, mPairs.size()))
        {
            mPairs.push_back(pair);
        }
    }
}

void MeshSimplifier::lockPosition(const float3& position) {}

float MeshSimplifier::simplify(
    uint32_t targetVertCount,
    uint32_t targetTriCount,
    float targetError,
    uint32_t limitVertCount,
    uint32_t limitTriCount,
    float limitError
)
{
    FALCOR_CHECK(targetVertCount < mVertices.size() || targetTriCount < mTriCount || targetError > 0.f, "Invalid target parameters");
    FALCOR_CHECK(targetVertCount >= limitVertCount, "Target vertex count must be greater than or equal to the limit vertex count");
    FALCOR_CHECK(targetTriCount >= limitTriCount, "Target triangle count must be greater than or equal to the limit triangle count");
    FALCOR_CHECK(targetError <= limitError, "Target error must be less than or equal to the limit error");

    mTriQuadrics.resize(mTriCount);
    for (uint32_t triIndex = 0; triIndex < mTriCount; triIndex++)
    {
        fixUpTriangle(triIndex);
    }

    for (uint32_t i = 0; i < mIndices.size(); i++)
    {
        computeEdgeQuadrics(i);
    }
#if 0
    using PairHeapItem = std::pair<float, uint32_t>;
    auto pairHeapCmp = [](const PairHeapItem& i1, const PairHeapItem& i2) { return i1.first > i2.first; };
    std::priority_queue<std::pair<float, uint32_t>, std::vector<float, uint32_t>, decltype(pairHeapCmp)> pairHeap(pairHeapCmp);

    for (uint32_t PairIndex = 0, Num = mPairs.size(); PairIndex < Num; PairIndex++)
    {
        auto& Pair = mPairs[PairIndex];

        float MergeError = EvaluateMerge(Pair.Position0, Pair.Position1, false);
        PairHeap.Add(MergeError, PairIndex);
    }

    float MaxError = 0.0f;

    while (PairHeap.Num() > 0)
    {
        uint32 PrevNumVerts = RemainingNumVerts;
        uint32 PrevNumTris = RemainingNumTris;

        if (PairHeap.GetKey(PairHeap.Top()) > LimitError)
            break;

        {
            uint32 PairIndex = PairHeap.Top();
            PairHeap.Pop();

            FPair& Pair = Pairs[PairIndex];

            PairHash0.Remove(HashPosition(Pair.Position0), PairIndex);
            PairHash1.Remove(HashPosition(Pair.Position1), PairIndex);

            float MergeError = EvaluateMerge(Pair.Position0, Pair.Position1, true);
            MaxError = FMath::Max(MaxError, MergeError);
        }

        if (RemainingNumVerts <= TargetNumVerts && RemainingNumTris <= TargetNumTris && MaxError >= TargetError)
        {
            break;
        }

        if (RemainingNumVerts <= LimitNumVerts || RemainingNumTris <= LimitNumTris || MaxError >= LimitError)
        {
            break;
        }

        for (uint32 PairIndex : ReevaluatePairs)
        {
            FPair& Pair = Pairs[PairIndex];

            float MergeError = EvaluateMerge(Pair.Position0, Pair.Position1, false);
            PairHeap.Add(MergeError, PairIndex);
        }
        ReevaluatePairs.Reset();
    }

    // If couldn't hit targets through regular edge collapses, resort to randomly removing triangles.
    {
        uint32 TriIndex = 0;
        while (1)
        {
            if (RemainingNumVerts <= TargetNumVerts && RemainingNumTris <= TargetNumTris && MaxError >= TargetError)
            {
                break;
            }

            if (RemainingNumVerts <= LimitNumVerts || RemainingNumTris <= LimitNumTris || MaxError >= LimitError)
            {
                break;
            }

            while (TriRemoved[TriIndex])
                TriIndex++;

            RemoveTri(TriIndex);
        }
    }

    return MaxError;
#endif
    return 0.0;
}
void MeshSimplifier::preserveSurfaceArea() {}
void MeshSimplifier::compact() {}

void MeshSimplifier::fixUpTriangle(uint32_t triIndex)
{
    FALCOR_CHECK(!mTriangleRemoved[triIndex], "Triangle already removed");

    const float3& p0 = getVertex(triIndex * 3 + 0).position;
    const float3& p1 = getVertex(triIndex * 3 + 1).position;
    const float3& p2 = getVertex(triIndex * 3 + 2).position;

    bool removeTri = is_set(mCornerFlags[triIndex * 3], CornerFlags::RemoveTriMask);

    if (!removeTri)
    {
        // remove if degenerate
        removeTri = all(p0 == p1) || all(p0 == p2) || all(p1 == p2);
    }

    if (!removeTri)
    {
        removeDuplicatedVertex(triIndex * 3 + 0);
        removeDuplicatedVertex(triIndex * 3 + 1);
        removeDuplicatedVertex(triIndex * 3 + 2);

        // if the triangle is duplicated
        uint32_t i0 = mIndices[triIndex * 3 + 0];
        uint32_t i1 = mIndices[triIndex * 3 + 1];
        uint32_t i2 = mIndices[triIndex * 3 + 2];

        uint32_t hash = hashPosition(getVertex(triIndex * 3 + 0).position);
        for (auto it = mVertHashmap.equal_range(hash).first; it != mVertHashmap.equal_range(hash).second; it++)
        {
            uint32_t corner = it->second;
            if (corner != triIndex * 3 && i0 == mIndices[corner] && i1 == mIndices[triangleIndexCycle(corner)] &&
                i0 == mIndices[triangleIndexCycle(corner, 2)])
            {
                removeTri = true;
                break;
            }
        }
    }

    if (removeTri)
    {
        removeTriangle(triIndex);
    }
    else
    {
        computeTriangleQuadrics(triIndex);
    }
}

void MeshSimplifier::removeDuplicatedVertex(uint32_t localIndex)
{
    auto& vertIndex = mIndices[localIndex];
    auto& vert = mVertices[vertIndex];

    uint32_t hash = hashPosition(vert.position);
    auto range = mVertHashmap.equal_range(hash);
    for (auto it = range.first; it != range.second; it++)
    {
        uint32_t otherVertIndex = it->second;
        auto& otherVertex = mVertices[otherVertIndex];

        if (PackedStaticVertexDataEqual<false>()(vert, otherVertex))
        {
            // First entry in hashtable for this vert value is authoritative.
            setVertexIndex(localIndex, otherVertIndex);
            break;
        }
    }
}

bool MeshSimplifier::addUniquePair(Float3Pair& pair, uint32_t pairIndex)
{
    uint32_t hash0 = hashPosition(pair.first);
    uint32_t hash1 = hashPosition(pair.second);

    if (hash0 > hash1)
    {
        std::swap(hash0, hash1);
        std::swap(pair.first, pair.second);
    }

    uint32_t otherPairIndex;
    auto range = mPairHash0.equal_range(hash0);
    for (auto it = range.first; it != range.second; it++)
    {
        otherPairIndex = it->second;
        FALCOR_CHECK(pairIndex != otherPairIndex, "Duplicate pair found");

        auto& otherPair = mPairs[otherPairIndex];

        if (all(pair.first == otherPair.first) && all(pair.second == otherPair.second))
        {
            // Found a duplicate
            return false;
        }
    }

    mPairHash0.emplace(hash0, pairIndex);
    mPairHash1.emplace(hash1, pairIndex);

    return true;
}

void MeshSimplifier::setVertexIndex(uint32_t corner, uint32_t newIndex)
{
    uint32_t& vertIndex = mIndices[corner];
    FALCOR_CHECK(vertIndex != ~0u, "Invalid vertex index: {}", vertIndex);
    FALCOR_CHECK(mVertRefCount[vertIndex] > 0, "Vertex {} has no reference", vertIndex);

    if (vertIndex == newIndex)
        return;

    uint32_t refCount = --mVertRefCount[vertIndex];
    if (refCount == 0)
    {
        // mVertHashmap.erase(hashPosition(getVertex(corner).position));
        auto range = mVertHashmap.equal_range(hashPosition(getVertex(corner).position));
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second == vertIndex)
            {
                mVertHashmap.erase(it);
                break;
            }
        }
        mRemainingVertCount--;
    }

    vertIndex = newIndex;
    if (vertIndex != ~0u)
    {
        mVertRefCount[vertIndex]++;
    }
}

void MeshSimplifier::removeTriangle(uint32_t triIndex)
{
    if (mTriangleRemoved[triIndex])
    {
        return;
    }

    mTriangleRemoved[triIndex] = true;
    mRemainingTriCount--;

    for (uint32_t i = 0; i < 3; i++)
    {
        uint32_t corner = triIndex * 3 + i;
        uint32_t hash = hashPosition(getVertex(corner).position);

        auto range = mVertHashmap.equal_range(hash);
        for (auto it = range.first; it != range.second; it++)
        {
            if (it->second == corner)
            {
                mVertHashmap.erase(it);
                break;
            }
        }
        mEdgeQuadricsValid[corner] = false;

        setVertexIndex(corner, ~0u);
    }
}

void MeshSimplifier::computeTriangleQuadrics(uint32_t triIndex)
{
    uint32_t i0 = mIndices[triIndex * 3 + 0];
    uint32_t i1 = mIndices[triIndex * 3 + 1];
    uint32_t i2 = mIndices[triIndex * 3 + 2];

    mTriQuadrics[triIndex] = Quadric(getVertexByVIndex(i0).position, getVertexByVIndex(i1).position, getVertexByVIndex(i2).position);
}

void MeshSimplifier::computeEdgeQuadrics(uint32_t edgeIndex)
{
    uint32_t triIndex = edgeIndex / 3;
    if (mTriangleRemoved[triIndex])
    {
        mEdgeQuadricsValid[edgeIndex] = false;
        return;
    }

    uint32_t vertIndex0 = mIndices[edgeIndex];
    uint32_t vertIndex1 = mIndices[triangleIndexCycle(edgeIndex)];

    const float3& position0 = getVertexByVIndex(vertIndex0).position;
    const float3& position1 = getVertexByVIndex(vertIndex1).position;

    // Find edge with opposite direction that shares these 2 verts.
    // If none then we need to add an edge constraint.
    /*
              /\
             /  \
            o-<<-o
            o->>-o
             \  /
              \/
    */
    uint32_t hash = hashPosition(position1);
    {
        auto range = mCornerHashmap.equal_range(hash);
        for (auto it = range.first; it != range.second; it++)
        {
            uint32_t corner = it->second;
            uint32_t otherVertIndex0 = mIndices[corner];
            uint32_t otherVertIndex1 = mIndices[triangleIndexCycle(corner)];

            if (vertIndex0 == otherVertIndex1 && vertIndex1 == otherVertIndex0)
            {
                // Found matching edge.
                // No constraints needed so remove any that exist.
                mEdgeQuadricsValid[edgeIndex] = false;
                return;
            }
        }
    }

    // Don't double count attribute discontinuities.
    float weight = mEdgeWeight;
    {
        auto range = mCornerHashmap.equal_range(hash);

        for (auto it = range.first; it != range.second; it++)
        {
            uint32_t corner = it->second;
            uint32_t otherVertIndex0 = mIndices[corner];
            uint32_t otherVertIndex1 = mIndices[triangleIndexCycle(corner)];

            if (all(position0 == getVertexByVIndex(otherVertIndex1).position && position1 == getVertexByVIndex(otherVertIndex0).position))
            {
                // Found matching edge.
                weight *= 0.5f;
                break;
            }
        }
    }

    // Didn't find matching edge. Add edge constraint.
    mEdgeQuadrics[edgeIndex] = EdgeQuadric(getVertexByVIndex(vertIndex0).position, getVertexByVIndex(vertIndex1).position, weight);
    mEdgeQuadricsValid[edgeIndex] = true;
}

float MeshSimplifier::evalMerge(float3 position0, float3 position1, bool executeMerge)
{
    // check( Position0 != Position1 );
    return 0.0;
}
} // namespace Falcor
