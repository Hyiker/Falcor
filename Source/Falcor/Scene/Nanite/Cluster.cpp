#include "Cluster.h"
namespace Falcor
{
Cluster::Cluster(
    const fstd::span<const PackedStaticVertexData>& vertices,
    std::function<uint32_t(uint32_t)> convertIndex,
    const GraphPartitioner& partitioner,
    const EdgeAdjacency& adjacency,
    const GraphPartitioner::Range& triangleRange,
    std::function<void(uint32_t, uint64_t)> clusterCallback
)
{
    mGUID = (uint64_t(triangleRange.first) << 32) | triangleRange.second;
    uint32_t numTriangles = triangleRange.second - triangleRange.first;

    mVertices.reserve(numTriangles * 3);
    mIndices.reserve(numTriangles * 3);
    mExternalEdges.reserve(numTriangles * 3);

    std::map<uint32_t, uint32_t> indexMap; // from global index to cluster local index

    for (uint32_t i = triangleRange.first; i < triangleRange.second; i++)
    {
        uint32_t triangleIndex = partitioner.getIndex(i);
        clusterCallback(triangleIndex, mGUID);
        for (uint32_t j = 0; j < 3; j++)
        {
            uint32_t globalIndex = convertIndex(triangleIndex * 3 + j);
            auto it = indexMap.find(globalIndex);

            if (it == indexMap.end())
            {
                uint32_t localIndex = uint32_t(mVertices.size());
                indexMap[globalIndex] = localIndex;
                auto vertex = vertices[globalIndex];

                mVertices.push_back(std::move(vertex));
                mIndices.push_back(localIndex);
            }
            else
            {
                mIndices.push_back(it->second);
            }

            uint32_t edgeIndex = triangleIndex * 3 + j;
            int adjNum = 0;
            // accumulate edge adjacency count
            adjacency.forEachAdjacentEdges(
                edgeIndex,
                [&](uint32_t edgeIndex0, int edgeIndex1)
                {
                    FALCOR_ASSERT(edgeIndex1 >= 0, "Invalid edge index");
                    uint32_t adjTriangle = edgeIndex1 / 3;
                    if (adjTriangle < triangleRange.first || adjTriangle >= triangleRange.second)
                    {
                        adjNum++;
                    }
                }
            );

            mExternalEdges.push_back(uint8_t(adjNum));
            mExternalEdgeCount += adjNum != 0 ? 1 : 0;
        }
    }

    // mBoundaryEdges = boundaryEdges;
    mVertices.shrink_to_fit();
    mIndices.shrink_to_fit();
    mExternalEdges.shrink_to_fit();

    computeBounds();
}

Cluster::Cluster(
    const Cluster& srcCluster,
    const GraphPartitioner& partitioner,
    const EdgeAdjacency& adjacency,
    const GraphPartitioner::Range& triangleRange
)
    : mGUID(murmurFinalize64(srcCluster.mGUID) ^ ((uint64_t(triangleRange.first) << 32) | triangleRange.second))
{
    uint32_t numTriangles = triangleRange.second - triangleRange.first;

    mVertices.reserve(numTriangles * 3);
    mIndices.reserve(numTriangles * 3);
    mExternalEdges.reserve(numTriangles * 3);

    std::map<uint32_t, uint32_t> indexMap; // from global index to cluster local index

    for (uint32_t i = triangleRange.first; i < triangleRange.second; i++)
    {
        uint32_t triangleIndex = partitioner.getIndex(i);
        for (uint32_t j = 0; j < 3; j++)
        {
            uint32_t srcIndex = srcCluster.mIndices[triangleIndex * 3 + j];
            auto it = indexMap.find(srcIndex);

            if (it == indexMap.end())
            {
                uint32_t localIndex = uint32_t(mVertices.size());
                indexMap[srcIndex] = localIndex;
                auto vertex = srcCluster.mVertices[srcIndex];

                mVertices.push_back(std::move(vertex));
                mIndices.push_back(localIndex);
            }
            else
            {
                mIndices.push_back(it->second);
            }

            uint32_t edgeIndex = triangleIndex * 3 + j;
            int adjNum = (int)srcCluster.mExternalEdges[edgeIndex];
            // accumulate uncounted external edges
            adjacency.forEachAdjacentEdges(
                edgeIndex,
                [&](uint32_t edgeIndex0, int edgeIndex1)
                {
                    FALCOR_ASSERT(edgeIndex1 >= 0, "Invalid edge index");
                    uint32_t adjTriangle = edgeIndex1 / 3;
                    if (adjTriangle < triangleRange.first || adjTriangle >= triangleRange.second)
                    {
                        adjNum++;
                    }
                }
            );

            mExternalEdges.push_back(uint8_t(adjNum));
            mExternalEdgeCount += adjNum != 0 ? 1 : 0;
        }
    }

    // mBoundaryEdges = boundaryEdges;
    mVertices.shrink_to_fit();
    mIndices.shrink_to_fit();
    mExternalEdges.shrink_to_fit();

    computeBounds();
}

void Cluster::computeBounds()
{
    mBoundingBox = AABB();
    for (const auto& vertex : mVertices)
    {
        mBoundingBox.include(vertex.position);
    }
}

float Cluster::simplify(uint32_t targetTriangleCount)
{
#if 0
    if ((TargetNumTris >= NumTris && TargetError == 0.0f) || LimitNumTris >= NumTris)
    {
        return 0.0f;
    }

    float UVArea[MAX_STATIC_TEXCOORDS] = {0.0f};

    for (uint32 TriIndex = 0; TriIndex < NumTris; TriIndex++)
    {
        uint32 Index0 = Indexes[TriIndex * 3 + 0];
        uint32 Index1 = Indexes[TriIndex * 3 + 1];
        uint32 Index2 = Indexes[TriIndex * 3 + 2];

        FVector2f* UV0 = GetUVs(Index0);
        FVector2f* UV1 = GetUVs(Index1);
        FVector2f* UV2 = GetUVs(Index2);

        for (uint32 UVIndex = 0; UVIndex < NumTexCoords; UVIndex++)
        {
            FVector2f EdgeUV1 = UV1[UVIndex] - UV0[UVIndex];
            FVector2f EdgeUV2 = UV2[UVIndex] - UV0[UVIndex];
            float SignedArea = 0.5f * (EdgeUV1 ^ EdgeUV2);
            UVArea[UVIndex] += FMath::Abs(SignedArea);

            // Force an attribute discontinuity for UV mirroring edges.
            // Quadric could account for this but requires much larger UV weights which raises error on meshes which have no visible issues
            // otherwise.
            MaterialIndexes[TriIndex] |= (SignedArea >= 0.0f ? 1 : 0) << (UVIndex + 24);
        }
    }

    float TriangleSize = FMath::Sqrt(SurfaceArea / (float)NumTris);

    FFloat32 CurrentSize(FMath::Max(TriangleSize, THRESH_POINTS_ARE_SAME));
    FFloat32 DesiredSize(0.25f);
    FFloat32 FloatScale(1.0f);

    // Lossless scaling by only changing the float exponent.
    int32 Exponent = FMath::Clamp((int)DesiredSize.Components.Exponent - (int)CurrentSize.Components.Exponent, -126, 127);
    FloatScale.Components.Exponent = Exponent + 127; // ExpBias
    // Scale ~= DesiredSize / CurrentSize
    // When generating nanite fallback meshes, use the same weights as in FQuadricSimplifierMeshReduction::ReduceMeshDescription
    // to ensure consistent LOD transition screen size.
    float PositionScale = bForNaniteFallback ? 1.f : FloatScale.FloatValue;

    for (uint32 i = 0; i < NumVerts; i++)
    {
        GetPosition(i) *= PositionScale;
    }
    TargetError *= PositionScale;

    uint32 NumAttributes = GetVertSize() - 3;
    float* AttributeWeights = (float*)FMemory_Alloca(NumAttributes * sizeof(float));
    float* WeightsPtr = AttributeWeights;

    // Normal
    *WeightsPtr++ = bForNaniteFallback ? 16.f : 1.0f;
    *WeightsPtr++ = bForNaniteFallback ? 16.f : 1.0f;
    *WeightsPtr++ = bForNaniteFallback ? 16.f : 1.0f;

    if (bHasTangents)
    {
        // Tangent X
        *WeightsPtr++ = 0.0625f;
        *WeightsPtr++ = 0.0625f;
        *WeightsPtr++ = 0.0625f;

        // Tangent Y Sign
        *WeightsPtr++ = 0.5f;
    }

    if (bHasColors)
    {
        *WeightsPtr++ = bForNaniteFallback ? 0.1f : 0.0625f;
        *WeightsPtr++ = bForNaniteFallback ? 0.1f : 0.0625f;
        *WeightsPtr++ = bForNaniteFallback ? 0.1f : 0.0625f;
        *WeightsPtr++ = bForNaniteFallback ? 0.1f : 0.0625f;
    }

    // Normalize UVWeights
    for (uint32 UVIndex = 0; UVIndex < NumTexCoords; UVIndex++)
    {
        if (bForNaniteFallback)
        {
            float MinUV = +FLT_MAX;
            float MaxUV = -FLT_MAX;

            for (uint32 TriIndex = 0; TriIndex < NumTris; TriIndex++)
            {
                uint32 Index0 = Indexes[TriIndex * 3 + 0];
                uint32 Index1 = Indexes[TriIndex * 3 + 1];
                uint32 Index2 = Indexes[TriIndex * 3 + 2];

                FVector2f UV0 = GetUVs(Index0)[UVIndex];
                FVector2f UV1 = GetUVs(Index1)[UVIndex];
                FVector2f UV2 = GetUVs(Index2)[UVIndex];

                MinUV = FMath::Min(MinUV, UV0.X);
                MinUV = FMath::Min(MinUV, UV0.Y);
                MinUV = FMath::Min(MinUV, UV1.X);
                MinUV = FMath::Min(MinUV, UV1.Y);
                MinUV = FMath::Min(MinUV, UV2.X);
                MinUV = FMath::Min(MinUV, UV2.Y);

                MaxUV = FMath::Max(MaxUV, UV0.X);
                MaxUV = FMath::Max(MaxUV, UV0.Y);
                MaxUV = FMath::Max(MaxUV, UV1.X);
                MaxUV = FMath::Max(MaxUV, UV1.Y);
                MaxUV = FMath::Max(MaxUV, UV2.X);
                MaxUV = FMath::Max(MaxUV, UV2.Y);
            }

            *WeightsPtr++ = 0.5f / FMath::Max(1.f, MaxUV - MinUV);
            *WeightsPtr++ = 0.5f / FMath::Max(1.f, MaxUV - MinUV);
        }
        else
        {
            float TriangleUVSize = FMath::Sqrt(UVArea[UVIndex] / (float)NumTris);
            TriangleUVSize = FMath::Max(TriangleUVSize, THRESH_UVS_ARE_SAME);

            *WeightsPtr++ = 1.0f / (128.0f * TriangleUVSize);
            *WeightsPtr++ = 1.0f / (128.0f * TriangleUVSize);
        }
    }
    check((WeightsPtr - AttributeWeights) == NumAttributes);

    FMeshSimplifier Simplifier(Verts.GetData(), NumVerts, Indexes.GetData(), Indexes.Num(), MaterialIndexes.GetData(), NumAttributes);

    TMap<TTuple<FVector3f, FVector3f>, int8> LockedEdges;

    for (int32 EdgeIndex = 0; EdgeIndex < ExternalEdges.Num(); EdgeIndex++)
    {
        if (ExternalEdges[EdgeIndex])
        {
            uint32 VertIndex0 = Indexes[EdgeIndex];
            uint32 VertIndex1 = Indexes[Cycle3(EdgeIndex)];

            const FVector3f& Position0 = GetPosition(VertIndex0);
            const FVector3f& Position1 = GetPosition(VertIndex1);

            Simplifier.LockPosition(Position0);
            Simplifier.LockPosition(Position1);

            LockedEdges.Add(MakeTuple(Position0, Position1), ExternalEdges[EdgeIndex]);
        }
    }

    Simplifier.SetAttributeWeights(AttributeWeights);
    Simplifier.SetCorrectAttributes(CorrectAttributesFunctions[bHasTangents][bHasColors]);
    Simplifier.SetEdgeWeight(bForNaniteFallback ? 512.f : 2.0f);

    if (bForNaniteFallback)
    {
        Simplifier.SetLimitErrorToSurfaceArea(false);
        Simplifier.DegreePenalty = 100.0f;
        Simplifier.InversionPenalty = 1000000.0f;
    }

    float MaxErrorSqr = Simplifier.Simplify(NumVerts, TargetNumTris, FMath::Square(TargetError), 0, LimitNumTris, MAX_flt);

    check(Simplifier.GetRemainingNumVerts() > 0);
    check(Simplifier.GetRemainingNumTris() > 0);

    if (bPreserveArea)
        Simplifier.PreserveSurfaceArea();

    Simplifier.Compact();

    Verts.SetNum(Simplifier.GetRemainingNumVerts() * GetVertSize());
    Indexes.SetNum(Simplifier.GetRemainingNumTris() * 3);
    MaterialIndexes.SetNum(Simplifier.GetRemainingNumTris());
    ExternalEdges.Init(0, Simplifier.GetRemainingNumTris() * 3);

    NumVerts = Simplifier.GetRemainingNumVerts();
    NumTris = Simplifier.GetRemainingNumTris();

    NumExternalEdges = 0;
    for (int32 EdgeIndex = 0; EdgeIndex < ExternalEdges.Num(); EdgeIndex++)
    {
        auto Edge = MakeTuple(GetPosition(Indexes[EdgeIndex]), GetPosition(Indexes[Cycle3(EdgeIndex)]));
        int8* AdjCount = LockedEdges.Find(Edge);
        if (AdjCount)
        {
            ExternalEdges[EdgeIndex] = *AdjCount;
            NumExternalEdges++;
        }
    }

    float InvScale = 1.0f / PositionScale;
    for (uint32 i = 0; i < NumVerts; i++)
    {
        GetPosition(i) *= InvScale;
        Bounds += GetPosition(i);
    }

    for (uint32 TriIndex = 0; TriIndex < NumTris; TriIndex++)
    {
        // Remove UV mirroring bits
        MaterialIndexes[TriIndex] &= 0xffffff;
    }

    return FMath::Sqrt(MaxErrorSqr) * InvScale;
#endif
    return 0.0f;
}

void Cluster::split(GraphPartitioner& partitioner, const EdgeAdjacency& adjacency) const
{
    uint32_t numTriangles = mIndices.size() / 3;
    DisjointSet djs(numTriangles);
    for (int edgeIndex = 0; edgeIndex < mIndices.size(); edgeIndex++)
    {
        adjacency.forEachAdjacentEdges(
            edgeIndex,
            [&djs](uint32_t edgeIndex0, int edgeIndex1)
            {
                FALCOR_CHECK(edgeIndex1 >= 0, "Invalid edge index");
                if (edgeIndex0 > uint32_t(edgeIndex1))
                    djs.unionSeq(edgeIndex0 / 3, edgeIndex1 / 3);
            }
        );
    }

    auto getCenter = [this](uint32_t triangleIndex)
    {
        float3 center;
        center = getVertex(triangleIndex * 3).position;
        center += getVertex(triangleIndex * 3 + 1).position;
        center += getVertex(triangleIndex * 3 + 2).position;
        return center / 3.f;
    };

    partitioner.buildLocalityLinks(djs, {}, mBoundingBox, getCenter);

    auto graph = partitioner.createGraph(mIndices.size());

    for (uint32_t i = 0; i < numTriangles; i++)
    {
        graph->adjOffset[i] = graph->adj.size();

        uint32_t triangleIndex = partitioner.getIndex(i);

        // Add shared edges
        for (int k = 0; k < 3; k++)
        {
            adjacency.forEachAdjacentEdges(
                3 * triangleIndex + k, [&graph](uint32_t edgeIndex, int adjIndex) { graph->addAdjacency(adjIndex / 3, 4 * 65); }
            );
        }

        graph->addLocalityLinks(triangleIndex, 1);
    }
    graph->adjOffset[numTriangles] = graph->adj.size();

    partitioner.partition(*graph, kSize - 4, kSize);
}

Cluster Cluster::merge(fstd::span<const Cluster> clusters)
{
    std::vector<const Cluster*> pClusters(clusters.size());
    std::transform(clusters.begin(), clusters.end(), pClusters.begin(), [](const Cluster& c) { return &c; });
    return merge(pClusters);
}

Cluster Cluster::merge(fstd::span<const Cluster*> pClusters)
{
    FALCOR_CHECK(pClusters.size() > 0, "No clusters to merge");
    Cluster merged;

    const uint32_t predTriCount = pClusters.size() * kSize;

    merged.mVertices.reserve(predTriCount * 3);
    merged.mIndices.reserve(predTriCount * 3);
    merged.mExternalEdges.reserve(predTriCount * 3);
    merged.mExternalEdgeCount = 0;

    PackedStaticVertexHashMap<uint32_t> vertexMap;
    for (const auto& pCluster : pClusters)
    {
        merged.mBoundingBox |= pCluster->mBoundingBox;

        for (uint32_t i = 0; i < pCluster->mIndices.size(); i++)
        {
            uint32_t oldIndex = pCluster->mIndices[i];
            auto it = vertexMap.find(pCluster->mVertices[oldIndex]);
            uint32_t newIndex;
            if (it == vertexMap.end())
            {
                newIndex = uint32_t(merged.mVertices.size());
                vertexMap[pCluster->mVertices[oldIndex]] = newIndex;
                merged.mVertices.push_back(pCluster->mVertices[oldIndex]);
            }
            else
            {
                newIndex = it->second;
            }
            merged.mIndices.push_back(newIndex);
        }
        merged.mExternalEdges.insert(merged.mExternalEdges.end(), pCluster->mExternalEdges.begin(), pCluster->mExternalEdges.end());

        merged.mGUID = murmurFinalize64(merged.mGUID) ^ pCluster->mGUID;
    }

    EdgeAdjacency adjacency = merged.getEdgeAdjacency();

    int childIndex = 0;
    int minIndex = 0;
    int maxIndex = pClusters[0]->mExternalEdges.size();
    // Rebuild adjacency count
    for (int edgeIndex = 0; edgeIndex < merged.mExternalEdges.size(); edgeIndex++)
    {
        if (edgeIndex >= maxIndex)
        {
            childIndex++;
            minIndex = maxIndex;
            maxIndex += pClusters[childIndex]->mExternalEdges.size();
        }

        int adjCount = merged.mExternalEdges[edgeIndex];

        adjacency.forEachAdjacentEdges(
            edgeIndex,
            [&adjCount, minIndex, maxIndex](uint32_t edgeIndex, int adjIndex)
            {
                if (adjIndex < minIndex || adjIndex >= maxIndex)
                    adjCount--;
            }
        );

        // This seems like a sloppy workaround for a bug elsewhere but it is possible an interior edge is moved during simplifiation to
        // match another cluster and it isn't reflected in this count. Sounds unlikely but any hole closing could do this.
        // The only way to catch it would be to rebuild full adjacency after every pass which isn't practical.
        adjCount = std::max(adjCount, 0);

        merged.mExternalEdges[edgeIndex] = (uint8_t)adjCount;
        merged.mExternalEdgeCount += adjCount != 0 ? 1 : 0;
    }

    merged.mVertices.shrink_to_fit();
    merged.mIndices.shrink_to_fit();
    merged.mExternalEdges.shrink_to_fit();

    return merged;
}

EdgeAdjacency Cluster::getEdgeAdjacency() const
{
    EdgeAdjacency adjacency(mIndices.size());
    EdgeHash edgeHash(mIndices.size());

    for (uint32_t i = 0; i < mIndices.size(); i++)
    {
        adjacency.direct[i] = -1;

        // Directly copied from UE5, but this line seems to be useless?
        edgeHash.forEachEdgeWithOppositeDirection(
            i,
            true,
            [&](uint32_t index) { return getVertex(index).position; },
            [&](uint32_t edgeIndex0, uint32_t edgeIndex1) { adjacency.link(edgeIndex0, edgeIndex1); }
        );
    }
    return adjacency;
}
} // namespace Falcor
