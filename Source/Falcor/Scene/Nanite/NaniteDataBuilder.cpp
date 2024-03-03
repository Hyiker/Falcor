#include "NaniteDataBuilder.h"
#include "Utils/Timing/TimeReport.h"
#include "Cluster.h"
#include "TriangleUtils.h"
#include "Utils/Algorithm/FastDisjointSet.h"

namespace Falcor
{
void NaniteDataBuilder::buildNaniteData(
    fstd::span<const PackedStaticVertexData> vertices,
    fstd::span<const uint32_t> triangleIndices,
    fstd::span<const MeshDesc> meshDescs
)
{
    TimeReport report;

    for (const auto& meshDesc : meshDescs)
    {
        clusterTriangles(vertices, triangleIndices, meshDesc);
    }
    report.measure("Cluster triangles");

    buildDAG();
    report.measure("Build DAG");

    buildCoarseRepresentation();
    report.measure("Build coarse repr");

    encodeNaniteMeshes();
    report.measure("Encode Nanite meshes");

    report.printToLog();
}

void NaniteDataBuilder::clusterTriangles(
    fstd::span<const PackedStaticVertexData> vertices,
    fstd::span<const uint32_t> triangleIndices,
    const MeshDesc& meshDesc
)
{
    bool use16Bit = meshDesc.use16BitIndices();
    uint32_t indexOffset = meshDesc.ibOffset * (use16Bit ? 2 : 1);
    uint32_t indexCount = meshDesc.indexCount;

    std::vector<uint32_t> sharedEdges(indexCount);
    std::vector<bool> boundaryEdges(indexCount, false);

    // adjoint vertex hash -> edge index
    EdgeHash edgeHash;
    EdgeAdjacency edgeAdjacency(indexCount);

    auto convertIndex = [&](uint32_t index) -> uint32_t
    {
        return use16Bit ? reinterpret_cast<const uint16_t*>(triangleIndices.data())[indexOffset + index]
                        : triangleIndices[indexOffset + index];
    };

    auto getPosition = [&](uint32_t index) -> float3 { return vertices[convertIndex(index)].position; };

    // 1. construct edge hash map
    // TODO: parallelize this
    for (uint32_t edgeIndex = 0; edgeIndex < indexCount; edgeIndex++)
    {
        edgeHash.addEdge(edgeIndex, getPosition);
    }

    // 2. find adjacent edges, build adjacency list
    // TODO: parallelize this
    for (uint32_t edgeIndex = 0; edgeIndex < indexCount; edgeIndex++)
    {
        edgeHash.forEachEdgeWithOppositeDirection(
            edgeIndex,
            getPosition,
            [&](uint32_t edgeIndex, uint32_t otherEdgeIndex)
            {
                int adjIndex = -1;
                int adjCount = 0;

                adjCount++;
                adjIndex = otherEdgeIndex;

                // the edge is shared by more than 2 triangles
                if (adjCount > 1)
                {
                    adjIndex = -2;
                }
                edgeAdjacency.direct[edgeIndex] = adjIndex;
            }
        );
    }

    // 3. construct disjoint set of triangles
    DisjointSet disjointSet(indexCount / 3);
    for (uint32_t edgeIndex = 0; edgeIndex < indexCount; edgeIndex++)
    {
        if (edgeAdjacency.direct[edgeIndex] == -2)
        {
            // EdgeHash is built in parallel, so we need to sort before use to ensure determinism.
            // This path is only executed in the rare event that an edge is shared by more than two triangles,
            // so performance impact should be negligible in practice.
            std::vector<std::pair<int, int>> edges;
            edgeAdjacency.forEachAdjacentEdges(
                edgeIndex, [&](uint32_t edgeIndex, int adjIndex) { edges.emplace_back(edgeIndex, adjIndex); }
            );
            std::sort(edges.begin(), edges.end());

            for (const auto& edge : edges)
            {
                edgeAdjacency.link(edge.first, edge.second);
            }
        }
        edgeAdjacency.forEachAdjacentEdges(
            edgeIndex,
            [&](uint32_t edgeIndex0, int edgeIndex1)
            {
                FALCOR_CHECK(edgeIndex1 >= 0, "Invalid edge index");
                if (edgeIndex0 > uint32_t(edgeIndex1))
                {
                    disjointSet.unionSeq(edgeIndex0 / 3, edgeIndex1 / 3);
                }
            }
        );
    }
}

void NaniteDataBuilder::buildDAG()
{
    // Build DAG data for mesh simplification.
}

void NaniteDataBuilder::buildCoarseRepresentation()
{
    // Build the coarse representation of original high precision meshes.
}

void NaniteDataBuilder::encodeNaniteMeshes()
{
    // Encode the Nanite meshes.
}
} // namespace Falcor
