#include "NaniteDataBuilder.h"
#include "Utils/Timing/TimeReport.h"
#include "Cluster.h"
#include "TriangleUtils.h"
#include "Utils/Algorithm/FastDisjointSet.h"
#include "GraphPartitioner.h"
#include "Scene/SceneBuilder.h"

namespace Falcor
{
void NaniteDataBuilder::buildNaniteData(
    fstd::span<const PackedStaticVertexData> vertices,
    fstd::span<const uint32_t> triangleIndices,
    fstd::span<const SceneBuilder::MeshSpec> meshList
)
{
    TimeReport report;

    for (const auto& meshSpec : meshList)
    {
        clusterTriangles(vertices, triangleIndices, meshSpec);
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
    const SceneBuilder::MeshSpec& meshSpec
)
{
    bool use16Bit = meshSpec.use16BitIndices;
    uint32_t indexOffset = meshSpec.indexOffset * (use16Bit ? 2 : 1);
    uint32_t indexCount = meshSpec.indexCount;
    uint32_t triangleCount = indexCount / 3;

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
    DisjointSet disjointSet(triangleCount);
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

    // 4. Graph partition
    GraphPartitioner partitioner(triangleCount);

    {
        auto getCenter = [&](uint32_t triangleIndex)
        {
            float3 center = getPosition(triangleIndex * 3);
            center += getPosition(triangleIndex * 3 + 1);
            center += getPosition(triangleIndex * 3 + 2);
            return center / 3.f;
        };

        partitioner.buildLocalityLinks(disjointSet, {}, meshSpec.boundingBox, getCenter);

        auto graph = partitioner.createGraph(indexCount);

        for (uint32_t i = 0; i < triangleCount; i++)
        {
            // add edgeAdjacency to graph
            graph->adjOffset[i] = graph->adj.size();

            uint32_t triangleIndex = partitioner.getIndex(i);

            for (int j = 0; j < 3; j++)
            {
                edgeAdjacency.forEachAdjacentEdges(
                    triangleIndex * 3 + j, [&](uint32_t edgeIndex, int adjIndex) { graph->addAdjacency(adjIndex / 3, 4 * 65); }
                );
            }
            graph->addLocalityLink(triangleIndex, 1);
        }

        graph->adjOffset[triangleCount] = graph->adj.size();

        constexpr int kClusterSize = 128;
        partitioner.partition(*graph, kClusterSize - 4, kClusterSize);
        FALCOR_CHECK(partitioner.getClustersCount() > 0, "No clusters found");
        logInfo("Partitioner partition result: {} clusters", partitioner.getClustersCount());
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
