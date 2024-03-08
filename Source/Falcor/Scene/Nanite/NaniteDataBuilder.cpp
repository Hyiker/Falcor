#include <tbb/parallel_for.h>

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
    // manually calculate the triangle count due to the existence of 16bit indices
    uint32_t triangleCount = std::accumulate(
        meshList.begin(), meshList.end(), 0, [](uint32_t sum, const SceneBuilder::MeshSpec& meshSpec) { return sum + meshSpec.indexCount; }
    );
    triangleCount /= 3;
    mClusterGUIDs.resize(triangleCount);
    TimeReport report;

    for (const auto& meshSpec : meshList)
    {
        if (meshSpec.isStatic)
        {
            auto clusters = clusterTriangles(vertices, triangleIndices, meshSpec);
            mClusters.insert(mClusters.end(), std::move_iterator(clusters.begin()), std::move_iterator(clusters.end()));
        }
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

std::vector<Cluster> NaniteDataBuilder::clusterTriangles(
    fstd::span<const PackedStaticVertexData> vertices,
    fstd::span<const uint32_t> triangleIndices,
    const SceneBuilder::MeshSpec& meshSpec
)
{
    FALCOR_CHECK(meshSpec.isStatic, "Nanite only supports static meshes for now.");
    constexpr int kClusterSize = 128;
    bool use16Bit = meshSpec.use16BitIndices;
    uint32_t indexOffset = meshSpec.indexOffset * (use16Bit ? 2 : 1);
    uint32_t indexCount = meshSpec.indexCount;
    uint32_t vertexOffset = meshSpec.staticVertexOffset;
    uint32_t triangleCount = indexCount / 3;

    std::vector<uint32_t> sharedEdges(indexCount);
    std::vector<bool> boundaryEdges(indexCount, false);

    // adjoint vertex hash -> edge index
    EdgeHash edgeHash(indexCount);
    EdgeAdjacency edgeAdjacency(indexCount);

    auto convertIndex = [&](uint32_t index) -> uint32_t
    {
        return use16Bit ? reinterpret_cast<const uint16_t*>(triangleIndices.data())[indexOffset + index]
                        : triangleIndices[indexOffset + index];
    };

    auto getPosition = [&](uint32_t index) -> float3 { return vertices[convertIndex(index) + vertexOffset].position; };

    // 1. construct edge hash map
    tbb::parallel_for(0u, indexCount, [&](uint32_t edgeIndex) { edgeHash.addEdge(edgeIndex, getPosition); });

    // 2. find adjacent edges, build adjacency list
    tbb::parallel_for(
        0u,
        indexCount,
        [&](uint32_t edgeIndex)
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
    );

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

        partitioner.partition(*graph, kClusterSize - 4, kClusterSize);
        FALCOR_CHECK(partitioner.getRangesCount() > 0, "No clusters found");
    }

    // 5. Create clusters
    std::vector<Cluster> clusters(partitioner.getRangesCount());
    const uint32_t kOptimalClusterCount = div_round_up(indexCount, uint32_t(kClusterSize));

    // TODO: parallelize this
    const auto& ranges = partitioner.getRanges();
    // index straight to uint32 index array
    const uint32_t triangleBase = meshSpec.indexOffset / 3;
    auto clusterCreationCallback = [&](uint32_t triangleOffset, uint64_t clusterGUID)
    { mClusterGUIDs[triangleBase + triangleOffset] = clusterGUID; };
    tbb::parallel_for(
        0u,
        uint32_t(partitioner.getRangesCount()),
        [&](uint32_t i) { clusters[i] = Cluster(vertices, convertIndex, partitioner, edgeAdjacency, ranges[i], clusterCreationCallback); }
    );
    return clusters;
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
