#include <tbb/parallel_for.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_vector.h>
#include <atomic>

#include "NaniteDataBuilder.h"
#include "Utils/Timing/TimeReport.h"
#include "Cluster.h"
#include "TriangleUtils.h"
#include "Utils/Algorithm/FastDisjointSet.h"
#include "GraphPartitioner.h"
#include "Scene/SceneBuilder.h"

namespace Falcor
{

// Optimize an original mesh by:
// 1. Dedup mesh vertices
void optimizeMesh(NaniteDataBuilder::MeshSpec& meshSpec)
{
    StaticVertexHashMap<uint32_t> vertexHash;
    std::vector<StaticVertexData> newVertexData;
    newVertexData.reserve(meshSpec.staticData.size());
    for (uint32_t& index : meshSpec.indexData)
    {
        const auto& vert = meshSpec.staticData[index];
        auto it = vertexHash.find(vert);
        if (it == vertexHash.end())
        {
            index = newVertexData.size();
            vertexHash[vert] = index;
            newVertexData.push_back(vert);
        }
        else
        {
            index = it->second;
        }
    }
    newVertexData.shrink_to_fit();
    meshSpec.vertexCount = newVertexData.size();
    meshSpec.staticData = std::move(newVertexData);
}

std::vector<uint32_t> NaniteDataBuilder::computeDAGDegree()
{
    tbb::concurrent_vector<int> clusterDegree(mClusters.size(), 0);
    std::vector<uint32_t> degree(mClusters.size());

    constexpr int kParallelThreshold = 1024;
    if (mClusters.size() >= kParallelThreshold)
    {
        tbb::parallel_for(
            0u,
            uint32_t(mClusters.size()),
            [&](uint32_t clusterIndex)
            {
                for (uint32_t child : mClusters[clusterIndex].children)
                {
                    clusterDegree[child]++;
                }
            }
        );
    }
    else
    {
        for (uint32_t clusterIndex = 0; clusterIndex < mClusters.size(); clusterIndex++)
        {
            for (uint32_t child : mClusters[clusterIndex].children)
            {
                clusterDegree[child]++;
            }
        }
    }

    std::copy(clusterDegree.begin(), clusterDegree.end(), degree.begin());
    return degree;
}

void NaniteDataBuilder::buildNaniteData(std::vector<MeshSpec>& meshList)
{
    // manually calculate the triangle count due to the existence of 16bit indices
    uint32_t triangleCount = std::accumulate(
                                 meshList.begin(),
                                 meshList.end(),
                                 0,
                                 [](uint32_t sum, const SceneBuilder::MeshSpec& meshSpec) { return sum + meshSpec.indexCount; }
                             ) /
                             3;
    std::vector<SceneBuilder::MeshSpec> staticMeshList;
    std::vector<uint32_t> clusterCounts;
    staticMeshList.reserve(meshList.size());
    clusterCounts.reserve(meshList.size());

    // Move all static meshes to the list, and remove them from meshList
    // TBN, isStatic indicates whether mesh is instanced. Instanced mesh supported will be implemented in the future.
    auto canClusterize = [](const MeshSpec& mesh)
    { return mesh.isStatic && !mesh.isDynamic() && mesh.topology == Vao::Topology::TriangleList; };
    auto p = std::stable_partition(meshList.begin(), meshList.end(), std::not_fn(canClusterize));
    staticMeshList.insert(staticMeshList.end(), std::make_move_iterator(p), std::make_move_iterator(meshList.end()));
    meshList.erase(p, meshList.end());
    staticMeshList.shrink_to_fit();

    TimeReport report;

    for (auto& staticMesh : staticMeshList)
    {
        // optimizeMesh(staticMesh);
        FALCOR_CHECK(staticMesh.indexOffset == 0, "MeshSpec::indexOffset is not zero");
        FALCOR_CHECK(staticMesh.staticVertexOffset == 0, "MeshSpec::staticVertexOffset is not zero");

        auto clusters = clusterTriangles(staticMesh.staticData, staticMesh.indexData, staticMesh);
        clusterCounts.push_back(uint32_t(clusters.size()));
        mClusters.insert(mClusters.end(), std::move_iterator(clusters.begin()), std::move_iterator(clusters.end()));
    }
    clusterCounts.shrink_to_fit();
    report.measure("Cluster triangles");

    uint32_t clusterOffset = 0;
    for (uint32_t clusterSize : clusterCounts)
    {
        buildDAG(clusterOffset, clusterSize);
        clusterOffset += clusterSize;
    }
    report.measure("Build DAG");

    encodeNaniteMeshes();
    report.measure("Encode Nanite meshes");

    report.printToLog();
}

std::vector<Cluster> NaniteDataBuilder::clusterTriangles(
    fstd::span<const StaticVertexData> vertices,
    fstd::span<const uint32_t> triangleIndices,
    const MeshSpec& meshSpec
)
{
    FALCOR_CHECK(meshSpec.isStatic, "Nanite only supports static meshes for now.");
    constexpr int kClusterSize = Cluster::kSize;
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
                false,
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
                FALCOR_ASSERT(edgeIndex1 >= 0);
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
            graph->addLocalityLinks(triangleIndex, 1);
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
    std::vector<PackedStaticVertexData> packedVertices(meshSpec.vertexCount);
    std::transform(
        std::move_iterator(vertices.begin()),
        std::move_iterator(vertices.end()),
        packedVertices.begin(),
        [](const StaticVertexData& unpackedData) { return PackedStaticVertexData(unpackedData); }
    );
    tbb::parallel_for(
        0u,
        uint32_t(partitioner.getRangesCount()),
        [&](uint32_t i)
        {
            clusters[i] =
                Cluster(packedVertices, convertIndex, meshSpec.materialId, partitioner, edgeAdjacency, ranges[i], meshSpec.isFrontFaceCW);
            clusters[i].instances = meshSpec.instances;
        }
    );
    return clusters;
}

void NaniteDataBuilder::reduceDAG(std::atomic_uint32_t& clusterCount, fstd::span<const uint32_t> children, uint32_t groupIndex)
{
    // Merge
    std::vector<const Cluster*> mergeList(children.size());
    std::transform(children.begin(), children.end(), mergeList.begin(), [&](uint32_t index) { return &mClusters[index]; });

    // Force a deterministic order
    std::sort(mergeList.begin(), mergeList.end(), [](const Cluster* a, const Cluster* b) { return a->getGUID() < b->getGUID(); });

    auto merged = Cluster::merge(mergeList);
    merged.children = {children.begin(), children.end()};

    int numParents = div_round_up(int(merged.mIndices.size()), int(Cluster::kSize * 6));
    int parentStart = 0;
    int parentEnd = 0;

    float parentMaxLODError = 0.0f;

    for (int targetClusterSize = Cluster::kSize - 2; targetClusterSize > Cluster::kSize / 2; targetClusterSize -= 2)
    {
        int targetNumTris = numParents * targetClusterSize;
        targetNumTris = targetNumTris;

        // Simplify merged cluster here
        parentMaxLODError = merged.simplify(targetNumTris, 0.3f, 0.7f);

        if (numParents == 1)
        {
            parentEnd = clusterCount += numParents;
            parentStart = parentEnd - numParents;

            merged.finalize();
            mClusters[parentStart] = std::move(merged);
            break;
        }
        else
        {
            EdgeAdjacency edgeAdj = merged.getEdgeAdjacency();

            GraphPartitioner partitioner(merged.mIndices.size() / 3);
            merged.split(partitioner, edgeAdj);

            if (partitioner.getRanges().size() <= numParents)
            {
                numParents = partitioner.mRanges.size();
                parentEnd = (clusterCount += numParents);
                parentStart = parentEnd - numParents;

                int parent = parentStart;
                for (const auto& range : partitioner.getRanges())
                {
                    mClusters[parent] = Cluster(merged, partitioner, edgeAdj, range);
                    parent++;
                }
                break;
            }

            // Start over from scratch. Continuing from simplified cluster screws up ExternalEdges and LODError.
            merged = Cluster::merge(mergeList);
            merged.children = {children.begin(), children.end()};
        }
    }

    ClusterGroup group;
    group.mipLevel = merged.mMipLevel - 1;

    for (uint32_t child : children)
    {
        mClusters[child].mGroupIndex = groupIndex;
        group.childrenIndices.push_back(child);
    }
    FALCOR_CHECK(uint32_t(group.childrenIndices.size()) <= ClusterGroup::kMaxChildren, "Too many children in a cluster group");

    // ignore lod related problem so far
    // FSphere3f ParentLODBounds(Children_LODBounds.GetData(), Children_LODBounds.Num());
    // FSphere3f ParentBounds(Children_SphereBounds.GetData(), Children_SphereBounds.Num());

    // // Force parents to have same LOD data. They are all dependent.
    // for (int32 Parent = ParentStart; Parent < ParentEnd; Parent++)
    // {
    //     Clusters[Parent].LODBounds = ParentLODBounds;
    //     Clusters[Parent].LODError = ParentMaxLODError;
    //     Clusters[Parent].GeneratingGroupIndex = GroupIndex;
    // }

    // Groups[GroupIndex].Bounds = ParentBounds;
    // Groups[GroupIndex].LODBounds = ParentLODBounds;
    // Groups[GroupIndex].MinLODError = ChildMinLODError;
    // Groups[GroupIndex].MaxParentLODError = ParentMaxLODError;
    // Groups[GroupIndex].MipLevel = Merged.MipLevel - 1;
    // Groups[GroupIndex].MeshIndex = MeshIndex;
    // Groups[GroupIndex].bTrimmed = false;
    mClusterGroups[groupIndex] = std::move(group);
}
void NaniteDataBuilder::buildDAG(uint32_t clusterRangeStart, uint32_t clusterRangeCount)
{
    // Build DAG data for mesh simplification.
    bool isFirstLevel = true;
    uint32_t clusterLevelOffset = clusterRangeStart;
    std::atomic_uint32_t clusterCount = mClusters.size();
    AABB meshBounds;

    while (true)
    {
        fstd::span<Cluster> levelClusters(
            mClusters.data() + clusterLevelOffset, isFirstLevel ? clusterRangeCount : mClusters.size() - clusterLevelOffset
        );
        // Resizing clusters will invalidate levelClusters, so we need to reassign it after resize.
        auto resizeClusters = [&, clusterLevelOffset](uint32_t newSize)
        {
            mClusters.resize(newSize);
            levelClusters =
                fstd::span<Cluster>(mClusters.data() + clusterLevelOffset, isFirstLevel ? clusterRangeCount : newSize - clusterLevelOffset);
        };
        isFirstLevel = false;
        uint32_t numExternalEdges = 0;

        for (auto& cluster : levelClusters)
        {
            numExternalEdges += cluster.getExternalEdgesCount();
            meshBounds |= cluster.getBounds();
        }

        if (levelClusters.size() < 2)
            break;

        if (levelClusters.size() <= ClusterGroup::kMaxSize)
        {
            // Reduce children if cluster count small enough to fit in a single cluster group.
            std::vector<uint32_t> children;
            children.reserve(levelClusters.size());

            uint32_t maxParents = 0;
            for (const auto& cluster : levelClusters)
            {
                maxParents += div_round_up(uint32_t(cluster.getIndexCount()), Cluster::kSize * 6);
                children.push_back(clusterLevelOffset++);
            }

            clusterLevelOffset = mClusters.size();
            resizeClusters(mClusters.size() + maxParents);
            mClusterGroups.emplace_back();

            reduceDAG(clusterCount, children, mClusterGroups.size() - 1);

            resizeClusters(clusterCount);

            continue;
        }

        struct ExternalEdge
        {
            uint32_t clusterIndex;
            int edgeIndex;
        };
        std::vector<ExternalEdge> externalEdges;
        tbb::concurrent_unordered_multimap<uint32_t, int> externalEdgeHash(1ull << uint32_t(std::floor(std::log2(numExternalEdges))));
        std::atomic_uint32_t externalEdgeOffset(0);

        // We have a total count of NumExternalEdges so we can allocate a hash table without growing.
        externalEdges.resize(numExternalEdges);

        // Add edges to hash table
        tbb::parallel_for(
            0u,
            uint32_t(levelClusters.size()),
            [&](uint32_t clusterIndex)
            {
                auto& cluster = levelClusters[clusterIndex];

                for (int edgeIndex = 0; edgeIndex < cluster.mExternalEdges.size(); edgeIndex++)
                {
                    if (cluster.mExternalEdges[edgeIndex])
                    {
                        uint32_t vertIndex0 = edgeIndex;
                        uint32_t vertIndex1 = triangleIndexCycle(edgeIndex);

                        const float3& position0 = cluster.getVertex(vertIndex0).position;
                        const float3& position1 = cluster.getVertex(vertIndex1).position;

                        uint32_t hash0 = hashPosition(position0);
                        uint32_t hash1 = hashPosition(position1);
                        uint32_t hash = murmur32({hash0, hash1});

                        uint32_t externalEdgeIndex = externalEdgeOffset.fetch_add(1);
                        externalEdges[externalEdgeIndex] = {clusterIndex, edgeIndex};
                        externalEdgeHash.emplace(hash, externalEdgeIndex);
                    }
                }
            }
        );

        FALCOR_CHECK(
            externalEdgeOffset == externalEdges.size(),
            "External edge offset mismatch: {} != {}",
            externalEdgeOffset.load(),
            externalEdges.size()
        );

        std::atomic_uint32_t numAdjacency(0);

        // Find matching edge in other clusters
        tbb::parallel_for(
            0u,
            uint32_t(levelClusters.size()),
            [&](uint32_t clusterIndex)
            {
                auto& cluster = levelClusters[clusterIndex];

                for (int edgeIndex = 0; edgeIndex < cluster.mExternalEdges.size(); edgeIndex++)
                {
                    if (cluster.mExternalEdges[edgeIndex])
                    {
                        uint32_t vertIndex0 = edgeIndex;
                        uint32_t vertIndex1 = triangleIndexCycle(edgeIndex);

                        float3 position0 = cluster.getVertex(vertIndex0).position;
                        float3 position1 = cluster.getVertex(vertIndex1).position;

                        uint32_t hash0 = hashPosition(position0);
                        uint32_t hash1 = hashPosition(position1);
                        uint32_t hash = murmur32({hash1, hash0});

                        auto range = externalEdgeHash.equal_range(hash);
                        for (auto it = range.first; it != range.second; it++)
                        {
                            uint32_t externalEdgeIndex = it->second;
                            const auto& externalEdge = externalEdges[externalEdgeIndex];

                            auto& otherCluster = levelClusters[externalEdge.clusterIndex];

                            if (otherCluster.mExternalEdges[externalEdge.edgeIndex])
                            {
                                uint32_t otherVertIndex0 = externalEdge.edgeIndex;
                                uint32_t otherVertIndex1 = triangleIndexCycle(externalEdge.edgeIndex);

                                if (all(position0 == otherCluster.getVertex(otherVertIndex1).position) &&
                                    all(position1 == otherCluster.getVertex(otherVertIndex0).position))
                                {
                                    if (clusterIndex != externalEdge.clusterIndex)
                                    {
                                        // Increase it's count
                                        // cluster.mAdjacentClusters.FindOrAdd(ExternalEdge.ClusterIndex, 0)++;
                                        cluster.mAdjacentClusters.insert({externalEdge.clusterIndex, 0u}).first->second++;

                                        // Can't break or a triple edge might be non-deterministically connected.
                                        // Need to find all matching, not just first.
                                    }
                                }
                            }
                        }
                    }
                }
                numAdjacency += cluster.mAdjacentClusters.size();

                // TODO: Force deterministic order of adjacency.
                // Cluster.AdjacentClusters.KeySort([&LevelClusters](uint32 A, uint32 B)
                //                                  { return LevelClusters[A].GUID < LevelClusters[B].GUID; });
            }
        );

        DisjointSet disjointSet(levelClusters.size());

        for (uint32_t clusterIndex = 0; clusterIndex < (uint32_t)levelClusters.size(); clusterIndex++)
        {
            for (auto& pair : levelClusters[clusterIndex].mAdjacentClusters)
            {
                uint32_t otherClusterIndex = pair.first;

                auto it = levelClusters[otherClusterIndex].mAdjacentClusters.find(clusterIndex);
                FALCOR_ASSERT(it != levelClusters[otherClusterIndex].mAdjacentClusters.end());
                uint32_t count = it->second;
                FALCOR_ASSERT(count == pair.second);

                if (clusterIndex > otherClusterIndex)
                {
                    disjointSet.unionSeq(clusterIndex, otherClusterIndex);
                }
            }
        }

        // Partition adjacent clusters again to form better cluster groups.
        GraphPartitioner partitioner(levelClusters.size());

        // Sort to force deterministic order
        std::sort(
            partitioner.mIndices.begin(),
            partitioner.mIndices.end(),
            [&](uint32_t a, uint32_t b) { return levelClusters[a].getGUID() < levelClusters[b].getGUID(); }
        );

        auto getCenter = [&](uint32_t index) { return levelClusters[index].getBounds().center(); };
        partitioner.buildLocalityLinks(disjointSet, {}, meshBounds, getCenter);

        auto graph = partitioner.createGraph(numAdjacency);

        for (int i = 0; i < levelClusters.size(); i++)
        {
            graph->adjOffset[i] = graph->adj.size();

            uint32_t clusterIndex = partitioner.mIndices[i];

            for (auto& pair : levelClusters[clusterIndex].mAdjacentClusters)
            {
                uint32_t otherClusterIndex = pair.first;
                uint32_t numSharedEdges = pair.second;

                const auto& cluster0 = mClusters[clusterLevelOffset + clusterIndex];
                const auto& cluster1 = mClusters[clusterLevelOffset + otherClusterIndex];

                bool isSiblings =
                    cluster0.mGroupIndex != std::numeric_limits<uint32_t>::max() && cluster0.mGroupIndex == cluster1.mGroupIndex;

                graph->addAdjacency(otherClusterIndex, numSharedEdges * (isSiblings ? 1 : 16) + 4);
            }

            graph->addLocalityLinks(clusterIndex, 1);
        }
        graph->adjOffset[graph->num] = graph->adj.size();

        // bool isSingleThreaded = LevelClusters.Num() <= 32;

        partitioner.partition(*graph, ClusterGroup::kMinSize, ClusterGroup::kMaxSize);

        uint32_t maxParents = 0;
        for (const auto& range : partitioner.getRanges())
        {
            uint32_t numParentIndices = 0;
            for (uint32_t i = range.first; i < range.second; i++)
            {
                // Global indexing is needed in Reduce()
                partitioner.mIndices[i] += clusterLevelOffset;
                numParentIndices += mClusters[partitioner.mIndices[i]].mIndices.size();
            }
            maxParents += div_round_up(numParentIndices, uint32_t(Cluster::kSize * 6));
        }

        clusterLevelOffset = mClusters.size();

        const uint32_t parentsStartOffset = mClusters.size();
        resizeClusters(mClusters.size() + maxParents);
        mClusterGroups.resize(mClusterGroups.size() + partitioner.mRanges.size());

        tbb::parallel_for(
            0u,
            (uint32_t)partitioner.getRangesCount(),
            [&](uint32_t partitionIndex)
            {
                const auto& range = partitioner.mRanges[partitionIndex];

                fstd::span<uint32_t> children(partitioner.mIndices.data() + range.first, range.second - range.first);

                // Force a deterministic order
                std::sort(
                    children.begin(),
                    children.end(),
                    [&](uint32_t a, uint32_t b) { return mClusters[a].getGUID() < mClusters[b].getGUID(); }
                );

                uint32_t clusterGroupIndex = partitionIndex + mClusterGroups.size() - partitioner.mRanges.size();

                reduceDAG(clusterCount, children, clusterGroupIndex);
            }
        );

        // Correct num to atomic count
        resizeClusters(clusterCount);

        // Force a deterministic order of the generated parent clusters
        {
            // TODO: Optimize me.
            // Just sorting the array directly seems like the safest option at this stage (right before UE5 final build).
            // On AOD_Shield this seems to be on the order of 0.01s in practice.
            // As the Clusters array is already conservatively allocated, it seems storing the parent clusters in their designated
            // conservative ranges and then doing a compaction pass at the end would be a more efficient solution that doesn't involve
            // sorting.

            fstd::span<Cluster> parents(mClusters.data() + parentsStartOffset, clusterCount - parentsStartOffset);
            std::sort(parents.begin(), parents.end(), [&](const Cluster& a, const Cluster& b) { return a.getGUID() < b.getGUID(); });
        }
    }

    // Max out root node
    FALCOR_ASSERT(mClusters.size() > 0, "No clusters found");
    uint32_t rootIndex = std::min(clusterLevelOffset, uint32_t(mClusters.size() - 1));
    ClusterGroup rootClusterGroup{
        mClusters[rootIndex].getMipLevel() + 1, ///< MipLevel
        Sphere(),                               ///< Bounds
        Sphere(),                               ///< LODBounds
        {rootIndex}                             ///< Children
    };
    // RootClusterGroup.Children.Add(RootIndex);
    // RootClusterGroup.Bounds = Clusters[RootIndex].SphereBounds;
    // RootClusterGroup.LODBounds = FSphere3f(0);
    // RootClusterGroup.MaxParentLODError = 1e10f;
    // RootClusterGroup.MinLODError = -1.0f;
    // RootClusterGroup.MeshIndex = MeshIndex;
    // RootClusterGroup.bTrimmed = false;
    mClusters[rootIndex].mGroupIndex = mClusterGroups.size();
    mClusterGroups.push_back(rootClusterGroup);
}

void NaniteDataBuilder::encodeNaniteMeshes()
{
    // Encode the Nanite meshes.
}

} // namespace Falcor
