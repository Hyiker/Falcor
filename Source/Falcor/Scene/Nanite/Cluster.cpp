#include <assimp/Exporter.hpp>
#include <assimp/scene.h>

#include "Cluster.h"
#include "MeshOptimizer.h"
namespace Falcor
{
Cluster::Cluster(
    const fstd::span<const PackedStaticVertexData>& vertices,
    std::function<uint32_t(uint32_t)> convertIndex,
    MaterialID materialID,
    const GraphPartitioner& partitioner,
    const EdgeAdjacency& adjacency,
    const GraphPartitioner::Range& triangleRange,
    bool isFrontFaceCW
)
    : materialID(materialID), isFrontFaceCW(isFrontFaceCW)
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

    finalize();
}

Cluster::Cluster(
    const Cluster& srcCluster,
    const GraphPartitioner& partitioner,
    const EdgeAdjacency& adjacency,
    const GraphPartitioner::Range& triangleRange
)
    : materialID(srcCluster.materialID)
    , isFrontFaceCW(srcCluster.isFrontFaceCW)
    , mGUID(murmurFinalize64(srcCluster.mGUID) ^ ((uint64_t(triangleRange.first) << 32) | triangleRange.second))
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

    finalize();
}

void Cluster::finalize()
{
    mBoundingBox = AABB();
    mSurfaceArea = 0.f;

    for (const auto& vertex : mVertices)
    {
        mBoundingBox.include(vertex.position);
    }

    for (uint32_t i = 0; i < mIndices.size(); i += 3)
    {
        float3 v0 = mVertices[mIndices[i + 0]].position;
        float3 v1 = mVertices[mIndices[i + 1]].position;
        float3 v2 = mVertices[mIndices[i + 2]].position;

        mSurfaceArea += length(cross(v1 - v0, v2 - v0));
    }

    mSurfaceArea *= 0.5f;
}

float Cluster::simplify(uint32_t targetTriangleCount, float targetError, uint32_t maxTriangles)
{
    uint32_t triangleCount = mIndices.size() / 3;
    if ((targetTriangleCount >= triangleCount && targetError == 0.0f) || maxTriangles >= triangleCount)
    {
        return 0.0f;
    }

    float triangleSize = std::sqrt(mSurfaceArea / (float)triangleCount);

    // save border edges
    auto float3Cmp = [](const float3& a, const float3& b)
    {
        if (a.x != b.x)
            return a.x < b.x;
        if (a.y != b.y)
            return a.y < b.y;
        return a.z < b.z;
    };
    auto float3PairCmp = [&float3Cmp](const std::pair<float3, float3>& a, const std::pair<float3, float3>& b)
    {
        if (!all(a.first == b.first))
            return float3Cmp(a.first, b.first);
        return float3Cmp(a.second, b.second);
    };
    std::map<std::pair<float3, float3>, uint8_t, decltype(float3PairCmp)> lockedEdges(float3PairCmp);
    for (uint32_t edgeIndex = 0; edgeIndex < mExternalEdges.size(); edgeIndex++)
    {
        if (mExternalEdges[edgeIndex])
        {
            uint32_t vertIndex0 = mIndices[edgeIndex];
            uint32_t vertIndex1 = mIndices[triangleIndexCycle(edgeIndex)];

            const float3& position0 = getVertex(edgeIndex).position;
            const float3& position1 = getVertex(triangleIndexCycle(edgeIndex)).position;

            lockedEdges.emplace(std::make_pair(position0, position1), mExternalEdges[edgeIndex]);
        }
    }

    FALCOR_CHECK(targetError >= 0.0f && targetError <= 1.f, "Invalid target error: {}", targetError);

    MeshOptimizer optimizer(mVertices, mIndices);

    float resultError = optimizer.simplify(targetTriangleCount, targetError);

    FALCOR_CHECK(optimizer.getSimplifiedIndicesCount() > 0, "Invalid simplified indices size");
    FALCOR_CHECK(optimizer.getSimplifiedVerticesCount() > 0, "Invalid simplified vertices size");

    triangleCount = optimizer.getSimplifiedIndicesCount() / 3;

    mVertices.resize(optimizer.getSimplifiedVerticesCount());
    mIndices.resize(optimizer.getSimplifiedIndicesCount());
    mExternalEdges.resize(optimizer.getSimplifiedIndicesCount(), 0);

    mExternalEdgeCount = 0;
    for (int edgeIndex = 0; edgeIndex < mExternalEdges.size(); edgeIndex++)
    {
        auto edge = std::make_pair(getVertex(edgeIndex).position, getVertex(triangleIndexCycle(edgeIndex)).position);
        if (auto it = lockedEdges.find(edge); it != lockedEdges.end())
        {
            uint8_t adjCount = it->second;
            mExternalEdges[edgeIndex] = adjCount;
            mExternalEdgeCount++;
        }
    }

    return resultError;
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

    PackedStaticVertexHashMap<uint32_t, true> vertexMap;
    auto materialID = pClusters[0]->materialID;
    bool isFrontFaceCW = pClusters[0]->isFrontFaceCW;
    for (const auto& pCluster : pClusters)
    {
        FALCOR_CHECK(materialID == pCluster->materialID, "Merge cluster material ID mismatch: {} != {}", materialID, pCluster->materialID);
        FALCOR_CHECK(
            isFrontFaceCW == pCluster->isFrontFaceCW, "Merge cluster clockwise mismatch: {} != {}", isFrontFaceCW, pCluster->isFrontFaceCW
        );
        merged.mBoundingBox |= pCluster->mBoundingBox;
        merged.mSurfaceArea += pCluster->mSurfaceArea;
        merged.instances.insert(pCluster->instances.begin(), pCluster->instances.end());

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
    merged.materialID = materialID;
    merged.isFrontFaceCW = isFrontFaceCW;

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

aiScene* createAssimpScene(fstd::span<const PackedStaticVertexData> vertices, fstd::span<const uint32_t> indices)
{
    auto* scene = new aiScene();

    scene->mRootNode = new aiNode();

    scene->mMaterials = new aiMaterial*[1];
    scene->mNumMaterials = 1;
    scene->mMaterials[0] = new aiMaterial();

    scene->mMeshes = new aiMesh*[1];
    scene->mMeshes[0] = new aiMesh();
    scene->mNumMeshes = 1;

    scene->mRootNode->mMeshes = new unsigned int[1];
    scene->mRootNode->mMeshes[0] = 0;
    scene->mRootNode->mNumMeshes = 1;

    aiMesh* mesh = scene->mMeshes[0];
    mesh->mVertices = new aiVector3D[vertices.size()];
    mesh->mNumVertices = vertices.size();

    for (unsigned int i = 0; i < vertices.size(); ++i)
    {
        mesh->mVertices[i] = aiVector3D(vertices[i].position.x, vertices[i].position.y, vertices[i].position.z);
    }

    mesh->mFaces = new aiFace[indices.size() / 3];
    mesh->mNumFaces = indices.size() / 3;

    for (unsigned int i = 0; i < mesh->mNumFaces; ++i)
    {
        aiFace& face = mesh->mFaces[i];
        face.mIndices = new unsigned int[3];
        face.mNumIndices = 3;

        face.mIndices[0] = indices[i * 3];
        face.mIndices[1] = indices[i * 3 + 1];
        face.mIndices[2] = indices[i * 3 + 2];
    }

    return scene;
}

void Cluster::saveToFile(const std::filesystem::path& p) const
{
    FALCOR_CHECK(p.extension() == ".obj", "Only support .obj export");
    auto* scene = createAssimpScene(mVertices, mIndices);

    Assimp::Exporter exporter;
    aiReturn ret = exporter.Export(scene, "obj", p.string());

    if (ret != aiReturn_SUCCESS)
    {
        logError("Assimp export failure: {}", exporter.GetErrorString());
    }

    delete scene;
}
} // namespace Falcor
