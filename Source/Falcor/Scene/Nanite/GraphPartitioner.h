#pragma once
#include "Utils/Algorithm/FastDisjointSet.h"
#include <fstd/span.h>
#include <metis.h>
namespace Falcor
{
/**
 * @brief GraphPartioner class is responsible for mesh triangle graph partitioning.
 * Finally produce a triangle cluster graph for the mesh.
 */
class GraphPartitioner
{
public:
    /**
     * @brief GraphData prepared for METIS partitioning.
     */
    struct GraphData
    {
        int offset;
        int num;

        std::vector<idx_t> adj;       // adjacency edge index
        std::vector<idx_t> adjCost;   // edge cost
        std::vector<idx_t> adjOffset; // edge offset

        GraphData(GraphPartitioner& partitioner, uint32_t numAdjacency, uint32_t numElements) : mPartitioner(partitioner)
        {
            numAdjacency += partitioner.mLocalityLinks.size();

            offset = 0;
            num = numElements;
            adj.reserve(numAdjacency);
            adjCost.reserve(numAdjacency);
            adjOffset.resize(numElements + 1);
        }
        /**
         * @brief Add adjacency relationship in the graph.
         *
         * @param adjIndex Indexing the sorted indices.
         * @param cost The corresponding cost.
         */
        void addAdjacency(uint32_t adjIndex, idx_t cost)
        {
            adj.push_back(mPartitioner.mSortedIndices[adjIndex]);
            adjCost.push_back(cost);
        }
        /**
         * @brief Add adjacency by locality link index.
         *
         * @param index The index refers to a collection of locality links.
         * @param cost The corresponding cost.
         */
        void addLocalityLinks(uint32_t index, idx_t cost)
        {
            auto range = mPartitioner.mLocalityLinks.equal_range(index);
            for (auto it = range.first; it != range.second; it++)
            {
                addAdjacency(it->second, cost);
            }
        }

    private:
        GraphPartitioner& mPartitioner;
    };
    /* Range with left inclusive, right exclusive */
    using Range = std::pair<uint32_t, uint32_t>;

    /**
     * @brief Construct a new Graph Partitioner object.
     *
     * @param numElements The number of elements in the graph.
     */
    GraphPartitioner(uint32_t numElements);

    /**
     * @brief Create a Graph object with the same number of elements.
     *
     * @param numAdjacency The number of adjacencies edges.
     * @return std::unique_ptr<GraphData> The graph object.
     */
    std::unique_ptr<GraphData> createGraph(uint32_t numAdjacency) const;

    /**
     * @brief Build locality links inside the mesh to optimize the data locality.
     * This function outputs to the mSortedIndices, mIndices and mLocalityLinks.
     *
     * @param disjointSet The disjoint triangle set.
     * @param groupIndices The group indices, only element in a same group will be linked.
     * @param bounds Mesh bounding box.
     * @param getCenter A function to get the center of a triangle by index.
     */
    void buildLocalityLinks(
        DisjointSet& disjointSet,
        fstd::span<const uint32_t> groupIndices,
        const AABB& bounds,
        std::function<float3(uint32_t)> getCenter
    );

    /**
     * @brief Partition the graph into several ranges.
     *
     * @param graph The graph to be partitioned.
     * @param minPartitionSize The minimum partition(cluster) size.
     * @param maxPartitionSize The maximum partition(cluster) size.
     */
    void partition(GraphData& graph, uint32_t minPartitionSize, uint32_t maxPartitionSize);

    /**
     * @brief Get sorted triangle index.
     *
     * @param i The index of the indices.
     * @return The triangle index.
     */
    auto getIndex(uint32_t i) const { return mIndices[i]; }

    /**
     * @brief Get the Sorted index.
     *
     * @param i The index of the sorted indices.
     * @return The sorted index.
     */
    auto getSortedIndex(uint32_t i) const { return mSortedIndices[i]; }

    /**
     * @brief Get the number of ranges(namely, clusters).
     *
     * @return The number of ranges.
     */
    auto getRangesCount() const { return mRanges.size(); }

    /**
     * @brief Get the Range by index.
     *
     * @param i The index of the range.
     * @return The partition cluster range.
     */
    auto getRange(uint32_t i) const { return mRanges[i]; }

    /**
     * @brief Get the Ranges object.
     *
     * @return Partition result ranges.
     */
    const auto& getRanges() const { return mRanges; }

private:
    std::vector<Range> mRanges;                       // Partition result ranges
    std::vector<uint32_t> mIndices;                   // The key of sorted indices
    std::vector<uint32_t> mSortedIndices;             // The sorted indices
    std::multimap<uint32_t, uint32_t> mLocalityLinks; // Locally nearby nodes link

    std::vector<idx_t> mPartitionIDs; // The output partition ID for each triangle

    uint32_t mNumElements;
    friend struct GraphData;
    friend class NaniteDataBuilder;
};
} // namespace Falcor
