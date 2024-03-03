#pragma once
#include "Utils/Algorithm/FastDisjointSet.h"
#include <fstd/span.h>
namespace Falcor
{
class GraphPartitioner
{
public:
    struct GraphData
    {
        int offset;
        int num;

        std::vector<uint32_t> adj;       // adjacency(edge)
        std::vector<uint32_t> adjCost;   // edge cost
        std::vector<uint32_t> adjOffset; // edge offset
    };
    using Range = std::pair<uint32_t, uint32_t>;

    GraphPartitioner(uint32_t numElements);

    std::unique_ptr<GraphData> createGraph(uint32_t numAdjacencies) const;

    /**
     * @brief Build locality links inside the mesh to optimize the data locality.
     *
     * @param disjointSet The disjoint triangle set.
     * @param bounds Mesh bounding box.
     * @param groupIndices The grouping indices of the mesh, only link triangles in the same group.
     * @param getCenter A function to get the center of a triangle by index.
     */
    void buildLocalityLinks(
        DisjointSet& disjointSet,
        const AABB& bounds,
        fstd::span<const uint32_t> groupIndices,
        std::function<float3(uint32_t)> getCenter
    );

private:
    std::vector<Range> mRanges;
    std::vector<uint32_t> mIndices;
    std::vector<uint32_t> mSortedIndices;

    uint32_t mNumElements;
};
} // namespace Falcor
