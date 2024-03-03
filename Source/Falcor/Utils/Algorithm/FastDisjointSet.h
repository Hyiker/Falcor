#pragma once

#include <vector>
namespace Falcor
{
/**
 * @brief DisjointSet implementation copied from UE5 source code.
 *
 */
class DisjointSet
{
public:
    DisjointSet() = default;

    DisjointSet(uint32_t size);

    /**
     * @brief Reset the disjoint set with a new size.
     *
     * @param size
     */
    void reset(uint32_t size);

    /**
     * @brief Union two sets that contain x and y.
     * No special requirements for the sequence of x and y.
     * @param x
     * @param y
     */
    void unionUnseq(uint32_t x, uint32_t y);

    /**
     * @brief Union two sets that contain x and y.
     * Requires x >= y and x's parent must be itself.
     * Trade off for performance.
     * @param x
     * @param y
     */
    void unionSeq(uint32_t x, uint32_t y);

    /**
     * @brief Find the root parent of the set that contains i.
     * Compress the path during the search.
     * @param i: The index of the element.
     * @return uint32_t: Root parent of the set that contains i.
     */
    uint32_t find(uint32_t i);

private:
    std::vector<uint32_t> mParents;
};
} // namespace Falcor
