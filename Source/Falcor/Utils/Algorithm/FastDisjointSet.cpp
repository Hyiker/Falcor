#include "FastDisjointSet.h"
#include "Core/Error.h"
namespace Falcor
{
DisjointSet::DisjointSet(uint32_t size) : mParents(size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        mParents[i] = i;
    }
}

void DisjointSet::reset(uint32_t size)
{
    mParents.resize(size);
    for (uint32_t i = 0; i < size; i++)
    {
        mParents[i] = i;
    }
}

// Union with splicing
void DisjointSet::unionUnseq(uint32_t x, uint32_t y)
{
    uint32_t px = mParents[x];
    uint32_t py = mParents[y];

    while (px != py)
    {
        // Pick larger
        if (px < py)
        {
            mParents[x] = py;
            if (x == px)
            {
                return;
            }
            x = px;
            px = mParents[x];
        }
        else
        {
            mParents[y] = px;
            if (y == py)
            {
                return;
            }
            y = py;
            py = mParents[y];
        }
    }
}

void DisjointSet::unionSeq(uint32_t x, uint32_t y)
{
    FALCOR_CHECK(x >= y, "x must be greater than or equal to y");
    FALCOR_CHECK(x == mParents[x], "x's parent must be itself");

    uint32_t px = x;
    uint32_t py = mParents[y];
    while (px != py)
    {
        mParents[y] = px;
        if (y == py)
        {
            return;
        }
        y = py;
        py = mParents[y];
    }
}

uint32_t DisjointSet::find(uint32_t i)
{
    // Find root
    uint32_t start = i;
    uint32_t root = mParents[i];
    while (root != i)
    {
        i = root;
        root = mParents[i];
    }

    // Point all nodes on path to root
    i = start;
    uint32_t parent = mParents[i];
    while (parent != root)
    {
        mParents[i] = root;
        i = parent;
        parent = mParents[i];
    }

    return root;
}
} // namespace Falcor
