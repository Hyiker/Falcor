import Utils.Attributes;
__exported import LightClusterTypes;

struct LightCluster
{
    [root]
    StructuredBuffer<ClusterNode> nodes;   ///< Buffer containing all the cluster nodes.
    StructuredBuffer<uint> lightIndices;   ///< Buffer containing the indices of lights, sorted by clusters they belongs to.
    StructuredBuffer<uint> clusterIndices; ///< Buffer containing the indices of clusters for each light.

    ClusterNode getNode(uint nodeIndex) { return nodes[nodeIndex]; }

    uint getNodeIndexByLightIndex(uint lightIndex) { return clusterIndices[lightIndex]; }

    ClusterNode getNodeByLightIndex(uint lightIndex) { return getNode(getNodeIndexByLightIndex(lightIndex)); }
};
