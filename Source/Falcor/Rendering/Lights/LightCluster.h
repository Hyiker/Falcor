#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
#include "LightClusterTypes.slang"
#include "Rendering/Lights/LightClusterTypes.slang"
#include "Utils/Math/AABB.h"

namespace Falcor
{

class FALCOR_API LightCluster : public Object
{
    FALCOR_OBJECT(LightCluster)
public:
    /** Creates a LightBVHSampler for a given scene.
        \param[in] pRenderContext The render context.
        \param[in] pScene The scene.
    */
    LightCluster(RenderContext* pRenderContext, ref<IScene> pScene);

    struct ClusterStats
    {
        struct ClusterInfo
        {
            AABB aabb;         ///< Cluster aabb.
            float power = 0.f; ///< Total light power emission inside the cluster.

            uint32_t lightCount = 0; ///< Light count of the cluster.
        };
        std::vector<ClusterInfo> lightClusterInfos; ///< For each level in the tree, how many nodes are there.

        uint32_t byteSize = 0;     ///< Number of bytes occupied by the cluster.
        uint32_t clusterCount = 0; ///< Number of light clusters.
    };

    /** Returns stats.
     */
    const ClusterStats& getStats() const { return mClusterStats; }

    /** Render the UI. This default implementation just shows the stats.
     */
    void renderUI(Gui::Widgets& widget);

    /** Bind the light cluster into a shader variable.
        \param[in] var The shader variable to set the data into.
    */
    void bindShaderData(const ShaderVar& var) const;

    bool update(RenderContext* pRenderContext);

    const auto& getNodes() const { return mNodes; }

    const auto& getLightIndices() const { return mLightIndices; }

protected:
    void rebuildClusters(RenderContext* pRenderContext);

    using IndexedLight = std::pair<ref<Light>, uint32_t>;
    /** Cluster lights using Minimal distance cluster, store the result in mNodes.
     */
    void minimalDistanceCluster(fstd::span<IndexedLight> lights, uint32_t lightOffset, float distanceTolerance);

    void finalize();
    void computeStats();
    void renderStats(Gui::Widgets& widget, const ClusterStats& stats) const;

    void uploadCPUBuffers(const std::vector<uint32_t>& lightIndices, const std::vector<uint32_t>& clusterIndices);

    /** Is the cluster valid.
        \return true if the clusters are ready for use.
    */
    bool isValid() const { return mIsValid; }

    /** Invalidate the Cluster.
     */
    void clear();

    // Internal state
    ref<Device> mpDevice;
    ref<const ILightCollection> mpLightCollection;
    ref<IScene> mpScene;

    sigs::Connection mUpdateFlagsConnection; ///< Connection to the UpdateFlags signal.
    /// SceneUpdateFlags accumulated since last `beginFrame()`
    IScene::UpdateFlags mUpdateFlags = IScene::UpdateFlags::None;

    // CPU resources
    mutable std::vector<ClusterNode> mNodes; ///< CPU-side copy of cluster nodes.
    std::vector<uint32_t> mLightIndices;     ///< CPU-side copy of light indices.
    ClusterStats mClusterStats;
    bool mIsValid = false; ///< True when the cluster has been built.
    bool mNeedsRebuild = true;

    // GPU resources
    ref<Buffer> mpClusterNodesBuffer; ///< Buffer holding all cluster nodes.
    ref<Buffer> mpLightIndicesBuffer; ///< Light indices sorted by cluster node, the first light should be representative light of cluster.
    ref<Buffer> mpClusterIndicesBuffer; ///< Cluster indices of each light source.
};
} // namespace Falcor
