#include "LightCluster.h"

#include "Core/API/Buffer.h"
#include "Core/API/Formats.h"
#include "Core/API/RenderContext.h"
#include "Core/Error.h"
#include "Core/Program/ShaderVar.h"
#include "Rendering/Lights/LightCluster.h"
#include "Rendering/Lights/LightClusterTypes.slang"
#include "Utils/Math/VectorMath.h"
#include "Utils/Timing/Profiler.h"

namespace Falcor
{

// https://www.forceflow.be/2013/10/07/morton-encodingdecoding-through-bit-interleaving-implementations/
static uint64_t mortonCode(uint32_t a)
{
    uint64_t x = a & 0x1fffff;             // we only look at the first 21 bits
    x = (x | x << 32) & 0x1f00000000ffff;  // shift left 32 bits, OR with self, and 00011111000000000000000000000000000000001111111111111111
    x = (x | x << 16) & 0x1f0000ff0000ff;  // shift left 32 bits, OR with self, and 00011111000000000000000011111111000000000000000011111111
    x = (x | x << 8) & 0x100f00f00f00f00f; // shift left 32 bits, OR with self, and
                                           // 0001000000001111000000001111000000001111000000001111000000000000
    x = (x | x << 4) & 0x10c30c30c30c30c3; // shift left 32 bits, OR with self, and
                                           // 0001000011000011000011000011000011000011000011000011000100000000
    x = (x | x << 2) & 0x1249249249249249;
    return x;
}

LightCluster::LightCluster(RenderContext* pRenderContext, ref<Scene> pScene)
    : mpDevice(pScene->getDevice()), mpLightCollection(pScene->getLightCollection(pRenderContext)), mpScene(pScene)
{}

void LightCluster::renderUI(Gui::Widgets& widget)
{
    // Render the BVH stats.
    renderStats(widget, getStats());
}

bool LightCluster::update(RenderContext* pRenderContext)
{
    FALCOR_PROFILE(pRenderContext, "LightCluster::update");

    bool clusterChanged = false;

    // Check if light collection has changed.
    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::LightCollectionChanged))
    {
        mNeedsRebuild = true;
    }

    if (mNeedsRebuild)
    {
        rebuildClusters(pRenderContext);
        finalize();
        clusterChanged = true;
        mNeedsRebuild = false;
    }
    return clusterChanged;
}

void LightCluster::rebuildClusters(RenderContext* pRenderContext)
{
    const int kDesiredClusterCount = 300;
    using IndexedLight = std::pair<ref<Light>, uint32_t>;
    uint32_t analyticLightCount = mpScene->getLightCount();
    const int kDesiredClusterSize = std::max(1, int(analyticLightCount) / kDesiredClusterCount);
    mNodes.clear();

    // Lights initialization.
    std::vector<IndexedLight> lights(analyticLightCount);
    for (size_t i = 0; i < lights.size(); i++)
    {
        lights[i] = std::make_pair(mpScene->getLight(i), (uint32_t)i);
    }

    // Sort lights with morton encode for better locality.
    auto morton3D = [](const float3 p)
    {
        uint32_t morton = mortonCode((uint32_t)(p.x * 1023.f));
        morton |= mortonCode((uint32_t)(p.y * 1023.f)) << 1;
        morton |= mortonCode((uint32_t)(p.z * 1023.f)) << 2;
        return morton;
    };
    std::sort(
        lights.begin(),
        lights.end(),
        [&](const IndexedLight& l1, const IndexedLight& l2)
        { return morton3D(l1.first->getData().posW) < morton3D(l2.first->getData().posW); }
    );

    // Node cluster initialization.
    std::vector<ClusterNode> nodes;
    float averagePower = 0.0;
    averagePower /= (float)nodes.size();
    for (size_t i = 0; i < lights.size(); i++)
    {
        const auto& data = lights[i].first->getData();

        ClusterNode node;
        node.origin = data.posW;
        node.extent = float3(0.0);
        node.lightCount = 1;
        node.lightOffset = i;
        node.power = math::length(data.intensity);
        averagePower += node.power;

        nodes.push_back(node);
    }
    if (nodes.empty())
    {
        return;
    }

    const float sceneRadius = mpScene->getSceneBounds().radius();
    const float kClusterDistanceToleranceMax = sceneRadius / 10.f * 4.f, kClusterDistanceToleranceMin = sceneRadius / 10.f * 3.f;

    // Greedy clustering
    ClusterNode& cNode = nodes[0];
    float nodeDistance = 0.0;
    AABB nodeBB;
    cNode.getAABB(nodeBB.minPoint, nodeBB.maxPoint);
    for (size_t i = 1; i < nodes.size(); i++)
    {
        auto& node = nodes[i];

        float d = math::length(cNode.origin - node.origin);
        nodeDistance += d;
        float avgDistance = nodeDistance / float(cNode.lightCount + 1);
        if (!(avgDistance < kClusterDistanceToleranceMin ||
              (cNode.lightCount <= (uint32_t)kDesiredClusterSize && avgDistance < kClusterDistanceToleranceMax)))
        {
            mNodes.push_back(cNode);
            nodeDistance = 0.0f;
            cNode = node;
            cNode.getAABB(nodeBB.minPoint, nodeBB.maxPoint);
            continue;
        }

        // Merge this node to cNode otherwise
        nodeBB |= node.origin;
        cNode.origin = nodeBB.center();
        cNode.extent = nodeBB.extent() / 2.f;
        cNode.lightCount++;
        cNode.power += node.power;
    }

    mNodes.push_back(cNode);

    logInfo("LightCluster build {} clusters.", mNodes.size());

    std::vector<uint32_t> lightIndices(analyticLightCount);
    std::vector<uint32_t> clusterIndices(analyticLightCount);
    std::transform(lights.begin(), lights.end(), lightIndices.begin(), [](const IndexedLight& l) { return l.second; });

    for (size_t ni = 0; ni < mNodes.size(); ni++)
    {
        const auto& node = mNodes[ni];
        for (size_t i = 0; i < node.lightCount; i++)
        {
            uint32_t li = lightIndices[node.lightOffset + i];
            clusterIndices[li] = ni;
        }
    }

    uploadCPUBuffers(lightIndices, clusterIndices);

    mIsValid = true;
}

void LightCluster::finalize()
{
    computeStats();
}

void LightCluster::computeStats()
{
    FALCOR_ASSERT(isValid());

    mClusterStats.clusterCount = 0;
    mClusterStats.lightClusterInfos.clear();
    mClusterStats.byteSize = 0;

    mClusterStats.clusterCount = mNodes.size();

    for (const auto& node : mNodes)
    {
        ClusterStats::ClusterInfo info;
        node.getAABB(info.aabb.minPoint, info.aabb.maxPoint);
        info.lightCount = node.lightCount;
        info.power = node.power;
        mClusterStats.lightClusterInfos.push_back(info);
    }

    mClusterStats.byteSize = (uint32_t)(mNodes.size() * sizeof(mNodes[0]));
}

void LightCluster::renderStats(Gui::Widgets& widget, const ClusterStats& stats) const
{
    const std::string statsStr = "    Cluster count:         " + std::to_string(stats.clusterCount) + "\n" +
                                 "    Size:                " + std::to_string(stats.byteSize) + " bytes\n";
    widget.text(statsStr);

    if (auto clusterInfoGroup = widget.group("  Cluster info"))
    {
        uint32_t clusterID = 0;
        for (const auto& clusterInfo : stats.lightClusterInfos)
        {
            if (auto clusterGroup = clusterInfoGroup.group(std::to_string(clusterID)))
            {
                std::ostringstream oss;
                oss << "    Bounds: (" << clusterInfo.aabb.minPoint.x << "," << clusterInfo.aabb.minPoint.y << ","
                    << clusterInfo.aabb.minPoint.z << ")-(" << clusterInfo.aabb.maxPoint.x << "," << clusterInfo.aabb.maxPoint.y << ","
                    << clusterInfo.aabb.maxPoint.z << ")" << std::endl;
                oss << "    Power: " << clusterInfo.power << std::endl;
                oss << "    Light count: " << clusterInfo.lightCount << std::endl;

                clusterGroup.text(oss.str());
            }
            clusterID++;
        }
    }
}

void LightCluster::clear()
{
    // Reset all CPU data.
    mNodes.clear();
    mClusterStats = ClusterStats();
    mIsValid = false;
}

void LightCluster::uploadCPUBuffers(const std::vector<uint32_t>& lightIndices, const std::vector<uint32_t>& clusterIndices)
{
    if (!mpClusterNodesBuffer || mpClusterNodesBuffer->getElementCount() < mNodes.size())
    {
        mpClusterNodesBuffer = mpDevice->createStructuredBuffer(
            sizeof(ClusterNode), (uint32_t)mNodes.size(), ResourceBindFlags::ShaderResource, MemoryType::DeviceLocal, nullptr, false
        );
        mpClusterNodesBuffer->setName("LightBVH::mpClusterNodesBuffer");
    }

    if (!mpLightIndicesBuffer || mpLightIndicesBuffer->getElementCount() < lightIndices.size())
    {
        mpLightIndicesBuffer = mpDevice->createStructuredBuffer(
            sizeof(uint32_t), (uint32_t)lightIndices.size(), ResourceBindFlags::ShaderResource, MemoryType::DeviceLocal, nullptr, false
        );
        mpLightIndicesBuffer->setName("LightBVH::mpLightIndicesBuffer");
    }

    if (!mpClusterIndicesBuffer || mpClusterIndicesBuffer->getElementCount() < clusterIndices.size())
    {
        mpClusterIndicesBuffer = mpDevice->createStructuredBuffer(
            sizeof(uint32_t), (uint32_t)clusterIndices.size(), ResourceBindFlags::ShaderResource, MemoryType::DeviceLocal, nullptr, false
        );
        mpLightIndicesBuffer->setName("LightBVH::mpClusterIndicesBuffer");
    }

    // Update our GPU side buffers.
    FALCOR_ASSERT(mpClusterNodesBuffer->getElementCount() >= mNodes.size());
    FALCOR_ASSERT(mpClusterNodesBuffer->getStructSize() == sizeof(mNodes[0]));
    mpClusterNodesBuffer->setBlob(mNodes.data(), 0, mNodes.size() * sizeof(mNodes[0]));

    FALCOR_ASSERT(mpLightIndicesBuffer->getSize() >= lightIndices.size() * sizeof(lightIndices[0]));
    mpLightIndicesBuffer->setBlob(lightIndices.data(), 0, lightIndices.size() * sizeof(lightIndices[0]));

    FALCOR_ASSERT(mpClusterIndicesBuffer->getSize() >= clusterIndices.size() * sizeof(clusterIndices[0]));
    mpClusterIndicesBuffer->setBlob(clusterIndices.data(), 0, clusterIndices.size() * sizeof(clusterIndices[0]));
}

void LightCluster::bindShaderData(const ShaderVar& var) const
{
    if (isValid())
    {
        FALCOR_ASSERT(var.isValid());
        var["nodes"] = mpClusterNodesBuffer;
        var["lightIndices"] = mpLightIndicesBuffer;
        var["clusterIndices"] = mpClusterIndicesBuffer;
    }
}

} // namespace Falcor
