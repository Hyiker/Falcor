#include "LightCluster.h"
#include <limits>

#include "Core/API/Buffer.h"
#include "Core/API/Formats.h"
#include "Core/API/RenderContext.h"
#include "Core/Error.h"
#include "Core/Program/ShaderVar.h"
#include "Rendering/Lights/LightCluster.h"
#include "Rendering/Lights/LightClusterTypes.slang"
#include "Scene/Lights/Light.h"
#include "Scene/Lights/LightData.slang"
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
    uint32_t analyticLightCount = mpScene->getLightCount();
    const int kDesiredClusterSize = std::max(1, int(analyticLightCount) / kDesiredClusterCount);
    mNodes.clear();
    const float sceneRadius = mpScene->getSceneBounds().radius();

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

    // Greedy clustering
    minimalDistanceCluster(lights, 0, sceneRadius / 10.f);

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

void LightCluster::minimalDistanceCluster(fstd::span<IndexedLight> lights, uint32_t lightOffset, float distanceTolerance)
{
    FALCOR_ASSERT(!lights.empty());

    if (lights.size() == 1)
    {
        return;
    }
    const auto& centerLight = lights[0].first->getData();
    // Find the point have maximum distance from center.
    auto distanceToCenter = [&centerLight](const LightData& l1) { return length(l1.posW - centerLight.posW); };
    std::vector<IndexedLight> inside, outside;
    inside.reserve(lights.size());
    outside.reserve(lights.size());

    inside.push_back(lights[0]);

    // Cluster properties
    AABB bounds;
    uint32_t lightCount = 1;
    float power = length(centerLight.intensity);
    bounds |= centerLight.posW;

    // New cluster center selection
    float maxDist = -1.0;
    int maxDistIdx = -1;

    for (size_t i = 1; i < lights.size(); i++)
    {
        auto l = lights[i];
        const auto& ld = l.first->getData();
        float dist = distanceToCenter(ld);
        if (dist > distanceTolerance)
        {
            if (dist > maxDist)
            {
                maxDist = dist;
                maxDistIdx = (int)outside.size();
            }
            outside.push_back(l);
        }
        else
        {
            inside.push_back(l);
            bounds |= ld.posW;
            power += length(ld.intensity);
            lightCount++;
        }
    }

    inside.shrink_to_fit();
    outside.shrink_to_fit();

    // Swap the new center to idx 0 in outside
    if (outside.size() >= 2)
    {
        std::swap(outside[maxDistIdx], outside[0]);
    }

    std::copy(inside.begin(), inside.end(), lights.begin());
    std::copy(outside.begin(), outside.end(), lights.begin() + inside.size());

    inside.clear();
    outside.clear();

    // Create new light cluster
    ClusterNode node;
    node.origin = bounds.center();
    node.extent = bounds.extent() / 2.0f;
    node.power = power;
    node.lightCount = lightCount;
    node.lightOffset = lightOffset;

    mNodes.push_back(node);
    if (maxDist >= 0)
    {
        minimalDistanceCluster(lights.subspan(lightCount, lights.size() - lightCount), lightOffset + lightCount, distanceTolerance);
    }
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
