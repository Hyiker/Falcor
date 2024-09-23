#pragma once
#include "Utils/HostDeviceShared.slangh"

#ifdef HOST_CODE
#include "Utils/Math/PackedFormats.h"
#else
import Utils.Math.PackedFormats;
#endif

BEGIN_NAMESPACE_FALCOR

enum class ClusterFlags : uint32_t
{
    None = 0x0,
    Enabled = 0x1,    ///< Cluster is enabled.
    UseReprVis = 0x2, ///< Use representitive light for visibility testing.
};

struct ClusterNode
{
    float3 origin;                            ///< Center of the node bounding box.
    float3 extent;                            ///< Extent of the node bounding box. The min/max corners are at origin +- extent.
    float power = 0.f;                        ///< Total emitted power.
    uint lightCount = 0;                      ///< Number of lights inside the cluster.
    uint lightOffset = 0;                     ///< Offset into the light index list.
    uint flags = (uint)ClusterFlags::Enabled; ///< see ClusterFlags.

#ifdef HOST_CODE
    void getAABB(float3& aabbMin, float3& aabbMax)
#else
    void getAABB(out float3 aabbMin, out float3 aabbMax)
#endif
        CONST_FUNCTION
    {
        aabbMin = origin - extent;
        aabbMax = origin + extent;
    }

    SETTER_DECL void setAABB(float3 aabbMin, float3 aabbMax)
    {
        origin = (aabbMax + aabbMin) * 0.5f;
        extent = (aabbMax - aabbMin) * 0.5f;
    }

#ifdef HOST_CODE
    void getBoundSphere(float3& center, float& radius)
#else
    void getBoundSphere(out float3 center, out float radius)
#endif
        CONST_FUNCTION
    {
        center = origin;
        radius = length(extent);
    }

    bool isEnabled() CONST_FUNCTION { return (flags & (uint)ClusterFlags::Enabled) != 0; }

    bool useReprVis() CONST_FUNCTION { return (flags & (uint)ClusterFlags::UseReprVis) != 0; }
};

END_NAMESPACE_FALCOR
