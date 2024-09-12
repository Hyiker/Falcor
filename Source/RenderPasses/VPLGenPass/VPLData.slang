#pragma once

#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

struct VPLData
{
    float4 position;  ///< World space position.
    float4 normal;    ///< World space normal.
    float4 intensity; ///< Intensity component of VPLData
};

END_NAMESPACE_FALCOR
