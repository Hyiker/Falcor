#pragma once

#include "Utils/HostDeviceShared.slangh"

BEGIN_NAMESPACE_FALCOR

struct VPLData
{
    float4 position;  ///< World space position.
    float4 normal;    ///< World space normal.
    float3 intensity; ///< Intensity component of VPLData
    float pdf;        ///< Sampling PDF of the VPL.
};

END_NAMESPACE_FALCOR
