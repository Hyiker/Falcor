#pragma once
#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

struct RadiosityParams
{
    uint maxPathLength = 8;
    uint maxVPLCount = 500;
    uint numVPLSamples = 10; ///< Number of samples per emissive primitive.
};

END_NAMESPACE_FALCOR
