#include "Scene/SceneDefines.slangh"
#include "Utils/Math/MathConstants.slangh"

import Scene.RaytracingInline;
import Scene.Intersection;
import VPLData;

// Outputs
StructuredBuffer<VPLData> gVPL;
RWTexture2D<float4> gOutput;

// Static configurations
static const uint kPathsPerLight = PATHS_PER_LIGHT;
static const uint kMaxPathLength = MAX_PATH_LENGTH;

[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId: SV_DispatchThreadID)
{
    uint2 instanceID = dispatchThreadId.xy;
    if (all(instanceID == uint2(0, 0)))
        gOutput[uint2(0, 0)] = 1.0;
}
