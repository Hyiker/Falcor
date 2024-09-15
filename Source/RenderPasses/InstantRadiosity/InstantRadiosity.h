#include "Falcor.h"
#include "VPLData.slang"

using namespace Falcor;

static_assert(sizeof(VPLData) % 16 == 0, "VPLData struct should be 16-byte aligned.");

// Shared configurations
const uint32_t kMaxVPLCountLimit = 1000u;
