#include "VPLGenPass.h"
#include "IRShadingPass.h"

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, VPLGenPass>();
    registry.registerClass<RenderPass, IRShadingPass>();
}
