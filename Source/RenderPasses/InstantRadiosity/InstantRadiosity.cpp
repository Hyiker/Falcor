#include "VPLGenPass.h"
#include "VPLVisibilityPass.h"
#include "IRShadingPass.h"
#include "VPLData.slang"

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, VPLGenPass>();
    ScriptBindings::registerBinding(VPLGenPass::registerBindings);
    registry.registerClass<RenderPass, VPLVisibilityPass>();
    ScriptBindings::registerBinding(VPLVisibilityPass::registerBindings);
    registry.registerClass<RenderPass, IRShadingPass>();
}
