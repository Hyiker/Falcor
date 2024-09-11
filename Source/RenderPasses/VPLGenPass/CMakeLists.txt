add_plugin(VPLGenPass)

target_sources(VPLGenPass PRIVATE
    CreateVPL.cs.slang
    VPLData.slang
    VPLGenPass.cpp
    VPLGenPass.h
)

target_copy_shaders(VPLGenPass RenderPasses/VPLGenPass)

target_source_group(VPLGenPass "RenderPasses")
