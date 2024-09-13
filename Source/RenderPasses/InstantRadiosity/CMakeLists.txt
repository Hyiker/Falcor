add_plugin(InstantRadiosity)

target_sources(InstantRadiosity PRIVATE
    InstantRadiosity.cpp

    CreateVPL.cs.slang
    VPLData.slang
    VPLGenPass.cpp
    VPLGenPass.h

    IRShading.cs.slang
    IRShadingPass.h
    IRShadingPass.cpp
)

target_copy_shaders(InstantRadiosity RenderPasses/InstantRadiosity)

target_source_group(InstantRadiosity "RenderPasses")
