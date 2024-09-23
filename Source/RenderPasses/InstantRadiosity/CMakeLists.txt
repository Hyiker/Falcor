add_plugin(InstantRadiosity)

target_sources(InstantRadiosity PRIVATE
    InstantRadiosity.cpp
    InstantRadiosity.h

    CreateVPL.cs.slang
    VPLData.slang
    VPLGenPass.cpp
    VPLGenPass.h

    IRShading.cs.slang
    IRShadingPass.h
    IRShadingPass.cpp

    VPLVisibility.cs.slang
    VPLVisibilityPass.cpp
    VPLVisibilityPass.h
)

target_copy_shaders(InstantRadiosity RenderPasses/InstantRadiosity)

target_source_group(InstantRadiosity "RenderPasses")
