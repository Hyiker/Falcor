add_plugin(InstantRadiosity)

target_sources(InstantRadiosity PRIVATE
    InstantRadiosity.cpp
    InstantRadiosity.h
    PhotonGenPass.cs.slang
    ShadingPass.cs.slang
    SharedTypes.slang
    VPLData.slang
)

target_copy_shaders(InstantRadiosity RenderPasses/InstantRadiosity)

target_source_group(InstantRadiosity "RenderPasses")
