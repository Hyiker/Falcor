add_plugin(LightSample)

target_sources(LightSample PRIVATE
    LightSample.cpp
    LightSample.h
    LightSample.cs.slang
)

target_copy_shaders(LightSample RenderPasses/LightSample)

target_source_group(LightSample "RenderPasses")
