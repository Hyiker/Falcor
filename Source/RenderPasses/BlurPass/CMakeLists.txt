add_plugin(BlurPass)

target_sources(BlurPass PRIVATE
    BlurPass.cpp
    BlurPass.h
    BlurPass.ps.slang
)

target_copy_shaders(BlurPass RenderPasses/BlurPass)

target_source_group(BlurPass "RenderPasses")
