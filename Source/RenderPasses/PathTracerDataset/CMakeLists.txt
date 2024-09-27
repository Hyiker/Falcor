add_plugin(PathTracerDataset)

target_sources(PathTracerDataset PRIVATE
    ColorType.slang
    GeneratePaths.cs.slang
    GuideData.slang
    LoadShadingData.slang
    NRDHelpers.slang
    Params.slang
    PathState.slang
    PathTracer.slang
    PathTracer.cpp
    PathTracer.h
    PathTracerNRD.slang
    ReflectTypes.cs.slang
    ResolvePass.cs.slang
    StaticParams.slang
    TracePass.rt.slang
)

target_copy_shaders(PathTracerDataset RenderPasses/PathTracerDataset)

target_source_group(PathTracerDataset "RenderPasses")
