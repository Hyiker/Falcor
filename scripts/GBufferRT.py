from falcor import *


def render_graph_GBufferRT():
    g = RenderGraph("GBufferRT")
    GBufferRT = createPass("GBufferRT", {'samplePattern': 'Halton', 'sampleCount': 16, 'useAlphaTest': True})
    g.addPass(GBufferRT, "GBufferRT")

    g.markOutput("GBufferRT.normWRoughnessMaterialID")
    return g


gbufferRT = render_graph_GBufferRT()
try:
    m.addGraph(gbufferRT)
except NameError:
    None

