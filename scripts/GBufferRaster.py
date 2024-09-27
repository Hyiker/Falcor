from falcor import *

def render_graph_GBufferRaster():
    g = RenderGraph("GBufferRaster")
    g.addPass(createPass("GBufferRaster"), "GBufferRaster")

    g.markOutput("GBufferRaster.normW")

    return g

GBufferRaster = render_graph_GBufferRaster()
try: m.addGraph(GBufferRaster)
except NameError: None
