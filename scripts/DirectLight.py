from falcor import *

def render_graph_DirectLight():
    g = RenderGraph("DirectLight")
    DirectLight = createPass("PathTracer", {'samplesPerPixel': 1, 'useShadow': False, 'maxSurfaceBounces': 1})
    g.addPass(DirectLight, "PathTracer")
    VBufferRT = createPass("VBufferRT", {'samplePattern': 'Stratified', 'sampleCount': 16, 'useAlphaTest': True})
    g.addPass(VBufferRT, "VBufferRT")
    AccumulatePass = createPass("AccumulatePass", {'enabled': True, 'precisionMode': 'Single'})
    g.addPass(AccumulatePass, "AccumulatePass")
    g.addEdge("VBufferRT.vbuffer", "PathTracer.vbuffer")
    g.addEdge("VBufferRT.viewW", "PathTracer.viewW")
    g.addEdge("VBufferRT.mvec", "PathTracer.mvec")
    g.addEdge("PathTracer.color", "AccumulatePass.input")
    g.markOutput("AccumulatePass.output")
    return g

DirectLight = render_graph_DirectLight()
try: m.addGraph(DirectLight)
except NameError: None
