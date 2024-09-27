from falcor import *

def render_graph_PathTracerDataset():
    g = RenderGraph("PathTracerDataset")
    PathTracerDataset = createPass("PathTracerDataset", {'samplesPerPixel': 1})
    g.addPass(PathTracerDataset, "PathTracerDataset")
    VBufferRT = createPass("VBufferRT", {'samplePattern': 'Stratified', 'sampleCount': 16, 'useAlphaTest': True})
    g.addPass(VBufferRT, "VBufferRT")
    AccumulatePass = createPass("AccumulatePass", {'enabled': True, 'precisionMode': 'Single'})
    g.addPass(AccumulatePass, "AccumulatePass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})
    g.addPass(ToneMapper, "ToneMapper")
    g.addEdge("VBufferRT.vbuffer", "PathTracerDataset.vbuffer")
    g.addEdge("VBufferRT.viewW", "PathTracerDataset.viewW")
    g.addEdge("VBufferRT.mvec", "PathTracerDataset.mvec")
    g.addEdge("PathTracerDataset.directShadowless", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")
    g.markOutput("ToneMapper.dst")
    # g.markOutput("PathTracerDataset.directColor")
    # g.markOutput("PathTracerDataset.directShadowless")
    return g

PathTracerDataset = render_graph_PathTracerDataset()
m.clock.pause()
try: m.addGraph(PathTracerDataset)
except NameError: None
