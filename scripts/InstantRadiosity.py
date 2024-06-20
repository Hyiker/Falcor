from falcor import *

def render_graph_InstantRadiosity():
    g = RenderGraph('InstantRadiosity')
    VBufferRT = createPass("VBufferRT", {'samplePattern': 'Stratified', 'sampleCount': 16})
    InstantRadiosity = createPass('InstantRadiosity')
    AccumulatePass = createPass("AccumulatePass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'operator': 'Linear', 'clamp': False, 'outputFormat': 'RGBA32Float'})
    g.addPass(VBufferRT, 'VBufferRT')
    g.addPass(InstantRadiosity, 'InstantRadiosity')
    g.addPass(AccumulatePass, 'AccumulatePass')
    g.addPass(ToneMapper, 'ToneMapper')
    g.addEdge("VBufferRT.viewW", "InstantRadiosity.viewW")
    g.addEdge('VBufferRT.vbuffer', 'InstantRadiosity.vbuffer')
    g.addEdge("InstantRadiosity.output", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")
    g.markOutput("ToneMapper.dst")
    return g

InstantRadiosity = render_graph_InstantRadiosity()
try: m.addGraph(InstantRadiosity)
except NameError: None
