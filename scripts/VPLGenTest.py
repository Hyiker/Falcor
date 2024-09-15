from falcor import *

def render_graph_VPLGen():
    g = RenderGraph("VPLGenTest")
    VPLGenPass = createPass("VPLGenPass")
    VBufferRT = createPass("VBufferRT", {'samplePattern': 'Stratified', 'sampleCount': 16, 'useAlphaTest': True})
    IRShadingPass = createPass("IRShadingPass")
    AccumulatePass = createPass("AccumulatePass", {'enabled': True, 'precisionMode': 'Single'})
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})

    g.addPass(VPLGenPass, "VPLGenPass")
    g.addPass(VBufferRT, "VBufferRT")
    g.addPass(IRShadingPass, "IRShadingPass")
    g.addPass(AccumulatePass, "AccumulatePass")
    g.addPass(ToneMapper, "ToneMapper")

    g.addEdge("VPLGenPass.vpl", "IRShadingPass.vpl")
    g.addEdge("VPLGenPass.vplCounter", "IRShadingPass.vplCounter")

    g.addEdge("VBufferRT.vbuffer", "IRShadingPass.vbuffer")
    g.addEdge("VBufferRT.viewW", "IRShadingPass.viewW")

    g.addEdge("IRShadingPass.color", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")
    g.markOutput('ToneMapper.dst')
    return g

vplGen = render_graph_VPLGen()
try: m.addGraph(vplGen)
except NameError: None
