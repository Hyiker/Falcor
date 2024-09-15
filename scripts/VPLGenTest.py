from falcor import *

def render_graph_VPLGen():
    g = RenderGraph("VPLGenTest")
    VPLGenPass = createPass("VPLGenPass")
    GBufferRT = createPass("GBufferRT", {'samplePattern': 'Stratified', 'sampleCount': 16, 'useAlphaTest': True})
    IRShadingPass = createPass("IRShadingPass")
    AccumulatePass = createPass("AccumulatePass", {'enabled': True, 'precisionMode': 'Single'})
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})

    g.addPass(VPLGenPass, "VPLGenPass")
    g.addPass(GBufferRT, "GBufferRT")
    g.addPass(IRShadingPass, "IRShadingPass")
    g.addPass(AccumulatePass, "AccumulatePass")
    g.addPass(ToneMapper, "ToneMapper")

    g.addEdge("VPLGenPass.vpl", "IRShadingPass.vpl")
    g.addEdge("VPLGenPass.vplCounter", "IRShadingPass.vplCounter")

    g.addEdge("GBufferRT.viewW", "IRShadingPass.viewW")
    g.addEdge("GBufferRT.posW", "IRShadingPass.posW")
    g.addEdge("GBufferRT.guideNormalW", "IRShadingPass.normW")
    g.addEdge("GBufferRT.emissive", "IRShadingPass.emissive")
    g.addEdge("GBufferRT.diffuseOpacity", "IRShadingPass.diffuseOpacity")
    g.addEdge("GBufferRT.specRough", "IRShadingPass.specRough")

    g.addEdge("IRShadingPass.color", "AccumulatePass.input")
    g.addEdge("AccumulatePass.output", "ToneMapper.src")
    g.markOutput('ToneMapper.dst')
    return g

vplGen = render_graph_VPLGen()
try: m.addGraph(vplGen)
except NameError: None
