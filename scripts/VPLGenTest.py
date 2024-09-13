from falcor import *

def render_graph_VPLGen():
    g = RenderGraph("VPLGenTest")
    VPLGenPass = createPass("VPLGenPass")
    GBufferRT = createPass("GBufferRT")
    IRShadingPass = createPass("IRShadingPass")
    ToneMapper = createPass("ToneMapper", {'autoExposure': False, 'exposureCompensation': 0.0})

    g.addPass(VPLGenPass, "VPLGenPass")
    g.addPass(GBufferRT, "GBufferRT")
    g.addPass(IRShadingPass, "IRShadingPass")
    g.addPass(ToneMapper, "ToneMapper")

    g.addEdge("VPLGenPass.vpl", "IRShadingPass.vpl")

    g.addEdge("GBufferRT.viewW", "IRShadingPass.viewW")
    g.addEdge("GBufferRT.posW", "IRShadingPass.posW")
    g.addEdge("GBufferRT.guideNormalW", "IRShadingPass.normW")
    g.addEdge("GBufferRT.emissive", "IRShadingPass.emissive")
    g.addEdge("GBufferRT.diffuseOpacity", "IRShadingPass.diffuseOpacity")
    g.addEdge("GBufferRT.specRough", "IRShadingPass.specRough")

    g.addEdge("IRShadingPass.color", "ToneMapper.src")
    g.markOutput('ToneMapper.dst')
    return g

vplGen = render_graph_VPLGen()
try: m.addGraph(vplGen)
except NameError: None
