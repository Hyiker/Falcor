from falcor import *

def render_graph_VPLGen():
    g = RenderGraph("VPLGenTest")
    VPLGenPass = createPass("VPLGenPass")
    g.addPass(VPLGenPass, "VPLGenPass")
    g.markOutput('VPLGenPass.output')
    return g

vplGen = render_graph_VPLGen()
try: m.addGraph(vplGen)
except NameError: None
