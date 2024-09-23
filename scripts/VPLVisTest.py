from falcor import *


def render_graph_VPLVisTest():
    g = RenderGraph("VPLVisTest")
    VPLGenPass = createPass("VPLGenPass")
    VPLVisibilityPass = createPass("VPLVisibilityPass")

    g.addPass(VPLGenPass, "VPLGenPass")
    g.addPass(VPLVisibilityPass, "VPLVisibilityPass")

    g.addEdge("VPLGenPass.vpl", "VPLVisibilityPass.vpl")
    g.addEdge("VPLGenPass.vplCounter", "VPLVisibilityPass.vplCounter")

    g.markOutput("VPLVisibilityPass.visibility")
    return g


vplVis = render_graph_VPLVisTest()
try:
    m.addGraph(vplVis)
except NameError:
    None

m.clock.pause()
