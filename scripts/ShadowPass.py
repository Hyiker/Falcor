from falcor import *

def render_graph_ShadowPass():
    dict = {'lightCount':2}
    g = RenderGraph('ShadowPass')
    SMPass = createPass("SMPass",dict)
    g.addPass(SMPass,"SMPass")
    VisPass = createPass("VisPass",dict)
    g.addPass(VisPass,"VisPass")
    LightSample= createPass('LightSample',dict)
    g.addPass(LightSample, 'LightSample')
    Acc = createPass("AccumulatePass")
    g.addPass(Acc, 'AccumulatePass')
    g.addEdge("SMPass.cubeMap","VisPass.cubeMap")
    g.addEdge("LightSample.position","SMPass.position")
    g.addEdge("LightSample.normal","SMPass.normal")
    g.addEdge("VisPass.gToLightVec0","AccumulatePass.input")
    g.markOutput("AccumulatePass.output")
    return g

shadowPass = render_graph_ShadowPass()
try: m.addGraph(shadowPass)
except NameError: None
