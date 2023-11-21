import math

from abe_sim.world import getDictionaryEntry

def updateTiming(name, customDynamicsAPI):
    w = customDynamicsAPI["leetHAXXOR"]()
    if "customStateVariables" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["customStateVariables"] = {}
    if "timing" not in w._kinematicTrees[name]["customStateVariables"]:
        w._kinematicTrees[name]["customStateVariables"]["timing"] = {}
    if "timer" not in w._kinematicTrees[name]["customStateVariables"]["timing"]:
        w._kinematicTrees[name]["customStateVariables"]["timing"]["timer"] = 0.0
    w._kinematicTrees[name]["customStateVariables"]["timing"]["timer"] += w._frameStepCount/(1.0*w._sfr)

