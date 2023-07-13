import math
import numpy
import time

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateTemperatureGetter(name, customDynamicsAPI):
    #sT = time.perf_counter()
    w = customDynamicsAPI["leetHAXXOR"]()
    csv = w._kinematicTrees[name].get("customStateVariables", {})
    thermalTimeConstant = csv.get("thermal", {}).get("timeConstant", 1.0)
    if "temperature" not in csv:
        csv["temperature"] = {}
    ownTemperatures = csv["temperature"]
    backgroundTemperature, defaultHeatExchangeCoefficient = w._backgroundTemperature, w._heatExchangeCoefficient
    targetTemperature = backgroundTemperature
    #contacts = w.checkCollision((name,))
    #temps = [(w._kinematicTrees[x[1][0]].get("customStateVariables",{}).get("temperature",{}).get(x[1][1]) or backgroundTemperature) for x in contacts]
    overlaps = [x for x in w.checkOverlap(w.adjustAABBRadius(w.getAABB((name,)), 0.1)) if x[0] != name]
    temps = [(w._kinematicTrees[x[0]].get("customStateVariables",{}).get("temperature",{}).get(x[1]) or backgroundTemperature) for x in overlaps]
    if 0 < len(temps):
        targetTemperature = sum(temps)/len(temps)
    for l in w._kinematicTrees[name]["links"]:
        if l not in ownTemperatures:
            ownTemperatures[l] = backgroundTemperature
        ownTemperature = ownTemperatures[l]
        dt = 1.0/w._sfr
        nfr = 1.0/w._frameStepCount
        newTemperature = ownTemperature + (targetTemperature - ownTemperature)*(1-math.exp(-nfr*dt/thermalTimeConstant))
        ownTemperatures[l] = newTemperature
    #print("    temp %s %f" % (name, time.perf_counter()-sT))
    return
