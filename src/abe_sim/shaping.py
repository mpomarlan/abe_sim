import copy
import math
import numpy
import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateShaped(name, customDynamicsAPI):
    w = customDynamicsAPI["leetHAXXOR"]()
    aabb = w.getAABB((name,))
    aabbAdj = w.adjustAABBRadius(aabb, 1.0)
    closeObjects = set([x[0] for x in w.checkOverlap(aabbAdj)])
    for e in closeObjects:
        if name in (w._kinematicTrees[e].get("customStateVariables", {}).get("provenance") or []):
            w.removeObject((name,))
            return

def updatePressingIntoShape(name, customDynamicsAPI):
    def _distance(a, b):
        dx, dy, dz = a[0]-b[0], a[1]-b[1], a[2]-b[2]
        return (dx*dx + dy*dy + dz*dz)
    w = customDynamicsAPI["leetHAXXOR"]()
    fnShaping = w._kinematicTrees[name].get("fn", {}).get("shaping") or {}
    for a in fnShaping.get("actuators", []):
        position = w.getKinematicData((name,))[0]
        radius = fnShaping.get("radius", {}).get(a) or 0
        aabb = w.getAABB((name, a))
        aabbAdj = w.adjustAABBRadius(aabb, radius)
        closeObjects = set([x[0] for x in w.checkOverlap(aabbAdj) if w._kinematicTrees[x[0]].get("fn", {}).get("shapeable", False)])
        close2Type = {}
        type2Amount = {}
        type2Adds = {}
        for co in closeObjects:
            ct = w._kinematicTrees[co]["type"]
            toAdds = [x[0] for x in w.getProcessOutcome({"process": "shaping", "patient": ct, "instrument": w._kinematicTrees[name]["type"]}).get("toAdd", [])]
            if len(toAdds):
                type2Adds[ct] = toAdds
                type2Amount[ct] = len(w.getProcessResource({"process": "shaping", "patient": ct}))
                if ct not in close2Type:
                    close2Type[ct] = []
                close2Type[ct].append(co)
                close2Type[ct] = sorted(close2Type[ct], key=lambda x: _distance(w.getKinematicData((x,))[0], position))[:type2Amount[ct]]
        for ct, cobs in close2Type.items():
            if type2Amount[ct] > len(cobs):
                continue
            positions = [w.getKinematicData((x,))[0] for x in cobs]
            px, py, pz = sum([x[0] for x in positions])/len(positions), sum([x[1] for x in positions])/len(positions), sum([x[2] for x in positions])/len(positions)
            for shapedType in type2Adds[ct]:
                objDesc = w.getObjectTypeKnowledge(shapedType)
                if "customStateVariables" not in objDesc:
                    objDesc["customStateVariables"] = {}
                objDesc["customStateVariables"]["provenance"] = list(cobs)
                objDesc["orientation"] = [0,0,0,1]
                objDesc["position"] = [px, py, pz]
                objDesc["name"] = "%s_%d" % (shapedType, w.getNewObjectCounter())
                w.addObject(objDesc)
    
def updateShaping(name, customDynamicsAPI):
    w = customDynamicsAPI["leetHAXXOR"]()
    fnShaping = w._kinematicTrees[name].get("fn", {}).get("shaping") or {}
    if "customStateVariables" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["customStateVariables"] = {}
    if "shaping" not in w._kinematicTrees[name]["customStateVariables"]:
        w._kinematicTrees[name]["customStateVariables"]["shaping"] = {}
    csvShaping = w._kinematicTrees[name]["customStateVariables"]["shaping"]
    if "ingredients" not in csvShaping:
        csvShaping["ingredients"] = {}
    if "outcome" not in csvShaping:
        csvShaping["outcome"] = {}
    for a in (fnShaping.get("actuators") or []):
        shapedType = csvShaping["outcome"].get(a)
        ingredients = set((csvShaping["ingredients"].get(a) or []))
        if (shapedType is None) or (0 == len(ingredients)):
            continue
        radius = fnShaping.get("radius", {}).get(a) or 0
        handLink = w._kinematicTrees[name].get("fn", {}).get("kinematicControl", {}).get("efLink", {}).get(a)
        aabb = w.getAABB((name, handLink))
        aabbAdj = w.adjustAABBRadius(aabb, radius)
        closeObjects = set([x[0] for x in w.checkOverlap(aabbAdj)])
        if 0 == len(ingredients.difference(closeObjects)):
            handP, handQ, _, _ = w.getKinematicData((name, handLink))
            axis = stubbornTry(lambda : pybullet.rotateVector(handQ, [0,0,-0.07]))
            objDesc = w.getObjectTypeKnowledge(shapedType)
            if "customStateVariables" not in objDesc:
                objDesc["customStateVariables"] = {}
            objDesc["customStateVariables"]["provenance"] = list(ingredients)
            objDesc["orientation"] = [0,0,0,1]
            objDesc["position"] = [handP[0] + axis[0], handP[1] + axis[1], handP[2] + axis[2]]
            objDesc["name"] = "%s_%d" % (shapedType, w.getNewObjectCounter())
            w.addObject(objDesc)
            csvShaping["ingredients"][a] = []
            csvShaping["outcome"][a] = None

