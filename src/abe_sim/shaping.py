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

