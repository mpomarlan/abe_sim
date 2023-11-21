import math
import time

import pybullet

from abe_sim.world import getDictionaryEntry

def updateClopening(name, customDynamicsAPI):
    #startD = time.perf_counter()
    w = customDynamicsAPI["leetHAXXOR"]()
    fnClopening = w._kinematicTrees[name].get("fn", {}).get("clopening") or {}
    if "customStateVariables" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["customStateVariables"] = {}
    if "clopening" not in w._kinematicTrees[name]["customStateVariables"]:
        w._kinematicTrees[name]["customStateVariables"]["clopening"] = {}
    csvClopening = w._kinematicTrees[name]["customStateVariables"]["clopening"]
    clopenableLinks = fnClopening.get("clopenableLinks") or []
    for link in clopenableLinks:
        clopeningRadius = fnClopening.get("radius", {}).get(link) or 0.2
        handleLink = fnClopening.get("handle", {}).get(link)
        joint = w._kinematicTrees[name]["idx2Joint"][w._kinematicTrees[name]["links"][link]["idx"]] # parent joint of link
        if joint is None:
            continue
        maxVelocity = fnClopening.get("maxVelocity", {}).get(link)
        positionGain = fnClopening.get("positionGain", {}).get(link)
        velocityGain = fnClopening.get("velocityGain", {}).get(link)
        aabb = w.getAABB((name, handleLink))
        aabbAdj = w.adjustAABBRadius(aabb, clopeningRadius)
        closeObjects = w.checkOverlap(aabbAdj)
        action = csvClopening.get("action", {}).get(link)
        for candidate in closeObjects:
            obj, lnk = candidate
            if "fn" not in w._kinematicTrees[obj]:
                w._kinematicTrees[obj]["fn"] = {}
            objFn = w._kinematicTrees[obj]["fn"]
            ef = objFn.get("grasping", {}).get("link2EF", {}).get(lnk)
            if objFn.get("canClopen") and (ef in (objFn.get("clopening", {}).get("clopeningEFs") or [])):
                actionCandidate = w._kinematicTrees[obj].get("customStateVariables", {}).get("clopening", {}).get("action", {}).get(ef)
                if (action is None) or (actionCandidate is not None):
                    action = actionCandidate
        if "action" not in csvClopening:
            csvClopening["action"] = {}
        csvClopening["action"][link] = action
        angle = None
        if "open" == action:
            angle = fnClopening.get("openingAngle", {}).get(link) or 0
        elif "close" == action:
            angle = fnClopening.get("closingAngle", {}).get(link) or 0
        if angle is not None:
            w.applyJointControl((name,joint), mode="position", targetPosition=angle, maxVelocity=maxVelocity, positionGain=positionGain, velocityGain=velocityGain)
    #endD = time.perf_counter()
    #print("    clopen %s %f" % (name, endD-startD))
    return
