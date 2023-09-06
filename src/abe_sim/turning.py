import math
import time

import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateTurning(name, customDynamicsAPI):
    #startD = time.perf_counter()
    w = customDynamicsAPI["leetHAXXOR"]()
    fnTurning = w._kinematicTrees[name].get("fn", {}).get("turning") or {}
    if "customStateVariables" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["customStateVariables"] = {}
    if "turning" not in w._kinematicTrees[name]["customStateVariables"]:
        w._kinematicTrees[name]["customStateVariables"]["turning"] = {}
    turnableLinks = fnTurning.get("links") or []
    for link in turnableLinks:
        p, q, _, _ = w.getKinematicData((name, link))
        turningRadius = fnTurning.get("radius", {}).get(link) or 0.2
        turningAxisInLink = fnTurning.get("axis", {}).get(link) or [0,0,1]
        turningAxis = stubbornTry(lambda : pybullet.rotateVector(q, turningAxisInLink))
        joint = w._kinematicTrees[name]["idx2Joint"][w._kinematicTrees[name]["links"][link]["idx"]] # parent joint of link
        if joint is None:
            continue
        aabb = w.adjustAABBRadius(w.getAABB((name, link)), turningRadius)
        overlaps = w.checkOverlap(aabb)
        for o, l in overlaps:
            fnO = w._kinematicTrees[o].get("fn", {})
            if not fnO.get("canTurn", False):
                continue
            if (l in fnO.get("turning", {}).get("links")):
                if ((name, link) in w._kinematicTrees[o].get("customStateVariables", {}).get("turning",{}).get(l,[])) or ([name, link] in w._kinematicTrees[o].get("customStateVariables", {}).get("turning",{}).get(l,[])):
                    _, _, _, omega = w.getKinematicData((o, l))
                    turnOmega = [-turningAxis[0]*omega[0], -turningAxis[1]*omega[1], -turningAxis[2]*omega[2]]
                    qI = list(q)
                    qI[3] = -qI[3]
                    turnOmega = stubbornTry(lambda : pybullet.rotateVector(qI, turnOmega))
                    turnOmega = turnOmega[0]*turningAxisInLink[0] + turnOmega[1]*turningAxisInLink[1] + turnOmega[2]*turningAxisInLink[2]
                    w.applyJointControl((name,joint), mode="velocity", targetVelocity=turnOmega)
    #endD = time.perf_counter()
    #print("    turn %s %f" % (name, endD-startD))
    return

