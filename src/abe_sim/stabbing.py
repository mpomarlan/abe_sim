import math
import time
import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateStabbing(name, customDynamicsAPI):
    def _toIdentifier(x):
        if isinstance(x, str):
            return (x,)
        return tuple(x)
    w = customDynamicsAPI["leetHAXXOR"]()
    if "customStateVariables" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["customStateVariables"] = {}
    if "fn" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["fn"] = {}
    if "stabbing" not in w._kinematicTrees[name]["customStateVariables"]:
        w._kinematicTrees[name]["customStateVariables"]["stabbing"] = {}
    if "stabbing" not in w._kinematicTrees[name]["fn"]:
        w._kinematicTrees[name]["fn"]["stabbing"] = {}
    fnStabbing = w._kinematicTrees[name]["fn"]["stabbing"]
    csvStabbing = w._kinematicTrees[name]["customStateVariables"]["stabbing"]
    graspedBy = w._kinematicTrees[name]["customStateVariables"].get("graspedBy")
    stabbedBy = w._kinematicTrees[name]["customStateVariables"].get("stabbedBy",[])
    maxForce = fnStabbing.get("maxForce", 1000)
    # If anyone grasps this, then destroy all stabbing constraints
    if graspedBy is not None:
        for e in stabbedBy:
            w.removeObject((e,))
        w._kinematicTrees[name]["customStateVariables"]["stabbedBy"] = []
        return
    # Otherwise, check for contacting objects
    collisions = [x for x in w.checkCollision((name,)) if w._kinematicTrees[x[1][0]].get("fn",{}).get("canStab",False)]
    # Also, check what stabs are already known
    stabbers = set([w._kinematicConstraints[x]["parent"] for x in stabbedBy])
    # For every not yet recorded stabber, create a constraint
    for e in collisions:
        identifierA, identifierB, ponA, ponB, normal, distance, force, lateralFriction1, lateralFriction1Dir, lateralFriction2, lateralFrictionDir2 = e
        stabber = identifierB[0]
        stabberLink = identifierB[1]
        childLink = identifierA[1]
        if stabber in stabbers:
            continue
        posStabbee,_,_,_ = w.getKinematicData(identifierA)
        posStabber,ornStabber,_,_ = w.getKinematicData(identifierB)
        diff=[x-y for x,y in zip(posStabbee,posStabber)]
        normsq=math.sqrt(sum([x*x for x in diff]))
        if(normsq<0.01):
            continue
        diff=[x/norm for x in diff]
        axis=w._kinematicTrees[stabber]["fn"]["stabbing"]["axis"]
        axis=stubbornTry(lambda v,q: pybullet.rotateVector(ornStabber,axis))
        dot=sum([x*y for x,y in zip(diff,axis)])
        if 0.8>dot:
            continue
        constraintName = ("STABBING_%s_%s_%s_%s" % (stabber, stabberLink, name, childLink))
        w.addObject({"fn": {"stabbingConstraint": True}, "name": constraintName, "simtype": "kcon", "parent": stabber, "child": name, "parentLink": stabberLink, "childLink": childLink, "jointType": "fixed", "maxForce": maxForce})
        w._kinematicTrees[name]["customStateVariables"].append(constraintName)
                