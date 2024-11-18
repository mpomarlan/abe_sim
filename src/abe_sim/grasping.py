import math
import time

from abe_sim.world import getDictionaryEntry

graspParams = {"maxForce": 1000}

def updateGraspingConstraint(name, customDynamicsAPI):
    w = customDynamicsAPI["leetHAXXOR"]()
    parent = w._kinematicConstraints[name]["parent"]
    parentLink = w._kinematicConstraints[name]["parentLink"]
    child = w._kinematicConstraints[name]["child"]
    childLink = w._kinematicConstraints[name]["childLink"]
    parentEF = w._kinematicTrees[parent].get("fn", {}).get("grasping", {}).get("link2EF", {}).get(parentLink)
    parentActuallyGrasps = w._kinematicTrees[parent].get("customStateVariables", {}).get("grasping", {}).get("actuallyGrasping", {}).get(parentEF) or []
    if [[child], name] not in parentActuallyGrasps:
        w.removeObject((name,))
        if "customStateVariables" in w._kinematicTrees[child]:
            w._kinematicTrees[child]["customStateVariables"]["graspedBy"] = None

def updateGrasping(name, customDynamicsAPI):
    def _toIdentifier(x):
        if isinstance(x, str):
            return (x,)
        return tuple(x)
    w = customDynamicsAPI["leetHAXXOR"]()
    if "customStateVariables" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["customStateVariables"] = {}
    if "grasping" not in w._kinematicTrees[name]["customStateVariables"]:
        w._kinematicTrees[name]["customStateVariables"]["grasping"] = {}
    csvGrasping = w._kinematicTrees[name]["customStateVariables"]["grasping"]
    fn = w._kinematicTrees[name].get("fn") or {}
    fnGrasping = fn.get("grasping") or {}
    fnKinematicControl = fn.get("kinematicControl") or {}
    intendedGrasped = csvGrasping["intendToGrasp"]
    actuallyGrasped = csvGrasping["actuallyGrasping"]
    efs = fnGrasping.get("effectors") or []
    newActuallyGrasped = {}
    for ef in efs:
        intendedSet = set([_toIdentifier(x) for x in intendedGrasped.get(ef, [])])
        actuallyGraspedEF = {_toIdentifier(x[0]): x[1] for x in actuallyGrasped.get(ef, [])}
        if (0 == len(intendedSet)) and (0 == len(actuallyGraspedEF)):
            continue
        efLink = fnKinematicControl.get("efLink", {}).get(ef)
        graspingActivationRadius = fnGrasping.get("graspingActivationRadius", {}).get(ef) or 0.1
        graspingDeactivationRadius = fnGrasping.get("graspingDeactivationRadius", {}).get(ef) or 0.2
        aabb = w.getAABB((name, efLink))
        aabbActivation = w.adjustAABBRadius(aabb, graspingActivationRadius)
        aabbDeactivation = w.adjustAABBRadius(aabb, graspingDeactivationRadius)
        overlapsActivation = set([x[0] for x in w.checkOverlap(aabbActivation) if x[0]!=name])
        overlapsDeactivation = set([x[0] for x in w.checkOverlap(aabbDeactivation) if x[0]!=name])
        maxForce = graspParams["maxForce"] or 1000#fnGrasping.get("maxForce", {}).get(ef) or 1000
        dists = {}
        toRemove = []
        for e in actuallyGraspedEF.keys():
        # check whether actually grasped items are still intended; if not, remove from grasping and destroy the constraint
            if (e not in intendedSet) or (not w._kinematicTrees[e[0]].get("fn", {}).get("graspable")):
                toRemove.append(e)
        for e in toRemove:
            actuallyGraspedEF.pop(e)
        toRemove = []
        for e in actuallyGraspedEF.keys():
        # check whether actually grasped items have drifted out of grasping range; if so, remove from grasping and destroy the constraint
            if e[0] not in overlapsDeactivation:
                toRemove.append(e)
        for e in toRemove:
            actuallyGraspedEF.pop(e)
        for e in intendedSet:
            if not w._kinematicTrees.get(e[0],{}).get("fn", {}).get("graspable"):
                continue
        # check whether intended grasped items are not actually grasped, but close enough; if so, add a constraint to make them grasped
            if ((e not in actuallyGraspedEF) or (w._kinematicConstraints.get(actuallyGraspedEF[e], {}).get("name") is None)) and (e[0] in overlapsActivation):
                child = e[0]
                if 1 == len(e):
                    childLink = w._kinematicTrees[e[0]]["idx2Link"][-1] # baseLinkName
                else:
                    childLink = e[1]
                constraintName = ("GRASPING_%s_%s_%s_%s" % (name, efLink, child, childLink))
                w.addObject({"fn": {"graspingConstraint": True}, "name": constraintName, "simtype": "kcon", "parent": name, "child": child, "parentLink": efLink, "childLink": childLink, "jointType": "fixed", "maxForce": maxForce})
                actuallyGraspedEF[e] = constraintName
                if "customStateVariables" not in w._kinematicTrees[child]:
                    w._kinematicTrees[child]["customStateVariables"] = {}
                w._kinematicTrees[child]["customStateVariables"]["graspedBy"] = constraintName
        newActuallyGrasped[ef] = sorted([[list(k), v] for k, v in actuallyGraspedEF.items()])
    csvGrasping["actuallyGrasping"] = newActuallyGrasped

