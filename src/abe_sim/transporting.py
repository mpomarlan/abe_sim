import math
import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateTransportingConstraint(name, customDynamicsAPI):
    w = customDynamicsAPI["leetHAXXOR"]()
    child = w._kinematicConstraints[name].get("child") or ""
    transportedBy = w._kinematicTrees[child].get("customStateVariables", {}).get("transporting", {}).get("constraint")
    if transportedBy != name:
        w.removeObject((name,))

def updateTransporting(name, customDynamicsAPI):
    def ornCloseness(a, b, th):
        ax = stubbornTry(lambda : pybullet.rotateVector(a, [1,0,0]))
        ay = stubbornTry(lambda : pybullet.rotateVector(a, [0,1,0]))
        bx = stubbornTry(lambda : pybullet.rotateVector(b, [1,0,0]))
        by = stubbornTry(lambda : pybullet.rotateVector(b, [0,1,0]))
        return (th < ax[0]*bx[0]+ax[1]*bx[1]+ax[2]*bx[2]) and (th < ay[0]*by[0]+ay[1]*by[1]+ay[2]*by[2])
    w = customDynamicsAPI["leetHAXXOR"]()
    if "customStateVariables" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["customStateVariables"] = {}
    csv = w._kinematicTrees[name].get("customStateVariables")
    if "transporting" not in csv:
        csv["transporting"] = {}
    transportedBy = csv["transporting"].get("constraint")
    newTransportedBy = None
    at = w.atComponent((name,))
    if (at is None) and (transportedBy is not None):
        at = (w._kinematicConstraints[transportedBy].get("parent"), w._kinematicConstraints[transportedBy].get("parentLink"))
    if at is None:
        return
    atFn = w._kinematicTrees[at[0]].get("fn", {})
    atGraspable = atFn.get("graspable", False)
    relativeStillness = atGraspable
    _, atOrientation, atVelocity, _ = w.getKinematicData(at) # WARNING: assumes single-link transporter, or at least one where all links have the same orientation always.
    if atGraspable and (transportedBy is None):
        _, _, nameVelocity, _ = w.getKinematicData((name,))
        dv = [x-y for x,y in zip(atVelocity, nameVelocity)]
        dv = dv[0]*dv[0]+dv[1]*dv[1]+dv[2]*dv[2]
        if 0.0001 < dv:
            relativeStillness = False
    mingling = csv.get("mingling", {}).get("mingling") or False
    if relativeStillness and (not mingling) and (w._kinematicTrees[at[0]].get("customStateVariables", {}).get("graspedBy") is not None):
        pourAxisInAt = atFn.get("containment", {}).get("pouring", {}).get("outof", {}).get("axis") or (0,1,0)
        pourAxis = stubbornTry(lambda : pybullet.rotateVector(atOrientation, pourAxisInAt))
        down = w.getDown()
        tipped = (0.9 < down[0]*pourAxis[0]+down[1]*pourAxis[1]+down[2]*pourAxis[2])
        # is this itself grasped? : if so, remove
        if not tipped:
            childOf = w._kinematicTrees[name].get("childOf") or []
            grasped = False
            for x in childOf:
                parent = w._kinematicConstraints[x].get("parent")
                for _, grasps in w._kinematicTrees[parent].get("customStateVariables", {}).get("grasping", {}).get("actuallyGrasping", {}).items():
                    for grasp in grasps:
                        if name == grasp[0][0]:
                            grasped = True
                            break
                    if grasped:
                        break
                if grasped:
                    break
            if not grasped:
                newTransportedBy = transportedBy
                if newTransportedBy is None:
                    nameBaseLink = w._kinematicTrees[name]["idx2Link"][-1] # base link name
                    newTransportedBy = ("TRANSPORTING_%s_%s_%s_%s" % (at[0], at[1], name, nameBaseLink))
                    maxForce = atFn.get("transporting", {}).get("maxForce") or 1000
                    w.addNewObject({"fn": {"transportingConstraint": True}, "name": newTransportedBy, "simtype": "kcon", "parent": at[0], "child": name, "parentLink": at[1], "childLink": nameBaseLink, "jointType": "fixed", "maxForce": maxForce})
    csv["transporting"]["constraint"] = newTransportedBy

