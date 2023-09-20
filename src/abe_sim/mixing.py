import math

from abe_sim.world import getDictionaryEntry

def updateMingling(name, customDynamicsAPI):
    w = customDynamicsAPI["leetHAXXOR"]()
    fnMingling = w._kinematicTrees[name].get("fn") or {}
    if "customStateVariables" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["customStateVariables"] = {}
    if "mingling" not in w._kinematicTrees[name]["customStateVariables"]:
        w._kinematicTrees[name]["customStateVariables"]["mingling"] = {}
    csvMingling = w._kinematicTrees[name]["customStateVariables"]["mingling"]
    nameType = w._kinematicTrees[name]["type"]
    aabb = w.getAABB((name,))
    aabbAdj = w.adjustAABBRadius(aabb, (fnMingling.get("radius") or 0.3))
    overlaps = [x for x in customDynamicsAPI['checkOverlap'](aabbAdj) if name != x[0]]
    minglers = set([x[0] for x in overlaps if w._kinematicTrees[x[0]].get("fn", {}).get("canMingle")])
    isMingling = False
    for mingler in minglers:
        fnMingler = w._kinematicTrees[mingler].get("fn", {}).get("mingling") or {}
        minglerLinks = fnMingler.get("links") or []
        for minglerLink in minglerLinks:
            if (mingler, minglerLink) in overlaps:
                isMingling = True
                break
        if isMingling:
            break
    if isMingling:
        hp = csvMingling.get("hp", 0) - 1
        if 0 < hp:
            nameP, _, _, _ = w.getKinematicData((name,))
            minglerP, _, _, _ = w.getKinematicData((mingler, minglerLink))
            dx = nameP[0] - minglerP[0]
            dy = nameP[1] - minglerP[1]
            norm = math.sqrt(dx*dx+dy*dy)
            #mass = w.getObjectProperty((name,), "mass")
            #if 0.1 < norm:
            #    dx = 0.05*dx/norm
            #    dy = 0.05*dy/norm
            #force = [-20*mass*dy, 20*mass*dx, 0]
            w.setObjectProperty((name,), "linearVelocity", [-dy,dx,0])
            w.setObjectProperty((name,), "angularVelocity", [0,0,0.4])
            #w.applyExternalForce((name,), force, nameP, inWorldFrame=True)
            #w.applyExternalTorque((name,), force, inWorldFrame=True)
            csvMingling["mingling"] = True
        else:
            csvMingling["mingling"] = False
        csvMingling["hp"] = hp
    else:
        csvMingling["mingling"] = False

def updateMixing(name, customDynamicsAPI):
    def setStr(l):
        l = sorted(list(l))
        return ";".join(l)
    w = customDynamicsAPI["leetHAXXOR"]()
    if "customStateVariables" not in w._kinematicTrees[name]:
        w._kinematicTrees[name]["customStateVariables"] = {}
    if "mixing" not in w._kinematicTrees[name]["customStateVariables"]:
        w._kinematicTrees[name]["customStateVariables"]["mixing"] = {}
    csvMixing = w._kinematicTrees[name]["customStateVariables"]["mixing"]
    fnMixing = w._kinematicTrees[name].get("fn", {}).get("mixing") or {}
    nameType = w._kinematicTrees[name].get("type")
    aabb = w.getAABB((name,))
    aabbAdj = w.adjustAABBRadius(aabb, (fnMixing.get("mixableRadius") or 0.5))
    overlaps = set(w.checkOverlap(aabbAdj))
    closeObjects = set([x[0] for x in overlaps if name != x[0]])
    mixers = [x for x in closeObjects if w._kinematicTrees[x].get("fn", {}).get("canMix")]
    mixables = [x for x in closeObjects if w._kinematicTrees[x].get("fn", {}).get("mixable") or w._kinematicTrees[x].get("fn", {}).get("mixMakeable")]
    isMixing = False
    for mixer in mixers:
        fnMixer = w._kinematicTrees[mixer].get("fn").get("mixing") or {}
        mixerLinks = fnMixer.get("links") or []
        for mixerLink in mixerLinks:
            if (mixer, mixerLink) in overlaps:
                isMixing = True
                break
        if isMixing:
            break
    if isMixing:
        neighboringTypes = set([nameType])
        for particle in mixables:
            pt = w._kinematicTrees[particle].get("type")
            neighboringTypes.add(pt)
        bestMatch = None
        for ingredientList in w.getProcessResource({"process": "mixing", "patient": nameType}):
            if 0 == len(set(ingredientList).difference(neighboringTypes)):
                if (bestMatch is None) or (len(bestMatch) < len(ingredientList)):
                    bestMatch = ingredientList
        if bestMatch is not None:
            hp = (csvMixing.get("hp") or 0) - 1
            if 0 < hp:
                csvMixing["hp"] = hp
            else:
                w.concludeProcess({"process": "mixing", "patient": setStr(bestMatch)}, name)

