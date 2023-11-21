import yaml

import math
import time

import copy

from abe_sim.geom_utils import vector2Axis
from abe_sim.world import _setDictionaryEntry, getDictionaryEntry

conditionalModes = {"dummy": (lambda varVal, refVal: True),
                    "notInclude": (lambda varVal, refVal: (refVal not in varVal)),
                    "includes": (lambda varVal, refVal: (refVal in varVal)),
                    "wholeSubsumesSomeElement": (lambda varVal, refVal: any([varVal.issuperset(x) for x in refVal])),
                    "equal": (lambda varVal, refVal: varVal == refVal)}

def interpretVariableDescription(name, w, description, roleBdgs):
    if isinstance(description, int) or isinstance(description, float) or isinstance(description, bool) or (description is None):
        return description
    if isinstance(description, str):
        if "$" == description[0]:
            return roleBdgs[description[1:]]
        return description
    if "world" == description[0]:
        if "upAxis" == description[1]:
            return w.getUp()
        elif "downAxis" == description[1]:
            return w.getDown()
        elif "contact-normal" == description[1]:
            return roleBdgs["contact-normal"]
    newDescription = [interpretVariableDescription(name, w, x, roleBdgs) for x in description[1:]]
    if "self" == description[0]:
        return getDictionaryEntry(w._kinematicTrees[name], newDescription, None)
    if "trajector" == description[0]:
        return getDictionaryEntry(w._kinematicTrees[roleBdgs["trajector"]], newDescription, None)
    if "processKnowledge" == description[0]:
        return w.getProcessKnowledge(newDescription)

def getVariablePath(name, w, description, roleBdgs):
    obName = roleBdgs["trajector"]
    if "self" == description[0]:
        obName = name
    return obName, [interpretVariableDescription(name, w, x, roleBdgs) for x in description[1:]]

def getAxisInWorld(name, w, axis, roleBdgs):
    iniValue = interpretVariableDescription(name, w, axis, roleBdgs)
    if (iniValue is None) or ("world" == axis[0]):
        return iniValue
    if "self" == axis[0]:
        _, refOrientation, _, _ = w.getKinematicData((name,))
    elif "trajector" == axis[0]:
        _, refOrientation, _, _ = w.getKinematicData((roleBdgs["trajector"],))
    return w.objectPoseRelativeToWorld([0,0,0], refOrientation, iniValue, [0,0,0,1])[0]

def getCandidates(name, w, description, disposition, partsPath):
    retq = []
    tC = 0.0
    if "contact" == description["mode"]:
        collisions = w.checkCollision((name,))
        aux = {}
        for collision in collisions:
            identifierA, identifierB, posA, posB, normal, distance, force, _, _, _, _ = collision
            if (disposition is not None) and (not w._kinematicTrees[identifierB[0]].get("fn").get(disposition)):
                continue
            if ((identifierA, identifierB) not in aux) or (force > aux[(identifierA, identifierB)]["force"]):
                aux[(identifierA, identifierB)] = {"relatumId": identifierA, "trajectorId": identifierB, "posOnRelatum": posA, "posOnTrajector": posB, "normal": normal, "distance": distance, "force": force}
        retq = list(aux.values())
    elif "transitive-at" == description["mode"]:
        atOb = name
        while True:
            atOb = w.at((atOb,))
            if atOb is None:
                break
            if not w._kinematicTrees[atOb].get("fn").get(disposition):
                continue
            retq.append({"relatumId": (name,), "trajectorId": (atOb,), "posOnRelatum": None, "posOnTrajector": None, "normal": None, "distance": None, "force": 0})
    elif "close" ==  description["mode"]:
        radius = interpretVariableDescription(name, w, description["radius"], {})
        aabb = w.getAABB((name,))
        aabbAdj = w.addAABBRadius(aabb, radius)
        closeObjects = w.checkOverlap(aabbAdj)
        if disposition is not None:
            closeObjects = [x for x in closeObjects if w._kinematicTrees[x[0]].get("fn", {}).get(disposition)]
        if partsPath is not None:
            aux = {}
            closeNew = []
            for cob, clnk in closeObjects:
                if cob not in aux:
                    aux[cob] = getDictionaryEntry(w._kinematicTrees[cob], partsPath, [])
                if clnk in aux[cob]:
                    closeNew.append((cob, clnk))
            closeObjects = closeNew
        for cob, clnk in closeObjects:
            closePoints = w.checkClosestPoints((name,), (cob, clnk), maxDistance=radius)
            for closePoint in closePoints:
                identifierA, identifierB, posA, posB, normal, distance = closePoint
                retq.append({"relatumId": identifierA, "trajectorId": identifierB, "posOnRelatum": posA, "posOnTrajector": posB, "normal": normal, "distance": distance, "force": 0})
    elif "pose" == description["mode"]:
        links = w._kinematicTrees[name]["links"]
        for l in links:
            retq.append({"relatumId": (name, l), "trajectorId": (name, l), "posOnRelatum": None, "posOnTrajector": None, "normal": None, "distance": None, "force": 0})
    return retq

def filterObject(candidate, name, w, roleBdgs, role, description):
    if description is None:
        return True, roleBdgs
    if "disposition" in description:
        if not interpretVariableDescription(name, w, description["disposition"], roleBdgs):
            return False, roleBdgs
    if "part" in description:
        acceptableParts = interpretVariableDescription(name, w, description["part"], roleBdgs)
        if (acceptableParts is None) or (roleBdgs["%s-part" % role] not in acceptableParts):
            return False, roleBdgs
    if "substance" in description:
        substanceSet = interpretVariableDescription(name, w, description["substance"], roleBdgs)
        if (substanceSet is None) or (0 == len(substanceSet)):
            return False, roleBdgs
        roleBdgs["%s-substance" % role] = list(substanceSet.keys())[0]
    if "resources" in description:
        for res in description["resources"]:
            variable = interpretVariableDescription(name, w, res["variable"], roleBdgs)
            reference = interpretVariableDescription(name, w, res["reference"], roleBdgs)
            if (variable is None) or (not conditionalModes[res["mode"]](variable, reference)):
                return False, roleBdgs
    return True, roleBdgs

def filterSource(candidate, name, w, roleBdgs, description, kinCache):
    if description is None:
        return True, roleBdgs
    if "minforce" in description:
        threshold = interpretVariableDescription(name, w, description['minforce'], roleBdgs)
        if not (threshold < candidate["force"]):
            return False, roleBdgs
    if "constraints" in description:
        for c in description["constraints"]:
            if "axis" in c:
                axisInWorld = getAxisInWorld(name, w, c["axis"], roleBdgs)
                referenceInWorld = getAxisInWorld(name, w, c["reference"], roleBdgs)
                thresholdVal = interpretVariableDescription(name, w, c["threshold"], roleBdgs)
                mode = c["mode"]
                dot = axisInWorld[0]*referenceInWorld[0] + axisInWorld[1]*referenceInWorld[1] + axisInWorld[2]*referenceInWorld[2]
                if ("alignment" == mode) and (thresholdVal > dot):
                    return False, roleBdgs
                elif ("counter-alignment" == mode) and (thresholdVal > -dot):
                    return False, roleBdgs
                elif ("orthogonal" == mode) and (thresholdVal < abs(dot)):
                    return False, roleBdgs
    return True, roleBdgs

def filterPath(candidate, name, w, roleBdgs, description, kinCache):
    if description is None:
        return True, roleBdgs
    mode = description["mode"]
    reference = description.get("reference")
    threshold = description.get("threshold")
    referenceVal = getAxisInWorld(name, w, reference, roleBdgs)
    thresholdVal = interpretVariableDescription(name, w, threshold, roleBdgs)
    if (roleBdgs["trajector"], roleBdgs["trajector-part"]) not in kinCache:
        kinCache[(roleBdgs["trajector"], roleBdgs["trajector-part"])] = w.getKinematicData((roleBdgs["trajector"], roleBdgs["trajector-part"]))
    _, _, velT, _ = kinCache[(roleBdgs["trajector"], roleBdgs["trajector-part"])]
    if (roleBdgs["relatum"], roleBdgs["relatum-part"]) not in kinCache:
        kinCache[(roleBdgs["relatum"], roleBdgs["relatum-part"])] = w.getKinematicData((roleBdgs["relatum"], roleBdgs["relatum-part"]))
    _, _, velR, _ = kinCache[(roleBdgs["relatum"], roleBdgs["relatum-part"])]
    relativeVelocity = [velT[0]-velR[0], velT[1]-velR[1], velT[2]-velR[2]]
    speed = math.sqrt(relativeVelocity[0]*relativeVelocity[0]+relativeVelocity[1]*relativeVelocity[1]+relativeVelocity[2]*relativeVelocity[2])
    if 0.000001 > speed:
        velocityDirection = relativeVelocity
    else:
        velocityDirection = [relativeVelocity[0]/speed, relativeVelocity[1]/speed, relativeVelocity[2]/speed]
    dot = velocityDirection[0]*referenceVal[0] + velocityDirection[1]*referenceVal[1] + velocityDirection[2]*referenceVal[2]
    retq = False
    if ("minmagnitude" == mode) and (threshold < speed):
        retq = True
    elif ("alignment" == mode) and (threshold < dot):
        retq = True
    elif ("counter-alignment" == mode) and (threshold < -dot):
        retq = True
    elif ("orthogonal" == mode) and (threshold > abs(dot)):
        retq = True
    return retq, roleBdgs

def filterGoal(candidate, name, w, roleBdgs, description, kinCache):
    if description is None:
        return True, roleBdgs
    direction = description["direction"]
    threshold = description["threshold"]
    if (roleBdgs["trajector"], roleBdgs["trajector-part"]) not in kinCache:
        kinCache[(roleBdgs["trajector"], roleBdgs["trajector-part"])] = w.getKinematicData((roleBdgs["trajector"], roleBdgs["trajector-part"]))
    posT, _, velT, _ = kinCache[(roleBdgs["trajector"], roleBdgs["trajector-part"])]
    if (roleBdgs["relatum"], roleBdgs["relatum-part"]) not in kinCache:
        kinCache[(roleBdgs["relatum"], roleBdgs["relatum-part"])] = w.getKinematicData((roleBdgs["relatum"], roleBdgs["relatum-part"]))
    posR, _, velR, _ = kinCache[(roleBdgs["relatum"], roleBdgs["relatum-part"])]
    pd = [posR[0]-posT[0], posR[1]-posT[1], posR[2]-posT[2]]
    vd = [velT[0]-velR[0], velT[1]-velR[1], velT[2]-velR[2]]
    pdn = math.sqrt(pd[0]*pd[0]+pd[1]*pd[1]+pd[2]*pd[2])
    vdn = math.sqrt(vd[0]*vd[0]+vd[1]*vd[1]+vd[2]*vd[2])
    if 0.000001 > pdn:
        dispDir = pd
    else:
        dispDir = [pd[0]/pdn, pd[1]/pdn, pd[2]/pdn]
    if 0.000001 > vdn:
        velDir = pd
    else:
        velDir = [vd[0]/vdn, vd[1]/vdn, vd[2]/vdn]
    retq = False
    dot = velDir[0]*dispDir[0] + velDir[1]*dispDir[1] + velDir[2]*dispDir[2]
    if ("away" == direction) and (threshold < -dot):
        retq = True
    if ("towards" == direction) and (threshold < dot):
        retq = True
    if ("around" == direction) and (threshold > abs(dot)):
        retq = True
    return retq, roleBdgs

def checkProgress(name, w, description):
    def loadCandidate(candidate, name, w, roleBdgs):
        roleBdgs["relatum-part"] = None
        roleBdgs["trajector-part"] = None
        if 1 < len(candidate["relatumId"]):
            roleBdgs["relatum-part"] = candidate["relatumId"][1]
        if 1 < len(candidate["trajectorId"]):
            roleBdgs["trajector-part"] = candidate["trajectorId"][1]
        roleBdgs["trajector"] = candidate["trajectorId"][0]
        roleBdgs["trajector-type"] = w._kinematicTrees[roleBdgs["trajector"]].get("type")
        roleBdgs["contact-normal"] = candidate["normal"]
        roleBdgs["contact-force"] = candidate["force"]
        return True, roleBdgs
    retq = False
    roleBdgs = {"relatum": name, "relatum-type": w._kinematicTrees[name].get("type")}
    disposition = description.get("trajector", {}).get("disposition")
    partsPath = description.get("trajector", {}).get("part")
    if disposition is not None:
        disposition = disposition[-1]
    if partsPath is not None:
        partsPath = partsPath[1:]
    candidates = getCandidates(name, w, description["source"], disposition, partsPath)
    kinCache = {}
    filters = [loadCandidate]
    filters += [(lambda c, name, w, roleBdgs: filterObject(c, name, w, roleBdgs, "relatum", description.get("relatum")))]
    filters += [(lambda c, name, w, roleBdgs: filterObject(c, name, w, roleBdgs, "trajector", description.get("trajector")))]
    filters += [(lambda c, name, w, roleBdgs: filterSource(c, name, w, roleBdgs, description.get("source"), kinCache))]
    filters += [(lambda c, name, w, roleBdgs: filterPath(c, name, w, roleBdgs, description.get("path"), kinCache))]
    filters += [(lambda c, name, w, roleBdgs: filterGoal(c, name, w, roleBdgs, description.get("goal"), kinCache))]
    for c in candidates:
        retq = True
        for f in filters:
            retq, roleBdgs = f(c, name, w, roleBdgs)
            if retq is False:
                break
        if retq is True:
            break
    return retq, roleBdgs

def decrementResource(name, w, roleBdgs, previousResult, resourceDescription, resetDescription):
    newResult = False
    value = interpretVariableDescription(name, w, resourceDescription, roleBdgs)
    if value is None:
        return None
    value -= 1
    if 0 >= value:
        value = 0
        newResult = True
        if resetDescription is not None:
            doReset = True
            if 'lives' in resetDescription:
                lives = interpretVariableDescription(name, w, resetDescription["lives"], roleBdgs)
                if lives is not None:
                    lives -= 1
                    if 0 >= lives:
                        doReset = False
                        lives = 0
                    _, livesPath = getVariablePath(name, w, resourceDescription["lives"], roleBdgs)
                    _setDictionaryEntry(w._kinematicTrees[name], livesPath, lives)
            if doReset:
                value = interpretVariableDescription(name, w, resetDescription["value"], roleBdgs)
    _, varPath = getVariablePath(name, w, resourceDescription, roleBdgs)
    _setDictionaryEntry(w._kinematicTrees[name], varPath, value)
    if (previousResult is None) or (newResult is False):
        return newResult
    return previousResult
    
def accumulateResource(name, w, roleBdgs, previousResult, variableDescription, valueDescription, capacity, conditionFn, conditionVariableDescription, conditionReference):
    conditionVariableValue = interpretVariableDescription(name, w, conditionVariableDescription, roleBdgs)
    conditionReferenceValue = interpretVariableDescription(name, w, conditionReference, roleBdgs)
    newResult = previousResult
    if conditionFn(conditionVariableValue, conditionReferenceValue):
        previousAccumulatorValue = interpretVariableDescription(name, w, variableDescription, roleBdgs)
        _, accumulatorPath = getVariablePath(name, w, variableDescription, roleBdgs)
        increment = interpretVariableDescription(name, w, valueDescription, roleBdgs)
        numericAcc = True
        if isinstance(increment, str):
            numericAcc = False
        if previousAccumulatorValue is None:
            if numericAcc:
                previousAccumulatorValue = 0
            else:
                previousAccumulatorValue = {}
            _setDictionaryEntry(w._kinematicTrees[name], accumulatorPath, previousAccumulatorValue)
        if numericAcc:
            newAccumulatorValue = previousAccumulatorValue + increment
        else:
            newAccumulatorValue = previousAccumulatorValue
            newAccumulatorValue[increment] = True
        _setDictionaryEntry(w._kinematicTrees[name], accumulatorPath, newAccumulatorValue)
        capacity = interpretVariableDescription(name, w, capacity, roleBdgs)
        if capacity is not None:
            if numericAcc:
                newResult = (capacity <= newAccumulatorValue)
            else:
                newResult = (capacity <= len(newAccumulatorValue))
            if (previousResult is not None) or (newResult is True):
                newResult = previousResult
    return newResult

def processCompletion(name, w, completionDescription, roleBdgs):
    description = {k: interpretVariableDescription(name, w, v, roleBdgs) for k, v in completionDescription.items()}
    description["patient"] = roleBdgs["relatum-type"]
    description["process"] = completionDescription["process"]
    w.concludeProcess(description, name)

def clearVariable(name, w, description, roleBdgs):
    value = interpretVariableDescription(name, w, description[:-1], roleBdgs)
    last = interpretVariableDescription(name, w, description[-1], roleBdgs)
    if isinstance(value.get(last), bool):
        value.pop(last)
        _, varPath = getVariablePath(name, w, description[:-1], roleBdgs)
        _setDictionaryEntry(w._kinematicTrees[name], varPath, value)

def customDynamic(name, customDynamicsAPI, progressDescription, resourceDescription, completionDescription, dynname):
    sT = time.perf_counter()
    w = customDynamicsAPI["leetHAXXOR"]()
    isProgressing, roleBdgs = checkProgress(name, w, progressDescription)
    eP = time.perf_counter()
    if isProgressing:
        resourceTrigger = None
        if (resourceDescription["resource"] is None) and (resourceDescription["accumulators"] is None):
            resourceTrigger = True
        if (resourceDescription["resource"] is not None):
            ####
            resourceTrigger = decrementResource(name, w, roleBdgs, resourceTrigger, resourceDescription["resource"], resourceDescription["reset"])
        if (resourceDescription["accumulators"] is not None) and ((resourceTrigger is not None) or (resourceDescription["resource"] is None)):
            for acc in resourceDescription["accumulators"]:
                variableDescription = acc["variable"]
                capacity = acc.get("capacity", None)
                valueDescription = acc.get("value", None)
                conditionFn = lambda x, y: True
                conditionVariableDescription = None
                conditionReference = None
                if "conditional" in acc:
                    mode = acc["conditional"].get("mode", "dummy")
                    conditionFn = conditionalModes[mode]
                    conditionVariableDescription = acc["conditional"].get("variable")
                    conditionReference = acc["conditional"].get("reference")
                ####
                resourceTrigger = accumulateResource(name, w, roleBdgs, resourceTrigger, variableDescription, valueDescription, capacity, conditionFn, conditionVariableDescription, conditionReference)
        if resourceTrigger:
            if completionDescription["completion"] is not None:
                processCompletion(name, w, completionDescription["completion"], roleBdgs)
            if completionDescription["clear"] is not None:
                for clearDescription in completionDescription["clear"]:
                    clearVariable(name, w, clearDescription, roleBdgs)

def buildSpecs(processDescriptionFilename):
    procs = yaml.safe_load(open(processDescriptionFilename).read())
    retq = []
    for dynname, description in procs.items():
        disposition = tuple(description['disposition'][1:])
        progressDescription = copy.deepcopy(description['progress'])
        resetDescription = None
        if 'reset' in description:
            lives = None
            if 'lives' in description['reset']:
                lives = copy.deepcopy(description['reset']['lives'])
            resetDescription = {'lives': lives, 'value': description['reset']['value']}
        resourceDescription = {'resource': copy.deepcopy(description.get('resource')), 'accumulators': copy.deepcopy(description.get('accumulators')), 'reset': resetDescription}
        completionDescription = {'completion': copy.deepcopy(description.get('completion')), 'clear': copy.deepcopy(description.get('clear'))}
        updateFn = lambda name, customDynamicsAPI, progressDescription=progressDescription, resourceDescription=resourceDescription, completionDescription=completionDescription, dynname=dynname: customDynamic(name, customDynamicsAPI, progressDescription, resourceDescription, completionDescription, dynname)
        retq.append([disposition, updateFn])
    return retq

