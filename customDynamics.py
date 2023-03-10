import yaml

import math
import numpy as np

import copy

from geom_utils import vector2Axis

conditionalModes = {'dummy': (lambda varVal, refVal: True),
                    'notInclude': (lambda varVal, refVal: (refVal not in varVal)),
                    'includes': (lambda varVal, refVal: (refVal in varVal)),
                    'wholeSubsumesSomeElement': (lambda varVal, refVal: any([varVal.issuperset(x) for x in refVal])),
                    'equal': (lambda varVal, refVal: varVal == refVal)}

def interpretVariableDescription(name, customDynamicsAPI, description, roleBdgs):
    if isinstance(description, int) or isinstance(description, float) or isinstance(description, bool) or (description is None):
        return description
    if isinstance(description, str):
        if '$' == description[0]:
            return roleBdgs[description[1:]]
        return description
    if 'world' == description[0]:
        if 'upAxis' == description[1]:
            return customDynamicsAPI['getUp']()
        elif 'downAxis' == description[1]:
            return customDynamicsAPI['getDown']()
        elif 'contact-normal' == description[1]:
            return roleBdgs['contact-normal']
    newDescription = [interpretVariableDescription(name, customDynamicsAPI, x, roleBdgs) for x in description[1:]]
    if 'self' == description[0]:
        return customDynamicsAPI['getObjectProperty']((name,), newDescription)
    if 'trajector' == description[0]:
        return customDynamicsAPI['getObjectProperty']((roleBdgs['trajector'],), newDescription)
    if 'processKnowledge' == description[0]:
        return customDynamicsAPI['getProcessKnowledge'](newDescription)

def getVariablePath(name, customDynamicsAPI, description, roleBdgs):
    obName = roleBdgs['trajector']
    if 'self' == description[0]:
        obName = name
    return obName, [interpretVariableDescription(name, customDynamicsAPI, x, roleBdgs) for x in description[1:]]

def getAxisInWorld(name, customDynamicsAPI, axis, roleBdgs):
    iniValue = interpretVariableDescription(name, customDynamicsAPI, axis, roleBdgs)
    if (iniValue is None) or ('world' == axis[0]):
        return iniValue
    if 'self' == axis[0]:
        refOrientation = customDynamicsAPI['getObjectProperty']((name,), 'orientation')
    elif 'trajector' == axis[0]:
        refOrientation = customDynamicsAPI['getObjectProperty']((roleBdgs['trajector'],), 'orientation')
    return customDynamicsAPI['objectPoseRelativeToWorld']([0,0,0], refOrientation, iniValue, [0,0,0,1])[0]

def getCandidates(name, customDynamicsAPI, description):
    retq = []
    if 'contact' == description['mode']:
        collisions = customDynamicsAPI['checkCollision']((name,))
        for collision in collisions:
            identifierA, identifierB, posA, posB, normal, distance, force, _, _, _, _ = collision
            retq.append({'relatumId': identifierA, 'trajectorId': identifierB, 'posOnRelatum': posA, 'posOnTrajector': posB, 'normal': normal, 'distance': distance, 'force': force})
    elif 'transitive-at' == description['mode']:
        atOb = name
        while True:
            atOb = customDynamicsAPI['getObjectProperty']((atOb,), 'at')
            if atOb is None:
                break
            retq.append({'relatumId': (name,), 'trajectorId': (atOb,), 'posOnRelatum': None, 'posOnTrajector': None, 'normal': None, 'distance': None, 'force': 0})
    elif 'close' ==  description['mode']:
        radius = interpretVariableDescription(name, customDynamicsAPI, description['radius'], {})
        aabb = customDynamicsAPI['getObjectProperty']((name,), 'aabb')
        aabbAdj = customDynamicsAPI['addAABBRadius'](aabb, radius)
        closeObjects = customDynamicsAPI['checkOverlap'](aabbAdj)
        for cob in closeObjects:
            closePoints = customDynamicsAPI['checkClosestPoints']((name,), (cob,), maxDistance=radius)
            for closePoint in closePoints:
                identifierA, identifierB, posA, posB, normal, distance = closePoint
                retq.append({'relatumId': identifierA, 'trajectorId': identifierB, 'posOnRelatum': posA, 'posOnTrajector': posB, 'normal': normal, 'distance': distance, 'force': 0})
    elif 'pose' == description['mode']:
        links = customDynamicsAPI['getObjectProperty']((name,), 'links')
        for l in links:
            retq.append({'relatumId': (name, l), 'trajectorId': (name, l), 'posOnRelatum': None, 'posOnTrajector': None, 'normal': None, 'distance': None, 'force': 0})
    return retq

def filterObject(candidate, name, customDynamicsAPI, roleBdgs, role, description):
    if description is None:
        return True, roleBdgs
    if 'disposition' in description:
        if not interpretVariableDescription(name, customDynamicsAPI, description['disposition'], roleBdgs):
            return False, roleBdgs
    if 'part' in description:
        acceptableParts = interpretVariableDescription(name, customDynamicsAPI, description['part'], roleBdgs)
        if (acceptableParts is None) or (roleBdgs['%s-part' % role] not in acceptableParts):
            return False, roleBdgs
    if 'substance' in description:
        substanceSet = interpretVariableDescription(name, customDynamicsAPI, description['substance'], roleBdgs)
        if (substanceSet is None) or (0 == len(substanceSet)):
            return False, roleBdgs
        roleBdgs['%s-substance' % role] = list(substanceSet.keys())[0]
    if 'resources' in description:
        for res in description['resources']:
            variable = interpretVariableDescription(name, customDynamicsAPI, res['variable'], roleBdgs)
            reference = interpretVariableDescription(name, customDynamicsAPI, res['reference'], roleBdgs)
            print(res, variable, reference)
            if (variable is None) or (not conditionalModes[res['mode']](variable, reference)):
                return False, roleBdgs
    return True, roleBdgs

def filterSource(candidate, name, customDynamicsAPI, roleBdgs, description):
    if description is None:
        return True, roleBdgs
    if 'minforce' in description:
        threshold = interpretVariableDescription(name, customDynamicsAPI, description['minforce'], roleBdgs)
        if not (threshold < candidate['force']):
            return False, roleBdgs
    if 'constraints' in description:
        for c in description['constraints']:
            if 'axis' in c:
                axisInWorld = getAxisInWorld(name, customDynamicsAPI, c['axis'], roleBdgs)
                referenceInWorld = getAxisInWorld(name, customDynamicsAPI, c['reference'], roleBdgs)
                thresholdVal = interpretVariableDescription(name, customDynamicsAPI, c['threshold'], roleBdgs)
                mode = c['mode']
                if ('alignment' == mode) and (thresholdVal > np.dot(axisInWorld, referenceInWorld)):
                    return False, roleBdgs
                elif ('counter-alignment' == mode) and (thresholdVal > -np.dot(axisInWorld, referenceInWorld)):
                    return False, roleBdgs
                elif ('orthogonal' == mode) and (thresholdVal < abs(np.dot(axisInWorld, referenceInWorld))):
                    return False, roleBdgs
    return True, roleBdgs

def filterPath(candidate, name, customDynamicsAPI, roleBdgs, description):
    if description is None:
        return True, roleBdgs
    mode = description['mode']
    reference = description.get('reference')
    threshold = description.get('threshold')
    referenceVal = getAxisInWorld(name, customDynamicsAPI, reference, roleBdgs)
    thresholdVal = interpretVariableDescription(name, customDynamicsAPI, threshold, roleBdgs)
    velT = np.array(customDynamicsAPI['getObjectProperty']((roleBdgs['trajector'], roleBdgs['trajector-part']), 'linearVelocity'))
    velR = np.array(customDynamicsAPI['getObjectProperty']((roleBdgs['relatum'], roleBdgs['relatum-part']), 'linearVelocity'))
    relativeVelocity = velT-velR
    velocityDirection = vector2Axis(relativeVelocity)
    speed = np.linalg.norm(relativeVelocity)
    retq = False
    if ('minmagnitude' == mode) and (threshold < speed):
        retq = True
    elif ('alignment' == mode) and (threshold < np.dot(velocityDirection, referenceVal)):
        retq = True
    elif ('counter-alignment' == mode) and (threshold < -np.dot(velocityDirection, referenceVal)):
        retq = True
    elif ('orthogonal' == mode) and (threshold > abs(np.dot(velocityDirection, referenceVal))):
        retq = True
    return retq, roleBdgs

def filterGoal(candidate, name, customDynamicsAPI, roleBdgs, description):
    if description is None:
        return True, roleBdgs
    direction = description['direction']
    threshold = description['threshold']
    posT = np.array(customDynamicsAPI['getObjectProperty']((roleBdgs['trajector'], roleBdgs['trajector-part']), 'position'))
    posR = np.array(customDynamicsAPI['getObjectProperty']((roleBdgs['relatum'], roleBdgs['relatum-part']), 'position'))
    velT = np.array(customDynamicsAPI['getObjectProperty']((roleBdgs['trajector'], roleBdgs['trajector-part']), 'linearVelocity'))
    velR = np.array(customDynamicsAPI['getObjectProperty']((roleBdgs['relatum'], roleBdgs['relatum-part']), 'linearVelocity'))
    velDir = vector2Axis(velT - velR)
    dispDir = vector2Axis(posR - posT)
    retq = False
    if ('away' == direction) and (threshold < -np.dot(velDir, dispDir)):
        retq = True
    if ('towards' == direction) and (threshold < np.dot(velDir, dispDir)):
        retq = True
    if ('around' == direction) and (threshold > abs(np.dot(velDir, dispDir))):
        retq = True
    return retq, roleBdgs

def checkProgress(name, customDynamicsAPI, description):
    def loadCandidate(candidate, name, customDynamicsAPI, roleBdgs):
        roleBdgs['relatum-part'] = candidate['relatumId'][1]
        roleBdgs['trajector'] = candidate['trajectorId'][0]
        roleBdgs['trajector-part'] = candidate['trajectorId'][1]
        roleBdgs['trajector-type'] = customDynamicsAPI['getObjectProperty']((roleBdgs['trajector'],), 'type')
        roleBdgs['contact-normal'] = candidate['normal']
        roleBdgs['contact-force'] = candidate['force']
        return True, roleBdgs
    retq = False
    roleBdgs = {'relatum': name, 'relatum-type': customDynamicsAPI['getObjectProperty']((name,), 'type')}
    candidates = getCandidates(name, customDynamicsAPI, description['source'])
    filters = [loadCandidate]
    filters += [(lambda c, name, customDynamicsAPI, roleBdgs: filterObject(c, name, customDynamicsAPI, roleBdgs, 'relatum', description.get('relatum')))]
    filters += [(lambda c, name, customDynamicsAPI, roleBdgs: filterObject(c, name, customDynamicsAPI, roleBdgs, 'trajector', description.get('trajector')))]
    filters += [(lambda c, name, customDynamicsAPI, roleBdgs: filterSource(c, name, customDynamicsAPI, roleBdgs, description.get('source')))]
    filters += [(lambda c, name, customDynamicsAPI, roleBdgs: filterPath(c, name, customDynamicsAPI, roleBdgs, description.get('path')))]
    filters += [(lambda c, name, customDynamicsAPI, roleBdgs: filterGoal(c, name, customDynamicsAPI, roleBdgs, description.get('goal')))]
    for c in candidates:
        retq = True
        for f in filters:
            retq, roleBdgs = f(c, name, customDynamicsAPI, roleBdgs)
            if retq is False:
                break
        if retq is True:
            break
    return retq, roleBdgs

def decrementResource(name, customDynamicsAPI, roleBdgs, previousResult, resourceDescription, resetDescription):
    newResult = False
    value = interpretVariableDescription(name, customDynamicsAPI, resourceDescription, roleBdgs)
    if value is None:
        return None
    value -= 1
    if 0 >= value:
        value = 0
        newResult = True
        if resetDescription is not None:
            doReset = True
            if 'lives' in resetDescription:
                lives = interpretVariableDescription(name, customDynamicsAPI, resetDescription['lives'], roleBdgs)
                if lives is not None:
                    lives -= 1
                    if 0 >= lives:
                        doReset = False
                        lives = 0
                    _, livesPath = getVariablePath(name, customDynamicsAPI, resourceDescription['lives'], roleBdgs)
                    customDynamicsAPI['setObjectProperty']((), livesPath, lives)
            if doReset:
                value = interpretVariableDescription(name, customDynamicsAPI, resetDescription['value'], roleBdgs)
    _, varPath = getVariablePath(name, customDynamicsAPI, resourceDescription, roleBdgs)
    customDynamicsAPI['setObjectProperty']((), varPath, value)
    if (previousResult is None) or (newResult is False):
        return newResult
    return previousResult
    
def accumulateResource(name, customDynamicsAPI, roleBdgs, previousResult, variableDescription, valueDescription, capacity, conditionFn, conditionVariableDescription, conditionReference):
    conditionVariableValue = interpretVariableDescription(name, customDynamicsAPI, conditionVariableDescription, roleBdgs)
    conditionReferenceValue = interpretVariableDescription(name, customDynamicsAPI, conditionReference, roleBdgs)
    newResult = previousResult
    if conditionFn(conditionVariableValue, conditionReferenceValue):
        previousAccumulatorValue = interpretVariableDescription(name, customDynamicsAPI, variableDescription, roleBdgs)
        _, accumulatorPath = getVariablePath(name, customDynamicsAPI, variableDescription, roleBdgs)
        increment = interpretVariableDescription(name, customDynamicsAPI, valueDescription, roleBdgs)
        numericAcc = True
        if isinstance(increment, str):
            numericAcc = False
        if previousAccumulatorValue is None:
            if numericAcc:
                previousAccumulatorValue = 0
            else:
                previousAccumulatorValue = {}
            customDynamicsAPI['setObjectProperty']((), accumulatorPath, previousAccumulatorValue)
        if numericAcc:
            newAccumulatorValue = previousAccumulatorValue + increment
        else:
            newAccumulatorValue = previousAccumulatorValue
            newAccumulatorValue[increment] = True
        customDynamicsAPI['setObjectProperty']((), accumulatorPath, newAccumulatorValue)
        capacity = interpretVariableDescription(name, customDynamicsAPI, capacity, roleBdgs)
        if capacity is not None:
            if numericAcc:
                newResult = (capacity <= newAccumulatorValue)
            else:
                newResult = (capacity <= len(newAccumulatorValue))
            if (previousResult is not None) or (newResult is True):
                newResult = previousResult
    return newResult

def processCompletion(name, customDynamicsAPI, completionDescription, roleBdgs):
    description = {k: interpretVariableDescription(name, customDynamicsAPI, v, roleBdgs) for k, v in completionDescription.items()}
    description['patient'] = roleBdgs['relatum-type']
    description['process'] = completionDescription['process']
    customDynamicsAPI['concludeProcess'](description)

def clearVariable(name, customDynamicsAPI, description, roleBdgs):
    value = interpretVariableDescription(name, customDynamicsAPI, description[:-1], roleBdgs)
    last = interpretVariableDescription(name, customDynamicsAPI, description[-1], roleBdgs)
    if isinstance(value.get(last), bool):
        if last in value:
            value.pop(last)
            _, varPath = getVariablePath(name, customDynamicsAPI, description[:-1], roleBdgs)
            customDynamicsAPI['setObjectProperty']((), varPath, value)

def customDynamic(name, customDynamicsAPI, progressDescription, resourceDescription, completionDescription):
    isProgressing, roleBdgs = checkProgress(name, customDynamicsAPI, progressDescription)
    if isProgressing:
        resourceTrigger = None
        if (resourceDescription['resource'] is None) and (resourceDescription['accumulators'] is None):
            resourceTrigger = True
        if (resourceDescription['resource'] is not None):
            resourceTrigger = decrementResource(name, customDynamicsAPI, roleBdgs, resourceTrigger, resourceDescription['resource'], resourceDescription['reset'])
        if (resourceDescription['accumulators'] is not None) and ((resourceTrigger is not None) or (resourceDescription['resource'] is None)):
            for acc in resourceDescription['accumulators']:
                variableDescription = acc['variable']
                capacity = acc.get('capacity', None)
                valueDescription = acc.get('value', None)
                conditionFn = lambda x, y: True
                conditionVariableDescription = None
                conditionReference = None
                if 'conditional' in acc:
                    mode = acc['conditional'].get('mode', 'dummy')
                    conditionFn = conditionalModes[mode]
                    conditionVariableDescription = acc['conditional'].get('variable')
                    conditionReference = acc['conditional'].get('reference')
                resourceTrigger = accumulateResource(name, customDynamicsAPI, roleBdgs, resourceTrigger, variableDescription, valueDescription, capacity, conditionFn, conditionVariableDescription, conditionReference)
        if resourceTrigger:
            if completionDescription['completion'] is not None:
                processCompletion(name, customDynamicsAPI, completionDescription['completion'], roleBdgs)
            if completionDescription['clear'] is not None:
                for clearDescription in completionDescription['clear']:
                    clearVariable(name, customDynamicsAPI, clearDescription, roleBdgs)

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
        updateFn = lambda name, customDynamicsAPI, progressDescription=progressDescription, resourceDescription=resourceDescription, completionDescription=completionDescription: customDynamic(name, customDynamicsAPI, progressDescription, resourceDescription, completionDescription)
        retq.append([disposition, updateFn])
    return retq

