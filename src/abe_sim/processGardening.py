import copy
import math
import numpy

import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

from abe_sim.world import getDictionaryEntry, stubbornTry

baseFwdOffset = (1.5, 0, 0)

def rotationMatrixToQuaternion(m):
    m00,m01,m02 = m[0]
    m10,m11,m12 = m[1]
    m20,m21,m22 = m[2]
    tr = m00 + m11 + m22
    if (tr > 0):
        S = math.sqrt(tr+1.0) * 2; # S=4*qw 
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S 
        qz = (m10 - m01) / S 
    elif ((m00 > m11) and (m00 > m22)): 
        S = math.sqrt(1.0 + m00 - m11 - m22) * 2 # S=4*qx 
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif (m11 > m22):
        S = math.sqrt(1.0 + m11 - m00 - m22) * 2 # S=4*qy
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else: 
        S = math.sqrt(1.0 + m22 - m00 - m11) * 2 # S=4*qz
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return [qx/n, qy/n, qz/n, qw/n]

def quatFromVecPairs(pA, pB):
    vAI, vAO = pA
    vBI, vBO = pB
    vCI = numpy.cross(vAI, vBI)
    vCO = numpy.cross(vAO, vBO)
    RI = numpy.transpose(numpy.array([vAI, vBI, vCI]))
    RORI = numpy.transpose(numpy.array([vAO, vBO, vCO]))
    #print(RI, RORI)
    RO = numpy.matmul(RORI, numpy.linalg.inv(RI))
    retq = rotationMatrixToQuaternion(RO)
    return retq

def pointCloseness(a, b, t):
    d = [x-y for x,y in zip(a, b)]
    return bool(t > numpy.dot(d, d))

def quaternionCloseness(a, b, t):
    ax = stubbornTry(lambda : pybullet.rotateVector(a, [1,0,0]))
    ay = stubbornTry(lambda : pybullet.rotateVector(a, [0,1,0]))
    bx = stubbornTry(lambda : pybullet.rotateVector(b, [1,0,0]))
    by = stubbornTry(lambda : pybullet.rotateVector(b, [0,1,0]))
    return bool((t < numpy.dot(ax, bx)) or (t < numpy.dot(ay, by)))

def getBaseEFPosition(customDynamicsAPI, name):
    baseLink = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'mobileBaseLink'))
    basePosition = customDynamicsAPI['getObjectProperty']((name, baseLink), 'position')
    baseOrientation = customDynamicsAPI['getObjectProperty']((name, baseLink), 'orientation')
    baseEFPosition, _ = customDynamicsAPI['objectPoseRelativeToWorld'](basePosition, baseOrientation, baseFwdOffset, (0,0,0,1))
    return baseEFPosition

def _copyForModeSwitch(description):
    retq = copy.deepcopy(description)
    if 'goal' in retq:
        retq.pop('goal')
    if 'process' in retq:
        retq.pop('process')
    return retq

def _makeProcess(description, process, numerics=None, target=None):
    if numerics is None:
       numerics = {}
    descriptionC = _copyForModeSwitch(description)
    descriptionC['process'] = process
    retq = {'type': 'P', 'description': descriptionC, 'children': [], 'numerics': copy.deepcopy(numerics)}
    if target is not None:
        retq['target'] = target
    return retq

def _makeGoal(description, goal, numerics=None):
    if numerics is None:
       numerics = {}
    descriptionC = _copyForModeSwitch(description)
    descriptionC['goal'] = goal
    return {'type': 'G', 'description': descriptionC, 'children': [], 'previousStatus': None, 'numerics': copy.deepcopy(numerics)}

def getUprightOrientation(customDynamicsAPI, item, itemOrientation, pouringAxis):
    return customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'upright'), [0,0,0,1])
    
def getTippedOrientation(customDynamicsAPI, item, itemOrientation, pouringAxis):
    return customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'tipped'), [-0.707,0,0,0.707])
    
def getComponentUnderHandHeight(customDynamicsAPI, name, hand, handPosition):
    handLink = getHandLink(customDynamicsAPI, name, hand)
    held = getHeld(customDynamicsAPI, name, hand)
    aabbs = [customDynamicsAPI['getObjectProperty']((x,), 'aabb') for x in held]
    #aabb = [[handPosition[0]-0.1, handPosition[1]-0.1, -0.1], [handPosition[0]+0.1, handPosition[1]+0.1, handPosition[2]]]
    aabb = customDynamicsAPI['getObjectProperty']((name, handLink), 'aabb')
    aabb = [list(aabb[0]), list(aabb[1])]
    for e in aabbs:
        for k, v in enumerate(aabb[0]):
            if e[0][k] < v:
                aabb[0][k] = e[0][k]
            if e[1][k] > v:
                aabb[1][k] = e[1][k]
    aabb[0][2] = -0.1
    overlaps = [x for x in customDynamicsAPI['checkOverlap'](aabb) if (x != name) and (x not in held) and customDynamicsAPI['getObjectProperty']((x,), ('fn', 'canContain'), False)]
    maxO = None
    for overlap in overlaps:
        aabbsOverlap = [(x, customDynamicsAPI['getObjectProperty']((overlap, x), 'aabb')) for x in customDynamicsAPI['getObjectProperty']((overlap,), ('fn', 'containment', 'links'), [])]
        valid = [(x[1][1][2], overlap, x[0]) for x in aabbsOverlap if bool(x[1][1][2] < handPosition[2])]
        if 0 < len(valid):
            maxC = max(valid)
            if (maxO is None) or bool(maxO[0] < maxC[0]):
                maxO = maxC
    if maxO is not None:
        height = getComponentHeight(customDynamicsAPI, maxO[1], maxO[2]) + maxO[0]
    else:
        height = 1.0
    return height

def getEntryHeight(customDynamicsAPI, name, description, numerics):
    entryHeight = numerics.get('entryHeight', None)
    if entryHeight is not None:
        return entryHeight
    hand = description['hand']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    positionHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    entryHeight = getComponentUnderHandHeight(customDynamicsAPI, name, hand, positionHand)
    item = description.get('item', None)
    destination = description.get('destination', None)
    if item is not None:
        container, component = getContainerComponent(customDynamicsAPI, item)
        if container is None:
            return entryHeight
    elif destination is not None:
        container, component = destination
    if (item is not None) or (destination is not None):
        if component is not None:
            componentHeight = getComponentHeight(customDynamicsAPI, container, component)
            componentBase = customDynamicsAPI['getObjectProperty']((container, component), 'aabb')[1][2]
        else:
            componentBase = customDynamicsAPI['getObjectProperty']((item,), 'aabb')[1][2]
            componentHeight = 0.2
        entryHeight = componentHeight + componentBase
    return entryHeight

def getAxisInWorld(customDynamicsAPI, item, axis):
    axisInObject = customDynamicsAPI['getObjectProperty']((item,), axis, None)
    if axisInObject is None:
        return None
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    return stubbornTry(lambda: pybullet.rotateVector(itemOrientation, axisInObject))

def getObjectInHand(customDynamicsAPI, name, hand, item, itemPoint=None):
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    itemPosition = customDynamicsAPI['getObjectProperty']((item,), 'position')
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    if itemPoint is not None:
        itemPosition, itemOrientation = customDynamicsAPI['objectPoseRelativeToWorld'](itemPosition, itemOrientation, itemPoint, [0,0,0,1])
    return customDynamicsAPI['objectPoseRelativeToObject'](handPosition, handOrientation, itemPosition, itemOrientation)

def getItemEntryHeight(handEntryHeight, positionInHand, orientationHand):
    positionInWorld = stubbornTry(lambda : pybullet.rotateVector(orientationHand, positionInHand))
    return handEntryHeight + positionInWorld[2]

def checkIdentifierAboveLocation(customDynamicsAPI, identifier, node):
    locationP = node['numerics'].get('position', None)
    locationQ = node['numerics'].get('orientation', None)
    position = customDynamicsAPI['getObjectProperty'](identifier, 'position')
    orientation = customDynamicsAPI['getObjectProperty'](identifier, 'orientation')
    previous = node.get('previousStatus', False)
    posTh = 0.0001
    ornTh = 0.98
    if previous:
        posTh = 0.0004
        ornTh = 0.94
    d = [x-y for x,y in zip(position[:2], locationP[:2])]
    abovePosition = bool(posTh > numpy.dot(d,d))
    aligned = True
    if locationQ is not None:
        aligned = quaternionCloseness(orientation, locationQ, ornTh)
    node['previousStatus'] = abovePosition and aligned
    return node['previousStatus']

def _alwaysFalse(customDynamicsAPI, name, description, node):
    return False

def _checkNear(customDynamicsAPI, name, description, node):
    relatum = description['relatum']
    goalPosition = node['numerics'].get('position', None)
    previous = node.get('previousStatus', False)
    return checkNear(customDynamicsAPI, name, relatum, previous, position=goalPosition)

def _checkParkedArm(customDynamicsAPI, name, description, node):
    hand = description['hand']
    previous = node.get('previousStatus', False)
    return checkParked(customDynamicsAPI, name, hand, previous)

def _checkArmNearItemHandle(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    handleLink = getHandleLink(customDynamicsAPI, item)
    aligned = True
    handleAxisInWorld = getAxisInWorld(customDynamicsAPI, item, ('fn', 'grasping', 'axis', handleLink))
    maxDistance = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'grasping', 'graspingActivationRadius', hand), 0.5)
    ornTh = 0.99
    if node.get('previousStatus', False):
        maxDistance = 1.5*maxDistance
        ornTh = 0.95
    if handleAxisInWorld is not None:
        handleYaw = math.atan2(handleAxisInWorld[1], handleAxisInWorld[0])
        targetOrientation = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,handleYaw)))
        aligned = quaternionCloseness(targetOrientation, handOrientation, ornTh)
    return aligned and (0 < len(customDynamicsAPI['checkClosestPoints']((name, handLink), (item, handleLink), maxDistance=maxDistance)))

def _checkGrasped(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    return checkGrasped(customDynamicsAPI, name, hand, item)

def _checkUngrasped(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    return not checkGrasped(customDynamicsAPI, name, hand, item)

def _checkPickedItem(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    previous = node.get('previousStatus', False)
    grasped = checkGrasped(customDynamicsAPI, name, hand, item)
    if previous:
        return grasped
    else:
        return grasped and checkParked(customDynamicsAPI, name, hand, False)

def _checkPlacedItem(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    container = description['container']
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    return (not checkGrasped(customDynamicsAPI, name, None, item)) and checkItemInContainer(customDynamicsAPI, name, item, container) and parked

def _checkLoweredItemAt(customDynamicsAPI, name, description, node):
    item = description['item']
    container = description['container']
    return checkItemInContainer(customDynamicsAPI, name, item, container)
    
def _checkItemAboveLocation(customDynamicsAPI, name, description, node):
    item = description['item']
    return checkIdentifierAboveLocation(customDynamicsAPI, (item,), node)
    
def _checkLiftedItemToEntry(customDynamicsAPI, name, description, node):
    hand = description['hand']
    previous = node.get('previousStatus', False)
    handLink = getHandLink(customDynamicsAPI, name, hand)
    positionHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    entryHeight = getEntryHeight(customDynamicsAPI, name, {'destination': [description['container'], node['numerics'].get('component', None)]}, node['numerics'])
    posTh = 0.01
    if previous:
        posTh = 0.02
    node['previousStatus'] = bool(posTh > abs(positionHand[2] - entryHeight))
    return node['previousStatus']

def _checkTransferredAndStored(customDynamicsAPI, name, description, node):
    storage = description['storage']
    pouredType = description['pouredType']
    item = description['item']
    hand = description['hand']
    container = description['container']
    amount = description['amount']
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    return checkTransferred(customDynamicsAPI, name, item, pouredType, amount, container) and (not checkGrasped(customDynamicsAPI, name, None, item)) and checkItemInContainer(customDynamicsAPI, name, item, storage) and parked
    
def _checkTransferredAndUprighted(customDynamicsAPI, name, description, node):
    pouredType = description['pouredType']
    item = description['item']
    hand = description['hand']
    container = description['container']
    amount = description['amount']
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    pouringAxis = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'axis'), [1,0,0])
    ornTh = 0.1
    if node.get('previousStatus', False):
        ornTh = 0.4
    upright = checkUpright(itemOrientation, pouringAxis, th=ornTh)
    return checkTransferred(customDynamicsAPI, name, item, pouredType, amount, container) and upright

def _checkTransferredContents(customDynamicsAPI, name, description, node):
    pouredType = description['pouredType']
    item = description['item']
    hand = description['hand']
    container = description['container']
    amount = description['amount']
    return checkTransferred(customDynamicsAPI, name, item, pouredType, amount, container)

def _checkTippedItemAboveLocation(customDynamicsAPI, name, description, node):
    item = description['item']
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    previous = node.get('previousStatus', False)
    ornTh = 0.99
    if previous:
        ornTh = 0.95
    return quaternionCloseness(itemOrientation, node['numerics'].get('tippedOrientation', None), ornTh)

def _checkPreparedItemForTipping(customDynamicsAPI, name, description, node):
    item = description['item']
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    previous = node.get('previousStatus', False)
    if not previous:
        previous = quaternionCloseness(itemOrientation, node['numerics'].get('entryOrientation', None), 0.99)
    return previous    

def _checkLiftedItemToPouringEntry(customDynamicsAPI, name, description, node):
    hand = description['hand']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    positionHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    previous = node.get('previousStatus', False)
    dTh = 0.01
    if previous:
        dTh = 0.03
    return dTh > abs(node['numerics'].get('entryHeight') - positionHand[2])

def _checkConstraintFollowed(customDynamicsAPI, name, description, node): # constraintConjunctions, hand, isTop | waypoints, tolerances, entities
    def _equalp(a, b, t):
        d = [x-y for x,y in zip(a,b)]
        return t > numpy.dot(d,d)
    def _equalz(a, b, t):
        return t > abs(a-b)
    def _aligned(a, b, t):
        return t < numpy.dot(a,b)
    def _orthogonal(a, b, t):
        return t > abs(numpy.dot(a,b))
    def _interpret(entity, mode):
        retq = entity
        if mode in ['equalxy']:
            retq = retq[:2]
        elif (mode in ['equalz']) and (type(retq) in [list, tuple]):
            retq = retq[2]
        return retq
    constraintConjunctions = description['constraintConjunctions']
    if 0 == len(constraintConjunctions):
        return True
    hand = description['hand']
    fn = {'equalp': _equalp, 'equalxy': _equalp, 'equalz': _equalz, 'equalq': quaternionCloseness, 'aligned': _aligned, 'orthogonal': _orthogonal}
    hand = description['hand']
    isTop = description['isTop']
    waypoints = node['numerics']['waypoints']
    tolerances = node['numerics']['tolerances']
    entities = node['numerics']['entities']
    conjunction = constraintConjunctions[0]
    toleranceChoices = tolerances[0]
    previous = node.get('previousStatus', False)
    if previous:
        thresholds = [x[1] for x in toleranceChoices]
    else:
        thresholds = [x[0] for x in toleranceChoices]
    for constraint, threshold in zip(conjunction, thresholds):
        mode, entityA, entityB = constraint
        if not fn[mode](_interpret(entities[entityA], mode), _interpret(entities[entityB], mode), threshold):
            return False
    return True

def checkMixed(customDynamicsAPI, name, container, mixedType):
    # TODO: needs a smarter check: should be, mixable into mixedType
    notDones = [x for x in getContents(customDynamicsAPI, container) if (mixedType != customDynamicsAPI['getObjectProperty']((x,), 'type')) and (customDynamicsAPI['getObjectProperty']((x,), ('fn', 'mixable')))]
    return 0 == len(notDones)
    
def _checkMixedAndStored(customDynamicsAPI, name, description, node):
    storage = description['storage']
    tool = description['tool']
    mixedType = description['mixedType']
    hand = description['hand']
    container = description['container']
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    return checkMixed(customDynamicsAPI, name, container, mixedType) and (not checkGrasped(customDynamicsAPI, name, None, tool)) and checkItemInContainer(customDynamicsAPI, name, tool, storage) and parked

def _checkMixedAndUprighted(customDynamicsAPI, name, description, node):
    tool = description['tool']
    toolLink = description['toolLink']
    mixedType = description['mixedType']
    hand = description['hand']
    container = description['container']
    toolOrientation = customDynamicsAPI['getObjectProperty']((tool,), 'orientation')
    mixingAxis = customDynamicsAPI['getObjectProperty']((tool,), ('fn', 'mixing', 'axis', toolLink), [1,0,0])
    ornTh = 0.1
    if node.get('previousStatus', False):
        ornTh = 0.4
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    upright = checkUpright(toolOrientation, mixingAxis, th=ornTh) and checkUpright(handQ, [1,0,0], th=ornTh)
    #parked = node.get('parked', False)
    #parked = checkParked(customDynamicsAPI, name, hand, parked)
    #node['parked'] = parked
    return checkMixed(customDynamicsAPI, name, container, mixedType) and upright

def _checkMixed(customDynamicsAPI, name, description, node):
    mixedType = description['mixedType']
    container = description['container']
    return checkMixed(customDynamicsAPI, name, container, mixedType)

def _checkLinedAndParked(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    lining = description['lining']
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    return (not checkGrasped(customDynamicsAPI, name, None, lining)) and checkItemInContainer(customDynamicsAPI, name, lining, item) and parked

def _checkLined(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    lining = description['lining']
    return checkItemInContainer(customDynamicsAPI, name, lining, item)

def containsIngredients(customDynamicsAPI, item, ingredientTypes):
    containedItems = {}
    for x in getContents(customDynamicsAPI, item):
        t = customDynamicsAPI['getObjectProperty']((x,), 'type')
        if t not in containedItems:
            containedItems[t] = []
        containedItems[t].append(x)
    retq = []
    for k, v in ingredientTypes.items():
        if (k not in containedItems) or (v > len(containedItems[k])):
            return False, []
        retq = retq + sorted(containedItems[k])[:v]
    return True, retq

def containsShapes(customDynamicsAPI, item, shapeType):
    retq = [x for x in getContents(customDynamicsAPI, item, heightBonus=0.5) if shapeType == customDynamicsAPI['getObjectProperty']((x,), 'type')]
    return 0 != len(retq), retq

def isHoldingObjectOfType(customDynamicsAPI, name, hand, qtype):
    heldOfType = [x for x in getHeld(customDynamicsAPI, name, hand) if qtype == customDynamicsAPI['getObjectProperty']((x,), 'type')]
    holding = (0 < len(heldOfType))
    retq = None
    if holding:
        retq = sorted(heldOfType)[0]
    return holding, retq

def _checkShapedAndParked(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    destination = description.get('destination', None)
    shapedType = description.get('shapedType', None)
    ingredientTypes = description.get('ingredientTypes', [])
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    return (not containsIngredients(customDynamicsAPI, item, ingredientTypes)[0]) and (not containsShapes(customDynamicsAPI, item, shapedType)[0]) and (not isHoldingObjectOfType(customDynamicsAPI, name, hand, shapedType)[0]) and parked

def _checkSprinkled(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    shaker = description.get('shaker', None)
    storage = description.get('storage', None)
    sprinkledType = description.get('sprinkledType', None)
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    return checkSprinkled(customDynamicsAPI, name, item, sprinkledType)[0] and (not checkGrasped(customDynamicsAPI, name, None, shaker)) and checkItemInContainer(customDynamicsAPI, name, shaker, storage) and parked

def _checkBaked(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    oven = description.get('oven', None)
    destination = description.get('destination', None)
    bakedType = description.get('bakedType', None)
    baked = checkBakedContents(customDynamicsAPI, item, bakedType)
    container, component = getContainerComponent(customDynamicsAPI, item)
    atOven = (container == oven)
    atDestination = (container == destination)
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    if atOven:
        node['sourceContainer'] = oven
        node['sourceComponent'] = component
    return baked and atDestination and parked

def _checkCutItem(customDynamicsAPI, name, description, node):
    item = description['item']
    tool = description['tool']
    hand = description['hand']
    storage = description['storage']
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    return (not customDynamicsAPI['getObjectProperty']((item,), ('fn', 'cuttable'), False)) and (not checkGrasped(customDynamicsAPI, name, None, tool)) and checkItemInContainer(customDynamicsAPI, name, tool, storage) and parked

def _checkArmTriggeredPortalHandle(customDynamicsAPI, name, description, node):
    container = description.get('container', None)
    component = description.get('component', None)
    hand = description.get('hand', None)
    action = description.get('action', None)
    if (container is not None) and (action is not None):
        handle, door = getHandleAndDoor(customDynamicsAPI, container, component)
        if handle is None: # No handle -- nothing to open/close
            return True
        actualAction = customDynamicsAPI['getObjectProperty']((container,), ('customStateVariables', 'clopening', 'action', door), None)
        if actualAction == action: # Already performing action -- no need to keep arm there
            return True
        doingAction = (action == customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'clopening', 'action', hand), None))
        hand = description.get('hand', None)
        handLink = getHandLink(customDynamicsAPI, name, hand)
        maxDistance = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'radius', door), 0.5)
        if not node.get('previousStatus', False):
            maxDistance = 0.8*maxDistance
        return doingAction and (0 < len(customDynamicsAPI['checkClosestPoints']((name, handLink), (container, handle), maxDistance=maxDistance)))
    return True

def _checkStoppedOpening(customDynamicsAPI, name, description, node):
    hand = description.get('hand', None)
    if hand is not None:
        return 'open' != customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'clopening', 'action', hand), None)
    else:
        clactions = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'clopening', 'action'), {})
        return not any(['open' == v for _, v in clactions.items()])

def _checkOpened(customDynamicsAPI, name, description, node):
    container = description.get('container', None)
    component = description.get('component', None)
    handle, door = getHandleAndDoor(customDynamicsAPI, container, component)
    if handle is not None:
        openMinThreshold = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'openMin', door), None)
        openMaxThreshold = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'openMax', door), None)
        jointValue = customDynamicsAPI['getObjectProperty']((container, door), 'jointPosition')
        return ((openMinThreshold is None) or bool(openMinThreshold < jointValue)) and ((openMaxThreshold is None) or bool(openMaxThreshold > jointValue))
    return True

def _checkClosed(customDynamicsAPI, name, description, node):
    container = description.get('container', None)
    component = description.get('component', None)
    handle, door = getHandleAndDoor(customDynamicsAPI, container, component)
    if handle is not None:
        closedMinThreshold = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'closedMin', door), None)
        closedMaxThreshold = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'closedMax', door), None)
        jointValue = customDynamicsAPI['getObjectProperty']((container, door), 'jointPosition')
        return ((closedMinThreshold is None) or bool(closedMinThreshold < jointValue)) and ((closedMaxThreshold is None) or bool(closedMaxThreshold > jointValue))
    return True

def _checkBroughtNear(customDynamicsAPI, name, description, node):
    trajector = description.get('trajector', None)
    hand = description.get('hand', None)
    relatum = description.get('relatum', None)
    previous = node.get('previousStatus', False)
    return checkGrasped(customDynamicsAPI, name, hand, trajector) and checkNear(customDynamicsAPI, name, relatum, previous)
    
def _checkFollowedWaypoints(customDynamicsAPI, name, description, node):
    hand = description.get('hand', None)
    waypoints = node.get('updateable', {}).get('waypoints', [])
    if 0 == len(waypoints):
        return True
    positionInHand = description.get('positionInHand', None)
    orientationInHand = description.get('orientationInHand', None)
    handLink = getHandLink(customDynamicsAPI, name, hand)
    positionHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    orientationHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    posInWorld, ornInWorld = customDynamicsAPI['objectPoseRelativeToWorld'](positionHand, orientationHand, positionInHand, orientationInHand)
    wp = waypoints[-1]
    posThreshold = 0.0001
    ornThreshold = 0.99
    if node.get('previousStatus', False):
        posThreshold = 0.0004
        ornThreshold = 0.95
    return pointCloseness(posInWorld, wp[0], posThreshold) and quaternionCloseness(ornInWorld, wp[1], ornThreshold)

def _emptyList(customDynamicsAPI, name, description, node):
    return []

def _getNearingConditions(customDynamicsAPI, name, description, node):
    return [_makeGoal({'relatum': description['relatum']}, 'alignedBaseYaw')]

def _getAligningBaseYawConditions(customDynamicsAPI, name, description, node):
    return [_makeGoal({}, 'baseCentered')]

def _getParkingArmConditions(customDynamicsAPI, name, description, node):
    hand = description['hand']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handP = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    handParkedP, handParkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
    entryHeight = getComponentUnderHandHeight(customDynamicsAPI, name, hand, handP)
    #For parking:
    #  we want P/U: hand parked
    #  We can achieve P/U by a lowering process that maintains xy/U: handXY at parked XY, hand parked orientation
    #  We can achieve xy/U by a movement process that maintains U/Z: hand parked orientation, handZ at entryHeight
    #  we can achieve U/Z by an uprighting process that maintains Z: handZ at entryHeight
    #  we can achieve Z by a lifting process
    conP = ['equalp', 'handP', 'handParkedP']
    tolP = [0.0001, 0.0004]
    conU = ['equalq', 'handQ', 'handParkedQ']
    tolU = [0.99, 0.95]
    conxy = ['equalxy', 'handP', 'handParkedP']
    tolxy = [0.0001, 0.0004]
    conZ = ['equalz', 'handP', 'entryHeight']
    tolZ = [0.01, 0.05]
    constraintConjunctions = [[conP, conU], [conxy, conU], [conU, conZ], [conZ]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpPU = [handParkedP, handParkedQ, None, None, None, None, None]
    wpxyU = [[handParkedP[0], handParkedP[1], entryHeight], handParkedQ, None, None, None, None, None]
    wpUZ = [[handP[0], handP[1], entryHeight], handParkedQ, None, None, None, None, None]
    wpZ = [[handP[0], handP[1], entryHeight], handQ, None, None, None, None, None]
    waypoints = [wpPU, wpxyU, wpUZ, wpZ]
    tolerances = [[tolP, tolU], [tolxy, tolU], [tolU, tolZ], [tolZ]]
    entities = {'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': entryHeight}
    return [_makeGoal({'hand': hand, 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                      numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]

def _getArmNearingItemHandleConditions(customDynamicsAPI, name, description, node):
    hand = description['hand']
    item = description['item']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handP = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    aabbItem = customDynamicsAPI['getObjectProperty']((item,), 'aabb')
    handleLink = getHandleLink(customDynamicsAPI, item)
    aabbHandLocal = customDynamicsAPI['getObjectProperty']((item, handleLink), 'localAABB')
    handlePosition = customDynamicsAPI['getObjectProperty']((item, handleLink), 'position')
    handleAxisInWorld = getAxisInWorld(customDynamicsAPI, item, ('fn', 'grasping', 'axis', handleLink))
    graspedQ = handQ
    if handleAxisInWorld is not None:
        handleYaw = math.atan2(handleAxisInWorld[1], handleAxisInWorld[0])
        graspedQ = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,handleYaw)))
    aboveHandleP = list(handlePosition)
    aboveHandleP[2] = aabbItem[1][2] - aabbHandLocal[0][2]
    entryHeight = getEntryHeight(customDynamicsAPI, name, description, node['numerics'])
    #For nearing item handle:
    #  we want h/o: hand above handle, oriented to handle
    #  We can achieve h/o by a lowering process that maintains xy/o: handXY at handleXY, oriented to handle
    #  We can achieve xy/o by an orienting process that maintains xy/z: handXY at handleXY, handZ at entryHeight
    #  we can achieve xy/z by an bringing process that maintains z: handZ at entryHeight
    #  we can achieve z by a lifting process
    conh = ['equalp', 'handP', 'aboveHandleP']
    tolh = [0.0001, 0.0004]
    cono = ['equalq', 'handQ', 'graspedQ']
    tolo = [0.99, 0.95]
    conxy = ['equalxy', 'handP', 'aboveHandleP']
    tolxy = [0.0001, 0.0004]
    conz = ['equalz', 'handP', 'entryHeight']
    tolz = [0.01, 0.05]
    constraintConjunctions = [[conh, cono], [conxy, cono], [conxy, conz], [conz]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpho = [aboveHandleP, graspedQ, None, None, None, None, None]
    wpxyo = [[aboveHandleP[0], aboveHandleP[1], entryHeight], graspedQ, None, None, None, None, None]
    wpxyz = [[aboveHandleP[0], aboveHandleP[1], entryHeight], handQ, None, None, None, None, None]
    wpz = [[handP[0], handP[1], entryHeight], handQ, None, None, None, None, None]
    waypoints = [wpho, wpxyo, wpxyz, wpz]
    tolerances = [[tolh, tolo], [tolxy, tolo], [tolxy, tolz], [tolz]]
    entities = {'handP': handP, 'handQ': handQ, 'aboveHandleP': aboveHandleP, 'graspedQ': graspedQ, 'entryHeight': entryHeight}
    return [_makeGoal({'hand': hand, 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                      numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]

def _getHoveringLocationConditions(customDynamicsAPI, name, description, node):
    return [_makeGoal({'hand': description['hand'], 'item': description['item']}, 'liftedToEntry',
                      numerics={'entryHeight': node['numerics'].get('entryHeight', None)})]

def _getGraspingConditions(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    freeHand = getFreeHand(customDynamicsAPI, name, hand)
    container, component = getContainerComponent(customDynamicsAPI, item)
    return [_makeGoal({'container': container, 'component': component, 'hand': freeHand}, 'opened'),
            _makeGoal({'hand': freeHand}, 'stoppedOpening'),
            _makeGoal({'relatum': item}, 'near'),
            _makeGoal({'hand': hand, 'item': item}, 'armNearItemHandle')]
    
def _getPickingItemConditions(customDynamicsAPI, name, description, node):
    return [_makeGoal({'hand': description['hand'], 'item': description['item']}, 'grasped'), _makeGoal({'hand': description['hand']}, 'parkedArm')]

def _getPlacingItemConditions(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    container = description['container']
    # TODO: close containers once done?
    return [_makeGoal({'container': container, 'hand': hand, 'item': item}, 'loweredItemAt'),
            _makeGoal({'hand': hand, 'item': item}, 'ungrasped'),
            _makeGoal({'hand': hand}, 'parkedArm')]

def _getLoweringItemConditions(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    container = description['container']
    freeHand = getFreeHand(customDynamicsAPI, name, hand)
    return [_makeGoal({'item': item, 'hand': hand}, 'pickedItem'),
            _makeGoal({'relatum': container}, 'near', numerics={'position': node['numerics'].get('position', None)}),
            _makeGoal({'container': container, 'component': node['numerics'].get('component', None), 'hand': freeHand}, 'opened'),
            _makeGoal({'container': container, 'item': item, 'hand': hand}, 'itemAboveLocation',
                      numerics={'component': node['numerics'].get('component', None), 'position': node['numerics'].get('position', None), 'orientation': node['numerics'].get('orientation', None), 'entryHeight': node['numerics'].get('entryHeight', None)})]
                      
def _getHoveringItemAboveLocationConditions(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    container = description['container']
    return [_makeGoal({'container': container, 'item': item, 'hand': hand}, 'liftedItemToEntry',
                      numerics={'component': node['numerics'].get('component', None), 'position': node['numerics'].get('position', None), 'orientation': node['numerics'].get('orientation', None), 'entryHeight': node['numerics'].get('entryHeight', None)})]

def _getTransferringAndStoringConditions(customDynamicsAPI, name, description, node):
    storage = description['storage']
    pouredType = description['pouredType']
    item = description['item']
    hand = description['hand']
    container = description['container']
    amount = description['amount']
    return [_makeGoal({'amount': amount, 'container': container, 'hand': hand, 'item': item, 'pouredType': pouredType}, 'transferredAndUprighted'),
            _makeGoal({'container': storage, 'hand': hand, 'item': item}, 'placedItem')]
            
def _getTransferringAndUprightingConditions(customDynamicsAPI, name, description, node):
    pouredType = description['pouredType']
    item = description['item']
    hand = description['hand']
    container = description['container']
    amount = description['amount']
    positionInHand, orientationInHand = node['numerics'].get('positionInHand', None), node['numerics'].get('orientationInHand', None)
    return [_makeGoal({'amount': amount, 'container': container, 'hand': hand, 'item': item, 'pouredType': pouredType}, 'transferredContents',
                      numerics={'positionInHand': positionInHand, 'orientationInHand': orientationInHand})]

def _getTransferringContentsConditions(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    container = description['container']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    orientationHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    containerPosition = customDynamicsAPI['getObjectProperty']((container,), 'position')
    containerOrientation = customDynamicsAPI['getObjectProperty']((container,), 'orientation')
    pouringAxis = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'axis'), [1,0,0])
    position = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'pouring', 'into', 'point'))
    position, _ = customDynamicsAPI['objectPoseRelativeToWorld'](containerPosition, containerOrientation, position, [0,0,0,1])
    tippedOrientation = getTippedOrientation(customDynamicsAPI, item, itemOrientation, pouringAxis)
    entryOrientation = getUprightOrientation(customDynamicsAPI, item, itemOrientation, pouringAxis)
    entryHeight = getEntryHeight(customDynamicsAPI, name, {'hand': hand, 'item': container}, {})
    positionInHand, orientationInHand = node['numerics'].get('positionInHand', None), node['numerics'].get('orientationInHand', None)
    position = [position[0], position[1], getItemEntryHeight(entryHeight, positionInHand, orientationHand)]
    return [_makeGoal({'hand': hand, 'item': item}, 'pickedItem'),
            _makeGoal({'relatum': container}, 'near'),
            _makeGoal({'hand': hand, 'item': item, 'relatum': container}, 'tippedItemAboveLocation',
                      numerics={'position': position, 'tippedOrientation': tippedOrientation, 'entryOrientation': entryOrientation, 'positionInHand': positionInHand, 'orientationInHand': orientationInHand, 'entryHeight': entryHeight})]

def _getTippingItemAboveLocationConditions(customDynamicsAPI, name, description, node):
    relatum = description['relatum']
    item = description['item']
    hand = description['hand']
    positionInHand, orientationInHand = node['numerics'].get('positionInHand', None), node['numerics'].get('orientationInHand', None)
    position = node['numerics'].get('position', None)
    entryOrientation = node['numerics'].get('entryOrientation', None)
    entryHeight = node['numerics'].get('entryHeight', None)
    return [_makeGoal({'hand': hand, 'item': item, 'relatum': relatum}, 'preparedItemForTipping',
                      numerics={'position': position, 'entryOrientation': entryOrientation, 'positionInHand': positionInHand, 'orientationInHand': orientationInHand, 'entryHeight': entryHeight})]
    
def _getPreparingItemForTippingConditions(customDynamicsAPI, name, description, node):
    relatum = description['relatum']
    item = description['item']
    hand = description['hand']
    positionInHand, orientationInHand = node['numerics'].get('positionInHand', None), node['numerics'].get('orientationInHand', None)
    entryOrientation = node['numerics'].get('entryOrientation', None)
    entryHeight = node['numerics'].get('entryHeight', None)
    return [_makeGoal({'hand': hand, 'item': item, 'relatum': relatum}, 'liftedItemToPouringEntry',
                      numerics={'entryOrientation': entryOrientation, 'positionInHand': positionInHand, 'orientationInHand': orientationInHand, 'entryHeight': entryHeight})]
                      
def _getConstraintFollowingConditions(customDynamicsAPI, name, description, node):
    constraintConjunctions = description['constraintConjunctions']
    hand = description['hand']
    if 1 >= len(constraintConjunctions):
        return []
    return [_makeGoal({'constraintConjunctions': constraintConjunctions[1:], 'hand': hand, 'isTop': False}, 'constraintFollowed',
                      numerics={'waypoints': node['numerics']['waypoints'][1:], 'tolerances': node['numerics']['tolerances'][1:], 'entities': node['numerics']['entities']})]

def _getMixingAndStoringConditions(customDynamicsAPI, name, description, node): # container, hand, mixedType, storage, tool, toolLink |
    storage = description['storage']
    mixedType = description['mixedType']
    toolLink = description['toolLink']
    tool = description['tool']
    hand = description['hand']
    container = description['container']
    return [_makeGoal({'container': container, 'hand': hand, 'tool': tool, 'toolLink': toolLink, 'mixedType': mixedType}, 'mixedAndUprighted'),
            _makeGoal({'container': storage, 'hand': hand, 'item': tool}, 'placedItem')]

def _getMixingAndUprightingConditions(customDynamicsAPI, name, description, node): # container, hand, mixedType, tool, toolLink |
    mixedType = description['mixedType']
    toolLink = description['toolLink']
    tool = description['tool']
    hand = description['hand']
    container = description['container']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handP = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    toolP = customDynamicsAPI['getObjectProperty']((tool,), 'position')
    toolQ = customDynamicsAPI['getObjectProperty']((tool,), 'orientation')
    toolPositionInHand, toolOrientationInHand = customDynamicsAPI['objectPoseRelativeToObject'](handP, handQ, toolP, toolQ)
    handParkedP, handParkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
    entryHeight = getEntryHeight(customDynamicsAPI, name, {'hand': hand, 'item': container}, {})
    mixPointInTool = customDynamicsAPI['getObjectProperty']((tool,), ('fn', 'mixing', 'mixPoint', toolLink), [0,0,0])
    mixPointInHand = customDynamicsAPI['objectPoseRelativeToWorld'](toolPositionInHand, toolOrientationInHand, mixPointInTool, [0,0,0,1])[0]
    mixPoint = customDynamicsAPI['objectPoseRelativeToWorld'](handP, handQ, mixPointInHand, [0,0,0,1])[0]
    containerP = customDynamicsAPI['getObjectProperty']((container,), 'position')
    containerQ = customDynamicsAPI['getObjectProperty']((container,), 'orientation')
    containerEntryInContainer = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'pouring', 'into', 'point'))
    containerEntry, _ = customDynamicsAPI['objectPoseRelativeToWorld'](containerP, containerQ, containerEntryInContainer, [0,0,0,1])
    entryHeightMixPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, mixPointInHand))[2]
    #For uprighting:
    #  we want P/U: hand parked
    #  We can achieve P/U by a lowering process that maintains xy/U: handXY at parked XY, hand parked orientation
    #  We can achieve xy/U by a movement process that maintains U/Z: hand parked orientation, handZ at entryHeight
    #  We can achieve U/Z by a dummy process that maintains U/Z/XY: mixpointXY at containerEntryXY, handZ at entryHeight, hand parked orientation
    #  we can achieve U/Z/XY by an uprighting process that maintains Z/XY: mixpointXY at containerEntryXY, handZ at entryHeight
    #  we can achieve Z/XY by a lifting process that maintains XY/T: mixpointXY at containerEntryXY, mixer tipped
    conP = ['equalp', 'handP', 'handParkedP']
    tolP = [0.01, 0.02]
    conU = ['equalq', 'handQ', 'handParkedQ']
    tolU = [0.99, 0.95]
    conxy = ['equalxy', 'handP', 'handParkedP']
    tolxy = [0.01, 0.05]
    conZ = ['equalz', 'handP', 'entryHeight']
    tolZ = [0.01, 0.05]
    conXY = ['equalxy', 'mixPoint', 'containerEntry']
    tolXY = [0.01, 0.05]
    constraintConjunctions = [[conP, conU], [conxy, conU], [conU, conZ], [conU, conZ, conXY], [conZ, conXY]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpPU = [handParkedP, handParkedQ, None, None, None, None, None]
    wpxyU = [[handParkedP[0], handParkedP[1], entryHeight], handParkedQ, None, None, None, None, None]
    wpUZ = [[containerEntry[0], containerEntry[1], entryHeightMixPoint], handParkedQ, None, mixPointInHand, None, None, None]
    wpUZXY = wpUZ
    wpZXY = [[containerEntry[0], containerEntry[1], entryHeightMixPoint], toolQ, None, mixPointInHand, toolOrientationInHand, None, None]
    waypoints = [wpPU, wpxyU, wpUZ, wpUZXY, wpZXY]
    tolerances = [[tolP, tolU], [tolxy, tolU], [tolU, tolZ], [tolU, tolZ, tolXY], [tolZ, tolXY]]
    entities = {'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': entryHeight, 'mixPoint': mixPoint, 'containerEntry': containerEntry}
    return [_makeGoal({'container': container, 'hand': hand, 'tool': tool, 'toolLink': toolLink, 'mixedType': mixedType}, 'mixed'),
            _makeGoal({'hand': hand, 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                      numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]

def _getMixingConditions(customDynamicsAPI, name, description, node): # container, hand, mixedType, tool, toolLink |
    mixedType = description['mixedType']
    toolLink = description['toolLink']
    tool = description['tool']
    hand = description['hand']
    container = description['container']
    down = customDynamicsAPI['getDown']()
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handP = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    toolP = customDynamicsAPI['getObjectProperty']((tool,), 'position')
    toolQ = customDynamicsAPI['getObjectProperty']((tool,), 'orientation')
    toolPositionInHand, toolOrientationInHand = customDynamicsAPI['objectPoseRelativeToObject'](handP, handQ, toolP, toolQ)
    handParkedP, handParkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
    entryHeight = getEntryHeight(customDynamicsAPI, name, {'hand': hand, 'item': container}, {})
    mixAxisInTool = customDynamicsAPI['getObjectProperty']((tool,), ('fn', 'mixing', 'axis', toolLink), [1,0,0])
    mixAxis = stubbornTry(lambda : pybullet.rotateVector(toolQ, mixAxisInTool))
    mixAxisInHand = stubbornTry(lambda : pybullet.rotateVector(toolOrientationInHand, mixAxisInTool))
    mixPointInTool = customDynamicsAPI['getObjectProperty']((tool,), ('fn', 'mixing', 'mixPoint', toolLink), [0,0,0])
    mixPointInHand = customDynamicsAPI['objectPoseRelativeToWorld'](toolPositionInHand, toolOrientationInHand, mixPointInTool, [0,0,0,1])[0]
    mixPoint = customDynamicsAPI['objectPoseRelativeToWorld'](handP, handQ, mixPointInHand, [0,0,0,1])[0]
    containerP = customDynamicsAPI['getObjectProperty']((container,), 'position')
    containerQ = customDynamicsAPI['getObjectProperty']((container,), 'orientation')
    containerEntryInContainer = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'pouring', 'into', 'point'))
    containerEntry, _ = customDynamicsAPI['objectPoseRelativeToWorld'](containerP, containerQ, containerEntryInContainer, [0,0,0,1])
    entryHeightMixPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, mixPointInHand))[2]
    if tool in getHeld(customDynamicsAPI, name, hand):
        fwdInHand = [1,0,0]
        facingYaw = getFacingYaw(customDynamicsAPI, container, containerP)
        fwd = [math.cos(facingYaw), math.sin(facingYaw),0]
        #axis = numpy.cross(mixAxisInHand, down)
        #angle = math.acos(numpy.dot(mixAxisInHand, down))
        tippedOrientation = quatFromVecPairs((fwdInHand, fwd), (mixAxisInHand, down))
    else:
        tippedOrientation = [0,0,0,1]
    #tippedOrientation = customDynamicsAPI['getObjectProperty']((tool,), ('fn', 'mixing', 'tipped'), [-0.707,0,0,0.707])
    #For mixing:
    #  we want pc/ad: mixpoint at containerEntry and mixaxis down
    #  we can achieve pc/ad by a lowering process that maintains ad/xy: mixaxis down and mixpointXY at containerEntryXY
    #  we can achieve ad/xy by a tipping process that maintains xy/ez: mixpointXY at containerEntryXY and handZ at entryHeight
    #  we can achieve xy/ez by a bringing process that maintains ez/po: handZ at entryHeight and handOrientation at parkedOrientation
    #  we can achieve ez/po by a lifting process that maintains po: handOrientation at parkedOrientation
    conpc = ['equalp', 'mixPoint', 'containerEntry']
    tolpc = [0.0001, 0.0004]
    conad = ['aligned', 'mixAxis', 'down']
    tolad = [0.99, 0.9]
    conxy = ['equalxy', 'mixPoint', 'containerEntry']
    tolxy = [0.0001, 0.0004]
    conez = ['equalz', 'handP', 'entryHeight']
    tolez = [0.0001, 0.0004]
    conpo = ['equalq', 'handQ', 'handParkedQ']
    tolpo = [0.99, 0.9]
    constraintConjunctions = [[conpc, conad], [conad, conxy], [conxy, conez], [conez, conpo]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    #wppcad = [containerEntry, tippedOrientation, None, mixPointInHand, toolOrientationInHand, None, None]
    #wpadxy = [[containerEntry[0], containerEntry[1], entryHeightMixPoint], tippedOrientation, None, mixPointInHand, toolOrientationInHand, None, None]
    wppcad = [containerEntry, tippedOrientation, None, mixPointInHand, None, None, None]
    wpadxy = [[containerEntry[0], containerEntry[1], entryHeightMixPoint], tippedOrientation, None, mixPointInHand, None, None, None]
    wpxyez = [[containerEntry[0], containerEntry[1], entryHeightMixPoint], handParkedQ, None, mixPointInHand, None, None, None]
    wpezpo = [[handParkedP[0], handParkedP[1], entryHeight], handParkedQ, None, None, None, None, None]
    waypoints = [wppcad, wpadxy, wpxyez, wpezpo]
    tolerances = [[tolpc, tolad], [tolad, tolxy], [tolxy, tolez], [tolez, tolpo]]
    entities = {'down': down, 'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': entryHeight, 'mixPoint': mixPoint, 'containerEntry': containerEntry, 'mixAxis': mixAxis}
    return [_makeGoal({'hand': hand, 'item': tool}, 'pickedItem'),
            _makeGoal({'relatum': container}, 'near'),
            _makeGoal({'hand': hand, 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                      numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]

def _getLiningAndParkingConditions(customDynamicsAPI, name, description, node):
    lining = description['lining']
    hand = description['hand']
    item = description['item']
    #source = description.get('sourceContainer', None)
    #sourcePart = description.get('sourceComponent', None)
    return [_makeGoal({'lining': lining, 'item': item, 'hand': hand}, 'lined'),
            _makeGoal({'item': lining, 'hand': hand}, 'ungrasped'),
            _makeGoal({'hand': hand}, 'parkedArm')]

def _getLiningConditions(customDynamicsAPI, name, description, node):
    lining = description['lining']
    hand = description['hand']
    item = description['item']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handP = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    linP = customDynamicsAPI['getObjectProperty']((lining,), 'position')
    linQ = customDynamicsAPI['getObjectProperty']((lining,), 'orientation')
    itemP = customDynamicsAPI['getObjectProperty']((item,), 'position')
    itemQ = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    linPositionInHand, linOrientationInHand = customDynamicsAPI['objectPoseRelativeToObject'](handP, handQ, linP, linQ)
    handParkedP, handParkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
    entryHeight = getEntryHeight(customDynamicsAPI, name, {'hand': hand, 'item': item}, {})
    linAxisInLin = customDynamicsAPI['getObjectProperty']((lining,), ('fn', 'lining', 'axis'), [1,0,0])
    linAxis = stubbornTry(lambda : pybullet.rotateVector(linQ, linAxisInLin))
    itemAxisInItem = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'lining', 'axis'), [1,0,0])
    itemAxis = stubbornTry(lambda : pybullet.rotateVector(itemQ, itemAxisInItem))
    linPointInLin = customDynamicsAPI['getObjectProperty']((lining,), ('fn', 'lining', 'point'), [0,0,0])
    linPointInHand = customDynamicsAPI['objectPoseRelativeToWorld'](linPositionInHand, linOrientationInHand, linPointInLin, [0,0,0,1])[0]
    linPoint = customDynamicsAPI['objectPoseRelativeToWorld'](linP, linQ, linPointInLin, [0,0,0,1])[0]
    itemLinPointInItem = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'lining', 'point'))
    itemLinPoint, _ = customDynamicsAPI['objectPoseRelativeToWorld'](itemP, itemQ, itemLinPointInItem, [0,0,0,1])
    entryHeightLinPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, linPointInHand))[2]
    itemYaw = math.atan2(itemAxis[1], itemAxis[0])
    liningQ = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,itemYaw)))
    #For lining:
    #  we want lp/la: linpoint at itemlinpoint and linaxis aligned with itemaxis
    #  we can achieve lp/la by a lowering process that maintains la/xy: linaxis aligned with itemaxis and linpointXY at itemlinpointXY
    #  we can achieve la/xy by an aligning process that maintains xy/ez: linpointXY at itemlinpointXY and handZ at entryHeight
    #  we can achieve xy/ez by a bringing process that maintains ez/po: handZ at entryHeight and handOrientation at parkedOrientation
    #  we can achieve ez/po by a lifting process that maintains po: handOrientation at parkedOrientation
    conlp = ['equalp', 'linPoint', 'itemLinPoint']
    tollp = [0.0001, 0.0004]
    conla = ['aligned', 'linAxis', 'itemAxis']
    tolla = [0.99, 0.9]
    conxy = ['equalxy', 'linPoint', 'itemLinPoint']
    tolxy = [0.0001, 0.0004]
    conez = ['equalz', 'handP', 'entryHeight']
    tolez = [0.0001, 0.0004]
    conpo = ['equalq', 'handQ', 'handParkedQ']
    tolpo = [0.99, 0.9]
    constraintConjunctions = [[conlp, conla], [conla, conxy], [conxy, conez], [conez, conpo]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wplpla = [itemLinPoint, liningQ, None, linPointInHand, linOrientationInHand, None, None]
    wplaxy = [[itemLinPoint[0], itemLinPoint[1], entryHeightLinPoint], liningQ, None, linPointInHand, linOrientationInHand, None, None]
    wpxyez = [[itemLinPoint[0], itemLinPoint[1], entryHeightLinPoint], handParkedQ, None, linPointInHand, None, None, None]
    wpezpo = [[handParkedP[0], handParkedP[1], entryHeight], handParkedQ, None, None, None, None, None]
    waypoints = [wplpla, wplaxy, wpxyez, wpezpo]
    tolerances = [[tollp, tolla], [tolla, tolxy], [tolxy, tolez], [tolez, tolpo]]
    entities = {'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': entryHeight, 'linPoint': linPoint, 'itemLinPoint': itemLinPoint, 'linAxis': linAxis, 'itemAxis': itemAxis}
    return [_makeGoal({'hand': hand, 'item': lining}, 'pickedItem'),
            _makeGoal({'relatum': item}, 'near'),
            _makeGoal({'hand': hand, 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                      numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]

def _getShapingConditions(customDynamicsAPI, name, description, node):
    hand = description['hand']
    item = description['item']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handP = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    itemP = customDynamicsAPI['getObjectProperty']((item,), 'position')
    itemQ = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    handParkedP, handParkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
    containerEntryInContainer = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'into', 'point'))
    containerEntry, _ = customDynamicsAPI['objectPoseRelativeToWorld'](itemP, itemQ, containerEntryInContainer, [0,0,0,1])
    containerEntry = [containerEntry[0], containerEntry[1], customDynamicsAPI['getObjectProperty']((item,), 'aabb')[1][2] + 0.05]
    entryHeight = getEntryHeight(customDynamicsAPI, name, {'hand': hand, 'item': item}, {})
    #For shaping:
    #  we want hp/po: hand at container entry+offset and handOrientation at parkedOrientation
    #  we can achieve hp/po by a lowering process that maintains po/xy: handOrientation at parkedOrientation and handXY at container entryXY
    #  we can achieve po/xy by a bringing process that maintains ez/po: handZ at entryHeight and handOrientation at parkedOrientation
    #  we can achieve ez/po by a lifting process that maintains po: handOrientation at parkedOrientation
    conhp = ['equalp', 'handP', 'containerEntry']
    tolhp = [0.0001, 0.0004]
    conpo = ['equalq', 'handQ', 'handParkedQ']
    tolpo = [0.99, 0.9]
    conxy = ['equalxy', 'handP', 'containerEntry']
    tolxy = [0.0001, 0.0004]
    conez = ['equalz', 'handP', 'entryHeight']
    tolez = [0.0001, 0.0004]
    constraintConjunctions = [[conhp, conpo], [conpo, conxy], [conez, conpo]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wphppo = [containerEntry, handParkedQ, None, None, None, None, None]
    wppoxy = [[containerEntry[0], containerEntry[1], entryHeight], handParkedQ, None, None, None, None, None]
    wpezpo = [[handParkedP[0], handParkedP[1], entryHeight], handParkedQ, None, None, None, None, None]
    waypoints = [wphppo, wppoxy, wpezpo]
    tolerances = [[tolhp, tolpo], [tolpo, tolxy], [tolez, tolpo]]
    entities = {'handP': handP, 'handQ': handQ, 'handParkedQ': handParkedQ, 'entryHeight': entryHeight, 'containerEntry': containerEntry}
    return [_makeGoal({'relatum': item}, 'near'),
            _makeGoal({'hand': hand, 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                      numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]
            
def _getOpeningConditions(customDynamicsAPI, name, description, node):
    hand = getDoorManipulationHand(customDynamicsAPI, name, description.get('hand', None))
    container = description.get('container', None)
    component = description.get('component', None)
    return [_makeGoal({'relatum': container}, 'near'),
            _makeGoal({'container': container, 'component': component, 'hand': hand, 'action': 'open'}, 'armTriggeredPortalHandle'),
            _makeGoal({'hand': hand}, 'parkedArm')]

def _getClosingConditions(customDynamicsAPI, name, description, node):
    hand = getDoorManipulationHand(customDynamicsAPI, name, description.get('hand', None))
    container = description.get('container', None)
    component = description.get('component', None)
    return [_makeGoal({'relatum': container}, 'near'),
            _makeGoal({'container': container, 'component': component, 'hand': hand, 'action': 'close'}, 'armTriggeredPortalHandle'),
            _makeGoal({'hand': hand}, 'parkedArm')]

def _getBakingConditions(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    oven = description.get('oven', None)
    destination = description.get('destination', None)
    bakedType = description.get('bakedType', None)
    source = node.get('sourceContainer')
    sourcePart = node.get('sourceComponent')
    if checkBakedContents(customDynamicsAPI, item, bakedType):
        return [{'type': 'G', 'description': {'goal': 'placedItem', 'item': item, 'hand': hand, 'container': destination}}]
    return [{'type': 'G', 'description': {'goal': 'placedItem', 'item': item, 'hand': hand, 'container': oven}}]

def _getSprinklingConditions(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    shaker = description.get('shaker', None)
    storage = description.get('storage', None)
    sprinkledType = description.get('sprinkledType', None)
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    allSprinkled, toSprinkle = checkSprinkled(customDynamicsAPI, name, item, sprinkledType)
    toSprinkle = sorted(toSprinkle)
    down = customDynamicsAPI['getDown']()
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handP = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    handParkedP, handParkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
    entryHeight = getEntryHeight(customDynamicsAPI, name, {'hand': hand, 'item': item}, {})
    itemP = customDynamicsAPI['getObjectProperty']((item,), 'position')
    itemQ = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    if allSprinkled:
        containerEntryInContainer = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'into', 'point'))
        containerEntry, _ = customDynamicsAPI['objectPoseRelativeToWorld'](itemP, itemQ, containerEntryInContainer, [0,0,0,1])
    else:
        containerEntry = customDynamicsAPI['getObjectProperty']((toSprinkle[0],), 'position')
    shakerP = customDynamicsAPI['getObjectProperty']((shaker,), 'position')
    shakerQ = customDynamicsAPI['getObjectProperty']((shaker,), 'orientation')
    pourPointInShaker = customDynamicsAPI['getObjectProperty']((shaker,), ('fn', 'containment', 'pouring', 'outof', 'point'))
    pourAxisInShaker = customDynamicsAPI['getObjectProperty']((shaker,), ('fn', 'containment', 'pouring', 'outof', 'axis'))
    pourPoint, _ = customDynamicsAPI['objectPoseRelativeToWorld'](shakerP, shakerQ, pourPointInShaker, [0,0,0,1])
    pourAxis = stubbornTry(lambda : pybullet.rotateVector(shakerQ, pourAxisInShaker))
    pouring = bool(0.8 < abs(numpy.dot(pourAxis, down)))
    upright = node.get('upright', False)
    uTh = 0.2
    if upright:
        uTh = 0.4
    upright = bool(uTh > abs(numpy.dot(pourAxis, down)))
    node['upright'] = upright
    pourPointInHand, shakerOrientationInHand = customDynamicsAPI['objectPoseRelativeToObject'](handP, handQ, pourPoint, shakerQ)
    entryHeightPourPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, pourPointInHand))[2]
    tippedOrientation = getTippedOrientation(customDynamicsAPI, shaker, shakerQ, pourAxis)
    _, shakerParkedQ = customDynamicsAPI['objectPoseRelativeToWorld']([0,0,0], handParkedQ, [0,0,0], shakerOrientationInHand)
    handHoverP = handParkedP
    handHoverQ = handParkedQ
    if pouring:
        handHoverP, handHoverQ = customDynamicsAPI['handPoseToBringObjectToPose'](pourPointInHand, shakerOrientationInHand, [containerEntry[0], containerEntry[1], entryHeightPourPoint], tippedOrientation)
    if not allSprinkled:
        #For sprinkling:
        #  we want xy/ad/ez: pourpointXY at containerEntryXY and pouraxis down and handZ at entryHeight
        #  we can achieve xy/ad/ez by a tipping process that maintains xy/ez: pourpointXY at containerEntryXY and handZ at entryHeight
        #  we can achieve xy/ez by a bringing process that maintains ez: handZ at entryHeight
        #  we can achieve ez by a lifting process
        conad = ['aligned', 'pourAxis', 'down']
        tolad = [0.99, 0.9]
        conxy = ['equalxy', 'pourPoint', 'containerEntry']
        tolxy = [0.0001, 0.0004]
        conez = ['equalz', 'handP', 'entryHeight']
        tolez = [0.0001, 0.0004]
        conpo = ['equalq', 'handQ', 'handParkedQ']
        tolpo = [0.99, 0.9]
        constraintConjunctions = [[conad, conxy, conez], [conxy, conez], [conez]]
        #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
        wpadxyez = [[containerEntry[0], containerEntry[1], entryHeightPourPoint], tippedOrientation, None, pourPointInHand, shakerOrientationInHand, None, None]
        wpxyez = [[containerEntry[0], containerEntry[1], entryHeightPourPoint], handHoverQ, None, pourPointInHand, None, None, None]
        wpez = [[handHoverP[0], handHoverP[1], entryHeight], handHoverQ, None, None, None, None, None]
        waypoints = [wpadxyez, wpxyez, wpez]
        tolerances = [[tolad, tolxy, tolez], [tolxy, tolez], [tolez]]
        entities = {'down': down, 'handP': handP, 'handQ': handQ, 'handParkedQ': handParkedQ, 'entryHeight': entryHeight, 'pourPoint': pourPoint, 'containerEntry': containerEntry, 'pourAxis': pourAxis}
        return [_makeGoal({'hand': hand, 'item': shaker}, 'pickedItem'),
                _makeGoal({'relatum': item}, 'near'),
                _makeGoal({'hand': hand, 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                          numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]
    elif allSprinkled and checkGrasped(customDynamicsAPI, name, hand, shaker) and not upright:
        #For uprighting:
        #  we want po/xy/ez: handOrientation at parkedOrientation and handXY at container entryXY and handZ at entryHeight
        #  we can achieve po/xy/ez by an uprighting process
        ### TODO: a more robust uprighting: take shakerOrientationInHand into account rather than assume hand pitch 0 will do it
        roll, _, yaw = stubbornTry(lambda : pybullet.getEulerFromQuaternion(handQ))
        handUprightQ = stubbornTry(lambda : pybullet.getQuaternionFromEuler((roll, 0, yaw)))
        conpo = ['equalq', 'handQ', 'handUprightQ']
        ##conpo = ['equalq', 'handQ', 'handParkedQ']
        tolpo = [0.99, 0.9]
        conxy = ['equalxy', 'handP', 'containerEntry']
        tolxy = [0.0001, 0.0004]
        conez = ['equalz', 'handP', 'entryHeight']
        tolez = [0.0001, 0.0004]
        constraintConjunctions = [[conpo, conxy, conez]]
        #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
        ##wppoxyez = [[containerEntry[0], containerEntry[1], entryHeight], shakerParkedQ, None, pourPointInHand, shakerOrientationInHand, None, None]
        ##waypoints = [wppoxyez]
        ##tolerances = [[tolpo, tolxy, tolez]]
        wppoez = [[handP[0], handP[1], entryHeight], handUprightQ, None, None, None, None, None]
        waypoints = [wppoez]
        tolerances = [[tolpo, tolez]]
        entities = {'handP': handP, 'handQ': handQ, 'handParkedQ': handParkedQ, 'entryHeight': entryHeight, 'containerEntry': containerEntry, 'handUprightQ': handUprightQ}
        return [_makeGoal({'hand': hand, 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                          numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]
    elif (checkGrasped(customDynamicsAPI, name, hand, shaker)) or (not checkItemInContainer(customDynamicsAPI, name, shaker, storage)):
        return [_makeGoal({'container': storage, 'hand': hand, 'item': shaker}, 'placedItem')]
    else:
        return [_makeGoal({'hand': hand}, 'parkedArm')]
    return []

def _getCuttingItemConditions(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    tool = description.get('tool', None)
    storage = description.get('storage', None)
    location = customDynamicsAPI['getObjectProperty']((item,), 'at')
    if location is not None:
        node['location'] = location
    location = node.get('location', None)
    down = customDynamicsAPI['getDown']()
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handP = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    handParkedP, handParkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
    entryHeight = getEntryHeight(customDynamicsAPI, name, {'hand': hand, 'item': item}, {})
    itemP = customDynamicsAPI['getObjectProperty']((item,), 'position')
    itemQ = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    blade = customDynamicsAPI['getObjectProperty']((tool,), ('fn', 'cutting', 'links'))[0]
    toolP = customDynamicsAPI['getObjectProperty']((tool, blade), 'position')
    toolQ = customDynamicsAPI['getObjectProperty']((tool, blade), 'orientation')
    toolPositionInHand, toolOrientationInHand = customDynamicsAPI['objectPoseRelativeToObject'](handP, handQ, toolP, toolQ)
    bladeAxisInTool = customDynamicsAPI['getObjectProperty']((tool,), ('fn', 'cutting', 'axis', blade), [0,0,-1])
    bladeAxis = stubbornTry(lambda : pybullet.rotateVector(toolQ, bladeAxisInTool))
    bladeAxisInHand = stubbornTry(lambda : pybullet.rotateVector(toolOrientationInHand, bladeAxisInTool))
    facingYaw = getFacingYaw(customDynamicsAPI, item, itemP)
    fwdInHand = [1,0,0]
    fwd = [math.cos(facingYaw), math.sin(facingYaw),0]
    tippedOrientation = quatFromVecPairs((fwdInHand, fwd), (bladeAxisInHand, down))
    entryHeightBlade = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, toolPositionInHand))[2]
    overItem = customDynamicsAPI['getObjectProperty']((item,), 'aabb')[1][2] + (toolP[2] - customDynamicsAPI['getObjectProperty']((tool, blade), 'aabb')[0][2])
    #For cutting:
    #  we want xy/ad/bz: bladeXY at itemXY and blade axis down and bladeZ over item
    #  we can achieve xy/ad/bz by a lowering process that maintains xy/ad: bladeXY at itemXY and blade axis down
    #  we can achieve xy/ad by a tipping process that maintains xy/ez: bladeXY at itemXY, handZ at entryHeight
    #  we can achieve xy/ez by a bringing process that maintains ez: handZ at entryHeight
    #  we can achieve ez by a lifting process
    conad = ['aligned', 'bladeAxis', 'down']
    tolad = [0.99, 0.95]
    conbz = ['equalz', 'bladePoint', 'overItem']
    tolbz = [0.0001, 0.0004]
    conxy = ['equalxy', 'bladePoint', 'itemP']
    tolxy = [0.0001, 0.0004]
    conez = ['equalz', 'handP', 'entryHeight']
    tolez = [0.0001, 0.0004]
    constraintConjunctions = [[conad, conxy, conbz], [conad, conxy], [conxy, conez], [conez]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpadxybz = [[itemP[0], itemP[1], overItem], tippedOrientation, None, toolPositionInHand, None, None, None]
    wpadxy = [[itemP[0], itemP[1], entryHeightBlade], tippedOrientation, None, toolPositionInHand, None, None, None]
    wpxyez = [[itemP[0], itemP[1], entryHeightBlade], handQ, None, toolPositionInHand, None, None, None]
    wpez = [[handP[0], handP[1], entryHeight], handQ, None, None, None, None, None]
    waypoints = [wpadxybz, wpadxy, wpxyez, wpez]
    tolerances = [[tolad, tolxy, tolbz], [tolad, tolxy], [tolxy, tolez], [tolez]]
    entities = {'down': down, 'handP': handP, 'handQ': handQ, 'entryHeight': entryHeight, 'bladePoint': toolP, 'itemP': itemP, 'bladeAxis': bladeAxis, 'overItem': overItem}
    return [_makeGoal({'hand': hand, 'item': tool}, 'pickedItem'),
            _makeGoal({'relatum': item}, 'near'),
            _makeGoal({'hand': hand, 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                          numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]

def _getBringingNearConditions(customDynamicsAPI, name, description, node):
    trajector = description.get('trajector', None)
    hand = description.get('hand', None)
    relatum = description.get('relatum', None)
    source = description.get('sourceContainer', None)
    sourcePart = description.get('sourceComponent', None)
    picked = {'type': 'G', 'description': {'goal': 'pickedItem', 'item': trajector, 'hand': hand}}
    freeHand = getFreeHand(customDynamicsAPI, name, hand)
    closed = {'type': 'G', 'description': {'goal': 'closed', 'container': source, 'component': sourcePart, 'hand': freeHand}}
    near = {'type': 'G', 'description': {'goal': 'near', 'item': relatum}}
    return [picked, closed, near]

def _suggestNotifiedCompletion(customDynamicsAPI, name, description, node):
    return [{'type': 'P', 'description': {'process': 'notifyingCompletion'}, 'target': {'internalFlags': {'completion': True}}}] # bpt: (processGardening, completed): (True)

def _suggestNoped(customDynamicsAPI, name, description, node):
    return [{'type': 'P', 'description': {'process': 'notifyingCompletion'}, 'target': None}] # bpt: None

def _suggestNear(customDynamicsAPI, name, description, node):
    relatum = description.get('relatum', None)
    relatumPosition = list(customDynamicsAPI['getObjectProperty']((relatum,), 'position'))
    goalPosition = node['numerics'].get('position', None)
    targetPosition = relatumPosition
    if goalPosition is not None:
        targetPosition = goalPosition
    facingYaw = getFacingYaw(customDynamicsAPI, relatum, relatumPosition)
    facingQ = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,facingYaw)))
    return [_makeProcess({'relatum': description['relatum']}, 'nearing', 
                          numerics={'position': node['numerics'].get('position', None)},
                          target={'hand_left': {'target': None}, 'hand_right': {'target': None},
                                  'base': {'target': [targetPosition, facingQ], 'positionInLink': baseFwdOffset, 'orientationInLink': (0,0,0,1)}})]

def _suggestNearA(customDynamicsAPI, name, description, node):
    return [_makeProcess({'relatum': description['relatum']}, 'nearing', 
                          numerics={'position': node['numerics'].get('position', None)})]
def _getNearingConditionsA(customDynamicsAPI, name, description, node):
    relatum = description.get('relatum', None)
    relatumPosition = list(customDynamicsAPI['getObjectProperty']((relatum,), 'position'))
    goalPosition = node['numerics'].get('position', None)
    targetPosition = relatumPosition
    if goalPosition is not None:
        targetPosition = goalPosition
    baseLink = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'mobileBaseLink'))
    baseP = customDynamicsAPI['getObjectProperty']((name, baseLink), 'position')
    baseQ = customDynamicsAPI['getObjectProperty']((name, baseLink), 'orientation')
    baseFwdP, _ = customDynamicsAPI['objectPoseRelativeToWorld'](baseP, baseQ, baseFwdOffset, [0,0,0,1])
    facingYaw = getFacingYaw(customDynamicsAPI, relatum, relatumPosition)
    facingQ = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,facingYaw)))
    #For nearing:
    #  we want p/f: baseFwd at (relatum) position and facing orientation
    #  we can achieve p/f by a translation process that maintains f: facing orientation
    #  we can achieve f by a rotating process that maintains z: base at 0
    #  we can achieve z by a translation process that maintains hands put
    #  hands put: dummy waypoints for hand_left and hand_right with None for position and orientation
    conp = ['equalp', 'baseFwdP', 'targetP']
    tolp = [0.01, 0.05]
    conf = ['equalq', 'baseQ', 'targetQ']
    tolf = [0.95, 0.9]
    conz = ['equalp', 'baseP', 'zeroP']
    tolz = [0.05, 0.2]
    constraintConjunctions = [[conp, conf], [conf], [conz]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wppf = [[targetPosition[0], targetPosition[1], 0], facingQ, None, baseFwdOffset, None, None, None]
    wpf = [[0, 0, 0], facingQ, None, None, None, None, None]
    wpz = [[0, 0, 0], baseQ, None, None, None, None, None]
    waypoints = [wppf, wpf, wpz]
    tolerances = [[tolp, tolf], [tolf], [tolz]]
    entities = {'zeroP': [0,0,0], 'baseP': baseP, 'baseFwdP': baseFwdP, 'baseQ': baseQ, 'targetP': targetPosition, 'targetQ': facingQ}
    return [_makeGoal({}, 'stoppedHands'),
            _makeGoal({'hand': 'base', 'constraintConjunctions': constraintConjunctions, 'isTop': True}, 'constraintFollowed',
                      numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities})]
                      
def _checkStoppedHands(customDynamicsAPI, name, description, node):
    left = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'kinematicControl', 'target', 'hand_left'))
    right = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'kinematicControl', 'target', 'hand_right'))
    return (left is None) and (right is None)

def _suggestStoppedHands(customDynamicsAPI, name, description, node):
    return [_makeProcess({}, 'stoppingHands', target={'hand_right': {'target': None}, 'hand_left': {'target': None}})]

def _suggestParkedArm(customDynamicsAPI, name, description, node):
    hand = description['hand']
    return [_makeProcess({'hand': hand}, 'parkingArm')]

def _suggestArmNearItemHandle(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    return [_makeProcess({'hand': hand, 'item': item}, 'armNearingItemHandle')]
    
def _suggestGrasped(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    return [_makeProcess({'item': item, 'hand': hand}, 'grasping', target={'grasping': {hand: {'toAdd': [item]}}})]

def _suggestUngrasped(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    return [_makeProcess({'item': item, 'hand': hand}, 'ungrasping', target={'grasping': {hand: {'toRemove': [item]}}})]

def _suggestPickedItem(customDynamicsAPI, name, description, node):
    return [_makeProcess({'hand': description['hand'], 'item': description['item']}, 'pickingItem')]

def _suggestPlacedItem(customDynamicsAPI, name, description, node):
    return [_makeProcess({'container': description['container'], 'hand': description['hand'], 'item': description['item']}, 'placingItem')]

def _suggestLoweredItemAt(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    container = description['container']
    component, targetPosition = getItemPlacement(customDynamicsAPI, name, item, container, node['numerics'].get('component'), node['numerics'].get('position'))
    node['numerics']['component'] = component
    node['numerics']['position'] = targetPosition
    positionInHand, orientationInHand = getObjectInHand(customDynamicsAPI, name, hand, item)
    targetOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    if description.get('matchOrientation', False) is True:
        targetOrientation = customDynamicsAPI['getObjectProperty']((container,), 'orientation')
    entryHeight = getEntryHeight(customDynamicsAPI, name, {'hand': hand, 'destination': [container, component]}, node['numerics'])
    return [_makeProcess({'container': container, 'hand': hand, 'item': item}, 'loweringItem',
                         numerics={'component': component, 'position': targetPosition, 'orientation': targetOrientation, 'entryHeight': entryHeight},
                         target={hand: {'target': [targetPosition, targetOrientation], 'positionInLink': positionInHand, 'orientationInLink': orientationInHand, 'clopeningAction': None}})]

def _suggestItemAboveLocation(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    container = description['container']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    orientationHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    component = node['numerics'].get('component', None)
    position = node['numerics'].get('position', None)
    orientation = node['numerics'].get('orientation', None)
    entryHeight = node['numerics'].get('entryHeight', None)
    positionInHand, orientationInHand = getObjectInHand(customDynamicsAPI, name, hand, item)
    targetPosition = [position[0], position[1], getItemEntryHeight(entryHeight, positionInHand, orientationHand)]
    return [_makeProcess({'container': container, 'hand': hand, 'item': item}, 'hoveringItemAboveLocation',
                         numerics={'component': component, 'position': position, 'orientation': orientation, 'entryHeight': entryHeight},
                         target={hand: {'target': [targetPosition, orientation], 'positionInLink': positionInHand, 'orientationInLink': orientationInHand, 'clopeningAction': None}})]
                         
def _suggestLiftedItemToEntry(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    container = description['container']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    orientationHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    positionItem = customDynamicsAPI['getObjectProperty']((item,), 'position')
    orientation = node['numerics'].get('orientation', None)
    entryHeight = node['numerics'].get('entryHeight', None)
    positionInHand, orientationInHand = getObjectInHand(customDynamicsAPI, name, hand, item)
    targetPosition = [positionItem[0], positionItem[1], getItemEntryHeight(entryHeight, positionInHand, orientationHand)]
    return [_makeProcess({'container': container, 'hand': hand, 'item': item}, 'liftingItemToEntry',
                         numerics={},
                         target={hand: {'target': [targetPosition, orientation], 'positionInLink': positionInHand, 'orientationInLink': orientationInHand, 'clopeningAction': None}})]

def _suggestTransferredAndStored(customDynamicsAPI, name, description, node):
    return [_makeProcess({'amount': description['amount'], 'container': description['container'], 'hand': description['hand'], 'item': description['item'], 'pouredType': description['pouredType'], 'storage': description['storage']}, 'transferringAndStoring')]
    
def _suggestTransferredAndUprighted(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    itemPosition = customDynamicsAPI['getObjectProperty']((item,), 'position')
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    pouringPoint = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'point'))
    positionInHand, orientationInHand = getObjectInHand(customDynamicsAPI, name, hand, item, itemPoint=pouringPoint)
    targetPosition, _ = customDynamicsAPI['objectPoseRelativeToWorld'](itemPosition, itemOrientation, pouringPoint, [0,0,0,1])
    targetOrientation = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'upright'), [0,0,0,1])
    return [_makeProcess({'amount': description['amount'], 'container': description['container'], 'hand': hand, 'item': item, 'pouredType': description['pouredType']}, 'transferringAndUprighting',
                         numerics={'positionInHand': positionInHand, 'orientationInHand': orientationInHand},
                         target={hand: {'target': [targetPosition, targetOrientation], 'positionInLink': positionInHand, 'orientationInLink': orientationInHand, 'clopeningAction': None}})]
                         
def _suggestTransferredContents(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    container = description['container']
    positionInHand, orientationInHand = node['numerics'].get('positionInHand', None), node['numerics'].get('orientationInHand', None)
    return [_makeProcess({'container': container, 'hand': hand, 'item': item}, 'transferringContents',
                         numerics={'positionInHand': positionInHand, 'orientationInHand': orientationInHand})]

def _suggestTippedItemAboveLocation(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    relatum = description['relatum']
    positionInHand, orientationInHand = node['numerics'].get('positionInHand', None), node['numerics'].get('orientationInHand', None)
    entryHeight = node['numerics'].get('entryHeight', None)
    position = node['numerics'].get('position', None)
    orientation = node['numerics'].get('tippedOrientation', None)
    return [_makeProcess({'hand': hand, 'item': item, 'relatum': relatum}, 'tippingItemAboveLocation',
                         numerics={'position': position, 'entryOrientation': node['numerics'].get('entryOrientation'), 'entryHeight': entryHeight, 'positionInHand': positionInHand, 'orientationInHand': orientationInHand},
                         target={hand: {'target': [position, orientation], 'positionInLink': positionInHand, 'orientationInLink': orientationInHand, 'clopeningAction': None}})]

def _suggestPreparedItemForTipping(customDynamicsAPI, name, description, node): # hand, item, relatum | position, entryOrientation, positionInHand, orientationInHand, entryHeight
    item = description['item']
    hand = description['hand']
    relatum = description['relatum']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    positionHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    orientationHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    positionInHand, orientationInHand = node['numerics'].get('positionInHand', None), node['numerics'].get('orientationInHand', None)
    entryHeight = node['numerics'].get('entryHeight', None)
    position, _ = customDynamicsAPI['objectPoseRelativeToWorld'](positionHand, orientationHand, positionInHand, orientationInHand)
    position = [position[0], position[1], getItemEntryHeight(entryHeight, positionInHand, orientationHand)]
    orientation = node['numerics'].get('entryOrientation', None)
    return [_makeProcess({'hand': hand, 'item': item, 'relatum': relatum}, 'preparingItemForTipping',
                         numerics={'position': position, 'entryOrientation': node['numerics'].get('entryOrientation'), 'entryHeight': entryHeight, 'positionInHand': positionInHand, 'orientationInHand': orientationInHand},
                         target={hand: {'target': [position, orientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}})]

def _suggestLiftedItemToPouringEntry(customDynamicsAPI, name, description, node): # hand, item, relatum | entryOrientation, positionInHand, orientationInHand, entryHeight
    item = description['item']
    hand = description['hand']
    positionInHand, orientationInHand = node['numerics'].get('positionInHand', None), node['numerics'].get('orientationInHand', None)
    entryHeight = node['numerics'].get('entryHeight', None)
    orientation = node['numerics'].get('entryOrientation', None)
    parkedP, _, _ = getParkedPose(customDynamicsAPI, name, hand)
    position = [parkedP[0], parkedP[1], entryHeight]
    return [_makeProcess({'hand': hand, 'item': item}, 'liftingItemToPouringEntry',
                         numerics={},
                         target={hand: {'target': [position, orientation], 'positionInLink': None, 'orientationInLink': orientationInHand, 'clopeningAction': None}})]

def _suggestConstraintFollowed(customDynamicsAPI, name, description, node): # constraintConjunctions, hand, isTop | waypoints, tolerances
    waypoints = node['numerics']['waypoints']
    if 0 == len(waypoints):
        return []
    tolerances = node['numerics']['tolerances']
    entities = node['numerics']['entities']
    hand = description['hand']
    constraintConjunctions = description['constraintConjunctions']
    positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr = waypoints[0]
    targetQ = orientation
    targetP = positionA
    if positionB is not None:
        dA = [x-y for x,y in zip(positionA, positionCr)]
        nsqA = numpy.dot(dA, dA)
        dB = [x-y for x,y in zip(positionB, positionCr)]
        nsqB = numpy.dot(dB, dB)
        if 0.0001 > nsqA:
            targetP = positionB
        elif 0.0001 > nsqB:
            targetP = positionA
        else:
            nA = math.sqrt(nsqA)
            dirA = [x/nA for x in dA]
            nB = math.sqrt(nsqB)
            dirB = [x/nB for x in dB]
            if numpy.dot(dirB, linVelCr) > numpy.dot(dirA, linVelCr):
                targetP = positionB
    return [_makeProcess({'constraintConjunctions': constraintConjunctions, 'hand': hand, 'isTop': False}, 'constraintFollowing',
                         numerics={'waypoints': waypoints, 'tolerances': tolerances, 'entities': entities},
                         target={hand: {'target': [targetP, targetQ], 'positionInLink': positionInLink, 'orientationInLink': orientationInLink}})]

def _suggestMixedAndStored(customDynamicsAPI, name, description, node): # container, hand, mixedType, storage, tool, toolLink |
    return [_makeProcess({'container': description['container'], 'hand': description['hand'], 'tool': description['tool'], 'toolLink': description['toolLink'], 'mixedType': description['mixedType'], 'storage': description['storage']}, 'mixingAndStoring')]
    
def _suggestMixedAndUprighted(customDynamicsAPI, name, description, node): # container, hand, mixedType, tool, toolLink |
    return [_makeProcess({'container': description['container'], 'hand': description['hand'], 'tool': description['tool'], 'toolLink': description['toolLink'], 'mixedType': description['mixedType']}, 'mixingAndUprighting')]

def _suggestMixed(customDynamicsAPI, name, description, node): # container, hand, mixedType, tool, toolLink |
    return [_makeProcess({'container': description['container'], 'hand': description['hand'], 'tool': description['tool'], 'toolLink': description['toolLink'], 'mixedType': description['mixedType']}, 'mixing')]

def _suggestLinedAndParked(customDynamicsAPI, name, description, node):
    return [_makeProcess({'lining': description['lining'], 'hand': description['hand'], 'item': description['item']}, 'liningAndParking')]

def _suggestLined(customDynamicsAPI, name, description, node):
    return [_makeProcess({'lining': description['lining'], 'hand': description['hand'], 'item': description['item']}, 'lining')]

def _suggestShapedAndParked(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    destination = description.get('destination', None)
    shapedType = description.get('shapedType', None)
    ingredientTypes = description.get('ingredientTypes', [])
    ingredientsAvailable, pickedIngredients = containsIngredients(customDynamicsAPI, item, ingredientTypes)
    holdingShape, heldShape = isHoldingObjectOfType(customDynamicsAPI, name, hand, shapedType)
    handLink = getHandLink(customDynamicsAPI, name, hand)
    aabb = customDynamicsAPI['getObjectProperty']((name, handLink), 'aabb')
    aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, customDynamicsAPI['getObjectProperty']((name,), ('fn', 'grasping', 'graspingActivationRadius', hand), 0.5))
    closeShapes = [x for x in customDynamicsAPI['checkOverlap'](aabbAdj) if (shapedType == customDynamicsAPI['getObjectProperty']((x,), 'type') and (destination != customDynamicsAPI['getObjectProperty']((x,), 'at')))]
    closeShape = None
    containedShape = None
    itemHasShape, shapes = containsShapes(customDynamicsAPI, item, shapedType)
    if 0 < len(shapes):
        containedShape = sorted(shapes)[0]
    if 0 < len(closeShapes):
        closeShape = sorted(closeShapes)[0]
    if (not holdingShape) and (not ingredientsAvailable) and (not itemHasShape):
        parkedP, parkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
        return [_makeProcess({'hand': hand}, 'parkingArm', target={hand: {'target': [parkedP, parkedQ], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}})]
    elif holdingShape:
        made = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'processGardening', 'log', 'shaping', 'products'), [])
        if heldShape not in made:
            made.append(heldShape)
        customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'processGardening', 'log', 'shaping', 'products'), made)
        return [_makeProcess({'container': destination, 'hand': hand, 'item': heldShape}, 'placingItem')]
    elif closeShape is not None:
        return [_makeProcess({'hand': hand, 'item': heldShape}, 'movingArm',
                              target={'grasping': {hand: {'toAdd': [closeShape]}}})]
    elif containedShape is not None:
        return [_makeProcess({'hand': hand, 'item': containedShape}, 'pickingItem')]
    return [_makeProcess({'hand': hand, 'item': item, 'shapedType': shapedType, 'ingredientTypes': ingredientTypes}, 'shaping',
                         target={'shapingIngredients': {hand: {'toSet': pickedIngredients}},
                                 'shapingOutcome': {hand: shapedType}})]

def _suggestCutItem(customDynamicsAPI, name, description, node):
    hand = description['hand']
    item = description['item']
    storage = description['storage']
    tool = description['tool']
    if customDynamicsAPI['getObjectProperty']((item,), ('fn', 'cuttable')):
        return [_makeProcess({'hand': hand, 'item': item, 'storage': storage, 'tool': tool}, 'cuttingItem')]
    else:
        return [_makeProcess({'container': storage, 'hand': hand, 'item': tool}, 'placingItem')]

def _suggestOpened(customDynamicsAPI, name, description, node):
    container = description.get('container', None)
    component = description.get('component', None)
    hand = description.get('hand', None)
    return [{'type': 'P', 'description': {'process': 'opening', 'container': container, 'component': component, 'hand': hand}, 'children': []}]

def _suggestClosed(customDynamicsAPI, name, description, node):
    container = description.get('container', None)
    component = description.get('component', None)
    hand = description.get('hand', None)
    return [{'type': 'P', 'description': {'process': 'closing', 'container': container, 'component': component, 'hand': hand}, 'children': []}]

def _suggestStoppedOpening(customDynamicsAPI, name, description, node):
    hand = description.get('hand', None)
    target = {}
    if hand is None:
        for hand in customDynamicsAPI['getObjectProperty']((name,), ('fn', 'clopening', 'clopeningEFs'), {}).keys():
            target[hand] = {'clopeningAction': None}
    else:
        target[hand] = {'clopeningAction': None}
    return [{'type': 'P', 'description': {'process': 'stopOpening'}, 'target': target}] # bpt: (clopening, opening, _Hand): (False)

def _suggestStoppedClosing(customDynamicsAPI, name, description, node):
    hand = description.get('hand', None)
    target = {}
    if hand is None:
        for hand in customDynamicsAPI['getObjectProperty']((name,), ('fn', 'clopening', 'clopeningEFs'), {}).keys():
            target[hand] = {'clopeningAction': None}
    else:
        target[hand] = {'clopeningAction': None}
    return [{'type': 'P', 'description': {'process': 'stopOpening'}, 'target': target}] # bpt: (clopening, closing, _Hand): (False)

def _suggestBakedItem(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    oven = description.get('oven', None)
    destination = description.get('destination', None)
    bakedType = description.get('bakedType', None)
    return [{'type': 'P', 'description': {'process': 'bakingItem', 'item': item, 'hand': hand, 'oven': oven, 'destination': destination, 'bakedType': bakedType}, 'sourceContainer': node.get('sourceContainer'), 'sourceComponent': node.get('sourceComponent')}]

def _suggestBroughtNear(customDynamicsAPI, name, description, node):
    descriptionC = copy.deepcopy(description)
    descriptionC['process'] = 'bringingNear'
    descriptionC.pop('goal')
    return [{'type': 'P', 'description': descriptionC}]

def _suggestArmTriggeredPortalHandle(customDynamicsAPI, name, description, node):
    container = description.get('container', None)
    component = description.get('component', None)
    action = description.get('action', None)
    if (container is not None) and (action is not None):
        handle, door = getHandleAndDoor(customDynamicsAPI, container, component)
        hand = description.get('hand', None)
        handLink = getHandLink(customDynamicsAPI, name, hand)
        maxDistance = 0.3*customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'radius', component), 0.5)
        handLocalAABB = customDynamicsAPI['getObjectProperty']((name, handLink), 'localAABB')
        handXHalf = 0.5*(handLocalAABB[1][0] - handLocalAABB[0][0])
        handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
        handlePosition = customDynamicsAPI['getObjectProperty']((container, handle), 'position')
        containerPosition = customDynamicsAPI['getObjectProperty']((container,), 'position')
        offsetYaw = getFacingYaw(customDynamicsAPI, container, containerPosition) + math.pi
        offsetVector = stubbornTry(lambda : pybullet.rotateVector(pybullet.getQuaternionFromEuler((0,0,offsetYaw)), (maxDistance + handXHalf, 0, 0)))
        target = {hand: {'target': [[x+y for x,y in zip(handlePosition, offsetVector)], handOrientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': action}}
        return [{'type': 'P', 'description': {'process': 'movingArm', 'hand': hand}, 'target': target}]
    return []

def _suggestSprinkled(customDynamicsAPI, name, description, node):
    return [_makeProcess({'item': description['item'], 'hand': description['hand'], 'shaker': description['shaker'], 'storage': description['storage'], 'sprinkledType': description['sprinkledType']}, 'sprinkling')]
  
goalCheckers = {
    'noped': _alwaysFalse,
    'notifiedCompletion': _alwaysFalse,
    'stoppedHands': _checkStoppedHands,
    'near': _checkNear, # relatum | position
    'parkedArm': _checkParkedArm, # hand |
    'armNearItemHandle': _checkArmNearItemHandle, # hand, item | 
    'grasped': _checkGrasped, # hand, item | 
    'ungrasped': _checkUngrasped, # hand, item |
    'pickedItem': _checkPickedItem, # hand, item | 
    'placedItem': _checkPlacedItem, # container, hand, item | 
    'loweredItemAt': _checkLoweredItemAt, # container, hand, item | 
    'itemAboveLocation': _checkItemAboveLocation, # hand, item | position, orientation, entryHeight
    'liftedItemToEntry': _checkLiftedItemToEntry, # container, hand, item | component, entryHeight
    'transferredAndStored': _checkTransferredAndStored, # amount, container, hand, item, pouredType, storage |
    'transferredAndUprighted': _checkTransferredAndUprighted, # amount, container, hand, item, pouredType |
    'transferredContents': _checkTransferredContents, # amount, container, hand, item, pouredType |
    'tippedItemAboveLocation': _checkTippedItemAboveLocation, # hand, item, relatum | position, tippedOrientation, entryOrientation, positionInHand, orientationInHand, entryHeight
    'preparedItemForTipping': _checkPreparedItemForTipping, # hand, item, relatum | position, entryOrientation, positionInHand, orientationInHand, entryHeight
    'liftedItemToPouringEntry': _checkLiftedItemToPouringEntry, # hand, item, relatum | entryOrientation, positionInHand, orientationInHand, entryHeight
    'constraintFollowed': _checkConstraintFollowed, # constraintConjunctions, hand, isTop | waypoints, tolerances, entities
    'mixedAndStored': _checkMixedAndStored, # container, hand, mixedType, storage, tool, toolLink |
    'mixedAndUprighted': _checkMixedAndUprighted, # container, hand, mixedType, storage, tool, toolLink |
    'mixed': _checkMixed, # container, hand, mixedType, storage, tool, toolLink |
    'linedAndParked': _checkLinedAndParked, # hand, item, lining |
    'lined': _checkLined, # hand, item, lining |
    'shapedAndParked': _checkShapedAndParked, # destination, hand, ingredientTypes, item, shapedType |
    'sprinkled': _checkSprinkled, # hand, item, shaker, sprinkledType, storage |
    'bakedItem': _checkBaked,
    'cutItem': _checkCutItem, # hand, item, storage, tool |
    'armTriggeredPortalHandle': _checkArmTriggeredPortalHandle,
    'stoppedOpening': _checkStoppedOpening,
    'opened': _checkOpened,
    'closed': _checkClosed,
    'broughtNear': _checkBroughtNear,
    'followedWaypoints': _checkFollowedWaypoints
    }
processSuggesters = {
    'notifiedCompletion': _suggestNotifiedCompletion,
    'noped': _suggestNoped,
    'stoppedHands': _suggestStoppedHands,
    'near': _suggestNearA, # relatum | position
    'parkedArm': _suggestParkedArm, # hand | 
    'armNearItemHandle': _suggestArmNearItemHandle, # hand, item | 
    'grasped': _suggestGrasped, # hand, item | 
    'ungrasped': _suggestUngrasped, # hand, item | 
    'pickedItem': _suggestPickedItem, # hand, item | 
    'placedItem': _suggestPlacedItem, # container, hand, item | 
    'loweredItemAt': _suggestLoweredItemAt, # container, hand, item | component, position
    'itemAboveLocation': _suggestItemAboveLocation, # container, hand, item | component, position, orientation, entryHeight
    'liftedItemToEntry': _suggestLiftedItemToEntry, # container, hand, item | component, entryHeight
    'transferredAndStored': _suggestTransferredAndStored, # amount, container, hand, item, pouredType, storage |
    'transferredAndUprighted': _suggestTransferredAndUprighted, # amount, container, hand, item, pouredType |
    'transferredContents': _suggestTransferredContents, # amount, container, hand, item, pouredType |
    'tippedItemAboveLocation': _suggestTippedItemAboveLocation, # hand, item, relatum | position, tippedOrientation, entryOrientation, positionInHand, orientationInHand, entryHeight
    'preparedItemForTipping': _suggestPreparedItemForTipping, # hand, item, relatum | position, entryOrientation, positionInHand, orientationInHand, entryHeight
    'liftedItemToPouringEntry': _suggestLiftedItemToPouringEntry, # hand, item, relatum | entryOrientation, positionInHand, orientationInHand, entryHeight
    'constraintFollowed': _suggestConstraintFollowed, # constraintConjunctions, hand, isTop | waypoints, tolerances, entities
    'mixedAndStored': _suggestMixedAndStored, # container, hand, mixedType, storage, tool, toolLink |
    'mixedAndUprighted': _suggestMixedAndUprighted, # container, hand, mixedType, tool, toolLink |
    'mixed': _suggestMixed, # container, hand, mixedType, tool, toolLink |
    'linedAndParked': _suggestLinedAndParked, # hand, item, lining |
    'lined': _suggestLined, # hand, item, lining |
    'shapedAndParked': _suggestShapedAndParked, # destination, hand, ingredientTypes, item, shapedType |
    'sprinkled': _suggestSprinkled, # hand, item, shaker, sprinkledType, storage |
    'cutItem': _suggestCutItem, # hand, item, storage, tool |
    'opened': _suggestOpened,
    'closed': _suggestClosed,
    'stoppedOpening': _suggestStoppedOpening,
    'stoppedClosing': _suggestStoppedClosing,
    'bakedItem': _suggestBakedItem,
    'broughtNear': _suggestBroughtNear,
    'armTriggeredPortalHandle': _suggestArmTriggeredPortalHandle
    
    }
conditionListers = {
    'notifyingCompletion': _emptyList,
    'nearing': _getNearingConditionsA, # relatum | 
    'stoppingHands': _emptyList,
    'parkingArm': _getParkingArmConditions, # hand |
    'armNearingItemHandle': _getArmNearingItemHandleConditions, # hand, item |
    'hoveringLocation': _getHoveringLocationConditions, # hand, item | entryHeight
    'grasping': _getGraspingConditions, # hand, item |
    'ungrasping': _emptyList,
    'pickingItem': _getPickingItemConditions, # hand, item |
    'placingItem': _getPlacingItemConditions, # container, hand, item | 
    'loweringItem': _getLoweringItemConditions, # container, hand, item | component, position, orientation, entryHeight
    'hoveringItemAboveLocation': _getHoveringItemAboveLocationConditions, # container, hand, item | component, position, orientation, entryHeight
    'liftingItemToEntry': _emptyList, # <|> TODO: consider adding the pickedItem goal here; not urgent as long as calling supertree has it
    'transferringAndStoring': _getTransferringAndStoringConditions, # amount, container, hand, item, pouredType, storage |
    'transferringAndUprighting': _getTransferringAndUprightingConditions, # amount, container, hand, item, pouredType, storage | positionInHand, orientationInHand
    'transferringContents': _getTransferringContentsConditions, # container, hand, item | positionInHand, orientationInHand
    'tippingItemAboveLocation': _getTippingItemAboveLocationConditions, # hand, item, relatum | position, tippedOrientation, entryOrientation, positionInHand, orientationInHand, entryHeight
    'preparingItemForTipping': _getPreparingItemForTippingConditions, # hand, item, relatum | position, entryOrientation, positionInHand, orientationInHand, entryHeight
    'constraintFollowing': _getConstraintFollowingConditions, # constraintConjunctions, hand, isTop | waypoints, tolerances, entities
    'mixingAndStoring': _getMixingAndStoringConditions, # container, hand, mixedType, storage, tool, toolLink |
    'mixingAndUprighting': _getMixingAndUprightingConditions, # container, hand, mixedType, tool, toolLink |
    'mixing': _getMixingConditions, # container, hand, mixedType, tool, toolLink |
    'liningAndParking': _getLiningAndParkingConditions, # hand, item, lining |
    'lining': _getLiningConditions, # hand, item, lining |
    'shaping': _getShapingConditions, # destination, hand, ingredientTypes, item, shapedType |
    'sprinkling': _getSprinklingConditions, # hand, item, shaker, sprinkledType, storage |
    'cuttingItem': _getCuttingItemConditions, # hand, item, storage, tool |
    'liftingItemToPouringEntry': _emptyList,
    'stopOpening': _emptyList,
    'stopClosing': _emptyList,
    'movingArm': _emptyList,
    'opening': _getOpeningConditions,
    'closing': _getClosingConditions,
    'bakingItem': _getBakingConditions,
    'bringingNear': _getBringingNearConditions,
    'followingWaypoints': _emptyList
    }

def checkGoal(customDynamicsAPI, name, node):
    description = node.get('description', {})
    goal = description.get('goal', 'notifiedCompletion')
    return goalCheckers[goal](customDynamicsAPI, name, description, node)

def checkProcess(customDynamicsAPI, name, node):
    ### TODO
    return True
def checkNoal(customDynamicsAPI, name, node):
    ### TODO
    return True
def checkThreat(customDynamicsAPI, name, node):
    ### TODO
    return True

def getThreats(customDynamicsAPI, name, description):
    return [] ### TODO; for now leave as empty list

def getProcesses(customDynamicsAPI, name, description, node):
    goal = description.get('goal', 'notifiedCompletion')
    return processSuggesters[goal](customDynamicsAPI, name, description, node)

def getCoherenceConditions(customDynamicsAPI, name, description, node):
    process = description.get('process', 'notifyingCompletion')
    return conditionListers[process](customDynamicsAPI, name, description, node)
    
def getHandLink(customDynamicsAPI, name, hand):
    return customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'efLink', hand), None)

def getHandleLink(customDynamicsAPI, item):
    retq = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'grasping', 'handle'), None)
    if retq is None:
        retq = customDynamicsAPI['getObjectProperty']((item,), 'baseLinkName')
    return retq

def getFacingYaw(customDynamicsAPI, item, itemPosition):
    yaw = math.atan2(itemPosition[1], itemPosition[0])
    ownFacing = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'facing'), None)
    if ownFacing is not None:
        orientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
        facingV = stubbornTry(lambda : pybullet.rotateVector(orientation, ownFacing))
        yaw = math.atan2(facingV[1], facingV[0])
    at = customDynamicsAPI['getObjectProperty']((item,), 'at')
    if at is not None:
        facing = customDynamicsAPI['getObjectProperty']((at,), ('fn', 'containment', 'facing'), None)
        if facing is not None:
            orientation = customDynamicsAPI['getObjectProperty']((at,), 'orientation')
            facingV = stubbornTry(lambda : pybullet.rotateVector(orientation, facing))
            yaw = math.atan2(facingV[1], facingV[0])
    return yaw

def poseXYYawDistance(positionA, yawA, positionB, yawB, posThreshold, yawThreshold):
    d = [x-y for x,y in zip(positionA[:2], positionB[:2])]
    if bool(posThreshold*posThreshold < numpy.dot(d,d)):
        return False
    aV = stubbornTry(lambda : pybullet.rotateVector(pybullet.getQuaternionFromEuler((0,0,yawA)), (1, 0, 0)))
    bV = stubbornTry(lambda : pybullet.rotateVector(pybullet.getQuaternionFromEuler((0,0,yawB)), (1, 0, 0)))
    if bool(yawThreshold >= numpy.dot(aV, bV)):
        return False
    return True

def pose6DDistance(positionA, orientationA, positionB, orientationB, posThreshold, ornThreshold):
    d = [x-y for x,y in zip(positionA, positionB)]
    if bool(posThreshold*posThreshold < numpy.dot(d,d)):
        return False
    aX = stubbornTry(lambda : pybullet.rotateVector(orientationA, (1, 0, 0)))
    bX = stubbornTry(lambda : pybullet.rotateVector(orientationB, (1, 0, 0)))
    aY = stubbornTry(lambda : pybullet.rotateVector(orientationA, (0,1,0)))
    bY = stubbornTry(lambda : pybullet.rotateVector(orientationB, (0,1,0)))
    if bool(ornThreshold >= numpy.dot(aX, bX)) or bool(ornThreshold >= numpy.dot(aY, bY)):
        return False
    return True

def getHandleAndDoor(customDynamicsAPI, container, component):
    if (container is None) or (component is None):
        return None, None
    door = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'door', component), None)
    handle = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'handle', door), None)
    return handle, door

def getDoorManipulationHand(customDynamicsAPI, name, suggestion, avoid=None):
    if suggestion is not None:
        return suggestion
    retq = None
    clopenerEFs = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'clopening', 'clopeningEFs'), [])
    for clopenerEF in clopenerEFs:
        if clopenerEF == avoid:
            continue
        if 0 == len(customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'grasping', 'actuallyGrasping', clopenerEF), [])):
            retq = clopenerEF
            break
    if retq is None:
        retq = clopenerEFs[0]
    return retq

def getFreeHand(customDynamicsAPI, name, hand):
    return getDoorManipulationHand(customDynamicsAPI, name, None, avoid=hand)

def getHeld(customDynamicsAPI, name, hand):
    return set([x[0][0] for x in customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'grasping', 'actuallyGrasping', hand), [])])

def getParkedPose(customDynamicsAPI, name, hand):
    armBaseLink = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'efBaseLink', hand))
    armBasePosition = customDynamicsAPI['getObjectProperty']((name, armBaseLink), 'position')
    armBaseOrientation = customDynamicsAPI['getObjectProperty']((name, armBaseLink), 'orientation')
    armParkedPosition = list(customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'parkedPosition', hand)))
    height = None
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    aabbHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'aabb')
    held = getHeld(customDynamicsAPI, name, hand)
    aabbs = [aabbHand] + [customDynamicsAPI['getObjectProperty']((x,), 'aabb') for x in held]
    aabbsMin = [x[0] for x in aabbs]
    aabbsMax = [x[1] for x in aabbs]
    aabbMin = [min([x[0] for x in aabbsMin]), min([x[1] for x in aabbsMin]), min([x[2] for x in aabbsMin])]
    aabbMax = [max([x[0] for x in aabbsMax]), max([x[1] for x in aabbsMax]), max([x[2] for x in aabbsMax])]
    aabbMinAdj = list(aabbMin)
    aabbMinAdj[2] = -0.01
    overlaps = [x for x in customDynamicsAPI['checkOverlap']([aabbMinAdj, aabbMax]) if (x != name) and (x not in held) and customDynamicsAPI['getObjectProperty']((x,), ('fn', 'canContain'), False)]
    maxO = None
    for overlap in overlaps:
        aabbsOverlap = [(x, customDynamicsAPI['getObjectProperty']((overlap, x), 'aabb')) for x in customDynamicsAPI['getObjectProperty']((overlap,), ('fn', 'containment', 'links'), [])]
        valid = [(x[1][1][2], overlap, x[0]) for x in aabbsOverlap if bool(x[1][1][2] < handPosition[2])]
        if 0 < len(valid):
            maxC = max(valid)
            if (maxO is None) or bool(maxO[0] < maxC[0]):
                maxO = maxC
    if maxO is not None:
        height = getComponentHeight(customDynamicsAPI, maxO[1], maxO[2]) + maxO[0]
    armParkedOrientation = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'parkedOrientation', hand))
    #previousZ = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'processGardening', 'parkZ', hand), None)
    if height is not None:
        armParkedPosition[2] = height
    #customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'processGardening', 'parkZ', hand), previousZ)
    armParkedOrientation = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'parkedOrientation', hand))
    parkedPosition, parkedOrientation = customDynamicsAPI['objectPoseRelativeToWorld'](armBasePosition, armBaseOrientation, armParkedPosition, armParkedOrientation)
    return parkedPosition, parkedOrientation, height

def getParkingTarget(customDynamicsAPI, name, hand):
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    parkedPosition, parkedOrientation, height = getParkedPose(customDynamicsAPI, name, hand)
    parkedPosition = list(parkedPosition)
    previousXY = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'processGardening', 'auxiliary', 'parkXY', hand), None)
    if height is None:
        previousXY = None
    else:
        if bool(0.02 < abs(handPosition[2] - height)):
            if previousXY is None:
                previousXY = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')[:2]
            parkedPosition = [previousXY[0], previousXY[1], height]
        else:
            previousXY = None
    customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'processGardening', 'auxiliary', 'parkXY', hand), previousXY)
    #movedObjs = [x[0][0] for x in customDynamicsAPI['getObjectProperty']((name,), ('fn', 'grasping', 'actuallyGrasping', hand), [])]
    #ats = [customDynamicsAPI['getObjectProperty']((x,), 'at') for x in movedObjs]
    #if any([x is not None for x in ats]):
    #    handLinkPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    #    parkedPosition[2] = handLinkPosition[2] + 0.01
    return {hand: {'target': [parkedPosition, parkedOrientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}}

def getContainerComponent(customDynamicsAPI, item):
    component = None
    container = customDynamicsAPI['getObjectProperty']((item,), 'at')
    if container is not None:
        for candidate in customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'links'), []):
            if 0 < len(customDynamicsAPI['checkClosestPoints']((container, candidate), (item,), maxDistance=0.1)):
                component = candidate
                break
    return container, component

def getItemHandlePosition(customDynamicsAPI, name, handleLink, item):
    return customDynamicsAPI['getObjectProperty']((item, handleLink), 'position')

def getComponentHeight(customDynamicsAPI, container, component):
    retq = None
    if component is None:
        components = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'links'), [])
        if 0 < len(components):
            component = components[0]
    while retq is None:
        retq = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'height', component), None)
        if retq is None:
            container, component = getContainerComponent(customDynamicsAPI, container)
            if container is None:
                break
    return retq

#def getTippedOrientation(itemOrientation, pouringAxis):
#    v = stubbornTry(lambda : pybullet.rotateVector(itemOrientation, pouringAxis))
#    dv = numpy.dot(v, [0,0,-1])
#    if dv > 0.95:
#        return itemOrientation
#    if dv < -0.95:
#        return stubbornTry(lambda: pybullet.multiplyTransforms([0,0,0], [1,0,0,0], [0,0,0], itemOrientation))[1]
#    av = numpy.cross(v, [0,0,-1])
#    sa = numpy.linalg.norm(av)
#    av = av/sa
#    print("XXX", list(av), sa)
#    return stubbornTry(lambda: pybullet.multiplyTransforms([0,0,0], pybullet.getQuaternionFromAxisAngle(list(av), math.asin(sa)), [0,0,0], itemOrientation))[1]

def toOriginAABB(aabb):
    middle = [(x+y)/2 for x,y in zip(aabb[0], aabb[1])]
    return [[x-y for x,y in zip(aabb[0],middle)], [x-y for x,y in zip(aabb[1],middle)]]

def getLocations(customDynamicsAPI, container, component, aabb, canLine):
    aabbC = customDynamicsAPI['getObjectProperty']((container, component), 'aabb')
    arrangementAxis = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'arrangement', 'axis'), (1, 0, 0))
    position = customDynamicsAPI['getObjectProperty']((container, component), 'position')
    orientation = customDynamicsAPI['getObjectProperty']((container, component), 'orientation')
    arrangementAxis = stubbornTry(lambda : pybullet.rotateVector(orientation, arrangementAxis))
    referencePosition = [position[0], position[1], aabbC[1][2]-aabb[0][2]]
    if canLine:
        return referencePosition
    retq = [referencePosition]
    for inc in [1, -1]:
        k = 1
        while True:
            d = k*0.05
            v = [d*arrangementAxis[0]+referencePosition[0], d*arrangementAxis[1]+referencePosition[1], referencePosition[2]]
            vx, vX = v[0]+aabb[0][0], v[0]+aabb[1][0]
            vy, vY = v[1]+aabb[0][1], v[1]+aabb[1][1]
            if bool(vx < aabbC[0][0]) or bool(vX > aabbC[1][0]) or bool(vy < aabbC[0][1]) or bool(vY > aabbC[1][1]):
                break
            else:
                retq.append(v)
            k += inc
    return retq
    
def getItemPlacement(customDynamicsAPI, name, item, container, component, targetPosition):
    def feasible(customDynamicsAPI, aabb, component, targetPosition):
        aabbAdj = [[x+y for x,y in zip(aabb[0], targetPosition)], [x+y for x,y in zip(aabb[1], targetPosition)]]
        overlaps = customDynamicsAPI['checkOverlap'](aabbAdj)
        badOverlaps = []
        for e in overlaps:
            if e in [name, item, container]:
                continue
            if item == customDynamicsAPI['getObjectProperty']((e,), 'at'):
                continue
            if customDynamicsAPI['getObjectProperty']((e,), ('fn', 'canLine'), False):
                continue
            badOverlaps.append(e)
        #print("BAD", component, targetPosition, aabbAdj, badOverlaps)
        return 0 == len(badOverlaps)
    aabb = toOriginAABB(customDynamicsAPI['getObjectProperty']((item,), 'aabb'))
    if (component is not None) and (targetPosition is not None):
        if feasible(customDynamicsAPI, aabb, component, targetPosition):
            return component, targetPosition
    arrangement = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'arrangement', 'mode'), 'heap')
    components = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'links'), [])
    if 'heap' == arrangement:
        component = components[0]
        targetPosition = list(customDynamicsAPI['getObjectProperty']((container, component), 'position'))
        targetPosition[2] = customDynamicsAPI['getObjectProperty']((container, component), 'aabb')[1][2] - aabb[0][2]
        return component, targetPosition
    ### Assuming side by side arrangement
    if component is not None:
        components = [component] + components
    for c in components:
        #print("LOCS", getLocations(customDynamicsAPI, container, c, aabb, customDynamicsAPI['getObjectProperty']((item,), ('fn', 'canLine'), False)))
        for l in getLocations(customDynamicsAPI, container, c, aabb, customDynamicsAPI['getObjectProperty']((item,), ('fn', 'canLine'), False)):
            if feasible(customDynamicsAPI, aabb, c, l):
                return c, l
    return None, None

def getContents(customDynamicsAPI, item, heightBonus=None):
    aabb = customDynamicsAPI['getObjectProperty']((item,), 'aabb')
    if heightBonus is not None:
        aabb = [aabb[0], [aabb[1][0], aabb[1][1], aabb[1][2] + heightBonus]]
    overlaps = customDynamicsAPI['checkOverlap'](aabb)
    return [x for x in overlaps if (item == customDynamicsAPI['getObjectProperty']((x,), 'at')) or (heightBonus is not None)]

def getSprinklee(customDynamicsAPI, name, item, sprinkledType, previous):
    if (previous is not None) and (item == customDynamicsAPI['getObjectProperty']((previous,), 'at')):
        return previous
    for x in getContents(customDynamicsAPI, item):
        if (name != x) and (sprinkledType != customDynamicsAPI['getObjectProperty']((x,), 'type')) and (not customDynamicsAPI['getObjectProperty']((x,), ('fn', 'canLine'), False)):
            return x
    return None

def checkUpright(itemOrientation, pouringAxis, th=None):
    if th is None:
        th = 0.2
    return bool(th > abs(numpy.dot(stubbornTry(lambda : pybullet.rotateVector(itemOrientation, pouringAxis)), [0,0,-1])))

def checkGrasped(customDynamicsAPI, name, hand, item):
    if hand is None:
        hands = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'grasping', 'effectors'), [])
    else:
        hands = [hand]
    actuallyGrasping = set([])
    for hand in hands:
        actuallyGrasping = actuallyGrasping.union(getHeld(customDynamicsAPI, name, hand))
    return item in actuallyGrasping

def checkNear(customDynamicsAPI, name, item, previous, position=None):
    if position is None:
        itemPosition = customDynamicsAPI['getObjectProperty']((item,), 'position')
    else:
        itemPosition = position
    facingYaw = getFacingYaw(customDynamicsAPI, item, itemPosition)
    baseLink = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'mobileBaseLink'))
    basePosition = customDynamicsAPI['getObjectProperty']((name, baseLink), 'position')
    baseOrientation = customDynamicsAPI['getObjectProperty']((name, baseLink), 'orientation')
    baseYaw = stubbornTry(lambda : pybullet.getEulerFromQuaternion(baseOrientation))[2]
    basePosition, _ = customDynamicsAPI['objectPoseRelativeToWorld'](basePosition, baseOrientation, baseFwdOffset, (0,0,0,1))
    if previous:
        return poseXYYawDistance(itemPosition, facingYaw, basePosition, baseYaw, 1.5, 0.7)
    else:
        return poseXYYawDistance(itemPosition, facingYaw, basePosition, baseYaw, 0.1, 0.95)

def checkParked(customDynamicsAPI, name, hand, previous):
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    parkedPosition, parkedOrientation, _ = getParkedPose(customDynamicsAPI, name, hand)
    if previous:
        return pose6DDistance(parkedPosition, parkedOrientation, handPosition, handOrientation, 0.5, 0.7)
    else:
        return pose6DDistance(parkedPosition, parkedOrientation, handPosition, handOrientation, 0.1, 0.95)

def checkParkedXY(customDynamicsAPI, name, hand, previous):
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    parkedPosition, parkedOrientation, _ = getParkedPose(customDynamicsAPI, name, hand)
    handPosition = [handPosition[0], handPosition[1], 0]
    parkedPosition = [parkedPosition[0], parkedPosition[1], 0]
    if previous:
        return pose6DDistance(parkedPosition, parkedOrientation, handPosition, handOrientation, 0.5, 0.7)
    else:
        return pose6DDistance(parkedPosition, parkedOrientation, handPosition, handOrientation, 0.1, 0.95)


def checkTransferred(customDynamicsAPI, name, item, pouredType, amount, container):
    if amount is not None:
        return amount <= len([x for x in getContents(customDynamicsAPI, container) if (pouredType == customDynamicsAPI['getObjectProperty']((x,), 'type'))])
    return 0 == len([x for x in getContents(customDynamicsAPI, item) if (x != name) and (not customDynamicsAPI['getObjectProperty']((x,), ('fn', 'canLine'), False))])

def checkItemInContainer(customDynamicsAPI, name, item, container):
    ## TODO: test whether transitive closure of at includes container
    inLargeContainer = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'largeConcavity'), False) and (item in customDynamicsAPI['checkOverlap'](customDynamicsAPI['getObjectProperty']((container,), 'aabb')))
    return (customDynamicsAPI['getObjectProperty']((item,), 'position') is None) or (container == customDynamicsAPI['getObjectProperty']((item,), 'at')) or inLargeContainer

def checkBakedContents(customDynamicsAPI, item, bakedType):
    contents = getContents(customDynamicsAPI, item)
    for x in contents:
        if bakedType == customDynamicsAPI['getObjectProperty']((x,), 'type'):
            return True
    return False

def checkSprinkled(customDynamicsAPI, name, item, sprinkledType):
    retq = [x for x in getContents(customDynamicsAPI, item) if (sprinkledType == customDynamicsAPI['getObjectProperty']((x,), 'type'))]
    return 0 == len(retq), retq

#def getCausalConditions(customDynamicsAPI, name, description):
#    return []

def checkStatus(customDynamicsAPI, name, node):
    nodeType = node['type']
    status = {'G': checkGoal, 'P': checkProcess, 'N': checkNoal, 'T': checkThreat}[nodeType](customDynamicsAPI, name, node)
    return nodeType, status

def popSubtree(garden, nodeId):
    toPop = [nodeId]
    while toPop:
        cr = toPop.pop()
        if cr in garden:
            toPop += garden[cr]['children']
            garden.pop(cr)

def popSubforest(garden, nodeId, nodeType):
    newChildren = []
    for c in garden[nodeId].get('children', []):
        if garden[c]['type'] == nodeType:
            popSubtree(garden, c)
        else:
            newChildren.append(c)
    garden[nodeId]['children'] = newChildren

def updateSubforest(garden, nodeId, mode, newNodes):
    toRemove = []
    toKeep = []
    for c in garden[nodeId].get('children', []):
        found = False
        for n in newNodes:
           if garden[c]['description'] == n['description']:
               found = True
               break
        if not found:
            toRemove.append(c)
        else:
            toKeep.append(c)
    for c in toRemove:
        garden.pop(c)
    newChildren = []
    for n in newNodes:
        found = None
        for c in toKeep:
            if garden[c]['description'] == n['description']:
                found = c
                break
        if found is None:
            n['children'] = []
            nIdx = max(garden.keys()) + 1
            garden[nIdx] = n
            newChildren.append(nIdx)
        else:
            garden[found]['type'] = n['type']
            if 'target' in n: #P: nodes may need updating targets, because new targets may be computed when suggesting procs based on current world state
                garden[found]['target'] = n['target']
            if 'numerics' in n: # Likewise, numerical data, e.g. precise positions, may need updating even if the proc/goal is otherwise the same
                garden[found]['numerics'] = n['numerics']
            newChildren.append(found)
    garden[nodeId]['children'] = newChildren

def updateGarden(name, customDynamicsAPI):
    csvProcessGardening = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'processGardening'))
    fnProcessGardening = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'processGardening'))
    garden = csvProcessGardening.get('garden', {})
    if 0 not in garden:
        return
    toVisit = [0]
    bodyProcesses = {}
    goalStatuses = {}
    siblings = {}
    while toVisit:
        cr = toVisit.pop()
        nodeType, status = checkStatus(customDynamicsAPI, name, garden[cr])
        sibling = siblings.get(cr, None)
        goalStatuses[cr] = status
        if 'G' == nodeType:
            garden[cr]['previousStatus'] = status
            if status:
                popSubforest(garden, cr, 'P')
                if isinstance(garden[cr]['description'], list):
                    crGoal = (garden[cr].get('currentGoal', 0) + 1)%len(garden[cr]['description'])
                    garden[cr]['currentGoal'] = crGoal
                    updateSubforest(garden, cr, 'P', getProcesses(customDynamicsAPI, name, garden[cr]['description'][crGoal], garden[cr]))
                else:
                    updateSubforest(garden, cr, 'T', getThreats(customDynamicsAPI, name, garden[cr]['description'])) ### TODO; for now will just always return an empty list
            else:
                if isinstance(garden[cr]['description'], list):
                    crGoal = garden[cr].get('currentGoal', 0)
                    updateSubforest(garden, cr, 'P', getProcesses(customDynamicsAPI, name, garden[cr]['description'][crGoal], garden[cr]))
                else:
                    updateSubforest(garden, cr, 'P', getProcesses(customDynamicsAPI, name, garden[cr]['description'], garden[cr]))
        elif 'P' == nodeType:
            updateSubforest(garden, cr, 'G', getCoherenceConditions(customDynamicsAPI, name, garden[cr]['description'], garden[cr]))
            #if status:
            #    updateSubforest(garden, cr, 'G', getCoherenceConditions(customDynamicsAPI, name, garden[cr]['description'], garden[cr]))
            #else:
            #    updateSubforest(garden, cr, 'G', getCausalConditions(customDynamicsAPI, name, garden[cr]['description']))
            target = garden[cr].get('target', None)
            if target is None:
                target = {}
            actuators = target.keys() # Check whether this is a body process and if so make a note to check on its actuator targets later
            for actuator in actuators:
                if actuator not in bodyProcesses:
                    bodyProcesses[actuator] = []
                bodyProcesses[actuator].append(cr)
        for k,ch in enumerate(garden[cr]['children'][:-1]):
            if k < len(garden[cr]['children']) - 1:
                siblings[ch] = garden[cr]['children'][k+1]
        ### TODO: add similar branches for threats and noals
        if 0 < len(garden[cr]['children']):
            fc = garden[cr]['children'][0]
            toVisit.append(fc)
        if status and (sibling is not None):
            toVisit.append(sibling)
    toKeep = set([])
    toVisit = [0]
    while toVisit:
        cr = toVisit.pop()
        toKeep.add(cr)
        toVisit = toVisit + garden[cr].get('children', [])
    for nodeIdx in list(garden.keys()):
        if nodeIdx not in toKeep:
            garden.pop(nodeIdx)

    #for k in sorted(garden.keys()):
    #    print(k, garden[k]['description'], garden[k].get('children'), garden[k].get('previousStatus'))

    # customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'processGardening', 'garden'), garden)
    for actuator, processes in bodyProcesses.items():
        ### TODO: some kind of conflict resolution; for now assume we can safely choose the first active proc using the actuator
        target = None
        for process in processes:
            if any([(not goalStatuses.get(x, False)) for x in garden[process]['children']]):
                continue
            target = garden[process].get('target', {}).get(actuator, None)
            if target is not None:
                break
        if target is not None:
            #if 'hand_right' == actuator:
            #    print(target)
            for variable, value in target.items():
                mode = getDictionaryEntry(fnProcessGardening, ('actuator', 'mode', actuator, variable), 'value')
                targetVariable = getDictionaryEntry(fnProcessGardening, ('actuator', 'variablePath', actuator, variable), None)
                if 'set' == mode:
                    toSet = value.get('toSet', None)
                    if toSet is not None:
                        targetValue = toSet
                    else:
                        toAdd = value.get('toAdd', [])
                        toRemove = value.get('toRemove', [])
                        targetValue = set(customDynamicsAPI['getObjectProperty']((name,), targetVariable, set()))
                        targetValue = list(targetValue.difference(toRemove).union(toAdd))
                else:
                    targetValue = value
                customDynamicsAPI['setObjectProperty']((), targetVariable, targetValue)


'''
OBSERVATION VERY IMPORTANT:
    IK will not precisely impose constraints based on waypoints. E.g., if need to go from p,q1 to p,q2, there may be times of significant deviations from p along the way.
    CONSEQUENCES:
        ALWAYS COMPUTE WAYPOINTS "ABSOLUTELY": the positions etc. of the trajectors cannot themselves be used as waypoints
        use Schmitt-Trigger pattern: precise thresholds when going False->True, tolerant thresholds once True.


Suggest pattern: suppose we want constraint conjunctions (a,b), (b,c), (c,d) achieved in this order

Then need garden
G:constrainedWaypoint([c,d b,c a,b])
  P:constrainingWaypoint([c,d b,c a,b]) # current waypoint obeys both c and d
    G:constrainedWaypoint([b,c a,b])
      P:constrainingWaypoint([b,c a,b]) # current waypoint obeys both b and c
        G:constrainedWaypoint([a,b])
          P:constrainingWaypoint([a,b]) # current waypoint obeys both a and b
            []
For slicing:
  we want ~paad: slicepoint oscillates along blade axis over item and bladeaxis points down
  we can achieve ~paad by an oscillating process that maintains adSR: bladeaxis down and blade in XYZ segregion just over item
  we can achieve adSR by a lowering process that maintains adSRXY: bladeaxis down and blade in XY segregion over item
  we can achieve adSRXY by a tipping process that maintains SRXYez: blade in XY segregion over item and handZ at entryHeight
  we can achieve SRXYez by a bringing process that maintains ezpo: handZ at entryHeight and handOrientation at parkedOrientation
  we can achieve ezpo by a lifting process that maintains po: handOrientation at parkedOrientation

Just one issue: the process must actually maintain the constraints it says; i.e., a final waypoint is in general not sufficient. A quick and dirty alt. is to have the constraints be tolerant once met.

'''

'''
G:notifiedCompletion() // goal status never set to True -- wait for sim update loop to detect it and respond accordingly
  P:notifyCompletion() // bpt: (processGardening, completed): (True)

G:noped() // goal never completes, used to stop a goal loop
  P:notifyCompletion() // bpt: None


G:near
  P:nearing
    G:alignedBaseYaw
      P:aligningBaseYaw
        G:baseCentered
          P:baseCentering

G:parkedArm
  P:parkingArm
    G:parkedXY
      P:parkingXY
        G:liftedToEntry
          P:liftingToEntry

G:armNearItemHandle
  P:lowering (arm, handle)
    G:aboveLocation (arm, handle)
      P:hoveringLocation (arm, handle)
        G:liftedToEntry

G:pickedItem
  P:pickingItem
    G:grasped
    G:parkedArm

G:placedItem
  P:placingItem
    G:loweredItemAt,ungrasped,parked
      P:lowering (item)
        G:aboveLocation (item)
          P:hoveringLocation (item)
            G:liftedToEntry

G:transferred
  P:transferring
    G:transferredContents,itemUpright,placedItem
      P:transferringContents
        G:aboveLocation (pourPoint)
      P:itemUprighting

G:mixed
  P:mixing
    G:mixedContents,itemUpright,placedItem
      P:mixingContents
        G:mixerTippedAbove, lowered(mixer point)
         P:mixerTippingAbove
           G:aboveLocation (item)

G:shaped
  P:shaping
   G:depositedShape
     P:depositingShape
       G:createdShape,placedNearShape
         P:creatingShape
           G:lowering (hand)
         P:placingNearShape
           G:placedItem

G:baked
  P:baking
    G:placedItem


G:opened(Item, _Hand)
  P:opening(Item,_Hand) // bpt: (clopening, opening, _Hand): (True); (kinematicControl, target, _Hand): (next pos in handle traj, at-orn)
    G:near(Item)...
    G:armTriggeredPortalHandle(Item,_Hand)
      P:movingArm...
    G:parkedArm(_Hand)
      P:movingArm...
G:stoppedOpening(_Hand)
  P:stopOpening(_Hand) // bpt: (clopening, opening, _Hand): (False)

G:closed(Item, _Hand)
  P:closing(Item,_Hand) // bpt: (clopening, closing, _Hand): (True); (kinematicControl, target, _Hand): (next pos in handle traj, at-orn)
    G:near(Item)...
    G:armTriggeredPortalHandle(Item,_Hand)
      P:movingArm...
    G:parkedArm(_Hand)
      P:movingArm...
G:stoppedClosing(_Hand)
  P:stopClosing(_Hand) // bpt: (clopening, closing, _Hand): (False)

G:grasped(Item, _Hand)
  P:grasping(Item, _Hand) // bpt: (grasping, intendToGrasp, _Hand): (+Item)
    G:opened(at(Item))...
    G:stoppedOpening(_OHand)...
    G:near(Item)
      P:movingBaseTo(Item) // bpt: (kinematicControl, target, base): (Item pos, at-orn)
    G:armNearItemHandle(Item, _Hand) 
      P:movingArm(_Hand) // bpt: (kinematicControl, target, _Hand): (above Item handle pos, at-orn)

G:ungrasped(Item, _Hand)
  P: ungrasping(Item, _Hand) // bpt: (grasping, intendToGrasp, _Hand): (-Item)

G:pickedItem(Item, _Hand)
  P:pickingItem(Item, at(Item), _Hand)
    G:grasped(Item, _Hand)...
    G:parkedArm(_Hand)

G:placedItem(Item, Container, _Hand, _SourceContainer, _SourceComponent)
  P:placingItem(Item, Container, _Hand, _SourceContainer, _SourceComponent) OR ungrasping(Item, _Hand) OR parkingArm(hand) depending on item/arm pos
    G:pickedItem(Item,_Hand);closed(_SourceContainer, _SourceComponent, _OHand);parked(_OHand);near(Container);opened(Container,_Component,_OHand)

G:broughtNear(Trajector, Hand, Relatum)
  P:bringingNear(Trajector, Hand, Relatum, SourceContainer, SourceComponent)
    G:pickedItem(Item, Hand, SourceContainer, SourceComponent)
    G:closed(SourceContainer, SourceComponent, _OHand)
    G:near(Relatum)

G:bakedItem(Item, BakedType, Oven, Destination)
  P:bakingItem(Item, BakedType, Oven, Destination)
    G:placedItem(Item, Oven) OR G:placedItem(Item,Destination,Oven,_OvenComponent) depending on baked status

G:transferred(Item, Hand, Container, Storage, Amount, PouredType)
  P:transferring(Item, Hand, Container, Storage)
    G:broughtNear(Item, Hand, Container)
      OR
    G:parkedArm(Hand)
      OR
    G:placedItem(Item, Hand, Storage)

G:sprinkled(Item, SprinkledType, Hand, Shaker, Storage)
  P:sprinkling(Item, SprinkledType, Hand, Shaker, Storage)
    G:broughtNear(Shaker, Hand, Item)
      OR
    G:parkedArm(Hand)
      OR
    G:placedItem(Shaker, Hand, Storage)

G:mixed(Item, MixedType, Hand, Mixer, Storage)
  P:mixing(Item, MixedType, Hand, Mixer, Storage)
    G:broughtNear(Mixer, Hand, Item)
      OR
    G:placedItem(Mixer, Hand, Storage)

G:shaped(Item, ShapedType, Hand, Storage)
  P:shaping(Item, ShapedType, Hand, Storage)
    G:near(Item)
      OR
    G:placedItem(_NewShape,Hand,Storage)

// seeding, peeling, sprinkling; 
G:completedGestureTriggeredProcess(Item, Instrument, Process, _THand)
  P:completingGestureTriggeredProcess(Item, Instrument, Process) // bpt: (kinematicControl, target, _THand): (pose)
    G:pickedItem(Instrument)
    G:broughtItem(Item, working location, _PHand)

//mashing, grinding, mixing;
G:completedGestureTriggeredProcess(Item, Instrument, Process, _THand)
  P:completingGestureTriggeredProcess(Item, Instrument, Process) // bpt: (kinematicControl, target, _THand): (pose)
    G:pickedItem(Instrument)
    G:near(Item)

//baking, boiling, freezing
G:completedPlacementTriggeredProcess(Item, ...)
  P:completingPlacementTriggeredProcess(Item, ...)
    G:placedItem(Item, ...)

'''

