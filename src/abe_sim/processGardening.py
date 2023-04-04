import copy
import math
import numpy

import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

from abe_sim.world import getDictionaryEntry, stubbornTry
from abe_sim.crane import getWaypointTarget, point3DSegmentCloseness, quaternionCloseness, pointCloseness

baseFwdOffset = (1.5, 0, 0)

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
    held = getHeld(customDynamicsAPI, name, hand)
    aabb = [[handPosition[0]-0.1, handPosition[1]-0.1, -0.1], [handPosition[0]+0.1, handPosition[1]+0.1, handPosition[2]]]
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
    ornTh = 0.99
    if previous:
        posTh = 0.0004
        ornTh = 0.95
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

def _checkAlignedBaseYaw(customDynamicsAPI, name, description, node):
    relatum = description.get('relatum', None)
    previous = node.get('previousStatus', False)
    relatumPosition = list(customDynamicsAPI['getObjectProperty']((relatum,), 'position'))
    facingYaw = getFacingYaw(customDynamicsAPI, relatum, relatumPosition)
    facingQ = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,facingYaw)))
    baseLink = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'mobileBaseLink'))
    baseOrientation = customDynamicsAPI['getObjectProperty']((name, baseLink), 'orientation')
    ornThreshold = 0.99
    if previous:
        ornThreshold = 0.95
    node['previousStatus'] = quaternionCloseness(baseOrientation, facingQ, ornThreshold)
    return node['previousStatus']

def _checkBaseCentered(customDynamicsAPI, name, description, node):
    baseEFPosition = getBaseEFPosition(customDynamicsAPI, name)
    posThreshold = 0.01
    if node.get('previousStatus', False):
        posThreshold = 0.04
    node['previousStatus'] = bool(posThreshold > numpy.dot(baseEFPosition, baseEFPosition))
    return node['previousStatus']

def _checkParkedArm(customDynamicsAPI, name, description, node):
    hand = description['hand']
    previous = node.get('previousStatus', False)
    return checkParked(customDynamicsAPI, name, hand, previous)

def _checkParkedXY(customDynamicsAPI, name, description, node):
    hand = description['hand']
    previous = node.get('previousStatus', False)
    return checkParkedXY(customDynamicsAPI, name, hand, previous)

def _checkLiftedToEntry(customDynamicsAPI, name, description, node):
    hand = description['hand']
    previous = node.get('previousStatus', False)
    handLink = getHandLink(customDynamicsAPI, name, hand)
    positionHand = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    entryHeight = getEntryHeight(customDynamicsAPI, name, description, node['numerics'])
    posTh = 0.01
    if previous:
        posTh = 0.02
    node['previousStatus'] = bool(posTh > abs(positionHand[2] - entryHeight))
    return node['previousStatus']
    
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

def _checkAboveLocation(customDynamicsAPI, name, description, node):
    hand = description['hand']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    return checkIdentifierAboveLocation(customDynamicsAPI, (name, handLink), node)

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

def _checkLined(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    held = getHeld(customDynamicsAPI, name, hand)
    return (0 < len([x for x in getContents(customDynamicsAPI, item) if customDynamicsAPI['getObjectProperty']((x,), ('fn', 'canLine'), False) and x not in held]))

def _checkMixed(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    mixedType = description.get('mixedType', None)
    ingredientTypes = description.get('ingredientTypes', [])
    contents = getContents(customDynamicsAPI, item)
    mixed = [x for x in contents if mixedType == customDynamicsAPI['getObjectProperty']((x,), 'type')]
    ingredients = [x for x in contents if customDynamicsAPI['getObjectProperty']((x,), 'type') in ingredientTypes]
    return (0 == len(ingredients)) and (0 < len(mixed))

def _checkShaped(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    destination = description.get('destination', None)
    shapedType = description.get('shapedType', None)
    ingredientTypes = description.get('ingredientTypes', [])
    ingredients = [x for x in getContents(customDynamicsAPI, item) if customDynamicsAPI['getObjectProperty']((x,), 'type') in ingredientTypes]
    shapes = [x for x in getContents(customDynamicsAPI, destination) if shapedType == customDynamicsAPI['getObjectProperty']((x,), 'type')]
    return (0 == len(ingredients)) and (0 < len(shapes))
    
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
    upright = checkUpright(itemOrientation, pouringAxis)
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

def _checkSprinkled(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    shaker = description.get('shaker', None)
    storage = description.get('storage', None)
    sprinkledType = description.get('sprinkledType', None)
    parked = node.get('parked', False)
    parked = checkParked(customDynamicsAPI, name, hand, parked)
    node['parked'] = parked
    return checkSprinkled(customDynamicsAPI, name, item, sprinkledType) and (not checkGrasped(customDynamicsAPI, name, None, shaker)) and checkItemInContainer(customDynamicsAPI, name, shaker, storage) and parked

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
    #source = description.get('sourceContainer', None)
    #sourcePart = description.get('sourceComponent', None)
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

# Lined, Mixed, Shaped  

def _getNearingConditions(customDynamicsAPI, name, description, node):
    return [_makeGoal({'relatum': description['relatum']}, 'alignedBaseYaw')]

def _getAligningBaseYawConditions(customDynamicsAPI, name, description, node):
    return [_makeGoal({}, 'baseCentered')]

def _getParkingArmConditions(customDynamicsAPI, name, description, node):
    return [_makeGoal({'hand': description['hand']}, 'parkedXY')]

def _getParkingXYConditions(customDynamicsAPI, name, description, node):
    return [_makeGoal({'hand': description['hand']}, 'liftedToEntry')]

def _getLoweringConditions(customDynamicsAPI, name, description, node):
    return [_makeGoal({'relatum': description['item']}, 'near',
                      numerics={'position': node['numerics'].get('position', None)}),
            _makeGoal({'hand': description['hand'], 'item': description['item']}, 'aboveLocation',
                      numerics={'position': node['numerics'].get('position', None), 'orientation': node['numerics'].get('orientation', None), 'entryHeight': node['numerics'].get('entryHeight', None)})]

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
    freeHand = getFreeHand(customDynamicsAPI, name, hand)
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
    
'''    
    # TODO: source, sourcePart? just to make sure we close things when picking them up
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    pouringAxis = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'axis'), [1,0,0])
    upright = node.get('upright', False)
    upright = checkUpright(itemOrientation, pouringAxis)
    node['upright'] = upright
    if checkTransferred(customDynamicsAPI, name, item, pouredType, amount, container) and upright:
        previouslyParked = node.get('previouslyParked', False)
        if not previouslyParked:
            previouslyParked = checkParked(customDynamicsAPI, name, hand, False)
        node['previouslyParked'] = previouslyParked
        if previouslyParked:
            return [{'type': 'G', 'description': {'goal': 'placedItem', 'item': item, 'hand': hand, 'container': storage}}]
        else:
            return [{'type': 'G', 'description': {'goal': 'parkedArm', 'hand': hand}}]
    else:
        return [{'type': 'G', 'description': {'goal': 'broughtNear', 'trajector': item, 'hand': hand, 'relatum': container}}]
'''


def _getLiningConditions(customDynamicsAPI, name, description, node):
    lining = description.get('lining', None)
    hand = description.get('hand', None)
    item = description.get('item', None)
    source = description.get('sourceContainer', None)
    sourcePart = description.get('sourceComponent', None)
    placed = {'type': 'G', 'description': {'goal': 'placedItem', 'item': lining, 'hand': hand, 'container': item, 'sourceContainer': source, 'sourceComponent': sourcePart, 'matchOrientation': True}}
    return [placed]

def _getMixingConditions(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    tool = description.get('tool', None)
    hand = description.get('hand', None)
    picked = {'type': 'G', 'description': {'goal': 'pickedItem', 'item': tool, 'hand': hand}}
    near = {'type': 'G', 'description': {'goal': 'near', 'item': item}}
    source = description.get('sourceContainer', None)
    sourcePart = description.get('sourceComponent', None)
    retq = [picked]
    freeHand = getFreeHand(customDynamicsAPI, name, hand)
    if source is not None:
        closed = {'type': 'G', 'description': {'goal': 'closed', 'container': source, 'component': sourcePart, 'hand': freeHand}}
        parked = {'type': 'G', 'description': {'goal': 'parkedArm', 'hand': freeHand}}
        retq = retq + [closed, parked]
    retq.append(near)
    furniture, component = getContainerComponent(customDynamicsAPI, item)
    if furniture is not None:
        retq.append({'type': 'G', 'description': {'goal': 'opened', 'container': furniture, 'component': component, 'hand': freeHand}})
        retq.append(copy.deepcopy(parked))
    return retq

def _getShapingConditions(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    destination = description.get('destination', None)
    source = description.get('sourceContainer', None)
    sourcePart = description.get('sourceComponent', None)
    hand = description.get('hand', None)
    furniture, component = getContainerComponent(customDynamicsAPI, item)
    placed = {'type': 'G', 'description': {'goal': 'placedItem', 'item': destination, 'hand': hand, 'container': furniture, 'sourceContainer': source, 'sourceComponent': sourcePart}}
    near = {'type': 'G', 'description': {'goal': 'near', 'item': item}}
    # TODO: alternate between near to item and near to destination depending on whether holding a shapedType
    retq = [placed, near]
    return retq
    
def _getOpeningConditions(customDynamicsAPI, name, description, node):
    hand = getDoorManipulationHand(customDynamicsAPI, name, description.get('hand', None))
    container = description.get('container', None)
    component = description.get('component', None)
    near = {'type':'G', 'description':{'goal': 'near', 'item': container}, 'children':[]}
    touched = {'type':'G', 'description':{'goal': 'armTriggeredPortalHandle', 'container': container, 'component': component, 'hand': hand, 'action': 'open'}, 'children':[]}
    parked = {'type':'G', 'description':{'goal': 'parkedArm', 'hand': hand}}
    return [near, touched, parked]

def _getClosingConditions(customDynamicsAPI, name, description, node):
    hand = getDoorManipulationHand(customDynamicsAPI, name, description.get('hand', None))
    container = description.get('container', None)
    component = description.get('component', None)
    near = {'type':'G', 'description':{'goal': 'near', 'item': container}, 'children':[]}
    touched = {'type':'G', 'description':{'goal': 'armTriggeredPortalHandle', 'container': container, 'component': component, 'hand': hand, 'action': 'close'}, 'children':[]}
    parked = {'type':'G', 'description':{'goal': 'parkedArm', 'hand': hand}}
    return [near, touched, parked]

def _getBakingConditions(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    oven = description.get('oven', None)
    destination = description.get('destination', None)
    bakedType = description.get('bakedType', None)
    source = node.get('sourceContainer')
    sourcePart = node.get('sourceComponent')
    if checkBakedContents(customDynamicsAPI, item, bakedType):
        return [{'type': 'G', 'description': {'goal': 'placedItem', 'item': item, 'hand': hand, 'container': destination, 'sourceContainer': sourceContainer, 'sourceComponent': sourceComponent}}]
    return [{'type': 'G', 'description': {'goal': 'placedItem', 'item': item, 'hand': hand, 'container': oven}}]

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

def _getSprinklingConditions(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    storage = description.get('storage', None)
    shaker = description.get('shaker', None)
    sprinkledType = description.get('sprinkledType', None)
    shakerOrientation = customDynamicsAPI['getObjectProperty']((shaker,), 'orientation')
    pouringAxis = customDynamicsAPI['getObjectProperty']((shaker,), ('fn', 'containment', 'pouring', 'outof', 'axis'), [1,0,0])
    # source, sourcePart? just to make sure we close things when picking them up
    upright = node.get('upright', False)
    upright = checkUpright(shakerOrientation, pouringAxis)
    node['upright'] = upright
    if checkSprinkled(customDynamicsAPI, name, item, sprinkledType) and upright:
        previouslyParked = node.get('previouslyParked', False)
        if not previouslyParked:
            previouslyParked = checkParked(customDynamicsAPI, name, hand, False)
        node['previouslyParked'] = previouslyParked
        if previouslyParked:
            return [{'type': 'G', 'description': {'goal': 'placedItem', 'item': shaker, 'hand': hand, 'container': storage}}]
        else:
            return [{'type': 'G', 'description': {'goal': 'parkedArm', 'hand': hand}}]
    else:
        return [{'type': 'G', 'description': {'goal': 'broughtNear', 'trajector': shaker, 'hand': hand, 'relatum': item}}]

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

def _suggestAlignedBaseYaw(customDynamicsAPI, name, description, node):
    relatum = description.get('relatum', None)
    relatumPosition = list(customDynamicsAPI['getObjectProperty']((relatum,), 'position'))
    facingYaw = getFacingYaw(customDynamicsAPI, relatum, relatumPosition)
    facingQ = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,facingYaw)))
    return [_makeProcess({'relatum': description['relatum']}, 'aligningBaseYaw',
                         target={'hand_left': {'target': None}, 'hand_right': {'target': None},
                                 'base': {'target': [[0,0,0], facingQ], 'positionInLink': baseFwdOffset, 'orientationInLink': (0,0,0,1)}})]

def _suggestBaseCentered(customDynamicsAPI, name, description, node):
    baseLink = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'mobileBaseLink'))
    baseOrientation = customDynamicsAPI['getObjectProperty']((name, baseLink), 'orientation')
    return [_makeProcess(description, 'baseCentering', target={'hand_left': {'target': None}, 'hand_right': {'target': None},
                                                              'base': {'target': [[0,0,0], baseOrientation], 'positionInLink': baseFwdOffset, 'orientationInLink': (0,0,0,1)}})]

def _suggestParkedArm(customDynamicsAPI, name, description, node):
    hand = description['hand']
    parkedP, parkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
    return [_makeProcess({'hand': hand}, 'parkingArm', target={hand: {'target': [parkedP, parkedQ], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}})]

def _suggestParkedXY(customDynamicsAPI, name, description, node):
    hand = description['hand']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    parkedP, parkedQ, _ = getParkedPose(customDynamicsAPI, name, hand)
    parkedPAdj = [parkedP[0], parkedP[1], handPosition[2]]
    return [_makeProcess({'hand': hand}, 'parkingXY', target={hand: {'target': [parkedPAdj, parkedQ], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}})]

def _suggestLiftedToEntry(customDynamicsAPI, name, description, node):
    hand = description['hand']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    entryHeight = getEntryHeight(customDynamicsAPI, name, description, node['numerics'])
    liftedPosition = [handPosition[0], handPosition[1], entryHeight]
    return [_makeProcess({'hand': hand}, 'liftingToEntry',
                         numerics={},
                         target={hand: {'target': [liftedPosition, handOrientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}})]

def _suggestArmNearItemHandle(customDynamicsAPI, name, description, node):
    item = description['item']
    hand = description['hand']
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    aabbItem = customDynamicsAPI['getObjectProperty']((item,), 'aabb')
    handleLink = getHandleLink(customDynamicsAPI, item)
    aabbHandLocal = customDynamicsAPI['getObjectProperty']((item, handleLink), 'localAABB')
    handlePosition = customDynamicsAPI['getObjectProperty']((item, handleLink), 'position')
    handleAxisInWorld = getAxisInWorld(customDynamicsAPI, item, ('fn', 'grasping', 'axis', handleLink))
    handleAlignedOrientation = handOrientation
    if handleAxisInWorld is not None:
        handleYaw = math.atan2(handleAxisInWorld[1], handleAxisInWorld[0])
        handleAlignedOrientation = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,handleYaw)))
    touchingPosition = list(handlePosition)
    touchingPosition[2] = aabbItem[1][2] - aabbHandLocal[0][2]
    entryHeight = getEntryHeight(customDynamicsAPI, name, description, {})
    return [_makeProcess({'hand': hand, 'item': item, 'entryHeight': entryHeight}, 'lowering', 
                         numerics={'position': touchingPosition, 'orientation': handleAlignedOrientation, 'entryHeight': entryHeight},
                         target={hand: {'target': [touchingPosition, handleAlignedOrientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}})]

def _suggestAboveLocation(customDynamicsAPI, name, description, node):
    hand = description['hand']
    position = node['numerics'].get('position', None)
    orientation = node['numerics'].get('orientation', None)
    entryHeight = node['numerics'].get('entryHeight', None)
    abovePosition = list(position)
    abovePosition[2] = entryHeight
    return [_makeProcess({'hand': description['hand'], 'item': description['item']}, 'hoveringLocation',
                         numerics={'entryHeight': node['numerics'].get('entryHeight', None)},
                         target={hand: {'target': [abovePosition, orientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}})]
    
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

def _suggestLined(customDynamicsAPI, name, description, node):
    lining = description.get('lining', None)
    hand = description.get('hand', None)
    item = description.get('item', None)
    source = description.get('sourceContainer', None)
    sourcePart = description.get('sourceComponent', None)
    liningP = {'type': 'P', 'description': {'process': 'lining', 'lining': lining, 'hand': hand, 'item': item, 'sourceContainer': source, 'sourceComponent': sourcePart}}
    return [liningP]

def _suggestMixed(customDynamicsAPI, name, description, node):
    # Bring mixing tool near container
    # TODO: explicitly have upright orientation in waypoints
    item = description.get('item', None)
    hand = description.get('hand', None)
    tool = description.get('tool', None)
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    itemPosition = customDynamicsAPI['getObjectProperty']((item,), 'position')
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    toolPosition = customDynamicsAPI['getObjectProperty']((tool,), 'position')
    toolOrientation = customDynamicsAPI['getObjectProperty']((tool,), 'orientation')
    toolPoint = customDynamicsAPI['getObjectProperty']((tool,), ('fn', 'mixing', 'toolPoint'))
    toolPointInWorld, _ = customDynamicsAPI['objectPoseRelativeToWorld'](toolPosition, toolOrientation, toolPoint, (0,0,0,1))
    toolPInHand, toolOInHand = customDynamicsAPI['objectPoseRelativeToObject'](handPosition, handOrientation, toolPointInWorld, toolOrientation)
    containerEntry = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'into', 'point'))
    containerEntry, _ = customDynamicsAPI['objectPoseRelativeToWorld'](itemPosition, itemOrientation, containerEntry, (0,0,0,1))
    tippedOrientation = customDynamicsAPI['getObjectProperty']((tool,), ('fn', 'mixing', 'tipped'))
    container, component = getContainerComponent(customDynamicsAPI, item)
    componentHeight = getComponentHeight(customDynamicsAPI, container, component)
    componentBase = customDynamicsAPI['getObjectProperty']((container, component), 'aabb')[1][2]
    mixingEntryHeight = componentHeight + componentBase - handPosition[2] + toolPointInWorld[2]
    waypoints = [[[toolPosition[0], toolPosition[1], mixingEntryHeight], toolOrientation], [[containerEntry[0], containerEntry[1], mixingEntryHeight], tippedOrientation], [containerEntry, tippedOrientation]]
    print('Mixed')
    pose = getWaypointTarget(customDynamicsAPI, name, node['updateable'], toolPosition, toolOrientation, waypoints)
    target = {hand: {'target': pose, 'positionInLink': toolPInHand, 'orientationInLink': toolOInHand, 'clopeningAction': None}}
    return [{'type': 'P', 'description': {'process': 'movingArm', 'hand': hand}, 'target': target}]

def _suggestShaped(customDynamicsAPI, name, description, node):
    item = description.get('item', None)
    hand = description.get('hand', None)
    destination = description.get('destination', None)
    shapedType = description.get('shapedType', None)
    ingredientTypes = description.get('ingredientTypes', [])
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    destinationPosition = customDynamicsAPI['getObjectProperty']((destination,), 'position')
    destinationOrientation = customDynamicsAPI['getObjectProperty']((destination,), 'orientation')
    containerEntry = customDynamicsAPI['getObjectProperty']((destination,), ('fn', 'containment', 'pouring', 'into', 'point'))
    containerEntry, _ = customDynamicsAPI['objectPoseRelativeToWorld'](destinationPosition, destinationOrientation, containerEntry, [0,0,0,1])
    graspingShapes = [x for x in getHeld(customDynamicsAPI, name, hand) if shapedType == customDynamicsAPI['getObjectProperty']((x,), 'type')]
    graspingShape = None
    if 0 < len(graspingShapes):
        graspingShape = graspingShapes[0]
    aabb = customDynamicsAPI['getObjectProperty']((name, handLink), 'aabb')
    aabbAdj = [[x-0.1 for x in aabb[0]], [x+0.1 for x in aabb[1]]]
    overlaps = customDynamicsAPI['checkOverlap'](aabbAdj)
    closeShapes = [x for x in overlaps if (x not in graspingShapes) and (shapedType == customDynamicsAPI['getObjectProperty']((x,), 'type'))]
    closeShape = None
    if 0 < len(closeShapes):
        closeShape = closeShapes[0]
    ingredients = [x for x in overlaps if customDynamicsAPI['getObjectProperty']((x,), 'type') in ingredientTypes]
    closeToIngredients = (0 < len(ingredients))
    if graspingShape is not None:
        descriptionAux = {'item': graspingShape, 'container': destination, 'hand': hand}
        if 'updateable' not in node:
            node['updateable'] = {}
        nodeAux = {'updateable': node['updateable']}
        target = _suggestPlacedItem(customDynamicsAPI, name, descriptionAux, nodeAux)['target']
        target['shaping'] = {hand: None}
        return [{'type': 'P', 'description': {'process': 'movingArm', 'hand': hand}, 'target': target}]
    elif closeShape is not None:
        target = {'grasping': {}, 'shaping': {}}
        target['grasping'][hand] = {'toAdd': [closeShape]} # bpt: (grasping, intendToGrasp, _Hand): (+Item)
        target['shaping'][hand] = None
        return [{'type': 'P', 'description': {'process': 'movingArm', 'hand': hand}, 'target': target}]
    elif closeToIngredients:
        target = {'shaping': {}}
        target['shaping'][hand] = shapedType # bpt: (grasping, intendToGrasp, _Hand): (+Item)
        return [{'type': 'P', 'description': {'process': 'movingArm', 'hand': hand}, 'target': target}]
    else:
        descriptionAux = {'item': destination, 'hand': hand, 'handlePosition': containerEntry}
        if 'updateable' not in node:
            node['updateable'] = {}
        nodeAux = {'updateable': node['updateable']}
        retq = _suggestArmNearItemHandle(customDynamicsAPI, name, descriptionAux, nodeAux)
        retq['target']['shaping'] = {hand: None}
        return retq

def _suggestSprinkled(customDynamicsAPI, name, description, node):
    descriptionC = copy.deepcopy(description)
    descriptionC['process'] = 'transferring'
    descriptionC.pop('goal')
    item = description.get('item', None)
    itemPosition = customDynamicsAPI['getObjectProperty']((item,), 'position')
    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
    shaker = description.get('container', None)
    shakerPosition = customDynamicsAPI['getObjectProperty']((shaker,), 'position')
    shakerOrientation = customDynamicsAPI['getObjectProperty']((shaker,), 'orientation')
    pouringAxis = customDynamicsAPI['getObjectProperty']((shaker,), ('fn', 'containment', 'pouring', 'outof', 'axis'), [1,0,0])
    sprinkledType = description.get('sprinkledType', None)
    upright = checkUpright(shakerOrientation, pouringAxis)
    if checkSprinkled(customDynamicsAPI, name, item, sprinkledType) and upright:
        return [{'type': 'P', 'description': descriptionC, 'target': None}]
    hand = description.get('hand', None)
    componentHeight = getComponentHeight(customDynamicsAPI, item, None)
    componentBase = customDynamicsAPI['getObjectProperty']((item,), 'aabb')[1][2]
    handLink = getHandLink(customDynamicsAPI, name, hand)
    handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
    handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
    pouringPoint = customDynamicsAPI['getObjectProperty']((shaker,), ('fn', 'containment', 'pouring', 'outof', 'point'))
    pouringPointWorld, pouringOrientationWorld = customDynamicsAPI['objectPoseRelativeToWorld'](shakerPosition, shakerOrientation, pouringPoint, (0,0,0,1))
    pouringPointInHand, pouringOrientationInHand = customDynamicsAPI['objectPoseRelativeToObject'](handPosition, handOrientation, pouringPointWorld, pouringOrientationWorld)
    uprightOrientation = getUprightOrientation(customDynamicsAPI, shaker, shakerOrientation, pouringAxis)
    targetOrientation = getTippedOrientation(customDynamicsAPI, shaker, shakerOrientation, pouringAxis)
    pouringEntryHeight = componentHeight + componentBase - handPosition[2] + pouringPointWorld[2]
    if checkSprinkled(customDynamicsAPI, name, item, sprinkledType):
        waypoints = [[[shakerPosition[0], shakerPosition[1], pouringEntryHeight], uprightOrientation]]
    else:
        sprinklee = getSprinklee(customDynamicsAPI, name, item, sprinkledType, node.get('sprinklee', None))
        node['sprinklee'] = sprinklee
        sprinkleePosition = customDynamicsAPI['getObjectProperty']((sprinklee,), 'position')
        # TODO: enforce an upright orientation at the start of the maneuver
        waypoints = [[[shakerPosition[0], shakerPosition[1], pouringEntryHeight], shakerOrientation], [[sprinkleePosition[0], sprinkleePosition[1], pouringEntryHeight], targetOrientation]]
    if 'updateable' not in node:
        node['updateable'] = {}
    print('Sprinkling')
    pose = getWaypointTarget(customDynamicsAPI, name, node['updateable'], itemPosition, itemOrientation, waypoints)
    target = {hand: {'target': pose, 'positionInLink': pouringPointInHand, 'orientationInLink': pouringOrientationInHand, 'clopeningAction': None}}
    return [{'type': 'P', 'description': descriptionC, 'target': target}]
  
goalCheckers = {
    'noped': _alwaysFalse,
    'notifiedCompletion': _alwaysFalse,
    'near': _checkNear, # relatum | position
    'alignedBaseYaw': _checkAlignedBaseYaw, # relatum
    'baseCentered': _checkBaseCentered, # <|>
    'parkedArm': _checkParkedArm, # hand |
    'parkedXY': _checkParkedXY, # hand
    'liftedToEntry': _checkLiftedToEntry, # hand ** | entryHeight
    'armNearItemHandle': _checkArmNearItemHandle, # hand, item | 
    'aboveLocation': _checkAboveLocation, # hand, item | position, orientation, entryHeight
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
    'sprinkled': _checkSprinkled,
    'bakedItem': _checkBaked,
    'armTriggeredPortalHandle': _checkArmTriggeredPortalHandle,
    'stoppedOpening': _checkStoppedOpening,
    'opened': _checkOpened,
    'closed': _checkClosed,
    'broughtNear': _checkBroughtNear,
    'followedWaypoints': _checkFollowedWaypoints,
    'lined': _checkLined,
    'mixed': _checkMixed,
    'shaped': _checkShaped
    }
processSuggesters = {
    'notifiedCompletion': _suggestNotifiedCompletion,
    'noped': _suggestNoped,
    'near': _suggestNear, # relatum | position
    'alignedBaseYaw': _suggestAlignedBaseYaw, # relatum
    'baseCentered': _suggestBaseCentered, # <|>
    'parkedArm': _suggestParkedArm, # hand | 
    'parkedXY': _suggestParkedXY, # hand |
    'liftedToEntry': _suggestLiftedToEntry, # hand ** | entryHeight
    'armNearItemHandle': _suggestArmNearItemHandle, # hand, item | 
    'aboveLocation': _suggestAboveLocation, # hand, item | position, orientation, entryHeight
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
    'opened': _suggestOpened,
    'closed': _suggestClosed,
    'stoppedOpening': _suggestStoppedOpening,
    'stoppedClosing': _suggestStoppedClosing,
    'bakedItem': _suggestBakedItem,
    'broughtNear': _suggestBroughtNear,
    'armTriggeredPortalHandle': _suggestArmTriggeredPortalHandle,
    'sprinkled': _suggestSprinkled,
    'lined': _suggestLined,
    'mixed': _suggestMixed,
    'shaped': _suggestShaped
    
    }
conditionListers = {
    'notifyingCompletion': _emptyList,
    'nearing': _getNearingConditions, # relatum | 
    'aligningBaseYaw': _getAligningBaseYawConditions, # relatum
    'baseCentering': _emptyList,
    'parkingArm': _getParkingArmConditions, # hand |
    'parkingXY': _getParkingXYConditions, # hand |
    'liftingToEntry': _emptyList,
    'lowering': _getLoweringConditions, # hand, item | position, orientation, entryHeight
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
    'liftingItemToPouringEntry': _emptyList,
    'stopOpening': _emptyList,
    'stopClosing': _emptyList,
    'movingArm': _emptyList,
    'opening': _getOpeningConditions,
    'closing': _getClosingConditions,
    'bakingItem': _getBakingConditions,
    'bringingNear': _getBringingNearConditions,
    'sprinkling': _getSprinklingConditions,
    'followingWaypoints': _emptyList,
    'lining': _getLiningConditions,
    'mixing': _getMixingConditions,
    'shaping': _getShapingConditions
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
        badOverlaps = [x for x in overlaps if (x not in [name, item, container]) and (not customDynamicsAPI['getObjectProperty']((x,), ('fn', 'canLine'), False))]
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

def getContents(customDynamicsAPI, item):
    aabb = customDynamicsAPI['getObjectProperty']((item,), 'aabb')
    overlaps = customDynamicsAPI['checkOverlap'](aabb)
    return [x for x in overlaps if item == customDynamicsAPI['getObjectProperty']((x,), 'at')]

def getSprinklee(customDynamicsAPI, name, item, sprinkledType, previous):
    if (previous is not None) and (item == customDynamicsAPI['getObjectProperty']((previous,), 'at')):
        return previous
    for x in getContents(customDynamicsAPI, item):
        if (name != x) and (sprinkledType != customDynamicsAPI['getObjectProperty']((x,), 'type')) and (not customDynamicsAPI['getObjectProperty']((x,), ('fn', 'canLine'), False)):
            return x
    return None

def checkUpright(itemOrientation, pouringAxis):
    return bool(0.2 > (numpy.dot(stubbornTry(lambda : pybullet.rotateVector(itemOrientation, pouringAxis)), [0,0,-1])))

#def checkUpright(customDynamicsAPI, name, item, previous):
#    itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
#    uprightOrientation = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'upright'))
#    threshold = 0.95
#    if previous:
#        threshold = 0.85
#    return pose6DDistance([0,0,0], itemOrientation, [0,0,0], uprightOrientation, 0.1, threshold)

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
    return container == customDynamicsAPI['getObjectProperty']((item,), 'at')
    #if container == customDynamicsAPI['getObjectProperty']((item,), 'at'):
    #    return True
    #aabb = list(customDynamicsAPI['getObjectProperty']((item,), 'aabb'))
    #aabb[0] = list(aabb[0])
    #aabb[0][2] -= 0.15
    #return container in customDynamicsAPI['checkOverlap'](aabb)

def checkBakedContents(customDynamicsAPI, item, bakedType):
    contents = getContents(customDynamicsAPI, item)
    for x in contents:
        if bakedType == customDynamicsAPI['getObjectProperty']((x,), 'type'):
            return True
    return False

def checkSprinkled(customDynamicsAPI, name, item, sprinkledType):
    return 0 == len([x for x in getContents(customDynamicsAPI, item) if (x != name) and (sprinkledType != customDynamicsAPI['getObjectProperty']((x,), 'type')) and (not customDynamicsAPI['getObjectProperty']((x,), ('fn', 'canLine'), False))])

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
            for variable, value in target.items():
                mode = getDictionaryEntry(fnProcessGardening, ('actuator', 'mode', actuator, variable), 'value')
                targetVariable = getDictionaryEntry(fnProcessGardening, ('actuator', 'variablePath', actuator, variable), None)
                if 'set' == mode:
                    toAdd = value.get('toAdd', [])
                    toRemove = value.get('toRemove', [])
                    targetValue = set(customDynamicsAPI['getObjectProperty']((name,), targetVariable, set()))
                    targetValue = list(targetValue.difference(toRemove).union(toAdd))
                else:
                    targetValue = value
                customDynamicsAPI['setObjectProperty']((), targetVariable, targetValue)

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

