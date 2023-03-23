import copy
import math
import numpy

import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

baseFwdOffset = (1.5, 0, 0)

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

def checkNear(customDynamicsAPI, name, item, previous):
    itemPosition = customDynamicsAPI['getObjectProperty']((item,), 'position')
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

def checkGoal(customDynamicsAPI, name, node):
    description = node.get('description', {})
    goal = description.get('goal', 'notifiedCompletion')
    if 'notifiedCompletion' == goal:
        return False # goal status never set to True -- wait for sim update loop to detect it and respond accordingly
    elif 'transferred' == goal:
        item = description.get('item', None)
        hand = description.get('hand', None)
        container = description.get('container', None)
        storage = description.get('storage', None)
        amount = description.get('amount', None)
        pouredType = description.get('pouredType', None)
        parked = node.get('parked', False)
        parked = checkParked(customDynamicsAPI, name, hand, parked)
        node['parked'] = parked
        return checkTransferred(customDynamicsAPI, name, item, pouredType, amount, container) and (not checkGrasped(customDynamicsAPI, name, None, item)) and checkItemInContainer(customDynamicsAPI, name, item, storage) and parked
    elif 'sprinkled' == goal:
        item = description.get('item', None)
        shaker = description.get('shaker', None)
        storage = description.get('storage', None)
        sprinkledType = description.get('sprinkledType', None)
        parked = node.get('parked', False)
        parked = checkParked(customDynamicsAPI, name, hand, parked)
        node['parked'] = parked
        return checkSprinkled(customDynamicsAPI, name, item, sprinkledType) and (not checkGrasped(customDynamicsAPI, name, None, shaker)) and checkItemInContainer(customDynamicsAPI, name, shaker, storage) and parked
    elif 'bakedItem' == goal:
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
    elif 'placedItem' == goal:
        item = description.get('item')
        container = description.get('container')
        hand = description.get('hand', None)
        parked = node.get('parked', False)
        parked = checkParked(customDynamicsAPI, name, hand, parked)
        node['parked'] = parked
        return (not checkGrasped(customDynamicsAPI, name, None, item)) and checkItemInContainer(customDynamicsAPI, name, item, container) and parked
    elif 'armNearItemHandle' == goal:
        item = description.get('item', None)
        hand = description.get('hand', None)
        if (item is not None) and (hand is not None):
            handLink = getHandLink(customDynamicsAPI, name, hand)
            handleLink = getHandleLink(customDynamicsAPI, item)
            maxDistance = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'grasping', 'graspingActivationRadius', hand), 0.5)
            if node.get('previousStatus', False):
                maxDistance = 1.5*maxDistance
            return 0 < len(customDynamicsAPI['checkClosestPoints']((name, handLink), (item, handleLink), maxDistance=maxDistance))
    elif 'parkedArm' == goal:
        previous = node.get('previousStatus', False)
        hand = description.get('hand', None)
        return checkParked(customDynamicsAPI, name, hand, previous)
    elif 'armTriggeredPortalHandle' == goal:
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
    elif 'noped' == goal:
        return False # goal never completes, used to stop a goal loop
    elif 'near' == goal:
        item = description.get('item', None)
        if item is not None:
            previous = node.get('previousStatus', False)
            return checkNear(customDynamicsAPI, name, item, previous)
    elif 'stoppedOpening' == goal:
        hand = description.get('hand', None)
        if hand is not None:
            return 'open' != customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'clopening', 'action', hand), None)
        else:
            clactions = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'clopening', 'action'), {})
            return not any(['open' == v for _, v in clactions.items()])
    elif 'stoppedClosing' == goal:
        hand = description.get('hand', None)
        if hand is not None:
            return not 'close' == customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'clopening', 'action', hand), None)
        else:
            clactions = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'clopening', 'action'), {})
            return not any(['close' == v for _, v in clactions.items()])
    elif 'grasped' == goal:
        item = description.get('item', None)
        if item is not None:
            hand = description.get('hand', None)
            return checkGrasped(customDynamicsAPI, name, hand, item)
    elif 'ungrasped' == goal:
        item = description.get('item', None)
        if item is not None:
            hand = description.get('hand', None)
            return not checkGrasped(customDynamicsAPI, name, hand, item)
    elif 'pickedItem' == goal:
        item = description.get('item', None)
        hand = description.get('hand', None)
        if item is not None:
            previous = node.get('previousStatus', False)
            grasped = checkGrasped(customDynamicsAPI, name, hand, item)
            if previous:
                return grasped
            else:
                return grasped and checkParked(customDynamicsAPI, name, hand, False)
    elif 'opened' == goal:
        container = description.get('container', None)
        component = description.get('component', None)
        handle, door = getHandleAndDoor(customDynamicsAPI, container, component)
        if handle is not None:
            openMinThreshold = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'openMin', door), None)
            openMaxThreshold = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'openMax', door), None)
            jointValue = customDynamicsAPI['getObjectProperty']((container, door), 'jointPosition')
            return ((openMinThreshold is None) or bool(openMinThreshold < jointValue)) and ((openMaxThreshold is None) or bool(openMaxThreshold > jointValue))
    elif 'closed' == goal:
        container = description.get('container', None)
        component = description.get('component', None)
        handle, door = getHandleAndDoor(customDynamicsAPI, container, component)
        if handle is not None:
            closedMinThreshold = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'closedMin', door), None)
            closedMaxThreshold = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'clopening', 'closedMax', door), None)
            jointValue = customDynamicsAPI['getObjectProperty']((container, door), 'jointPosition')
            return ((closedMinThreshold is None) or bool(closedMinThreshold < jointValue)) and ((closedMaxThreshold is None) or bool(closedMaxThreshold > jointValue))
    elif 'broughtNear' == goal:
        trajector = description.get('trajector', None)
        hand = description.get('hand', None)
        relatum = description.get('relatum', None)
        #source = description.get('sourceContainer', None)
        #sourcePart = description.get('sourceComponent', None)
        previous = node.get('previousState', False)
        return checkGrasped(customDynamicsAPI, name, hand, trajector) and checkNear(customDynamicsAPI, name, relatum, previous)
    return True

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
    if 'notifiedCompletion' == goal:
        return [{'type': 'P', 'description': {'process': 'notifyingCompletion'}, 'target': {'internalFlags': {'completion': True}}}] # bpt: (processGardening, completed): (True)
    elif 'noped' == goal:
        return [{'type': 'P', 'description': {'process': 'notifyingCompletion'}, 'target': None}] # bpt: None
    elif 'placedItem' == goal:
        item = description.get('item')
        container = description.get('container')
        if checkItemInContainer(customDynamicsAPI, name, item, container):
            hand = description.get('hand', None)
            if checkGrasped(customDynamicsAPI, name, hand, item):
                target = {'grasping': {}}
                if hand is None:
                    for hande in customDynamicsAPI['getObjectProperty']((name,), ('fn', 'grasping', 'effectors'), []):
                        target['grasping'][hande] = {'toRemove': [item]}
                else:
                    target['grasping'][hand] = {'toRemove': [item]}
                return [{'type': 'P', 'description': {'process': 'ungrasping', 'item': item, 'hand': hand}, 'target': target}]
            parked = node.get('parked', False)
            parked = checkParked(customDynamicsAPI, name, hand, parked)
            node['parked'] = parked
            if not parked: ### redundant, should have a parkingArm process after all
                #handLink = getHandLink(customDynamicsAPI, name, hand)
                #parkedPosition, parkedOrientation, _ = getParkedPose(customDynamicsAPI, name, hand)
                #parkedPosition = list(parkedPosition)
                #movedObjs = getHeld(customDynamicsAPI, name, hand)
                #ats = [customDynamicsAPI['getObjectProperty']((x,), 'at') for x in movedObjs]
                #if any([x is not None for x in ats]):
                #    handLinkPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
                #    parkedPosition[2] = handLinkPosition[2] + 0.01
                #target = {hand: {'target': [parkedPosition, parkedOrientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}}
                target = getParkingTarget(customDynamicsAPI, name, hand)
                return [{'type': 'P', 'description': {'process': 'movingArm', 'hand': hand}, 'target': target}]
            return [] ## Should never be reached: this happens if item is in container, and not grasped, and arm is parked: goal should have then been true and no need for process suggestions
        hand = description.get('hand', None)
        if hand is None:
            hand = getFreeHand(customDynamicsAPI, name, None)
        handLink = getHandLink(customDynamicsAPI, name, hand)
        handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
        handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
        itemPosition = customDynamicsAPI['getObjectProperty']((item,), 'position')
        itemPositionInHand, _ = customDynamicsAPI['objectPoseRelativeToObject'](handPosition, handOrientation, itemPosition, (0,0,0,1))
        previousTargetPosition = node.get('targetPosition')
        component, targetPosition = getItemPlacement(customDynamicsAPI, name, item, container, node.get('component'), previousTargetPosition)
        node['component'] = component
        node['targetPosition'] = targetPosition
        if component is None:
            node['error'] = 'container full'
            return []
        node['error'] = None
        componentHeight = getComponentHeight(customDynamicsAPI, container, component)
        componentBase = customDynamicsAPI['getObjectProperty']((container, component), 'aabb')[1][2]
        if componentHeight is None:
            componentHeight = 0.1
        containerEntryHeight = componentHeight + componentBase
        itemEntryHeight = containerEntryHeight - handPosition[2] + itemPosition[2]
# 1) hand far from containerEntryHeight and itemXY far from targetPositionXY: bring hand to containerEntryHeight
# 2) hand close to containerEntryHeight and itemXY far from targetPositionXY: bring item to targetPositionXY
# 3) itemXY close to targetPositionXY: lower item to targetPosition
        dZ = (handPosition[2] - containerEntryHeight)
        dZ = dZ*dZ
        dXY = [itemPosition[0] - targetPosition[0], itemPosition[1] - targetPosition[1]]
        dXY = numpy.dot(dXY, dXY)
        previousZ = node.get('previousPlacingZ', False)
        ZThresh = 0.0025
        if previousZ:
            ZThresh = 0.01
        closeZ = bool(ZThresh > dZ)
        node['previousPlacingZ'] = closeZ
        previousXY = node.get('previousPlacingXY', False)
        XYThresh = 0.0025
        if previousXY:
            XYThresh = 0.01
        closeXY = bool(XYThresh > dXY)
        node['previousPlacingXY'] = closeXY
        if closeXY:
            target = {hand: {'target': [[targetPosition[0], targetPosition[1], itemPosition[2]-0.005], handOrientation], 'positionInLink': itemPositionInHand, 'orientationInLink': None, 'clopeningAction': None}}
        elif closeZ:
            target = {hand: {'target': [[targetPosition[0], targetPosition[1], itemEntryHeight], handOrientation], 'positionInLink': itemPositionInHand, 'orientationInLink': None, 'clopeningAction': None}}
        else:
            aux = node.get('targetXY', None)
            if (aux is None) or (not checkNear(customDynamicsAPI, name, container, False)):
                aux = [handPosition[0], handPosition[1]]
            if checkNear(customDynamicsAPI, name, container, False):
                node['targetXY'] = aux
            else:
                node['targetXY'] = None
            target = {hand: {'target': [[aux[0], aux[1], containerEntryHeight], handOrientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}}
        source = description.get('sourceContainer', None)
        sourcePart = description.get('sourceComponent', None)
        return [{'type': 'P', 'description': {'process': 'placingItem', 'item': item, 'hand': hand, 'container': container, 'sourceContainer': source, 'sourceComponent': sourcePart}, 'target': target, 'component': component, 'targetPosition': targetPosition}]
    elif 'opened' == goal:
        container = description.get('container', None)
        component = description.get('component', None)
        hand = description.get('hand', None)
        return [{'type': 'P', 'description': {'process': 'opening', 'container': container, 'component': component, 'hand': hand}, 'children': []}]
    elif 'closed' == goal:
        container = description.get('container', None)
        component = description.get('component', None)
        hand = description.get('hand', None)
        return [{'type': 'P', 'description': {'process': 'closing', 'container': container, 'component': component, 'hand': hand}, 'children': []}]
    elif 'armNearItemHandle' == goal:
        item = description.get('item', None)
        hand = description.get('hand', None)
        container, component = getContainerComponent(customDynamicsAPI, item)
        if component is not None:
            componentHeight = getComponentHeight(customDynamicsAPI, container, component)
            componentBase = customDynamicsAPI['getObjectProperty']((container, component), 'aabb')[1][2]
        else:
            componentBase = customDynamicsAPI['getObjectProperty']((item,), 'aabb')[1][2]
            componentHeight = 0.2
        handLink = getHandLink(customDynamicsAPI, name, hand)
        handleLink = getHandleLink(customDynamicsAPI, item)
        handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
        handlePosition = list(customDynamicsAPI['getObjectProperty']((item, handleLink), 'position'))
        handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
        handleAABB = customDynamicsAPI['getObjectProperty']((item, handleLink), 'aabb')
        handAABB = customDynamicsAPI['getObjectProperty']((name, handLink), 'aabb')
        handleZHalf = 0.5*(handleAABB[1][2] - handleAABB[0][2])
        handZHalf = 0.5*(handAABB[1][2] - handAABB[0][2])
        handlePosition[2] = handlePosition[2] + handZHalf + handleZHalf
        if (componentHeight is not None):
            handleApproachHeight = componentHeight + componentBase #handleAABB[1][2]
            dZ = (handPosition[2]-handleApproachHeight)
            dZ = dZ*dZ
            previousZ = node.get('previousZ', False)
            ZThresh = 0.0025
            if previousZ:
                ZThresh = 0.01
            closeZ = bool(ZThresh > dZ)
            node['previousZ'] = closeZ
            dXY = [handPosition[0]-handlePosition[0], handPosition[1]-handlePosition[1]]
            dXY = numpy.dot(dXY, dXY)
            previousXY = node.get('previousXY', False)
            XYThresh = 0.0025
            if previousXY:
                XYThresh = 0.01
            closeXY = bool(XYThresh > dXY)
            node['previousXY'] = closeXY
            if closeZ:
                heightOk = True
            else:
                heightOk = node.get('heightOk', False)
            node['heightOk'] = heightOk
            if heightOk and closeXY:
                xyOk = True
            else:
                xyOk = node.get('xyOk', False)
            node['xyOk'] = xyOk
            if heightOk and xyOk:
                handlePosition = [handlePosition[0], handlePosition[1], handPosition[2] - 0.02]
            elif heightOk:
                handlePosition = [handlePosition[0], handlePosition[1], handleApproachHeight]
            else:
                # TODO: this seems not enough to keep the hand XY-still while approaching the right height; need to record hand position upon entering near status
                handlePosition = [handPosition[0], handPosition[1], handleApproachHeight]
        target = {hand: {'target': [handlePosition, handOrientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}}
        return [{'type': 'P', 'description': {'process': 'movingArm', 'hand': hand}, 'target': target}]
    elif 'parkedArm' == goal:
        hand = description.get('hand', None)
        #handLink = getHandLink(customDynamicsAPI, name, hand)
        #handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
        #parkedPosition, parkedOrientation, height = getParkedPose(customDynamicsAPI, name, hand)
        #parkedPosition = list(parkedPosition)
        #previousXY = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'processGardening', 'auxiliary', 'parkXY', hand), None)
        #if height is None:
        #    previousXY = None
        #else:
        #    if 0.02 < abs(handPosition[2] - height):
        #        if previousXY is None:
        #            previousXY = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')[:2]
        #        parkedPosition = [previousXY[0], previousXY[1], height]
        #    else:
        #        previousXY = None
        #customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'processGardening', 'auxiliary', 'parkXY', hand), previousXY)
        ##movedObjs = [x[0][0] for x in customDynamicsAPI['getObjectProperty']((name,), ('fn', 'grasping', 'actuallyGrasping', hand), [])]
        ##ats = [customDynamicsAPI['getObjectProperty']((x,), 'at') for x in movedObjs]
        ##if any([x is not None for x in ats]):
        ##    handLinkPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
        ##    parkedPosition[2] = handLinkPosition[2] + 0.01
        #target = {hand: {'target': [parkedPosition, parkedOrientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': None}}
        target = getParkingTarget(customDynamicsAPI, name, hand)
        return [{'type': 'P', 'description': {'process': 'movingArm', 'hand': hand}, 'target': target}]
    elif 'armTriggeredPortalHandle' == goal:
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
    elif 'stoppedOpening' == goal:
        hand = description.get('hand', None)
        target = {}
        if hand is None:
            for hand in customDynamicsAPI['getObjectProperty']((name,), ('fn', 'clopening', 'clopeningEFs'), {}).keys():
                target[hand] = {'clopeningAction': None}
        else:
            target[hand] = {'clopeningAction': None}
        return [{'type': 'P', 'description': {'process': 'stopOpening'}, 'target': target}] # bpt: (clopening, opening, _Hand): (False)
    elif 'stoppedClosing' == goal:
        hand = description.get('hand', None)
        target = {}
        if hand is None:
            for hand in customDynamicsAPI['getObjectProperty']((name,), ('fn', 'clopening', 'clopeningEFs'), {}).keys():
                target[hand] = {'clopeningAction': None}
        else:
            target[hand] = {'clopeningAction': None}
        return [{'type': 'P', 'description': {'process': 'stopOpening'}, 'target': target}] # bpt: (clopening, closing, _Hand): (False)
    elif 'ungrasped' == goal:
        item = description.get('item', None)
        if item is not None:
            hand = description.get('hand', None)
            target = {'grasping': {}}
            if hand is None:
                for handC in customDynamicsAPI['getObjectProperty']((name,), ('fn', 'grasping', 'effectors'), []):
                    target['grasping'][handC] = {'toRemove': [item]}
            else:
                target['grasping'][hand] = {'toRemove': [item]}
            return [{'type': 'P', 'description': {'process': 'ungrasping', 'item': item, 'hand': hand}, 'target': target}] # bpt: (grasping, intendToGrasp, _Hand): (-Item)
    elif 'grasped' == goal:
        item = description.get('item', None)
        if item is not None:
            hand = description.get('hand', None)
            if hand is None:
                hand = getFreeHand(customDynamicsAPI, name, None)
            target = {'grasping': {}}
            target['grasping'][hand] = {'toAdd': [item]} # bpt: (grasping, intendToGrasp, _Hand): (+Item)
            return [{'type': 'P', 'description': {'process': 'grasping', 'item': item, 'hand': hand}, 'target': target}]
    elif 'near' == goal:
        item = description.get('item', None)
        if item is not None:
            itemPosition = list(customDynamicsAPI['getObjectProperty']((item,), 'position'))
            facingYaw = getFacingYaw(customDynamicsAPI, item, itemPosition)
            itemPosition[2] = 0
            previousAligned = node.get('previousAligned', False)
            posOffset = baseFwdOffset
            if not previousAligned:
                posOffset = [0,0,0]
            if not previousAligned:
                baseLink = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'mobileBaseLink'))
                basePosition = customDynamicsAPI['getObjectProperty']((name, baseLink), 'position')
                baseOrientation = customDynamicsAPI['getObjectProperty']((name, baseLink), 'orientation')
                baseYaw = stubbornTry(lambda : pybullet.getEulerFromQuaternion(baseOrientation))[2]
                basePositionAdj, _ = customDynamicsAPI['objectPoseRelativeToWorld'](basePosition, baseOrientation, posOffset, (0,0,0,1))
                previousAligned = poseXYYawDistance(basePositionAdj, baseYaw, [0,0,0], facingYaw, 0.1, 0.95)
            node['previousAligned'] = previousAligned
            if previousAligned:
                targetPosition = itemPosition
            else:
                targetPosition = [0,0,0]
            return [{'type': 'P', 'description': {'process': 'movingBase', 'item': item}, 'target': {'hand_left': {'target': None}, 'hand_right': {'target': None}, 'base': {'target': [targetPosition, stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,facingYaw)))], 'positionInLink': posOffset, 'orientationInLink': (0,0,0,1)}}}] # bpt: (kinematicControl, target, base): (Item pos, at-orn)
    elif 'pickedItem' == goal:
        item = description.get('item', None)
        hand = description.get('hand', None)
        if item is not None:
            return [{'type': 'P', 'description': {'process': 'pickingItem', 'item': item, 'hand': hand}}]
    elif 'bakedItem' == goal:
        item = description.get('item', None)
        hand = description.get('hand', None)
        oven = description.get('oven', None)
        destination = description.get('destination', None)
        bakedType = description.get('bakedType', None)
        return [{'type': 'P', 'description': {'process': 'bakingItem', 'item': item, 'hand': hand, 'oven': oven, 'destination': destination, 'bakedType': bakedType}, 'sourceContainer': node.get('sourceContainer'), 'sourceComponent': node.get('sourceComponent')}]
        #container, component = getContainerComponent(customDynamicsAPI, item)
        #if checkBakedContents(customDynamicsAPI, item, bakedType):
        #    return [{'type': 'P', 'description': {'process': 'placingItem', 'item': item, 'hand': hand, 'container': destination, 'sourceContainer': node.get('sourceContainer'), 'sourceComponent': node.get('sourceComponent')}}]
        #elif not (container == oven):
        #    return [{'type': 'P', 'description': {'process': 'placingItem', 'item': item, 'hand': hand, 'container': oven}}]
        #return []
    elif 'broughtNear' == goal:
        #trajector = description.get('trajector', None)
        #hand = description.get('hand', None)
        #relatum = description.get('relatum', None)
        #source = description.get('sourceContainer', None)
        #sourcePart = description.get('sourceComponent', None)
        descriptionC = copy.deepcopy(description)
        descriptionC['process'] = 'bringingNear'
        descriptionC.pop('goal')
        return [{'type': 'P', 'description': descriptionC}]
    elif 'transferred' == goal:
        descriptionC = copy.deepcopy(description)
        descriptionC['process'] = 'transferring'
        descriptionC.pop('goal')
        item = description.get('item', None)
        itemPosition = customDynamicsAPI['getObjectProperty']((item,), 'position')
        itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
        pouringAxis = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'axis'), [1,0,0])
        container = description.get('container', None)
        pouredType = description.get('pouredType', None)
        amount = description.get('amount', None)
        upright = node.get('upright', False)
        upright = checkUpright(itemOrientation, pouringAxis)
        node['upright'] = upright
        if checkTransferred(customDynamicsAPI, name, item, pouredType, amount, container) and upright:
            return [{'type': 'P', 'description': descriptionC, 'target': None}]
        hand = description.get('hand', None)
        component = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'pouring', 'into', 'link'))
        handLink = getHandLink(customDynamicsAPI, name, hand)
        handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
        handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
        containerPosition = customDynamicsAPI['getObjectProperty']((container,), 'position')
        containerOrientation = customDynamicsAPI['getObjectProperty']((container,), 'orientation')
        containerEntry = customDynamicsAPI['getObjectProperty']((container,), ('fn', 'containment', 'pouring', 'into', 'point'))
        containerEntry, _ = customDynamicsAPI['objectPoseRelativeToWorld'](containerPosition, containerOrientation, containerEntry, (0,0,0,1))
        pouringPoint = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'point'))
        pouringPointWorld, pouringOrientationWorld = customDynamicsAPI['objectPoseRelativeToWorld'](itemPosition, itemOrientation, pouringPoint, (0,0,0,1))
        pouringPointInHand, pouringOrientationInHand = customDynamicsAPI['objectPoseRelativeToObject'](handPosition, handOrientation, pouringPointWorld, pouringOrientationWorld)
        componentHeight = getComponentHeight(customDynamicsAPI, container, component)
        componentBase = customDynamicsAPI['getObjectProperty']((container,), 'aabb')[1][2]
        pouringEntryHeight = componentHeight + componentBase - handPosition[2] + pouringPointWorld[2]
        #print('PEH', pouringEntryHeight, componentHeight, componentBase)
        uprightOrientation = node.get('uprightOrientation', None)
        if (uprightOrientation is None) or (not checkUpright(uprightOrientation, pouringAxis)):
            if checkUpright(itemOrientation, pouringAxis):
                uprightOrientation = itemOrientation
            else:
                uprightOrientation = [0,0,0,1] # TODO: assumes the identity orientation is upright
        node['uprightOrietation'] = uprightOrientation
        dZ = (pouringEntryHeight - pouringPointWorld[2])
        dZ = dZ*dZ
        previousZ = node.get('previousZ', False)
        ZThresh = 0.0025
        if previousZ:
            ZThresh = 0.01
        closeZ = bool(ZThresh > dZ)
        node['previousZ'] = closeZ
        dXY = [pouringPointWorld[0]-containerEntry[0], pouringPointWorld[1]-containerEntry[1]]
        dXY = numpy.dot(dXY, dXY)
        previousXY = node.get('previousXY', False)
        XYThresh = 0.0025
        if previousXY:
            XYThresh = 0.01
        closeXY = bool(XYThresh > dXY)
        node['previousXY'] = closeXY
# if closeZ and closeXY: tip
# if not closeZ: get to closeZ, stay upright
# if closeZ but not closeXY: get to closeXY, stay upright
        if not checkTransferred(customDynamicsAPI, name, item, pouredType, amount, container):
            if closeZ and closeXY:
                targetPosition = [containerEntry[0], containerEntry[1], pouringEntryHeight]
                targetOrientation = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'tipped'), [-0.707,0,0,0.707])
            elif not closeZ:
                aux = node.get('transferXY', None)
                if aux is None:
                    aux = [pouringPointWorld[0], pouringPointWorld[1]]
                if checkNear(customDynamicsAPI, name, container, False):
                    node['transferXY'] = aux
                else:
                    node['transferXY'] = None
                targetPosition = [aux[0], aux[1], pouringEntryHeight]
                targetOrientation = uprightOrientation
            else:# closeZ but not closeXY
                targetPosition = [containerEntry[0], containerEntry[1], pouringEntryHeight]
                targetOrientation = uprightOrientation
        else:
            targetPosition = [containerEntry[0], containerEntry[1], pouringEntryHeight]
            targetOrientation = uprightOrientation
        target = {hand: {'target': [targetPosition, targetOrientation], 'positionInLink': pouringPointInHand, 'orientationInLink': pouringOrientationInHand, 'clopeningAction': None}}
        return [{'type': 'P', 'description': descriptionC, 'target': target}]
    elif 'sprinkled' == goal:
        descriptionC = copy.deepcopy(description)
        descriptionC['process'] = 'transferring'
        descriptionC.pop('goal')
        item = description.get('item', None)
        itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
        pouringAxis = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'axis'), [1,0,0])
        shaker = description.get('container', None)
        sprinkledType = description.get('pouredType', None)
        upright = node.get('upright', False)
        upright = checkUpright(itemOrientation, pouringAxis)
        node['upright'] = upright
        if checkSprinkled(customDynamicsAPI, name, item, sprinkledType) and upright:
            return [{'type': 'P', 'description': descriptionC, 'target': None}]
        hand = description.get('hand', None)
        componentHeight = getComponentHeight(customDynamicsAPI, item, None)
        componentBase = customDynamicsAPI['getObjectProperty']((item,), 'aabb')[1][2]
        handLink = getHandLink(customDynamicsAPI, name, hand)
        handPosition = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
        handOrientation = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
        shakerPosition = customDynamicsAPI['getObjectProperty']((shaker,), 'position')
        shakerOrientation = customDynamicsAPI['getObjectProperty']((shaker,), 'orientation')
        pouringPoint = customDynamicsAPI['getObjectProperty']((shaker,), ('fn', 'containment', 'pouring', 'outof', 'point'))
        pouringPointWorld, _ = customDynamicsAPI['objectPoseRelativeToWorld'](shakerPosition, shakerOrientation, pouringPoint, (0,0,0,1))
        pouringPointInHand, _ = customDynamicsAPI['objectPoseRelativeToObject'](handPosition, handOrientation, pouringPointWorld, (0,0,0,1))
        sprinklee = getSprinklee(customDynamicsAPI, name, item, sprinkledType, node.get('sprinklee', None))
        node['sprinklee'] = sprinklee
        sprinkleePosition = customDynamicsAPI['getObjectProperty']((sprinklee,), 'position')
        targetPosition = [sprinkleePosition[0], sprinkleePosition[1], componentHeight + componentBase]
        dXY = [pouringPointWorld[0]-sprinkleePosition[0], pouringPointWorld[1]-sprinkleePosition[1]]
        dXY = numpy.dot(dXY, dXY)
        previousXY = node.get('previousXY', False)
        XYThresh = 0.0025
        if previousXY:
            XYThresh = 0.01
        closeXY = bool(XYThresh > dXY)
        node['previousXY'] = closeXY
        if closeXY and (not checkSprinkled(customDynamicsAPI, name, item, sprinkledType)):
            targetOrientation = customDynamicsAPI['getObjectProperty']((shaker,), ('fn', 'containment', 'pouring', 'outof', 'tipped'))
        else:
            targetOrientation = customDynamicsAPI['getObjectProperty']((shaker,), ('fn', 'containment', 'pouring', 'outof', 'upright'))
        target = {hand: {'target': [targetPosition, targetOrientation], 'positionInLink': pouringPointInHand, 'orientationInLink': None, 'clopeningAction': None}}
        return [{'type': 'P', 'description': descriptionC, 'target': target}]
    return []

def getCoherenceConditions(customDynamicsAPI, name, description, node):
    process = description.get('process', 'notifiedCompletion')
    if 'notifyingCompletion' == process:
        return []
    elif 'stopOpening' == process:
        return []
    elif 'stopClosing' == process:
        return []
    elif 'ungrasping' == process:
        return []
    elif 'movingBase' == process:
        return []
    elif 'movingArm' == process:
        return []
    elif 'opening' == process:
        hand = getDoorManipulationHand(customDynamicsAPI, name, description.get('hand', None))
        container = description.get('container', None)
        component = description.get('component', None)
        near = {'type':'G', 'description':{'goal': 'near', 'item': container}, 'children':[]}
        touched = {'type':'G', 'description':{'goal': 'armTriggeredPortalHandle', 'container': container, 'component': component, 'hand': hand, 'action': 'open'}, 'children':[]}
        parked = {'type':'G', 'description':{'goal': 'parkedArm', 'hand': hand}}
        return [near, touched, parked]
    elif 'closing' == process:
        hand = getDoorManipulationHand(customDynamicsAPI, name, description.get('hand', None))
        container = description.get('container', None)
        component = description.get('component', None)
        near = {'type':'G', 'description':{'goal': 'near', 'item': container}, 'children':[]}
        touched = {'type':'G', 'description':{'goal': 'armTriggeredPortalHandle', 'container': container, 'component': component, 'hand': hand, 'action': 'close'}, 'children':[]}
        parked = {'type':'G', 'description':{'goal': 'parkedArm', 'hand': hand}}
        return [near, touched, parked]
    elif 'grasping' == process:
        item = description.get('item', None)
        hand = description.get('hand', None)
        freeHand = getFreeHand(customDynamicsAPI, name, hand)
        container, component = getContainerComponent(customDynamicsAPI, item)
        opened = {'type': 'G', 'description': {'goal': 'opened', 'container': container, 'component': component, 'hand': freeHand}}
        stoppedOpening = {'type': 'G', 'description': {'goal': 'stoppedOpening', 'hand': freeHand}}
        near = {'type': 'G', 'description': {'goal': 'near', 'item': item}}
        armNearItemHandle = {'type': 'G', 'description': {'goal': 'armNearItemHandle', 'item': item, 'hand': hand}}
        return [opened, stoppedOpening, near, armNearItemHandle]
    elif 'pickingItem' == process:
        item = description.get('item', None)
        hand = description.get('hand', None)
        grasped = {'type': 'G', 'description': {'goal': 'grasped', 'item': item, 'hand': hand}}
        parked = {'type':'G', 'description': {'goal': 'parkedArm', 'hand': hand}}
        return [grasped, parked]
    elif 'placingItem' == process:
        item = description.get('item', None)
        hand = description.get('hand', None)
        container = description.get('container', None)
        picked = {'type': 'G', 'description': {'goal': 'pickedItem', 'item': item, 'hand': hand}}
        near = {'type': 'G', 'description': {'goal': 'near', 'item': container}}
        source = description.get('sourceContainer', None)
        sourcePart = description.get('sourceComponent', None)
        retq = [picked]
        freeHand = getFreeHand(customDynamicsAPI, name, hand)
        if source is not None:
            closed = {'type': 'G', 'description': {'goal': 'closed', 'container': source, 'component': sourcePart, 'hand': freeHand}}
            parked = {'type': 'G', 'description': {'goal': 'parkedArm', 'hand': freeHand}}
            retq = retq + [closed, parked]
        component, targetPosition = getItemPlacement(customDynamicsAPI, name, item, container, node.get('component'), node.get('targetPosition'))
        retq.append(near)
        retq.append({'type': 'G', 'description': {'goal': 'opened', 'container': container, 'component': component, 'hand': freeHand}})
        return retq
    elif 'bakingItem' == process:
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
    elif 'bringingNear' == process:
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
    elif 'transferring' == process:
        item = description.get('item', None)
        hand = description.get('hand', None)
        storage = description.get('storage', None)
        container = description.get('container', None)
        pouredType = description.get('pouredType', None)
        amount = description.get('amount', None)
        itemOrientation = customDynamicsAPI['getObjectProperty']((item,), 'orientation')
        pouringAxis = customDynamicsAPI['getObjectProperty']((item,), ('fn', 'containment', 'pouring', 'outof', 'axis'), [1,0,0])
        # TODO: source, sourcePart? just to make sure we close things when picking them up
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
    elif 'sprinkling' == process:
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
    return []

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

G:bakedItem(Item, BakedType, Oven, Destination)
  P:bakingItem(Item, BakedType, Oven, Destination)
    G:placedItem(Item, Oven) OR G:placedItem(Item,Destination,Oven,_OvenComponent) depending on baked status

G:broughtNear(Trajector, Hand, Relatum)
  P:bringingNear(Trajector, Hand, Relatum, SourceContainer, SourceComponent)
    G:pickedItem(Item, Hand, SourceContainer, SourceComponent)
    G:closed(SourceContainer, SourceComponent, _OHand)
    G:near(Relatum)

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

