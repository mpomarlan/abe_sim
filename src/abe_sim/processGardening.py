import sys
import math
import numpy
import time

import pybullet

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
    vCI = [vAI[1]*vBI[2] - vAI[2]*vBI[1], vAI[2]*vBI[0] - vAI[0]*vBI[2], vAI[0]*vBI[1] - vAI[1]*vBI[0]]
    vCO = [vAO[1]*vBO[2] - vAO[2]*vBO[1], vAO[2]*vBO[0] - vAO[0]*vBO[2], vAO[0]*vBO[1] - vAO[1]*vBO[0]]
    RI = numpy.transpose(numpy.array([vAI, vBI, vCI]))
    RORI = numpy.transpose(numpy.array([vAO, vBO, vCO]))
    #print(RI, RORI)
    RO = numpy.matmul(RORI, numpy.linalg.inv(RI))
    retq = rotationMatrixToQuaternion(RO)
    return retq

def pointCloseness(a, b, t):
    d = [x-y for x,y in zip(a, b)]
    return (t > (d[0]*d[0] + d[1]*d[1] + d[2]*d[2]))

def quaternionCloseness(a, b, t):
    ax = stubbornTry(lambda : pybullet.rotateVector(a, [1,0,0]))
    ay = stubbornTry(lambda : pybullet.rotateVector(a, [0,1,0]))
    bx = stubbornTry(lambda : pybullet.rotateVector(b, [1,0,0]))
    by = stubbornTry(lambda : pybullet.rotateVector(b, [0,1,0]))
    return ((t < (ax[0]*bx[0] + ax[1]*bx[1] + ax[2]*bx[2])) and (t < (ay[0]*by[0] + ay[1]*by[1] + ay[2]*by[2])))

def getAxisInWorld(w, item, axis, predCache):
    if axis is not None:
        axis = tuple(axis)
    if ((item, axis), "axisInWorld") not in predCache:
        if axis is not None:
            _, itemOrientation, _, _ = getKinematicData(w, (item,), predCache)
            axisAdj = stubbornTry(lambda: pybullet.rotateVector(itemOrientation, axis))
        else:
            axisAdj = axis
        predCache[((item, axis), "axisInWorld")] = axisAdj
    return predCache[((item, axis), "axisInWorld")]

def getKinematicData(w, identifier, predCache):
    if (identifier, "kinematicData") not in predCache:
        predCache[(identifier, "kinematicData")] = w.getKinematicData(identifier)
    return predCache[(identifier, "kinematicData")]

def getJointData(w, identifier, predCache):
    if (identifier, "jointData") not in predCache:
        predCache[(identifier, "jointData")] = w.getJointData(identifier)
    return predCache[(identifier, "jointData")]

def getGraspingFn(w, name, predCache):
    if ((name,), "graspingFn") not in predCache:
        predCache[((name,), "graspingFn")] = w._kinematicTrees[name].get("fn", {}).get("grasping", {})
    return predCache[((name,), "graspingFn")]
    
def getContainmentFn(w, name, predCache):
    if ((name,), "containerFn") not in predCache:
        predCache[((name,), "containerFn")] = w._kinematicTrees[name].get("fn", {}).get("containment", {})
    return predCache[((name,), "containerFn")]
    
def getKinematicControlFn(w, name, predCache):
    if ((name,), "kinematicControlFn") not in predCache:
        predCache[((name,), "kinematicControlFn")] = w._kinematicTrees[name].get("fn", {}).get("kinematicControl", {})
    return predCache[((name,), "kinematicControlFn")]
    
def getFacingYaw(w, item, itemPosition, predCache):
    if (item, "facingYaw") not in predCache:
        yaw = math.atan2(itemPosition[1], itemPosition[0])
        itemContainmentFn = getContainmentFn(w, item, predCache)
        ownFacing = itemContainmentFn.get("facing")
        if ownFacing is not None:
            _, orientation, _, _ = getKinematicData(w, (item,), predCache)
            facingV = stubbornTry(lambda : pybullet.rotateVector(orientation, ownFacing))
            yaw = math.atan2(facingV[1], facingV[0])
        at = w.at((item,))
        if at is not None:
            atContainmentFn = getContainmentFn(w, at, predCache)
            facing = atContainmentFn.get("facing")
            if facing is not None:
                _, orientation, _, _ = getKinematicData(w, (at,), predCache)
                facingV = stubbornTry(lambda : pybullet.rotateVector(orientation, facing))
                yaw = math.atan2(facingV[1], facingV[0])
        predCache[(item, "facingYaw")] = yaw
    return predCache[(item, "facingYaw")]

def getBaseLink(w, name, predCache):
    if ((name,), "baseLinkName") not in predCache:
        predCache[((name,), "baseLinkName")] = w._kinematicTrees[name]["idx2Link"][-1]
    return predCache[((name,), "baseLinkName")]
    
def getHandLink(w, name, hand, predCache):
    if ((name, hand), "handLink") not in predCache:
        predCache[((name, hand), "handLink")] = w._kinematicTrees[name].get("fn", {}).get("kinematicControl", {}).get("efLink", {}).get(hand)
    return predCache[((name, hand), "handLink")]

def getHandleLink(w, item, predCache):
    if ((item,), "handleLink") not in predCache:
        aux = getGraspingFn(w, item, predCache).get("handle")
        if aux is None:
            aux = getBaseLink(w, item, predCache)
        predCache[((item,), "handleLink")] = aux
    return predCache[((item,), "handleLink")]

def getContents(w, item, predCache, heightBonus=None):
    if ((item, heightBonus), "contents") not in predCache:
        aabb = w.getAABB((item,))
        if heightBonus is not None:
            aabb = [aabb[0], [aabb[1][0], aabb[1][1], aabb[1][2] + heightBonus]]
        overlaps = set([x[0] for x in w.checkOverlap(aabb)])
        predCache[((item, heightBonus), "contents")] = [x for x in overlaps if (item == w.at((x,))) or (heightBonus is not None)]
    return predCache[((item, heightBonus), "contents")]

def getHeld(w, name, hand, predCache):
    if ((name, hand), 'held') not in predCache:
        if hand is None:
            hands = getGraspingFn(w, name, predCache).get("effectors") or []
        else:
            hands = [hand]
        actuallyGrasping = set([])
        for h in hands:
            if ((name, h), 'held') not in predCache:
                predCache[((name, h), 'held')] = set([x[0][0] for x in w._kinematicTrees[name].get("customStateVariables", {}).get("grasping", {}).get("actuallyGrasping", {}).get(h, [])])
            actuallyGrasping = actuallyGrasping.union(predCache[((name, h), 'held')])
        if hand is None:
            predCache[((name, None), 'held')] = actuallyGrasping
    return predCache[((name, hand), 'held')]

def getHeldAABB(w, name, hand, predCache):
    if ((name, hand), "heldAABB") not in predCache:
        handLink = getHandLink(w, name, hand, predCache)
        held = getHeld(w, name, hand, predCache)
        aabbs = [w.getAABB((x,)) for x in held]
        aabb = w.getAABB((name, handLink))
        aabb = [list(aabb[0]), list(aabb[1])]
        for e in aabbs:
            for k, v in enumerate(aabb[0]):
                if e[0][k] < aabb[0][k]:
                    aabb[0][k] = e[0][k]
                if e[1][k] > aabb[1][k]:
                    aabb[1][k] = e[1][k]
        predCache[((name, hand), "heldAABB")] = held, aabb
    return predCache[((name, hand), "heldAABB")]

def getComponentHeight(w, container, component, predCache):
    if ((container, component), "height") not in predCache:
        retq = None
        containerFn = w._kinematicTrees[container].get("fn", {}).get("containment", {})
        if component is None:
            components = containerFn.get("links", [])
            if 0 < len(components):
                component = components[0]
        while retq is None:
            retq = containerFn.get("height", {}).get(component)
            if retq is None:
                container, component = getContainerComponent(w, container)
                if container is None:
                    break
        predCache[((container, component), "height")] = retq
    return predCache[((container, component), "height")]

def getComponentUnderHandHeight(w, name, hand, handPosition, predCache, defaultHeight=None):
    if defaultHeight is None:
        defaultHeight = 1.0
    if ((hand,), "underHandHeight") not in predCache:
        held, aabb = getHeldAABB(w, name, hand, predCache)
        aabb[0][2] = -0.1
        overlaps = [x for x in w.checkOverlap(aabb) if (x[0] != name) and (x[0] not in held) and w._kinematicTrees[x[0]].get("fn", {}).get("canContain")]
        maxO = None
        for overlap in overlaps:
            obj, lnk = overlap
            if lnk not in w._kinematicTrees[obj].get("fn", {}).get("containment", {}).get("links", []):
                continue
            aabbOverlap = w.getAABB((obj, lnk))
            if aabbOverlap[1][2] < handPosition[2]:
                if (maxO is None) or (maxO[0] < aabbOverlap[1][2]):
                    maxO = [aabbOverlap[1][2], obj, lnk]
        if maxO is not None:
            height = getComponentHeight(w, maxO[1], maxO[2], predCache) + maxO[0]
        else:
            height = defaultHeight
        predCache[((hand,), "underHandHeight")] = height
    return predCache[((hand,), "underHandHeight")]

def getParkedPose(w, name, hand, predCache):
    if ((name, hand), "parkedPose") not in predCache:
        nameKinematicControlFn = getKinematicControlFn(w, name, predCache)
        armBaseLink = nameKinematicControlFn.get("efBaseLink", {}).get(hand)
        armBasePosition, armBaseOrientation, _, _ = getKinematicData(w, (name, armBaseLink), predCache)
        armParkedPosition = list(nameKinematicControlFn.get("parkedPosition", {}).get(hand))
        armParkedOrientation = nameKinematicControlFn.get("parkedOrientation", {}).get(hand)
        handLink = getHandLink(w, name, hand, predCache)
        handPosition, _, _, _ = getKinematicData(w, (name, handLink), predCache)
        aabbHand = w.getAABB((name, handLink))
        held = getHeld(w, name, hand, predCache)
        height = getComponentUnderHandHeight(w, name, hand, handPosition, predCache, defaultHeight=None)
        if height is not None:
            armParkedPosition[2] = height
        parkedPosition, parkedOrientation = w.objectPoseRelativeToWorld(armBasePosition, armBaseOrientation, armParkedPosition, armParkedOrientation)
        predCache[((name, hand), "parkedPose")] = parkedPosition, parkedOrientation, height
    return predCache[((name, hand), "parkedPose")]

def getCloseness(w, ia, ib, d, predCache):
    if (ia, ib, d) not in predCache:
        predCache[(ia, ib, d)] = (0 < len(w.checkClosestPoints(ia, ib, maxDistance=d)))
    return predCache[(ia, ib, d)]

def getContainerComponent(w, item):
    # atComponent is already cached by the world object.
    retq = w.atComponent((item,))
    if retq is None:
        return None, None
    return retq

def getHandleAndDoor(w, container, component, predCache):
    if ((container, component), "handleDoor") not in predCache:
        if (container is None) or (component is None):
            predCache[((container, component), "handleDoor")] = None, None
        else:
            door = w._kinematicTrees[container].get("fn", {}).get("containment", {}).get("door", {}).get(component)
            handle = w._kinematicTrees[container].get("fn", {}).get("clopening", {}).get("handle", {}).get(door)
            predCache[((container, component), "handleDoor")] = handle, door
    return predCache[((container, component), "handleDoor")]

def getEntryHeight(w, name, description, numerics, predCache):
    entryHeight = numerics.get('entryHeight', None)
    if entryHeight is not None:
        return entryHeight
    aux = description.get("destination")
    if aux is not None:
        aux = tuple(aux)
    predK = (description["hand"], description.get("item"), aux)
    if (predK, "entryHeight") not in predCache:
        hand = description['hand']
        handLink = getHandLink(w, name, hand, predCache)
        positionHand, _, _, _ = getKinematicData(w, (name, handLink), predCache)
        entryHeight = getComponentUnderHandHeight(w, name, hand, positionHand, predCache)
        item = description.get('item', None)
        destination = description.get('destination', None)
        container, component = getContainerComponent(w, item)
        if (item is None) or (container is not None):
            if destination is not None:
                container, component = destination
            if (item is not None) or (destination is not None):
                if component is not None:
                    componentHeight = getComponentHeight(w, container, component, predCache)
                    componentBase = w.getAABB((container, component))[1][2]
                else:
                    componentBase = w.getAABB((item,))[1][2]
                    componentHeight = 0.2
                entryHeight = componentHeight + componentBase
        predCache[(predK, "entryHeight")] = entryHeight
    return predCache[(predK, "entryHeight")]

def getDoorManipulationHand(w, name, suggestion, predCache, avoid=None):
    if suggestion is not None:
        return suggestion
    if (avoid, "doorHand") not in predCache:
        retq = None
        clopenerEFs = w._kinematicTrees[name].get("fn", {}).get("clopening", {}).get("clopeningEFs", [])
        for clopenerEF in clopenerEFs:
            if clopenerEF == avoid:
                continue
            if 0 == len(w._kinematicTrees[name].get("customStateVariables", {}).get("grasping", {}).get("actuallyGrasping", {}).get(clopenerEF, [])):
                retq = clopenerEF
                break
        if retq is None:
            retq = clopenerEFs[0]
        predCache[(avoid, "doorHand")] = retq
    return predCache[(avoid, "doorHand")]

def getFreeHand(w, name, hand, predCache):
    return getDoorManipulationHand(w, name, None, predCache, avoid=hand)

def getLocations(w, container, component, aabb, canLine, predCache):
    aabbC = w.getAABB((container, component))
    arrangementAxis = getContainmentFn(w, container, predCache).get("arrangement", {}).get("axis") or (1, 0, 0)
    position, orientation, _, _ = getKinematicData(w, (container, component), predCache)
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
            if (vx < aabbC[0][0]) or (vX > aabbC[1][0]) or (vy < aabbC[0][1]) or (vY > aabbC[1][1]):
                break
            else:
                retq.append(v)
            k += inc
    return retq

def getItemPlacement(w, name, item, container, component, targetPosition, predCache, allowedComponents=None):
    def _acceptable(e):
        return (e in [name, item, container]) or (item == w.at((e,))) or (w._kinematicTrees[e].get("fn", {}).get("canLine"))
    def feasible(w, aabb, component, targetPosition, handRadius, handGraspTolerance, itemHeight):
        aabbAdj = [[x+y for x,y in zip(aabb[0], targetPosition)], [x+y for x,y in zip(aabb[1], targetPosition)]]
        aabbAdjWide = [[aabbAdj[0][0]-handRadius, aabbAdj[0][1]-handRadius, aabbAdj[0][2]], [aabbAdj[1][0]+handRadius, aabbAdj[1][1]+handRadius, aabbAdj[1][2]]]
        overlaps = set([x[0] for x in w.checkOverlap(aabbAdj)])
        wideOverlaps = set([x[0] for x in w.checkOverlap(aabbAdjWide)])
        badOverlaps = []
        for e in overlaps:
            if _acceptable(e):
                continue
            badOverlaps.append(e)
        for e in wideOverlaps:
            if _acceptable(e):
                continue
            aabbE = w.getAABB((e,))
            if handGraspTolerance < abs(itemHeight - (aabbE[1][2] - aabbE[0][2])):
                badOverlaps.append(e)
        #print("BAD", component, targetPosition, aabbAdj, badOverlaps)
        return 0 == len(badOverlaps)
    itemP, _, _, _ = getKinematicData(w, (item,), predCache)
    aabb = toOriginAABB(w.getAABB((item,)), itemP)
    aabbLocal = w._kinematicTrees[item]["localAABB"][0]
    itemHeight = aabbLocal[1][2] - aabbLocal[0][2]
    # TODO: get hand radius from hand local aabb; hand grasp tolerance from ???
    handRadius = 0.1
    handGraspTolerance = 0.01
    if (component is not None) and (targetPosition is not None):
        if feasible(w, aabb, component, targetPosition):
            return component, targetPosition
    containmentFn = getContainmentFn(w, container, predCache)
    arrangement = containmentFn.get("arrangement", {}).get("mode") or "heap"
    components = containmentFn.get("links") or []
    if allowedComponents is not None:
        components = list(set(components).intersection(allowedComponents))
    if "heap" == arrangement:
        component = components[0]
        # TODO: re-enable this, or something like it, to detect "fullness" of heap containers
        aabbComponent = w.getAABB((container, component))
        #aabbOverComponent = [list(aabbComponent[0]), list(aabbComponent[1])]
        #aabbOverComponent[0][2] = aabbComponent[1][2]
        #aabbOverComponent[1][2] = aabbComponent[1][2] + 0.02
        #overlaps = set([x[0] for x in w.checkOverlap(aabbOverComponent)])
        #for e in overlaps:
        #    if (e not in [name, item, container]):
        #        atContainer, atComponent = w.atComponent((e,))
        #        if (container == atContainer) and (component == atComponent):
        #            return None, None
        compP, _, _, _ = getKinematicData(w, (container, component), predCache)
        targetPosition = list(compP)
        targetPosition[2] = aabbComponent[1][2] - aabb[0][2]
        return component, targetPosition
    ### Assuming side by side arrangement
    if component is not None:
        components = [component] + components
    for c in components:
        #print("LOCS", getLocations(w, container, c, aabb, w._kinematicTrees[item].get("fn", {}).get("canLine"), predCache))
        for l in getLocations(w, container, c, aabb, w._kinematicTrees[item].get("fn", {}).get("canLine"), predCache):
            if feasible(w, aabb, c, l, handRadius, handGraspTolerance, itemHeight):
                return c, l
    #print("OOPS, nothing available?")
    #print("  tried", components)
    #print("    ", [getLocations(w, container, c, aabb, w._kinematicTrees[item].get("fn", {}).get("canLine"), predCache) for c in components])
    #sys.exit(0)
    return None, None

def checkNear(w, name, item, previous, predCache, position=None):
    if ((name, item), "near") not in predCache:
        if position is None:
            itemPosition, _, _, _ = getKinematicData(w, (item,), predCache)
        else:
            itemPosition = position
        facingYaw = getFacingYaw(w, item, itemPosition, predCache)
        baseLink = w._kinematicTrees[name].get("fn", {}).get("kinematicControl", {}).get('mobileBaseLink')
        basePosition, baseOrientation, _, _ = getKinematicData(w, (name, baseLink), predCache)
        baseYaw = stubbornTry(lambda : pybullet.getEulerFromQuaternion(baseOrientation))[2]
        basePosition, _ = w.objectPoseRelativeToWorld(basePosition, baseOrientation, baseFwdOffset, (0,0,0,1))
        if previous:
            retq = poseXYYawDistance(itemPosition, facingYaw, basePosition, baseYaw, 1.5, 0.7)
        else:
            retq = poseXYYawDistance(itemPosition, facingYaw, basePosition, baseYaw, 0.1, 0.95)
        predCache[((name, item), "near")] = retq
    return predCache[((name, item), "near")]

def checkParked(w, name, hand, previous, predCache):
    if ((name, hand), "parked") not in predCache:
        handLink = getHandLink(w, name, hand, predCache)
        handPosition, handOrientation, _, _ = getKinematicData(w, (name, handLink), predCache)
        parkedPosition, parkedOrientation, _ = getParkedPose(w, name, hand, predCache)
        if previous:
            retq = pose6DDistance(parkedPosition, parkedOrientation, handPosition, handOrientation, 0.5, 0.7)
        else:
            retq = pose6DDistance(parkedPosition, parkedOrientation, handPosition, handOrientation, 0.1, 0.95)
        predCache[((name, hand), "parked")] = retq
    return predCache[((name, hand), "parked")]

def checkGrasped(w, name, hand, item, predCache):
    if ((name, hand), item, 'grasps') not in predCache:
        predCache[((name, hand), item, 'grasps')] = item in getHeld(w, name, hand, predCache)
    return predCache[((name, hand), item, 'grasps')]

def checkItemInContainer(w, name, item, container, predCache, allowedComponents=None):
    if isinstance(allowedComponents,list):
        allowedComponents = tuple(allowedComponents)
    ## TODO: test whether transitive closure of at includes container
    if item not in w._kinematicTrees:
        return True
    if ((container, item, allowedComponents), "contains") not in predCache:
        aabbItem = w.getAABB((item,))
        aabbItemMin, aabbItemMax = list(aabbItem[0]), aabbItem[1]
        aabbItemMin[2] -= 0.05
        containmentFn = getContainmentFn(w, container, predCache)
        if allowedComponents is None:
            allowedComponents = tuple(containmentFn.get("links"))
        largeContainer = containmentFn.get("largeConcavity")
        retq = False
        if largeContainer:
            #if allowedComponents is not None:
            for ac in allowedComponents:
                overlaps = set([x[0] for x in w.checkOverlap(w.getAABB((container, ac)))])
                if item in overlaps:
                    retq = True
                    break
            #else:
            #    retq = item in [x[0] for x in w.checkOverlap(w.getAABB((container,)))]
        else:
            overlaps = set([x for x in w.checkOverlap((aabbItemMin, aabbItemMax))])
            #if allowedComponents is not None:
            for ac in allowedComponents:
                if (container, ac) in overlaps:
                    aabbC = w.getAABB((container, ac))
                    retq = (aabbC[0][0] - 0.02 < aabbItemMin[0]) and (aabbC[0][1] - 0.02 < aabbItemMin[1]) and (aabbC[1][0] + 0.02 > aabbItemMax[0]) and (aabbC[1][1] + 0.02 > aabbItemMax[1])
                    if retq:
                        break
            #else:
            #    retq = container in set([x[0] for x in overlaps])
        predCache[((container, item, allowedComponents), "contains")] = retq
    return predCache[((container, item, allowedComponents), "contains")]

def checkItemAboveContainer(w, name, item, container, predCache, hand, sideHold, allowedComponents=None):
    if item not in w._kinematicTrees:
        return True
    if ((container, item, sideHold, allowedComponents), "aboveContainer") not in predCache:
        aabbItem = w.getAABB((item,))
        aabbItemMin, aabbItemMax = list(aabbItem[0]), aabbItem[1]
        aabbItemMin[2] -= 0.05
        containmentFn = getContainmentFn(w, container, predCache)
        if allowedComponents is None:
            allowedComponents = tuple(containmentFn.get("links"))
        itemP, _, _, _ = getKinematicData(w, (item,), predCache)
        retq = None
        if sideHold:
            handLink = getHandLink(w, name, hand, predCache)
            _, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
            if 0.1 < abs(stubbornTry(lambda : pybullet.rotateVector(handQ, [0,0,1]))[2]):
                retq = False
        if retq is None:
            pouringFn = w._kinematicTrees[container].get("fn", {}).get("containment", {}).get("pouring", {}).get("into", {})
            link = pouringFn.get("link")
            point = pouringFn.get("point")
            if link in allowedComponents:
                cP, cQ, _, _ = getKinematicData(w, (container, link), predCache)
                entryPoint, _ = w.objectPoseRelativeToWorld(cP, cQ, point, [0,0,0,1])
                dx = entryPoint[0] - itemP[0]
                dy = entryPoint[1] - itemP[1]
                if 0.0009 > (dx*dx + dy*dy):
                    retq = True
        if retq is None:
            retq = False
        predCache[((container, item, sideHold, allowedComponents), "aboveContainer")] = retq
    return predCache[((container, item, sideHold, allowedComponents), "aboveContainer")]

def checkItemCoversContainer(w, name, cover, item, predCache, covered, allowedComponents=None):
    if (item not in w._kinematicTrees) or (cover not in w._kinematicTrees):
        return True
    if ((cover, item, allowedComponents), "covers") not in predCache:
        if not covered:
            posTh = 0.0004
            ornTh = 0.99
        else:
            posTh = 0.004
            ornTh = 0.9
        itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
        coverP, coverQ, _, _ = getKinematicData(w, (cover,), predCache)
        itemFnCovering = w._kinematicTrees[item].get("fn", {}).get("covering", {})
        coverFnCovering = w._kinematicTrees[cover].get("fn", {}).get("covering", {})
        itemCoverPoint = itemFnCovering.get("point")
        itemCoverAxis = itemFnCovering.get("axis")
        coverCoverPoint = coverFnCovering.get("point")
        coverCoverAxis = coverFnCovering.get("axis")
        itemCoverAxisInWorld = stubbornTry(lambda : pybullet.rotateVector(itemQ, itemCoverAxis))
        coverCoverAxisInWorld = stubbornTry(lambda : pybullet.rotateVector(coverQ, coverCoverAxis))
        itemCoverPointInWorld, _ = w.objectPoseRelativeToWorld(itemP, itemQ, itemCoverPoint, [0,0,0,1])
        coverCoverPointInWorld, _ = w.objectPoseRelativeToWorld(coverP, coverQ, coverCoverPoint, [0,0,0,1])
        zGap = w.getAABB((cover,))[0][2] - w.getAABB((item,))[1][2]
        d = [coverCoverPointInWorld[0] - itemCoverPointInWorld[0], coverCoverPointInWorld[1] - itemCoverPointInWorld[1]]
        dot = coverCoverAxisInWorld[0]*itemCoverAxisInWorld[0] + coverCoverAxisInWorld[1]*itemCoverAxisInWorld[1] + coverCoverAxisInWorld[2]*itemCoverAxisInWorld[2]
        retq = (posTh > (d[0]*d[0] + d[1]*d[1])) and (ornTh < dot) and (-0.05 < zGap) and (0.05 > zGap)
        predCache[((cover, item, allowedComponents), "covers")] = retq
    return predCache[((cover, item, allowedComponents), "covers")]

def checkTransferred(w, name, item, pouredType, amount, container, predCache):
    if ((name, item, pouredType, amount, container), "transferred") not in predCache:
        if amount is not None:
            retq = amount <= len([x for x in getContents(w, container, predCache) if (pouredType == w._kinematicTrees[x]["type"])])
        else:
            retq = (0 == len([x for x in getContents(w, item, predCache) if (x != name) and (not w._kinematicTrees[x].get("fn", {}).get("canLine"))]))
        predCache[((name, item, pouredType, amount, container), "transferred")] = retq
    return predCache[((name, item, pouredType, amount, container), "transferred")]

def checkMixed(w, name, container, mixedType, predCache):
    if ((container, mixedType), "mixed") not in predCache:
        if mixedType is None:
            retq = all([0 == w._kinematicTrees[x].get("customStateVariables", {}).get("mingling", {}).get("hp", 0) for x in getContents(w, container, predCache)])
        else:
            # TODO: needs a smarter check: should be, mixable into mixedType
            notDones = [x for x in getContents(w, container, predCache) if (mixedType != w._kinematicTrees[x].get("type")) and (w._kinematicTrees[x].get("fn", {}).get("mixable"))]
            retq = (0 == len(notDones))
        predCache[((container, mixedType), "mixed")] = retq
    return predCache[((container, mixedType), "mixed")]

def checkMashed(w, name, container, disposition, predCache):
    return all([not w._kinematicTrees[x].get("fn", {}).get(disposition, False) for x in getContents(w, container, predCache)])

def checkUpright(w, itemOrientation, pouringAxis, predCache, th=None):
    itemOrientation = tuple(itemOrientation)
    pouringAxis = tuple(pouringAxis)
    if ((itemOrientation, pouringAxis), "upright") not in predCache:
        if th is None:
            th = 0.2
        vec = stubbornTry(lambda : pybullet.rotateVector(itemOrientation, pouringAxis))
        down = w.getDown()
        predCache[((itemOrientation, pouringAxis), "upright")] = (th > abs(vec[0]*down[0] + vec[1]*down[1] + vec[2]*down[2]))
    return predCache[((itemOrientation, pouringAxis), "upright")]

def checkControlSetting(w, name, hand, item, link, setting, predCache):
    if ((hand, item, link, setting), "controlSetting") not in predCache:
        angleDif = abs(setting - w.getJointData((item, link))[0])
        isSet = (0.05 > angleDif)
        handLink = getHandLink(w, name, hand, predCache)
        v = w._kinematicTrees[name].get("customStateVariables", {}).get("turning", {}).get(handLink, [])
        isSetting = ([item, link] in v) or ((item, link) in v)
        predCache[((hand, item, link, setting), "controlSetting")] = (isSet, isSetting)
    return predCache[((hand, item, link, setting), "controlSetting")]
    
def containsIngredients(w, item, ingredientTypes, predCache):
    ingredientTypesK = tuple(sorted(ingredientTypes.items()))
    if ((item, ingredientTypesK), "containsIngredients") not in predCache:
        containedItems = {}
        for x in getContents(w, item, predCache):
            t = w._kinematicTrees[x].get("type")
            if t not in containedItems:
                containedItems[t] = []
            containedItems[t].append(x)
        retq = []
        allOk = True
        for k, v in ingredientTypes.items():
            if (k not in containedItems) or (v > len(containedItems[k])):
                retq = []
                allOk = False
                break
            retq = retq + sorted(containedItems[k])[:v]
        predCache[((item, ingredientTypesK), "containsIngredients")] = allOk, retq
    return predCache[((item, ingredientTypesK), "containsIngredients")]

def containsShapes(w, item, shapeType, predCache):
    if ((item, shapeType), "containsShapes") not in predCache:
        retq = [x for x in getContents(w, item, predCache, heightBonus=0.5) if shapeType == w._kinematicTrees[x].get("type")]
        predCache[((item, shapeType), "containsShapes")] = 0 != len(retq), retq
    return predCache[((item, shapeType), "containsShapes")]

def isHoldingObjectOfType(w, name, hand, qtype, predCache):
    if ((hand, qtype), "isHoldingOfType") not in predCache:
        heldOfType = [x for x in getHeld(w, name, hand, predCache) if qtype == w._kinematicTrees[x].get("type")]
        holding = (0 < len(heldOfType))
        retq = None
        if holding:
            retq = sorted(heldOfType)[0]
        predCache[((hand, qtype), "isHoldingOfType")] = holding, retq
    return predCache[((hand, qtype), "isHoldingOfType")]

def checkSprinkled(w, name, item, sprinkledType, predCache):
    if ((item, sprinkledType), "sprinkled") not in predCache:
        retq = [x for x in getContents(w, item, predCache) if (sprinkledType == w._kinematicTrees[x].get("type"))]
        predCache[((item, sprinkledType), "sprinkled")] = (0 == len(retq), retq)
    return predCache[((item, sprinkledType), "sprinkled")]

def checkBakedContents(w, item, bakedType, process, predCache):
    if ((item, bakedType, process), process) not in predCache:
        contents = getContents(w, item, predCache)
        processed = True
        # TODO: need a smarter check here: all that is bakeable/processable into bakedType must be baked.
        for x in contents:
            if (w._kinematicTrees[x].get("fn", {}).get(process, False)) and (bakedType != w._kinematicTrees[x].get("type")):
                processed = False
                break
        predCache[((item, bakedType, process), process)] = processed
    return predCache[((item, bakedType, process), process)]

def checkOpened(w, container, component, predCache):
    if ((container, component), "open") not in predCache:
        handle, door = getHandleAndDoor(w, container, component, predCache)
        if handle is not None:
            aux = w._kinematicTrees[container].get("fn", {}).get("clopening") or {}
            openMinThreshold = aux.get("openMin", {}).get(door)
            openMaxThreshold = aux.get("openMax", {}).get(door)
            jointValue, _, _, _ = getJointData(w, (container, door), predCache)
            predCache[((container, component), "open")] = (((openMinThreshold is None) or (openMinThreshold < jointValue)) and ((openMaxThreshold is None) or (openMaxThreshold > jointValue)))
        else:
            predCache[((container, component), "open")] = True
    return predCache[((container, component), "open")]

def checkClosed(w, container, component, predCache):
    handle, door = getHandleAndDoor(w, container, component, predCache)
    if handle is not None:
        aux = w._kinematicTrees[container].get("fn", {}).get("clopening") or {}
        closedMinThreshold = aux.get("closedMin", {}).get(door)
        closedMaxThreshold = aux.get("closedMax", {}).get(door)
        jointValue, _, _, _ = getJointData(w, (container, door), predCache)
        return ((closedMinThreshold is None) or (closedMinThreshold < jointValue)) and ((closedMaxThreshold is None) or (closedMaxThreshold > jointValue))
    return True

def _alwaysFalse(w, name, description, node, predCache):
    return False
    
def _alwaysTrue(w, name, description, node, predCache):
    return True
    
def _checkWaited(w, name, description, node, predCache):
    timeEnd = description.get('timeEnd', 0)
    timeNow = w._kinematicTrees[name].get("customStateVariables", {}).get("timing", {}).get("timer") or 0
    return timeEnd <= timeNow

def _checkNear(w, name, description, node, predCache):
    relatum = description['relatum']
    goalPosition = node['numerics'].get('position', None)
    previous = node.get('previousStatus', False)
    retq = checkNear(w, name, relatum, previous, predCache, position=goalPosition)
    return retq

def _checkStoppedHands(w, name, description, node, predCache):
    csv = w._kinematicTrees[name].get("customStateVariables",{}).get("kinematicControl", {}).get("target") or {}
    left = csv.get("hand_left")
    right = csv.get("hand_right")
    return (left is None) and (right is None)

def _checkParkedArm(w, name, description, node, predCache):
    hand = description['hand']
    previous = node.get('previousStatus', False)
    return checkParked(w, name, hand, previous, predCache)

def _checkArmNearItemHandle(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    handLink = getHandLink(w, name, hand, predCache)
    _, handOrientation, _, _ = getKinematicData(w, (name, handLink), predCache)
    handleLink = getHandleLink(w, item, predCache)
    aligned = True
    graspingFn = getGraspingFn(w, name, predCache)
    itemGraspingFn = getGraspingFn(w, item, predCache)
    handleAxisInWorld = getAxisInWorld(w, item, itemGraspingFn.get("axis", {}).get(handleLink), predCache)
    maxDistance = graspingFn.get("graspingActivationRadius", {}).get(hand) or 0.5
    ornTh = 0.99
    if node.get('previousStatus', False):
        maxDistance = 1.5*maxDistance
        ornTh = 0.95
    if handleAxisInWorld is not None:
        handleYaw = math.atan2(handleAxisInWorld[1], handleAxisInWorld[0])
        targetOrientation = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,handleYaw)))
        aligned = quaternionCloseness(targetOrientation, handOrientation, ornTh)
    return aligned and getCloseness(w, (name, handLink), (item, handleLink), maxDistance, predCache)

def _checkGrasped(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    retq = checkGrasped(w, name, hand, item, predCache)
    return retq

def _checkUngrasped(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    return not checkGrasped(w, name, hand, item, predCache)

def _checkPickedItem(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    previous = node.get('previousStatus', False)
    grasped = checkGrasped(w, name, hand, item, predCache)
    if previous:
        retq = grasped
    else:
        retq = (grasped and checkParked(w, name, hand, False, predCache))
    return retq

def _checkPlacedItem(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    container = description['container']
    allowedComponents = description.get('allowedComponents', None)
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    retq = (not checkGrasped(w, name, None, item, predCache)) and checkItemInContainer(w, name, item, container, predCache, allowedComponents=allowedComponents) and parked
    return retq

def _checkLoweredItemAt(w, name, description, node, predCache):
    item = description['item']
    container = description['container']
    allowedComponents = description.get('allowedComponents', None)
    retq = checkItemInContainer(w, name, item, container, predCache, allowedComponents=allowedComponents)
    return retq
    
def _checkItemAboveContainer(w, name, description, node, predCache): # item, hand, container[, sideHold]
    item = description["item"]
    hand = description["hand"]
    container = description["container"]
    sideHold = description.get("sideHold", False)
    allowedComponents = description.get('allowedComponents', None)
    retq = checkItemAboveContainer(w, name, item, container, predCache, hand, sideHold, allowedComponents=allowedComponents)
    return retq

def _checkTurnedControlAndParked(w, name, description, node, predCache): # item, hand, link, setting
    item = description["item"]
    hand = description["hand"]
    link = description["link"]
    setting = description["setting"]
    isSet, isSetting = checkControlSetting(w, name, hand, item, link, setting, predCache)
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    return isSet and (not isSetting) and parked

def _checkTurnedHand(w, name, description, node, predCache): # item, hand, link, setting
    item = description["item"]
    hand = description["hand"]
    link = description["link"]
    setting = description["setting"]
    isSet, isSetting = checkControlSetting(w, name, hand, item, link, setting, predCache)
    return isSet and (not isSetting)

def _checkTransferredAndStored(w, name, description, node, predCache):
    storage = description['storage']
    pouredType = description['pouredType']
    item = description['item']
    hand = description['hand']
    container = description['container']
    amount = description['amount']
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    return checkTransferred(w, name, item, pouredType, amount, container, predCache) and (not checkGrasped(w, name, None, item, predCache)) and checkItemInContainer(w, name, item, storage, predCache) and parked
    
def _checkTransferredAndUprighted(w, name, description, node, predCache):
    pouredType = description['pouredType']
    item = description['item']
    hand = description['hand']
    container = description['container']
    amount = description['amount']
    _, itemOrientation, _, _ = getKinematicData(w, (item,), predCache)
    pouringAxis = getContainmentFn(w, item, predCache).get("pouring", {}).get("outof", {}).get("axis") or [1,0,0]
    ornTh = 0.1
    if node.get('previousStatus', False):
        ornTh = 0.4
    upright = checkUpright(w, itemOrientation, pouringAxis, predCache, th=ornTh)
    return checkTransferred(w, name, item, pouredType, amount, container, predCache) and upright

def _checkTransferredContents(w, name, description, node, predCache):
    pouredType = description['pouredType']
    item = description['item']
    hand = description['hand']
    container = description['container']
    amount = description['amount']
    return checkTransferred(w, name, item, pouredType, amount, container, predCache)

def _checkConstraintFollowed(w, name, description, node, predCache): # constraintConjunctions, hand, isTop | waypoints, tolerances, entities
    def _equalp(a, b, t):
        d = [x-y for x,y in zip(a,b)]
        return t > d[0]*d[0]+d[1]*d[1]+d[2]*d[2]
    def _equalxy(a, b, t):
        d = [x-y for x,y in zip(a,b)]
        return t > d[0]*d[0]+d[1]*d[1]
    def _equalz(a, b, t):
        return t > abs(a-b)
    def _aligned(a, b, t):
        return t < a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
    def _orthogonal(a, b, t):
        return t > abs(a[0]*b[0]+a[1]*b[1]+a[2]*b[2])
    def _interpret(entity, mode):
        retq = entity
        if "equalxy" == mode:
            retq = retq[:2]
        elif "equalz" == mode:
            retq = retq[2]
        return retq
    constraintConjunctions = description["constraintConjunctions"]
    if 0 == len(constraintConjunctions):
        return True
    hand = description["hand"]
    fn = {"equalp": _equalp, "equalxy": _equalxy, "equalz": _equalz, "equalq": quaternionCloseness, "aligned": _aligned, "orthogonal": _orthogonal}
    hand = description["hand"]
    isTop = description["isTop"]
    waypoints = node["numerics"]["waypoints"]
    tolerances = node["numerics"]["tolerances"]
    entities = node["numerics"]["entities"]
    conjunction = constraintConjunctions[0]
    toleranceChoices = tolerances[0]
    previous = node.get("previousStatus", False)
    retq = True
    if previous:
        thresholds = [x[1] for x in toleranceChoices]
    else:
        thresholds = [x[0] for x in toleranceChoices]
    for constraint, threshold in zip(conjunction, thresholds):
        mode, entityA, entityB = constraint
        if not fn[mode](_interpret(entities[entityA], mode), _interpret(entities[entityB], mode), threshold):
            retq = False
    return retq

def _checkMixedAndStored(w, name, description, node, predCache):
    storage = description['storage']
    tool = description['tool']
    mixedType = description['mixedType']
    hand = description['hand']
    container = description['container']
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    return checkMixed(w, name, container, mixedType, predCache) and (not checkGrasped(w, name, None, tool, predCache)) and checkItemInContainer(w, name, tool, storage, predCache) and parked

def _checkMixedAndUprighted(w, name, description, node, predCache):
    tool = description['tool']
    toolLink = description['toolLink']
    mixedType = description['mixedType']
    hand = description['hand']
    container = description['container']
    _, toolOrientation, _, _ = getKinematicData(w, (tool,), predCache)
    mixingAxis = w._kinematicTrees[tool].get("fn", {}).get("mixing", {}).get("axis", {}).get(toolLink) or [1,0,0]
    ornTh = 0.1
    if node.get('previousStatus', False):
        ornTh = 0.4
    handLink = getHandLink(w, name, hand, predCache)
    _, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    upright = checkUpright(w, toolOrientation, mixingAxis, predCache, th=ornTh) and checkUpright(w, handQ, [1,0,0], predCache, th=ornTh)
    #parked = node.get('parked', False)
    #parked = checkParked(w, name, hand, parked, predCache)
    #node['parked'] = parked
    return checkMixed(w, name, container, mixedType, predCache) and upright

def _checkMixed(w, name, description, node, predCache):
    mixedType = description['mixedType']
    container = description['container']
    return checkMixed(w, name, container, mixedType, predCache)

def _checkMashedAndStored(w, name, description, node, predCache): # container, hand, storage, tool, toolLink |
    storage = description['storage']
    tool = description['tool']
    hand = description['hand']
    disposition = description["disposition"]
    container = description['container']
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    return checkMashed(w, name, container, disposition, predCache) and (not checkGrasped(w, name, None, tool, predCache)) and checkItemInContainer(w, name, tool, storage, predCache) and parked

def _checkMashedAndUprighted(w, name, description, node, predCache): # container, hand, storage, tool, toolLink |
    tool = description['tool']
    toolLink = description['toolLink']
    hand = description['hand']
    disposition = description["disposition"]
    container = description['container']
    _, toolOrientation, _, _ = getKinematicData(w, (tool,), predCache)
    mashingAxis = w._kinematicTrees[tool].get("fn", {}).get("mashing", {}).get("axis", {}).get(toolLink) or [1,0,0]
    ornTh = 0.1
    if node.get('previousStatus', False):
        ornTh = 0.4
    handLink = getHandLink(w, name, hand, predCache)
    _, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    upright = checkUpright(w, toolOrientation, mashingAxis, predCache, th=ornTh) and checkUpright(w, handQ, [1,0,0], predCache, th=ornTh)
    return checkMashed(w, name, container, disposition, predCache) and upright

def _checkMashed(w, name, description, node, predCache): # container, hand, storage, tool, toolLink |
    disposition = description["disposition"]
    container = description['container']
    return checkMashed(w, name, container, disposition, predCache)
    
def _checkLinedAndParked(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    lining = description['lining']
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    return (not checkGrasped(w, name, None, lining, predCache)) and checkItemInContainer(w, name, lining, item, predCache) and parked

def _checkLined(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    lining = description['lining']
    return checkItemInContainer(w, name, lining, item, predCache)

def _checkCoveredAndParked(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    cover = description['cover']
    parked = node.get('parked', False)
    covered = node.get("covered", False)
    parked = checkParked(w, name, hand, parked, predCache)
    covered = checkItemCoversContainer(w, name, cover, item, predCache, covered)
    node['parked'] = parked
    node["covered"] = covered
    return (not checkGrasped(w, name, None, cover, predCache)) and covered and parked

def _checkCovered(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    cover = description['cover']
    covered = node.get("covered", False)
    node["covered"] = checkItemCoversContainer(w, name, cover, item, predCache, covered)
    return node["covered"]

def _checkShapedAndParked(w, name, description, node, predCache):
    item = description.get('item', None)
    hand = description.get('hand', None)
    destination = description.get('destination', None)
    shapedType = description.get('shapedType', None)
    ingredientTypes = description.get('ingredientTypes', [])
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    if description.get('itemIsShaped', False):
        shapingDone = (shapedType == w._kinematicTrees[item].get("type"))
    else:
        shapingDone = (not containsIngredients(w, item, ingredientTypes, predCache)[0]) and (not containsShapes(w, item, shapedType, predCache)[0])
    retq = shapingDone and (not isHoldingObjectOfType(w, name, hand, shapedType, predCache)[0]) and parked
    return retq

def _checkSprinkled(w, name, description, node, predCache):
    item = description.get('item', None)
    hand = description.get('hand', None)
    shaker = description.get('shaker', None)
    storage = description.get('storage', None)
    sprinkledType = description.get('sprinkledType', None)
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    return checkSprinkled(w, name, item, sprinkledType, predCache)[0] and (not checkGrasped(w, name, None, shaker, predCache)) and checkItemInContainer(w, name, shaker, storage, predCache) and parked

def _checkBaked(w, name, description, node, predCache):
    item = description.get('item', None)
    hand = description.get('hand', None)
    oven = description.get('oven', None)
    destination = description.get('destination', None)
    bakedType = description.get('bakedType', None)
    processDisposition = description.get('processDisposition', 'bakable')
    baked = checkBakedContents(w, item, bakedType, processDisposition, predCache)
    container, component = getContainerComponent(w, item)
    atOven = (container == oven)
    atDestination = (container == destination)
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    if atOven:
        node['sourceContainer'] = oven
        node['sourceComponent'] = component
    return baked and atDestination and parked

def _checkCutItem(w, name, description, node, predCache):
    item = description['item']
    tool = description['tool']
    hand = description['hand']
    storage = description['storage']
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    return (not w._kinematicTrees.get(item, {}).get("fn", {}).get("cuttable")) and (not checkGrasped(w, name, None, tool, predCache)) and checkItemInContainer(w, name, tool, storage, predCache) and parked

def _checkPeeledAndStored(w, name, description, node, predCache): # container, containerForPeels, hand, item, storage, tool
    item = description['item']
    tool = description['tool']
    hand = description['hand']
    storage = description['storage']
    disposition = description.get("disposition")
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    return (not w._kinematicTrees.get(item, {}).get("fn", {}).get(disposition)) and (not checkGrasped(w, name, None, item, predCache)) and (not checkGrasped(w, name, None, tool, predCache)) and checkItemInContainer(w, name, item, container, predCache) and checkItemInContainer(w, name, tool, storage, predCache) and parked

def _checkArmTriggeredPortalHandle(w, name, description, node, predCache):
    container = description.get('container', None)
    component = description.get('component', None)
    hand = description.get('hand', None)
    action = description.get('action', None)
    if (container is not None) and (action is not None):
        handle, door = getHandleAndDoor(w, container, component, predCache)
        if handle is None: # No handle -- nothing to open/close
            return True
        actualAction = w._kinematicTrees[container].get("customStateVariables", {}).get("clopening", {}).get("action", {}).get(door)
        return actualAction == action
        #if actualAction == action: # Already performing action -- no need to keep arm there
        #    return True
        #doingAction = (action == w._kinematicTrees[name].get("customStateVariables", {}).get("clopening", {}).get("action", {}).get(hand))
        #hand = description.get('hand', None)
        #handLink = getHandLink(w, name, hand, predCache)
        #maxDistance = w._kinematicTrees[container].get("fn", {}).get("clopening", {}).get("radius", {}).get(door) or 0.5
        #if not node.get('previousStatus', False):
        #    maxDistance = 0.8*maxDistance
        #return action == actualAction
    return True

def _checkStoppedOpening(w, name, description, node, predCache):
    hand = description.get('hand', None)
    if hand is not None:
        retq = ("open" != w._kinematicTrees[name].get("customStateVariables", {}).get("clopening", {}).get("action", {}).get(hand))
    else:
        clactions = w._kinematicTrees[name].get("customStateVariables", {}).get("clopening", {}).get("action", {})
        retq = (not any(["open" == v for _, v in clactions.items()]))
    return retq

def _checkStoppedClosing(w, name, description, node, predCache):
    hand = description.get('hand', None)
    if hand is not None:
        return "close" != w._kinematicTrees[name].get("customStateVariables", {}).get("clopening", {}).get("action", {}).get(hand)
    else:
        clactions = w._kinematicTrees[name].get("customStateVariables", {}).get("clopening", {}).get("action", {})
        return not any(["close" == v for _, v in clactions.items()])

def _checkOpened(w, name, description, node, predCache):
    container = description.get('container', None)
    component = description.get('component', None)
    hand = description.get('hand', None)
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    retq = checkOpened(w, container, component, predCache) and parked
    return retq

def _checkClosed(w, name, description, node, predCache):
    container = description.get('container', None)
    component = description.get('component', None)
    hand = description.get('hand', None)
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    return checkClosed(w, container, component, predCache) and parked

def _checkBroughtNear(w, name, description, node, predCache):
    trajector = description.get('trajector', None)
    hand = description.get('hand', None)
    relatum = description.get('relatum', None)
    previous = node.get('previousStatus', False)
    return checkGrasped(w, name, hand, trajector, predCache) and checkNear(w, name, relatum, previous, predCache)
    
def _suggestNotifiedCompletion(w, name, description, node, predCache):
    return [{'type': 'P', 'description': {'process': 'notifyingCompletion'}, 'target': {'internalFlags': {'completion': True}}}] # bpt: (processGardening, completed): (True)

def _suggestNoped(w, name, description, node, predCache):
    return [{'type': 'P', 'description': {'process': 'notifyingCompletion'}, 'target': None}] # bpt: None

def _suggestWaited(w, name, description, node, predCache):
    timeEnd = description.get('timeEnd', 0)
    timeNow = w._kinematicTrees[name].get("customStateVariables", {}).get("timing", {}).get("timer") or 0
    if timeNow < timeEnd:
        w.setFrameStepCount(1 + math.ceil((timeEnd-timeNow)*w._sfr))
    return [{"type": "P", "description": {"process": "waiting"}, 'children': [], 'numerics': {}, "target": None}]

def _suggestStoppedHands(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "stoppingHands"}, 'children': [], 'numerics': {}, "target": {"hand_right": {"target": None}, "hand_left": {"target": None}}}]

def _suggestNearA(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "nearing", "relatum": description["relatum"]}, 'children': [], 'numerics': {"position": node["numerics"].get("position")}, "target": None}]

def _suggestParkedArm(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "parkingArm", "hand": description["hand"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestArmNearItemHandle(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "armNearingItemHandle", "hand": description["hand"], "item": description["item"]}, 'children': [], 'numerics': {}, "target": None}]
    
def _suggestGrasped(w, name, description, node, predCache):
    hand = description["hand"]
    item = description["item"]
    return [{"type": "P", "description": {"process": "grasping", "hand": hand, "item": item}, 'children': [], 'numerics': {}, "target": {"grasping": {hand: {"toAdd": [item]}}}}]

def _suggestUngrasped(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    return [{"type": "P", "description": {"process": "ungrasping", "hand": hand, "item": item}, 'children': [], 'numerics': {}, "target": {"grasping": {hand: {"toRemove": [item]}}}}]

def _suggestPickedItem(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "pickingItem", "hand": description["hand"], "item": description["item"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestPlacedItem(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "placingItem", "hand": description["hand"], "item": description["item"], "container": description["container"], "allowedComponents": description.get("allowedComponents")}, 'children': [], 'numerics': {}, "target": None}]

def _suggestLoweredItemAt(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "loweringItem", "hand": description["hand"], "item": description["item"], "container": description["container"], "allowedComponents": description.get("allowedComponents")}, 'children': [], 'numerics': {}, "target": None}]

def _suggestItemAboveContainer(w, name, description, node, predCache): # item, hand, container[, sideHold]
    return [{"type": "P", "description": {"process": "holdingItemAbove", "hand": description["hand"], "item": description["item"], "container": description["container"], "allowedComponents": description.get("allowedComponents"), "sideHold": description.get("sideHold", False)}, 'children': [], 'numerics': {}, "target": None}]

def _suggestTransferredAndStored(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "transferringAndStoring", "hand": description["hand"], "item": description["item"], "amount": description["amount"], "container": description["container"], "pouredType": description["pouredType"], "storage": description["storage"], "allowedComponents": description.get("allowedComponents")}, 'children': [], 'numerics': {}, "target": None}]
    
def _suggestTransferredAndUprighted(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "transferringAndUprighting", "hand": description["hand"], "item": description["item"], "amount": description["amount"], "container": description["container"], "pouredType": description["pouredType"]}, 'children': [], 'numerics': {}, "target": None}]
                         
def _suggestTransferredContents(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "transferringContents", "hand": description["hand"], "item": description["item"], "container": description["container"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestTurnedControlAndParked(w, name, description, node, predCache): # item, hand, link, setting
    hand = description["hand"]
    item = description["item"]
    link = description["link"]
    setting = description["setting"]
    isSet, isSetting = checkControlSetting(w, name, hand, item, link, setting, predCache)
    if isSet and (not isSetting):
        return [{"type": "P", "description": {"process": "parkingArm", "hand": hand}, 'children': [], 'numerics': {}, "target": None}]
    else:
        return [{"type": "P", "description": {"process": "turningControl", "hand": hand, "item": item, "link": link, "setting": setting}, 'children': [], 'numerics': {}, "target": None}]

def _suggestTurnedHand(w, name, description, node, predCache): # item, hand, link, setting
    hand = description["hand"]
    item = description["item"]
    link = description["link"]
    setting = description["setting"]
    isSet, isSetting = checkControlSetting(w, name, hand, item, link, setting, predCache)
    if isSet:
        target = {"turning": {hand: {"toRemove": [(item, link)]}}}
    else:
        target = {"turning": {hand: {"toAdd": [(item, link)]}}}
    return [{"type": "P", "description": {"process": "turningHand", "hand": hand, "item": item, "link": link, "setting": setting}, 'children': [], 'numerics': {}, "target": target}]
    
def _suggestConstraintFollowed(w, name, description, node, predCache): # constraintConjunctions, hand, isTop | waypoints, tolerances
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
        nsqA = (dA[0]*dA[0] + dA[1]*dA[1] + dA[2]*dA[2])
        dB = [x-y for x,y in zip(positionB, positionCr)]
        nsqB = (dB[0]*dB[0] + dB[1]*dB[1] + dB[2]*dB[2])
        if 0.0001 > nsqA:
            targetP = positionB
        elif 0.0001 > nsqB:
            targetP = positionA
        else:
            nA = math.sqrt(nsqA)
            dirA = [x/nA for x in dA]
            nB = math.sqrt(nsqB)
            dirB = [x/nB for x in dB]
            if (dirB[0]*linVelCr[0] + dirB[1]*linVelCr[1] + dirB[2]*linVelCr[2]) > (dirA[0]*linVelCr[0] + dirA[1]*linVelCr[1] + dirA[2]*linVelCr[2]):
                targetP = positionB
    return [{"type": "P", "description": {"process": "constraintFollowing", "constraintConjunctions": constraintConjunctions, "hand": hand, "isTop": False}, 'children': [], 'numerics': {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}, "target": {hand: {"target": [targetP, targetQ], "positionInLink": positionInLink, "orientationInLink": orientationInLink}}}]

def _suggestMixedAndStored(w, name, description, node, predCache): # container, hand, mixedType, storage, tool, toolLink |
    return [{"type": "P", "description": {"process": "mixingAndStoring", "container": description["container"], "hand": description["hand"], "tool": description["tool"], "toolLink": description["toolLink"], "mixedType": description["mixedType"], "storage": description["storage"]}, 'children': [], 'numerics': {}, "target": None}]
    
def _suggestMixedAndUprighted(w, name, description, node, predCache): # container, hand, mixedType, tool, toolLink |
    return [{"type": "P", "description": {"process": "mixingAndUprighting", "container": description["container"], "hand": description["hand"], "tool": description["tool"], "toolLink": description["toolLink"], "mixedType": description["mixedType"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestMixed(w, name, description, node, predCache): # container, hand, mixedType, tool, toolLink |
    return [{"type": "P", "description": {"process": "mixing", "container": description["container"], "hand": description["hand"], "tool": description["tool"], "toolLink": description["toolLink"], "mixedType": description["mixedType"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestMashedAndStored(w, name, description, node, predCache): # container, hand, storage, tool, toolLink |
    return [{"type": "P", "description": {"process": "mashingAndStoring", "disposition": description["disposition"], "container": description["container"], "hand": description["hand"], "tool": description["tool"], "toolLink": description["toolLink"], "storage": description["storage"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestMashedAndUprighted(w, name, description, node, predCache): # container, hand, tool, toolLink |
    return [{"type": "P", "description": {"process": "mashingAndUprighting", "disposition": description["disposition"], "container": description["container"], "hand": description["hand"], "tool": description["tool"], "toolLink": description["toolLink"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestMashed(w, name, description, node, predCache): # container, hand, tool, toolLink |
    return [{"type": "P", "description": {"process": "mashing", "disposition": description["disposition"], "container": description["container"], "hand": description["hand"], "tool": description["tool"], "toolLink": description["toolLink"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestLinedAndParked(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "liningAndParking", "lining": description["lining"], "hand": description["hand"], "item": description["item"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestLined(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "lining", "lining": description["lining"], "hand": description["hand"], "item": description["item"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestCoveredAndParked(w, name, description, node, predCache): # hand, item, cover |
    return [{"type": "P", "description": {"process": "coveringAndParking", "cover": description["cover"], "hand": description["hand"], "item": description["item"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestCovered(w, name, description, node, predCache): # hand, item, cover |
    return [{"type": "P", "description": {"process": "covering", "cover": description["cover"], "hand": description["hand"], "item": description["item"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestShapedAndParked(w, name, description, node, predCache):
    item = description.get('item', None)
    hand = description.get('hand', None)
    destination = description.get('destination', None)
    shapedType = description.get('shapedType', None)
    ingredientTypes = description.get('ingredientTypes', [])
    if description.get('itemIsShaped', False):
        ingredientsAvailable, pickedIngredients = False, []
        itemHasShape, shapes = False, []
        if (shapedType != w._kinematicTrees[item].get("type")):
            ingredientsAvailable, pickedIngredients = True, [item]
        elif (destination != w.at((item,))):
            itemHasShape, shapes = True, [item]
    else:
        ingredientsAvailable, pickedIngredients = containsIngredients(w, item, ingredientTypes, predCache)
        itemHasShape, shapes = containsShapes(w, item, shapedType, predCache)
    holdingShape, heldShape = isHoldingObjectOfType(w, name, hand, shapedType, predCache)
    handLink = getHandLink(w, name, hand, predCache)
    aabb = w.getAABB((name, handLink))
    aabbAdj = w.adjustAABBRadius(aabb, w._kinematicTrees[name].get("fn", {}).get("grasping", {}).get("graspingActivationRadius", {}).get(hand, 0.5))
    closeShapes = set([x[0] for x in w.checkOverlap(aabbAdj)])
    closeShapes = [x for x in closeShapes if (shapedType == w._kinematicTrees[x].get("type") and (destination != w.at((x,))))]
    closeShape = None
    containedShape = None
    if 0 < len(shapes):
        containedShape = sorted(shapes)[0]
    if 0 < len(closeShapes):
        closeShape = sorted(closeShapes)[0]
    if (not holdingShape) and (not ingredientsAvailable) and (not itemHasShape):
        parkedP, parkedQ, _ = getParkedPose(w, name, hand, predCache)
        return [{"type": "P", "description": {"process": "parkingArm", "hand": hand}, 'children': [], 'numerics': {}, "target": {hand: {"target": [parkedP, parkedQ], "positionInLink": None, "orientationInLink": None, "clopeningAction": None}}}]
    elif holdingShape:
        csvPG = w._kinematicTrees[name].get("customStateVariables", {}).get("processGardening", {})
        if "log" not in csvPG:
            csvPG["log"] = {}
        if "shaping" not in csvPG["log"]:
            csvPG["log"]["shaping"] = {}
        if "products" not in csvPG["log"]["shaping"]:
            csvPG["log"]["shaping"]["products"] = []
        made = csvPG.get("log", {}).get("shaping", {}).get("products", [])
        if heldShape not in made:
            made.append(heldShape)
        csvPG["log"]["shaping"]["products"] = made
        return [{"type": "P", "description": {"process": "placingItem", "hand": hand, "item": heldShape, "container": destination, "allowedComponents": None}, 'children': [], 'numerics': {}, "target": None}]
    elif closeShape is not None:
        return [{"type": "P", "description": {"process": "movingArm", "hand": hand, "item": heldShape}, 'children': [], 'numerics': {}, "target": {"grasping": {hand: {"toAdd": [closeShape]}}}}]
    elif containedShape is not None:
        return [{"type": "P", "description": {"process": "pickingItem", "hand": hand, "item": containedShape}, 'children': [], 'numerics': {}, "target": None}]
    return [{"type": "P", "description": {"process": "shaping", "shapedType": shapedType, "hand": hand, "item": item, "ingredientTypes": ingredientTypes}, 'children': [], 'numerics': {}, "target": {"shapingIngredients": {hand: {"toSet": pickedIngredients}}, "shapingOutcome": {hand: shapedType}}}]

def _suggestCutItem(w, name, description, node, predCache):
    item = description["item"]
    if w._kinematicTrees.get(item, {}).get("fn", {}).get("cuttable"):
        return [{"type": "P", "description": {"process": "cuttingItem", "hand": description["hand"], "item": item, "storage": description["storage"], "tool": description["tool"]}, 'children': [], 'numerics': {}, "target": None}]
    else:
        return [{"type": "P", "description": {"process": "placingItem", "hand": description["hand"], "item": description["tool"], "container": description["storage"], "allowedComponents": description.get("allowedComponents")}, 'children': [], 'numerics': {}, "target": None}]

def _suggestPeeledAndStored(w, name, description, node, predCache): # container, containerForPeels, hand, item, storage, tool
    item = description["item"]
    hand = description["hand"]
    otherHand = description["otherHand"]
    disposition = description.get("disposition")
    if w._kinematicTrees.get(item, {}).get("fn", {}).get(disposition):
        return [{"type": "P", "description": {"process": "peelingItem", "hand": hand, "item": item, "otherHand": otherHand, "storage": description["storage"], "tool": description["tool"]}, 'children': [], 'numerics': {}, "target": None}]
    elif (not w._kinematicTrees.get(item, {}).get("fn", {}).get("disposition")) and (checkGrasped(w, name, None, item, predCache)):
        return [{"type": "P", "description": {"process": "placingItem", "hand": otherHand, "item": description["item"], "container": description["container"], "allowedComponents": description.get("allowedComponents")}, 'children': [], 'numerics': {}, "target": None}]
    else:
        return [{"type": "P", "description": {"process": "placingItem", "hand": hand, "item": description["tool"], "container": description["storage"], "allowedComponents": description.get("allowedComponentsStorage")}, 'children': [], 'numerics': {}, "target": None}]

def _suggestOpened(w, name, description, node, predCache):
    container = description.get('container', None)
    component = description.get('component', None)
    hand = description.get('hand', None)
    if not checkOpened(w, container, component, predCache):
        return [{'type': 'P', 'description': {'process': 'opening', 'container': container, 'component': component, 'hand': hand}, 'children': [], 'numerics': {}, "target": None}]
    return [{"type": "P", "description": {"process": "parkingArm", "hand": description["hand"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestClosed(w, name, description, node, predCache):
    container = description.get('container', None)
    component = description.get('component', None)
    hand = description.get('hand', None)
    if not checkClosed(w, container, component, predCache):
        return [{'type': 'P', 'description': {'process': 'closing', 'container': container, 'component': component, 'hand': hand}, 'children': [], 'numerics': {}, "target": None}]
    return [{"type": "P", "description": {"process": "parkingArm", "hand": description["hand"]}, 'children': [], 'numerics': {}, "target": None}]

def _suggestStoppedOpening(w, name, description, node, predCache):
    hand = description.get('hand', None)
    target = {}
    if hand is None:
        for hand in w._kinematicTrees[name].get("fn", {}).get("clopening", {}).get("clopeningEFs", {}).keys():
            target[hand] = {'clopeningAction': None}
    else:
        target[hand] = {'clopeningAction': None}
    return [{'type': 'P', 'description': {'process': 'stopOpening'}, 'target': target}] # bpt: (clopening, opening, _Hand): (False)

def _suggestStoppedClosing(w, name, description, node, predCache):
    hand = description.get('hand', None)
    target = {}
    if hand is None:
        for hand in w._kinematicTrees[name]("fn", {}).get("clopening", {}).get("clopeningEFs", {}).keys():
            target[hand] = {'clopeningAction': None}
    else:
        target[hand] = {'clopeningAction': None}
    return [{'type': 'P', 'description': {'process': 'stopClosing'}, 'target': target}] # bpt: (clopening, closing, _Hand): (False)

def _suggestBakedItem(w, name, description, node, predCache):
    item = description.get('item', None)
    hand = description.get('hand', None)
    oven = description.get('oven', None)
    destination = description.get('destination', None)
    bakedType = description.get('bakedType', None)
    processDisposition = description.get('processDisposition', 'bakable')
    timeAmount = description.get('timeAmount')
    timeUnit = description.get('timeUnit')
    return [{'type': 'P', 'description': {'process': 'bakingItem', 'item': item, 'hand': hand, 'oven': oven, 'destination': destination, 'bakedType': bakedType, 'processDisposition': processDisposition, 'timeAmount': timeAmount, 'timeUnit': timeUnit}, 'sourceContainer': node.get('sourceContainer'), 'sourceComponent': node.get('sourceComponent')}]

def _suggestBroughtNear(w, name, description, node, predCache):
    return [{'type': 'P', 'description': {"process": "bringingNear", "trajector": description["trajector"], "hand": description["hand"], "relatum": description["relatum"]}, "children": [], "numerics": {}, "target": None}]

def _suggestArmTriggeredPortalHandle(w, name, description, node, predCache):
    container = description.get('container', None)
    component = description.get('component', None)
    action = description.get('action', None)
    if (container is not None) and (action is not None):
        handle, door = getHandleAndDoor(w, container, component, predCache)
        hand = description.get('hand', None)
        handLink = getHandLink(w, name, hand, predCache)
        maxDistance = 0.3*w._kinematicTrees[container].get("fn", {}).get("clopening", {}).get("radius", {}).get(component, 0.5)
        handLocalAABB = w._kinematicTrees[name]["localAABB"][handLink]
        handXHalf = 0.5*(handLocalAABB[1][0] - handLocalAABB[0][0])
        _, handOrientation, _, _ = getKinematicData(w, (name, handLink), predCache)
        handlePosition, _, _, _ = getKinematicData(w, (container, handle), predCache)
        containerPosition, _, _, _ = getKinematicData(w, (container,), predCache)
        offsetYaw = getFacingYaw(w, container, containerPosition, predCache) + math.pi
        offsetVector = stubbornTry(lambda : pybullet.rotateVector(pybullet.getQuaternionFromEuler((0,0,offsetYaw)), (maxDistance + handXHalf, 0, 0)))
        target = {hand: {'target': [[x+y for x,y in zip(handlePosition, offsetVector)], handOrientation], 'positionInLink': None, 'orientationInLink': None, 'clopeningAction': action}}
        return [{'type': 'P', 'description': {'process': 'movingArm', 'hand': hand}, "children": [], "numerics": {}, "target": target}]
    return []

def _suggestSprinkled(w, name, description, node, predCache):
    return [{"type": "P", "description": {"process": "sprinkling", "hand": description["hand"], "item": description["item"], "shaker": description["shaker"], "storage": description["storage"], "sprinkledType": description["sprinkledType"]}, 'children': [], 'numerics': {}, "target": None}]
  
def _emptyList(w, name, description, node, predCache):
    return []

def _getNearingConditionsA(w, name, description, node, predCache):
    relatum = description.get('relatum', None)
    relatumPosition, _, _, _ = getKinematicData(w, (relatum,), predCache)
    relatumPosition = list(relatumPosition)
    goalPosition = node['numerics'].get('position', None)
    targetPosition = relatumPosition
    if goalPosition is not None:
        targetPosition = goalPosition
    baseLink = w._kinematicTrees[name].get("fn", {}).get("kinematicControl", {}).get("mobileBaseLink")
    baseP, baseQ, _, _ = getKinematicData(w, (name, baseLink), predCache)
    baseFwdP, _ = w.objectPoseRelativeToWorld(baseP, baseQ, baseFwdOffset, [0,0,0,1])
    facingYaw = getFacingYaw(w, relatum, relatumPosition, predCache)
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
    tolf2 = [0.95, 0.7]
    conz = ['equalp', 'baseP', 'zeroP']
    tolz = [0.05, 0.2]
    constraintConjunctions = [[conp, conf], [conf], [conz]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wppf = [[targetPosition[0], targetPosition[1], 0], facingQ, None, baseFwdOffset, None, None, None]
    wpf = [[0, 0, 0], facingQ, None, None, None, None, None]
    wpz = [[0, 0, 0], baseQ, None, None, None, None, None]
    waypoints = [wppf, wpf, wpz]
    tolerances = [[tolp, tolf], [tolf2], [tolz]]
    entities = {'zeroP': [0,0,0], 'baseP': baseP, 'baseFwdP': baseFwdP, 'baseQ': baseQ, 'targetP': targetPosition, 'targetQ': facingQ}
    retq = [{"type": "G", "description": {"goal": "parkedArm", "hand": "hand_left"}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "parkedArm", "hand": "hand_right"}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "stoppedHands"}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": "base", "constraintConjunctions": constraintConjunctions, "isTop": True}, "children":[], "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]
    return retq
                      
def _getParkingArmConditions(w, name, description, node, predCache):
    hand = description["hand"]
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getComponentUnderHandHeight(w, name, hand, handP, predCache)
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
    entities = {'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight]}
    return [{"type": "G", "description": {"goal": "stoppedOpening", "hand": hand}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "stoppedClosing", "hand": hand}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getArmNearingItemHandleConditions(w, name, description, node, predCache):
    hand = description['hand']
    item = description['item']
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    aabbItem = w.getAABB((item,))
    handleLink = getHandleLink(w, item, predCache)
    aabbHandLocal = w._kinematicTrees[item]["localAABB"][handleLink]
    handlePosition, _, _, _ = getKinematicData(w, (item, handleLink), predCache)
    graspingFn = getGraspingFn(w, item, predCache)
    handleAxisInWorld = getAxisInWorld(w, item, graspingFn.get("axis", {}).get(handleLink), predCache)
    graspedQ = handQ
    if handleAxisInWorld is not None:
        handleYaw = math.atan2(handleAxisInWorld[1], handleAxisInWorld[0])
        graspedQ = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,handleYaw)))
    aboveHandleP = list(handlePosition)
    aboveHandleP[2] = aabbItem[1][2] - aabbHandLocal[0][2]
    entryHeight = getEntryHeight(w, name, description, node['numerics'], predCache)
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
    entities = {'handP': handP, 'handQ': handQ, 'aboveHandleP': aboveHandleP, 'graspedQ': graspedQ, 'entryHeight': [0,0,entryHeight]}
    return [{"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getGraspingConditions(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    freeHand = getFreeHand(w, name, hand, predCache)
    container, component = getContainerComponent(w, item)
    return [{"type": "G", "description": {"goal": "opened", "container": container, "component": component, "hand": freeHand}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "stoppedOpening", "hand": freeHand}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "armNearItemHandle", "hand": hand, "item": item}, "children": [], "previousStatus": None, "numerics": {}}]
    
def _getPickingItemConditions(w, name, description, node, predCache):
    hand = description.get("hand")
    return [{"type": "G", "description": {"goal": "grasped", "hand": hand, "item": description["item"]}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "parkedArm", "hand": hand}, "children": [], "previousStatus": None, "numerics": {}}]

def _getPlacingItemConditions(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    container = description['container']
    allowedComponents = description.get('allowedComponents', None)
    # TODO: close containers once done?
    return [{"type": "G", "description": {"goal": "loweredItemAt", "hand": hand, "item": item, "container": description.get("container"), "allowedComponents": description.get("allowedComponents")}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "ungrasped", "hand": hand, "item": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "parkedArm", "hand": hand}, "children": [], "previousStatus": None, "numerics": {}}]

def _getLoweringItemConditions(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    container = description['container']
    allowedComponents = description.get('allowedComponents', None)
    freeHand = getFreeHand(w, name, hand, predCache)
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
    containerP, _, _, _ = getKinematicData(w, (container,), predCache)
    itemInHandP, itemInHandQ = w.objectPoseRelativeToObject(handP, handQ, itemP, itemQ)
    facingYaw = getFacingYaw(w, container, containerP, predCache)
    _, fwdItemQ = w.objectPoseRelativeToWorld((0,0,0), handQ, (0,0,0), itemInHandQ)
    component, placementP = getItemPlacement(w, name, item, container, node['numerics'].get('component'), node['numerics'].get('position'), predCache, allowedComponents=allowedComponents)
    if component is None:
        node["error"] = "Container %s is full" % container
        return []
    node['numerics']['component'] = component
    node['numerics']['position'] = placementP
    placementQ = fwdItemQ
    if description.get('matchOrientation', False) is True:
        _, placementQ, _, _ = getKinematicData(w, (container,), predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'destination': [container, component]}, node['numerics'], predCache)
    itemEntryHeight = entryHeight + stubbornTry(lambda : pybullet.rotateVector(itemInHandQ, itemInHandP))[2]
    #For lowering item:
    #  we want p: item at placement
    #  We can achieve p by a lowering process that maintains xy: itemXY at placementXY
    #  We can achieve xy by a bringing process that maintains z: itemZ at entryHeight (adjusted for item in hand)
    #  we can achieve z by a lifting process: handZ at entryHeight
    conp = ['equalp', 'itemP', 'placementP']
    tolp = [0.0001, 0.0004]
    conxy = ['equalxy', 'itemP', 'placementP']
    tolxy = [0.0001, 0.0004]
    conz = ['equalz', 'handP', 'entryHeight']
    tolz = [0.01, 0.05]
    constraintConjunctions = [[conp], [conxy], [conz]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpp = [placementP, placementQ, None, itemInHandP, itemInHandQ, None, None]
    wpxy = [[placementP[0], placementP[1], itemEntryHeight], placementQ, None, itemInHandP, itemInHandQ, None, None]
    wpz = [[handP[0], handP[1], entryHeight], handQ, None, None, None, None, None]
    waypoints = [wpp, wpxy, wpz]
    tolerances = [[tolp], [tolxy], [tolz]]
    entities = {'handP': handP, 'itemP': itemP, 'placementP': placementP, 'entryHeight': [0,0,entryHeight]}
    retq = [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": container}, "children": [], "previousStatus": None, "numerics": {"position": node["numerics"].get("position")}},
            {"type": "G", "description": {"goal": "opened", "container": container, "component": node["numerics"].get("component"), "hand": freeHand}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]
    return retq
                      
def _getHoldingItemAboveConditions(w, name, description, node, predCache): # container, hand, item, sideHold[, allowedComponents] |
    item = description['item']
    hand = description['hand']
    container = description['container']
    allowedComponents = description.get('allowedComponents', None)
    sideHold = description.get("sideHold", False)
    freeHand = getFreeHand(w, name, hand, predCache)
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
    containerP, _, _, _ = getKinematicData(w, (container,), predCache)
    itemInHandP, itemInHandQ = w.objectPoseRelativeToObject(handP, handQ, itemP, itemQ)
    facingYaw = getFacingYaw(w, container, containerP, predCache)
    _, fwdItemQ = w.objectPoseRelativeToWorld((0,0,0), handQ, (0,0,0), itemInHandQ)
    pouringFn = w._kinematicTrees[container].get("fn", {}).get("containment", {}).get("pouring", {}).get("into", {})
    component = pouringFn.get("link")
    pointC = pouringFn.get("point")
    cP, cQ, _, _ = getKinematicData(w, (container, component), predCache)
    placementP, _ = w.objectPoseRelativeToWorld(cP, cQ, pointC, [0,0,0,1])
    placementP = [placementP[0], placementP[1], aabbComponent[1][2] + 0.05 + itemP[2] - aabbItem[0][2]]
    #component, placementP = getItemPlacement(w, name, item, container, node['numerics'].get('component'), node['numerics'].get('position'), predCache, allowedComponents=allowedComponents)
    if component is None:
        node["error"] = "Container %s is full" % container
        return []
    node['numerics']['component'] = component
    node['numerics']['position'] = placementP
    placementQ = fwdItemQ
    if description.get('matchOrientation', False) is True:
        _, placementQ, _, _ = getKinematicData(w, (container,), predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'destination': [container, component]}, node['numerics'], predCache)
    itemEntryHeight = entryHeight + stubbornTry(lambda : pybullet.rotateVector(itemInHandQ, itemInHandP))[2]
    placementQAdj = placementQ
    if sideHold:
        xtg = [math.cos(facingYaw), math.sin(facingYaw), 0]
        ztg = [-math.sin(facingYaw), math.cos(facingYaw), 0]
        if "hand_left" == hand:
            ztg[0] = -ztg[0]
        sideHoldQ = quatFromVecPairs(([0,0,1], xtg), ([0,0,1], ztg))
        _, placementQAdj = w.objectPoseRelativeToWorld([0,0,0], sideHoldQ, [0,0,0], itemInHandQ)
    _, handPlacementQ = w.handPoseToBringObjectToPose(itemInHandP, itemInHandQ, placementP, placementQAdj)
    #For holding item above container:
    #  we want p/q: item at placement, hand at orientation selected via sideHold
    #  We can achieve p by a lowering process that maintains xy/q: itemXY at placementXY, hand at sideHold-selected orientation
    #  we can achieve xy/q by a rotating process that maintains xy: itemXY at placementXY
    #  We can achieve xy by a bringing process that maintains z: itemZ at entryHeight (adjusted for item in hand)
    #  we can achieve z by a lifting process: handZ at entryHeight
    conp = ['equalp', 'itemP', 'placementP']    
    tolp = [0.0001, 0.0004]
    conq = ['equalq', 'handQ', 'handPlacementQ']
    tolq = [0.99, 0.95]
    conxy = ['equalxy', 'itemP', 'placementP']
    tolxy = [0.0001, 0.0004]
    conz = ['equalz', 'handP', 'entryHeight']
    tolz = [0.01, 0.05]
    constraintConjunctions = [[conp, conq], [conxy, conq], [conxy], [conz]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wppq = [placementP, placementQAdj, None, itemInHandP, itemInHandQ, None, None]
    wpxyq = [[placementP[0], placementP[1], itemEntryHeight], placementQAdj, None, itemInHandP, itemInHandQ, None, None]
    wpxy = [[placementP[0], placementP[1], itemEntryHeight], placementQ, None, itemInHandP, itemInHandQ, None, None]
    wpz = [[handP[0], handP[1], entryHeight], handQ, None, None, None, None, None]
    waypoints = [wpp, wpxy, wpz]
    tolerances = [[tolp, tolq], [tolxy, tolq], [tolxy], [tolz]]
    entities = {'handP': handP, 'handQ': handQ, 'itemP': itemP, 'placementP': placementP, 'handPlacementQ': handPlacementQ, 'entryHeight': [0,0,entryHeight]}
    retq = [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": container}, "children": [], "previousStatus": None, "numerics": {"position": node["numerics"].get("position")}},
            {"type": "G", "description": {"goal": "opened", "container": container, "component": node["numerics"].get("component"), "hand": freeHand}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]
    return retq

def _getTransferringAndStoringConditions(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    return [{"type": "G", "description": {"goal": "transferredAndUprighted", "hand": hand, "item": item, "pouredType": description["pouredType"], "amount": description["amount"], "container": description["container"]}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "placedItem", "hand": hand, "item": item, "container": description.get("storage")}, "children": [], "previousStatus": None, "numerics": {}}]
            
def _getTransferringAndUprightingConditions(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    container = description['container']
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': container}, {}, predCache)
    _, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    #For uprighting:
    #  we want U: hand at parked orientation
    #  We can achieve U by an uprighting process that maintains z: handZ at entryHeight
    conU = ['equalq', 'handQ', 'handParkedQ']
    tolU = [0.99, 0.95]
    constraintConjunctions = [[conU]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpU = [[handP[0], handP[1], entryHeight], handParkedQ, None, None, None, None, None]
    waypoints = [wpU]
    tolerances = [[tolU]]
    entities = {'handP': handP, 'handQ': handQ, 'handParkedQ': handParkedQ, 'entryHeight': entryHeight}
    return [{"type": "G", "description": {"goal": "transferredContents", "hand": hand, "item": item, "container": container, "amount": description["amount"], "pouredType": description["pouredType"]}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getTransferringContentsConditions(w, name, description, node, predCache):
    item = description['item']
    hand = description['hand']
    container = description['container']
    down = w.getDown()
    handLink = getHandLink(w, name, hand, predCache)
    _, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': container}, {}, predCache)
    containerP, containerQ, _, _ = getKinematicData(w, (container,), predCache)
    containerEntryInContainer = getContainmentFn(w, container, predCache).get("pouring", {}).get("into", {}).get("point")
    containerEntry, _ = w.objectPoseRelativeToWorld(containerP, containerQ, containerEntryInContainer, [0,0,0,1])
    itemOutOfFn = getContainmentFn(w, item, predCache).get("pouring", {}).get("outof", {})
    pourPointInItem = itemOutOfFn.get("point") or [0,0,0]
    pourAxisInItem = itemOutOfFn.get("axis") or [1,0,0]
    pourPoint, _ = w.objectPoseRelativeToWorld(itemP, itemQ, pourPointInItem, [0,0,0,1])
    pourPointInHand, itemQInHand = w.objectPoseRelativeToObject(handP, handQ, pourPoint, itemQ)
    pourAxis = stubbornTry(lambda : pybullet.rotateVector(itemQ, pourAxisInItem))
    pourAxisInHand = stubbornTry(lambda : pybullet.rotateVector(itemQInHand, pourAxisInItem))
    entryHeightPourPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, pourPointInHand))[2]
    if item in getHeld(w, name, hand, predCache):
        fwdInHand = [1,0,0]
        facingYaw = getFacingYaw(w, container, containerP, predCache)
        fwd = [math.cos(facingYaw), math.sin(facingYaw),0]
        #axis = ([mixAxisInHand[1]*down[2]-mixAxisInHand[2]*down[1], mixAxisInHand[2]*down[0]-mixAxisInHand[0]*down[2], mixAxisInHand[0]*down[1]-mixAxisInHand[1]*down[0]])
        #angle = math.acos((mixAxisInHand[0]*down[0] + mixAxisInHand[1]*down[1] + mixAxisInHand[2]*down[2]))
        tippedQ = quatFromVecPairs((fwdInHand, fwd), (pourAxisInHand, down))
    else:
        tippedQ = [0,0,0,1]
    #For tipping:
    #  we want XYZ/T: pourpointXY at container entryXY, handZ at entryHeight, hand orientation at parked orientation
    #  we can achieve XYZ/T by a tipping process that maintains XYZ: pourpointXY at container entryXY, handZ at entryHeight
    #  we can achieve XY by a bringing process that maintains Z: handZ at entryHeight
    # we can achieve Z by a lifting process
    conT = ['equalq', 'itemQ', 'tippedQ']
    tolT = [0.99, 0.95]
    conXY = ['equalxy', 'pourPoint', 'containerEntry']
    tolXY = [0.0001, 0.0025] # [0.01, 0.05]
    conZ = ['equalz', 'handP', 'entryHeight']
    tolZ = [0.01, 0.05]
    constraintConjunctions = [[conT, conXY, conZ], [conXY, conZ], [conZ]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpXYZT = [[containerEntry[0], containerEntry[1], entryHeightPourPoint], tippedQ, None, pourPointInHand, None, None, None]
    wpXYZ = [[containerEntry[0], containerEntry[1], entryHeightPourPoint], handParkedQ, None, pourPointInHand, None, None, None]
    wpZ = [[handP[0], handP[1], entryHeight], handParkedQ, None, None, None, None, None]
    waypoints = [wpXYZT, wpXYZ, wpZ]
    tolerances = [[tolT, tolXY, tolZ], [tolXY, tolZ], [tolZ]]
    entities = {'handP': handP, 'itemQ': itemQ, 'tippedQ': tippedQ, 'entryHeight': [0,0,entryHeight], 'pourPoint': pourPoint, 'containerEntry': containerEntry}
    return [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": container}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getTurningControlConditions(w, name, description, node, predCache): # item, hand, link, setting
    hand = description["hand"]
    item = description["item"]
    link = description["link"]
    setting = description["setting"]
    #### TODO CF
    fnTurning = w._kinematicTrees[item].get("fn", {}).get("turning", {})
    axisInLink = fnTurning.get("axis", {}).get(link, [0,0,1])
    pointInLink = fnTurning.get("point", {}).get(link, [0,0,0.1])
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    linkP, linkQ, _, _ = getKinematicData(w, (item, link), predCache)
    handTipInHand = w._kinematicTrees[name].get("fn", {}).get("turning", {}).get("point", {}).get(handLink, [0.2, 0, 0])
    handTipInWorld, _ = w.objectPoseRelativeToWorld(handP, handQ, handTipInHand, [0,0,0,1])
    controlPointInWorld, _ = w.objectPoseRelativeToWorld(linkP, linkQ, pointInLink, [0,0,0,1])
    axisInWorld = stubbornTry(lambda : pybullet.rotateVector(linkQ, axisInLink))
    if 0.8 > abs(axisInWorld[2]):
        handTwist = [-1,0,0]
        handOrthogonal = [0,0,1]
        orthogonalInWorld = [0,0,1]
    else:
        handTwist = [0,0,1]
        facingYaw = getFacingYaw(w, item, itemP, predCache)
        s = math.sin(facingYaw)
        c = math.cos(facingYaw)
        handOrthogonal = [1,0,0]
        orthogonalInWorld = [c,s,0]
    alignedQ = quatFromVecPairs((handTwist, axisInWorld), (handOrthogonal, orthogonalInWorld))
    #For twist prep:
    #  we want XYZ: hand tip at control point
    #  we can achieve XYZ by a bringing process that maintains Q: hand orientation aligned to control
    #  we can achieve Q by a rotating process
    conXYZ = ['equalp', 'handTip', 'controlPoint']
    tolXYZ = [0.01, 0.05]
    conQ = ['equalq', 'handQ', 'alignedQ']
    tolQ = [0.99, 0.95]
    constraintConjunctions = [[conXYZ], [conQ]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpXYZ = [controlPointInWorld, alignedQ, None, handTipInHand, None, None, None]
    wpQ = [handP, alignedQ, None, None, None, None, None]
    waypoints = [wpXYZ, wpQ]
    tolerances = [[tolXYZ], [tolQ]]
    entities = {'handTip': handTipInWorld, 'handQ': handQ, 'alignedQ': alignedQ, 'controlPoint': controlPointInWorld}
    return [{"type": "G", "description": {"goal": "near", "relatum": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}},
            {"type": "G", "description": {"goal": "turnedHand", "hand": hand, "item": item, "link": link, "setting": setting}, "previousStatus": None, "numerics": {}}]
    
def _getTurningHandConditions(w, name, description, node, predCache): # item, hand, link, setting
    hand = description["hand"]
    item = description["item"]
    link = description["link"]
    setting = description["setting"]
    isSet, isSetting = checkControlSetting(w, name, hand, item, link, setting, predCache)
    if (not isSet) and isSetting:
        angle = w.getJointData((item, link))[0]
        angleDiff = setting - angle
        # TODO: assumes here the axis given is such that counterclockwise rotation increases joint angle. This should be checked.
        fnTurning = w._kinematicTrees[item].get("fn", {}).get("turning", {})
        axisInLink = fnTurning.get("axis", {}).get(link, [0,0,1])
        pointInLink = fnTurning.get("point", {}).get(link, [0,0,0.1])
        handLink = getHandLink(w, name, hand, predCache)
        handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
        linkP, linkQ, _, _ = getKinematicData(w, (item, link), predCache)
        pointInWorld, _ = w.objectPoseRelativeToWorld(linkP, linkQ, pointInLink, [0,0,0,1])
        linkQInv = [linkQ[0], linkQ[1], linkQ[2], -linkQ[3]]
        _, handQInLink = w.objectPoseRelativeToWorld([0,0,0], linkQInv, [0,0,0], handQ)
        halfIncAngle = 0.05
        if 0 < angleDiff:
            halfIncAngle = -halfIncAngle
        s = math.sin(halfIncAngle)
        c = math.cos(halfIncAngle)
        _, handQInLinkAdj = w.objectPoseRelativeToWorld([0,0,0], [axisInLink[0]*s, axisInLink[1]*s, axisInLink[2]*s, c], [0,0,0], handQInLink)
        _, twistedQ = w.objectPoseRelativeToWorld([0,0,0], linkQ, [0,0,0], handQInLinkAdj)
        handTipInHand = w._kinematicTrees[name].get("fn", {}).get("turning", {}).get("point", {}).get(handLink, [0.2, 0, 0])
        #For twisting:
        #  we want q: hand orientation at twisted orientation (and very precisely so!)
        #  we can achieve q by a twisting process
        conQ = ['equalq', 'handQ', 'twistedQ']
        tolQ = [0.9999, 0.9999]
        constraintConjunctions = [[conQ]]
        #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
        wpQ = [pointInWorld, twistedQ, None, handTipInHand, None, None, None]
        waypoints = [wpQ]
        tolerances = [[tolQ]]
        entities = {'handQ': handQ, 'twistedQ': twistedQ}
        return [{"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]
    else:
        return []

def _getConstraintFollowingConditions(w, name, description, node, predCache):
    constraintConjunctions = description['constraintConjunctions']
    hand = description['hand']
    if 1 >= len(constraintConjunctions):
        return []
    return [{"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions[1:], "isTop": False}, "children": [], "previousStatus": None, "numerics": {"waypoints": node['numerics']['waypoints'][1:], "tolerances": node['numerics']['tolerances'][1:], "entities": node['numerics']['entities']}}]

def _getMixingAndStoringConditions(w, name, description, node, predCache): # container, hand, mixedType, storage, tool, toolLink |
    tool = description['tool']
    hand = description['hand']
    return [{"type": "G", "description": {"goal": "mixedAndUprighted", "container": description["container"], "hand": hand, "tool": tool, "toolLink": description["toolLink"], "mixedType": description["mixedType"]}, "children": [], "previousStatus": None, "numerics": None},
            {"type": "G", "description": {"goal": "placedItem", "hand": hand, "item": tool, "container": description["storage"]}, "children": [], "previousStatus": None, "numerics": {}}]

def _getMixingAndUprightingConditions(w, name, description, node, predCache): # container, hand, mixedType, tool, toolLink |
    mixedType = description['mixedType']
    toolLink = description['toolLink']
    tool = description['tool']
    hand = description['hand']
    container = description['container']
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    toolP, toolQ, _, _ = getKinematicData(w, (tool,), predCache)
    containerP, containerQ, _, _ = getKinematicData(w, (container,), predCache)
    toolPositionInHand, toolOrientationInHand = w.objectPoseRelativeToObject(handP, handQ, toolP, toolQ)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': container}, {}, predCache)
    mixPointInTool = w._kinematicTrees[tool].get("fn", {}).get("mixing", {}).get("mixPoint", {}).get(toolLink) or [0,0,0]
    mixPointInHand = w.objectPoseRelativeToWorld(toolPositionInHand, toolOrientationInHand, mixPointInTool, [0,0,0,1])[0]
    mixPoint = w.objectPoseRelativeToWorld(handP, handQ, mixPointInHand, [0,0,0,1])[0]
    containerEntryInContainer = w._kinematicTrees[container].get("fn", {}).get("containment", {}).get("pouring", {}).get("into", {}).get("point") or [0,0,0]
    containerEntry, _ = w.objectPoseRelativeToWorld(containerP, containerQ, containerEntryInContainer, [0,0,0,1])
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
    entities = {'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight], 'mixPoint': mixPoint, 'containerEntry': containerEntry}
    return [{"type": "G", "description": {"goal": "mixed", "container": container, "hand": hand, "tool": tool, "toolLink": toolLink, "mixedType": mixedType}, "children": [], "previousStatus": None, "numerics": None},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getMixingConditions(w, name, description, node, predCache): # container, hand, mixedType, tool, toolLink |
    toolLink = description['toolLink']
    tool = description['tool']
    hand = description['hand']
    container = description['container']
    down = w.getDown()
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    toolP, toolQ, _, _ = getKinematicData(w, (tool,), predCache)
    containerP, containerQ, _, _ = getKinematicData(w, (container,), predCache)
    toolPositionInHand, toolOrientationInHand = w.objectPoseRelativeToObject(handP, handQ, toolP, toolQ)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': container}, {}, predCache)
    mixAxisInTool = w._kinematicTrees[tool].get("fn", {}).get("mixing", {}).get("axis", {}).get(toolLink) or [1,0,0]
    mixAxis = stubbornTry(lambda : pybullet.rotateVector(toolQ, mixAxisInTool))
    mixAxisInHand = stubbornTry(lambda : pybullet.rotateVector(toolOrientationInHand, mixAxisInTool))
    mixPointInTool = w._kinematicTrees[tool].get("fn", {}).get("mixing", {}).get("mixPoint", {}).get(toolLink) or [0,0,0]
    mixPointInHand = w.objectPoseRelativeToWorld(toolPositionInHand, toolOrientationInHand, mixPointInTool, [0,0,0,1])[0]
    mixPoint = w.objectPoseRelativeToWorld(handP, handQ, mixPointInHand, [0,0,0,1])[0]
    containerEntryInContainer = w._kinematicTrees[container].get("fn", {}).get("containment", {}).get("pouring", {}).get("into", {}).get("point") or [0,0,0]
    containerEntry, _ = w.objectPoseRelativeToWorld(containerP, containerQ, containerEntryInContainer, [0,0,0,1])
    entryHeightMixPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, mixPointInHand))[2]
    if tool in getHeld(w, name, hand, predCache):
        fwdInHand = [1,0,0]
        facingYaw = getFacingYaw(w, container, containerP, predCache)
        fwd = [math.cos(facingYaw), math.sin(facingYaw),0]
        #axis = ([mixAxisInHand[1]*down[2]-mixAxisInHand[2]*down[1], mixAxisInHand[2]*down[0]-mixAxisInHand[0]*down[2], mixAxisInHand[0]*down[1]-mixAxisInHand[1]*down[0]])
        #angle = math.acos((mixAxisInHand[0]*down[0] + mixAxisInHand[1]*down[1] + mixAxisInHand[2]*down[2]))
        tippedOrientation = quatFromVecPairs((fwdInHand, fwd), (mixAxisInHand, down))
    else:
        tippedOrientation = [0,0,0,1]
    #For mixing:
    #  we want pc/ad: mixpoint at containerEntry and mixaxis down
    #  we can achieve pc/ad by a lowering process that maintains ad/xy: mixaxis down and mixpointXY at containerEntryXY
    #  we can achieve ad/xy by a tipping process that maintains xy/ez: mixpointXY at containerEntryXY and handZ at entryHeight
    #  we can achieve xy/ez by a bringing process that maintains ez/po: handZ at entryHeight and handOrientation at parkedOrientation
    #  we can achieve ez/po by a lifting process that maintains po: handOrientation at parkedOrientation
    conpc = ['equalp', 'mixPoint', 'containerEntry']
    tolpc = [0.0001, 0.0004]
    conad = ['aligned', 'mixAxis', 'down']
    tolad = [0.95, 0.9]
    conxy = ['equalxy', 'mixPoint', 'containerEntry']
    tolxy = [0.0001, 0.0025]
    conez = ['equalz', 'handP', 'entryHeight']
    tolez = [0.01, 0.05]
    conpo = ['equalq', 'handQ', 'handParkedQ']
    tolpo = [0.95, 0.9]
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
    entities = {'down': down, 'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight], 'mixPoint': mixPoint, 'containerEntry': containerEntry, 'mixAxis': mixAxis}
    return [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": tool}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": container}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]


def _getMashingAndStoringConditions(w, name, description, node, predCache): # container, hand, storage, tool, toolLink |
    tool = description['tool']
    hand = description['hand']
    return [{"type": "G", "description": {"goal": "mashedAndUprighted", "disposition": description["disposition"], "container": description["container"], "hand": hand, "tool": tool, "toolLink": description["toolLink"]}, "children": [], "previousStatus": None, "numerics": None},
            {"type": "G", "description": {"goal": "placedItem", "hand": hand, "item": tool, "container": description["storage"]}, "children": [], "previousStatus": None, "numerics": {}}]

def _getMashingAndUprightingConditions(w, name, description, node, predCache): # container, hand, tool, toolLink |
    disposition = description["disposition"]
    if "mashable" == disposition:
        fnGroup = "mashing"
        fnPoint = "mashPoint"
    elif "grindable" == disposition:
        fnGroup = "grinding"
        fnPoint = "grindPoint"
    elif "flattenable" == disposition:
        fnGroup = "flattening"
        fnPoint = "flattenPoint"
    toolLink = description['toolLink']
    tool = description['tool']
    hand = description['hand']
    container = description['container']
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    toolP, toolQ, _, _ = getKinematicData(w, (tool,), predCache)
    containerP, containerQ, _, _ = getKinematicData(w, (container,), predCache)
    toolPositionInHand, toolOrientationInHand = w.objectPoseRelativeToObject(handP, handQ, toolP, toolQ)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': container}, {}, predCache)
    mashPointInTool = w._kinematicTrees[tool].get("fn", {}).get(fnGroup, {}).get(fnPoint, {}).get(toolLink) or [0,0,0]
    mashPointInHand = w.objectPoseRelativeToWorld(toolPositionInHand, toolOrientationInHand, mashPointInTool, [0,0,0,1])[0]
    mashPoint = w.objectPoseRelativeToWorld(handP, handQ, mashPointInHand, [0,0,0,1])[0]
    containerEntryInContainer = w._kinematicTrees[container].get("fn", {}).get("containment", {}).get("pouring", {}).get("into", {}).get("point") or [0,0,0]
    containerEntry, _ = w.objectPoseRelativeToWorld(containerP, containerQ, containerEntryInContainer, [0,0,0,1])
    entryHeightMashPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, mashPointInHand))[2]
    #For uprighting:
    #  we want P/U: hand parked
    #  We can achieve P/U by a lowering process that maintains xy/U: handXY at parked XY, hand parked orientation
    #  We can achieve xy/U by a movement process that maintains U/Z: hand parked orientation, handZ at entryHeight
    #  We can achieve U/Z by a dummy process that maintains U/Z/XY: mashpointXY at containerEntryXY, handZ at entryHeight, hand parked orientation
    #  we can achieve U/Z/XY by an uprighting process that maintains Z/XY: mashpointXY at containerEntryXY, handZ at entryHeight
    #  we can achieve Z/XY by a lifting process that maintains XY/T: mashpointXY at containerEntryXY, masher tipped
    conP = ['equalp', 'handP', 'handParkedP']
    tolP = [0.01, 0.02]
    conU = ['equalq', 'handQ', 'handParkedQ']
    tolU = [0.99, 0.95]
    conxy = ['equalxy', 'handP', 'handParkedP']
    tolxy = [0.01, 0.05]
    conZ = ['equalz', 'handP', 'entryHeight']
    tolZ = [0.01, 0.05]
    conXY = ['equalxy', 'mashPoint', 'containerEntry']
    tolXY = [0.01, 0.05]
    constraintConjunctions = [[conP, conU], [conxy, conU], [conU, conZ], [conU, conZ, conXY], [conZ, conXY]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpPU = [handParkedP, handParkedQ, None, None, None, None, None]
    wpxyU = [[handParkedP[0], handParkedP[1], entryHeight], handParkedQ, None, None, None, None, None]
    wpUZ = [[containerEntry[0], containerEntry[1], entryHeightMashPoint], handParkedQ, None, mashPointInHand, None, None, None]
    wpUZXY = wpUZ
    wpZXY = [[containerEntry[0], containerEntry[1], entryHeightMashPoint], toolQ, None, mashPointInHand, toolOrientationInHand, None, None]
    waypoints = [wpPU, wpxyU, wpUZ, wpUZXY, wpZXY]
    tolerances = [[tolP, tolU], [tolxy, tolU], [tolU, tolZ], [tolU, tolZ, tolXY], [tolZ, tolXY]]
    entities = {'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight], 'mashPoint': mashPoint, 'containerEntry': containerEntry}
    return [{"type": "G", "description": {"goal": "mashed", "disposition": description["disposition"], "container": container, "hand": hand, "tool": tool, "toolLink": toolLink}, "children": [], "previousStatus": None, "numerics": None},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getMashingConditions(w, name, description, node, predCache): # container, hand, tool, toolLink |
    disposition = description["disposition"]
    if "mashable" == disposition:
        fnGroup = "mashing"
        fnPoint = "mashPoint"
    elif "grindable" == disposition:
        fnGroup = "grinding"
        fnPoint = "grindPoint"
    elif "flattenable" == disposition:
        fnGroup = "flattening"
        fnPoint = "flattenPoint"
    toolLink = description['toolLink']
    tool = description['tool']
    hand = description['hand']
    container = description['container']
    down = w.getDown()
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    toolP, toolQ, _, _ = getKinematicData(w, (tool,), predCache)
    containerP, containerQ, _, _ = getKinematicData(w, (container,), predCache)
    toolPositionInHand, toolOrientationInHand = w.objectPoseRelativeToObject(handP, handQ, toolP, toolQ)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': container}, {}, predCache)
    mashAxisInTool = w._kinematicTrees[tool].get("fn", {}).get(fnGroup, {}).get("axis", {}).get(toolLink) or [1,0,0]
    mashAxis = stubbornTry(lambda : pybullet.rotateVector(toolQ, mashAxisInTool))
    mashAxisInHand = stubbornTry(lambda : pybullet.rotateVector(toolOrientationInHand, mashAxisInTool))
    mashPointInTool = w._kinematicTrees[tool].get("fn", {}).get(fnGroup, {}).get(fnPoint, {}).get(toolLink) or [0,0,0]
    mashPointInHand = w.objectPoseRelativeToWorld(toolPositionInHand, toolOrientationInHand, mashPointInTool, [0,0,0,1])[0]
    mashPoint = w.objectPoseRelativeToWorld(handP, handQ, mashPointInHand, [0,0,0,1])[0]
    containerEntryInContainer = w._kinematicTrees[container].get("fn", {}).get("containment", {}).get("pouring", {}).get("into", {}).get("point") or [0,0,0]
    containerEntry, _ = w.objectPoseRelativeToWorld(containerP, containerQ, containerEntryInContainer, [0,0,0,1])
    entryHeightMashPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, mashPointInHand))[2]
    if tool in getHeld(w, name, hand, predCache):
        fwdInHand = [1,0,0]
        facingYaw = getFacingYaw(w, container, containerP, predCache)
        fwd = [math.cos(facingYaw), math.sin(facingYaw),0]
        #axis = ([mashAxisInHand[1]*down[2]-mashAxisInHand[2]*down[1], mashAxisInHand[2]*down[0]-mashAxisInHand[0]*down[2], mashAxisInHand[0]*down[1]-mashAxisInHand[1]*down[0]])
        #angle = math.acos((mashAxisInHand[0]*down[0] + mashAxisInHand[1]*down[1] + mashAxisInHand[2]*down[2]))
        tippedOrientation = quatFromVecPairs((fwdInHand, fwd), (mashAxisInHand, down))
    else:
        tippedOrientation = [0,0,0,1]
    #For mashing:
    #  we want pc/ad: mashpoint at containerEntry and mashaxis down
    #  we can achieve pc/ad by a lowering process that maintains ad/xy: mashaxis down and mashpointXY at containerEntryXY
    #  we can achieve ad/xy by a tipping process that maintains xy/ez: mashpointXY at containerEntryXY and handZ at entryHeight
    #  we can achieve xy/ez by a bringing process that maintains ez/po: handZ at entryHeight and handOrientation at parkedOrientation
    #  we can achieve ez/po by a lifting process that maintains po: handOrientation at parkedOrientation
    conpc = ['equalp', 'mashPoint', 'containerEntry']
    tolpc = [0.0001, 0.0004]
    conad = ['aligned', 'mashAxis', 'down']
    tolad = [0.95, 0.9]
    conxy = ['equalxy', 'mashPoint', 'containerEntry']
    tolxy = [0.0001, 0.0004]
    conez = ['equalz', 'handP', 'entryHeight']
    tolez = [0.01, 0.05]
    conpo = ['equalq', 'handQ', 'handParkedQ']
    tolpo = [0.95, 0.9]
    constraintConjunctions = [[conpc, conad], [conad, conxy], [conxy, conez], [conez, conpo]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    #wppcad = [containerEntry, tippedOrientation, None, mashPointInHand, toolOrientationInHand, None, None]
    #wpadxy = [[containerEntry[0], containerEntry[1], entryHeightMashPoint], tippedOrientation, None, mashPointInHand, toolOrientationInHand, None, None]
    wppcad = [containerEntry, tippedOrientation, None, mashPointInHand, None, None, None]
    wpadxy = [[containerEntry[0], containerEntry[1], entryHeightMashPoint], tippedOrientation, None, mashPointInHand, None, None, None]
    wpxyez = [[containerEntry[0], containerEntry[1], entryHeightMashPoint], handParkedQ, None, mashPointInHand, None, None, None]
    wpezpo = [[handParkedP[0], handParkedP[1], entryHeight], handParkedQ, None, None, None, None, None]
    waypoints = [wppcad, wpadxy, wpxyez, wpezpo]
    tolerances = [[tolpc, tolad], [tolad, tolxy], [tolxy, tolez], [tolez, tolpo]]
    entities = {'down': down, 'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight], 'mashPoint': mashPoint, 'containerEntry': containerEntry, 'mashAxis': mashAxis}
    return [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": tool}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": container}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getLiningAndParkingConditions(w, name, description, node, predCache):
    lining = description['lining']
    hand = description['hand']
    #source = description.get('sourceContainer', None)
    #sourcePart = description.get('sourceComponent', None)
    return [{"type": "G", "description": {"goal": "lined", "hand": hand, "item": description["item"], "lining": lining}, "children": [], "previousStatus": None, "numerics": None},
            {"type": "G", "description": {"goal": "ungrasped", "hand": hand, "item": lining}, "children": [], "previousStatus": None, "numerics": None},
            {"type": "G", "description": {"goal": "parkedArm", "hand": hand}, "children": [], "previousStatus": None, "numerics": {}}]

def _getLiningConditions(w, name, description, node, predCache):
    lining = description['lining']
    hand = description['hand']
    item = description['item']
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    linP, linQ, _, _ = getKinematicData(w, (lining,), predCache)
    itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
    linPositionInHand, linOrientationInHand = w.objectPoseRelativeToObject(handP, handQ, linP, linQ)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': item}, {}, predCache)
    liningFn = w._kinematicTrees[lining].get("fn", {}).get("lining", {})
    itemLiningFn = w._kinematicTrees[item].get("fn", {}).get("lining", {})
    linAxisInLin = liningFn.get("axis") or [1,0,0]
    linPointInLin = liningFn.get("point") or [0,0,0]
    linAxis = stubbornTry(lambda : pybullet.rotateVector(linQ, linAxisInLin))
    itemAxisInItem = itemLiningFn.get("axis") or [1,0,0]
    itemLinPointInItem = itemLiningFn.get("point") or [0,0,0]
    itemAxis = stubbornTry(lambda : pybullet.rotateVector(itemQ, itemAxisInItem))
    linPointInHand = w.objectPoseRelativeToWorld(linPositionInHand, linOrientationInHand, linPointInLin, [0,0,0,1])[0]
    linPoint = w.objectPoseRelativeToWorld(linP, linQ, linPointInLin, [0,0,0,1])[0]
    itemLinPoint, _ = w.objectPoseRelativeToWorld(itemP, itemQ, itemLinPointInItem, [0,0,0,1])
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
    tolez = [0.01, 0.05]
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
    entities = {'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight], 'linPoint': linPoint, 'itemLinPoint': itemLinPoint, 'linAxis': linAxis, 'itemAxis': itemAxis}
    return [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": lining}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getCoveringAndParkingConditions(w, name, description, node, predCache): # hand, item, cover |
    cover = description['cover']
    hand = description['hand']
    #source = description.get('sourceContainer', None)
    #sourcePart = description.get('sourceComponent', None)
    return [{"type": "G", "description": {"goal": "covered", "hand": hand, "item": description["item"], "cover": cover}, "children": [], "previousStatus": None, "numerics": None},
            {"type": "G", "description": {"goal": "ungrasped", "hand": hand, "item": cover}, "children": [], "previousStatus": None, "numerics": None},
            {"type": "G", "description": {"goal": "parkedArm", "hand": hand}, "children": [], "previousStatus": None, "numerics": {}}]

def _getCoveringConditions(w, name, description, node, predCache): # hand, item, cover |
    cover = description['cover']
    hand = description['hand']
    item = description['item']
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    covP, covQ, _, _ = getKinematicData(w, (cover,), predCache)
    itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
    covPositionInHand, covOrientationInHand = w.objectPoseRelativeToObject(handP, handQ, covP, covQ)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': item}, {}, predCache)
    coveringFn = w._kinematicTrees[cover].get("fn", {}).get("covering", {})
    itemCoveringFn = w._kinematicTrees[item].get("fn", {}).get("covering", {})
    covAxisInCov = coveringFn.get("axis") or [1,0,0]
    covPointInCov = coveringFn.get("point") or [0,0,0]
    covAxis = stubbornTry(lambda : pybullet.rotateVector(covQ, covAxisInCov))
    itemAxisInItem = itemCoveringFn.get("axis") or [1,0,0]
    itemCovPointInItem = itemCoveringFn.get("point") or [0,0,0]
    itemAxis = stubbornTry(lambda : pybullet.rotateVector(itemQ, itemAxisInItem))
    covPointInHand = w.objectPoseRelativeToWorld(covPositionInHand, covOrientationInHand, covPointInCov, [0,0,0,1])[0]
    covPoint = w.objectPoseRelativeToWorld(covP, covQ, covPointInCov, [0,0,0,1])[0]
    itemCovPoint, _ = w.objectPoseRelativeToWorld(itemP, itemQ, itemCovPointInItem, [0,0,0,1])
    entryHeightCovPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, covPointInHand))[2]
    itemYaw = math.atan2(itemAxis[1], itemAxis[0])
    coveringQ = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,itemYaw)))
    #For lining:
    #  we want lp/la: linpoint at itemlinpoint and linaxis aligned with itemaxis
    #  we can achieve lp/la by a lowering process that maintains la/xy: linaxis aligned with itemaxis and linpointXY at itemlinpointXY
    #  we can achieve la/xy by an aligning process that maintains xy/ez: linpointXY at itemlinpointXY and handZ at entryHeight
    #  we can achieve xy/ez by a bringing process that maintains ez/po: handZ at entryHeight and handOrientation at parkedOrientation
    #  we can achieve ez/po by a lifting process that maintains po: handOrientation at parkedOrientation
    conlp = ['equalp', 'covPoint', 'itemCovPoint']
    tollp = [0.0001, 0.0004]
    conla = ['aligned', 'covAxis', 'itemAxis']
    tolla = [0.99, 0.9]
    conxy = ['equalxy', 'covPoint', 'itemCovPoint']
    tolxy = [0.0001, 0.0004]
    conez = ['equalz', 'handP', 'entryHeight']
    tolez = [0.01, 0.05]
    conpo = ['equalq', 'handQ', 'handParkedQ']
    tolpo = [0.99, 0.9]
    constraintConjunctions = [[conlp, conla], [conla, conxy], [conxy, conez], [conez, conpo]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wplpla = [itemCovPoint, coveringQ, None, covPointInHand, covOrientationInHand, None, None]
    wplaxy = [[itemCovPoint[0], itemCovPoint[1], entryHeightCovPoint], coveringQ, None, covPointInHand, covOrientationInHand, None, None]
    wpxyez = [[itemCovPoint[0], itemCovPoint[1], entryHeightCovPoint], handParkedQ, None, covPointInHand, None, None, None]
    wpezpo = [[handParkedP[0], handParkedP[1], entryHeight], handParkedQ, None, None, None, None, None]
    waypoints = [wplpla, wplaxy, wpxyez, wpezpo]
    tolerances = [[tollp, tolla], [tolla, tolxy], [tolxy, tolez], [tolez, tolpo]]
    entities = {'handP': handP, 'handQ': handQ, 'handParkedP': handParkedP, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight], 'covPoint': covPoint, 'itemCovPoint': itemCovPoint, 'covAxis': covAxis, 'itemAxis': itemAxis}
    return [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": cover}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getShapingConditions(w, name, description, node, predCache):
    hand = description['hand']
    item = description['item']
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
    itemAABB = w.getAABB((item,))
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    if description.get('itemIsShaped', False):
        containerEntryInContainer = [0, 0, (itemAABB[1][2]-itemP[2])]
        containerEntry = [itemP[0], itemP[1], itemAABB[1][2]]
    else:
        containerEntryInContainer = w._kinematicTrees[item].get("fn", {}).get("containment", {}).get("pouring", {}).get("into", {}).get("point") or [0,0,0]
        containerEntry, _ = w.objectPoseRelativeToWorld(itemP, itemQ, containerEntryInContainer, [0,0,0,1])
    containerEntry = [containerEntry[0], containerEntry[1], itemAABB[1][2] + 0.05]
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': item}, {}, predCache)
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
    tolez = [0.01, 0.05]
    constraintConjunctions = [[conhp, conpo], [conpo, conxy], [conez, conpo]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wphppo = [containerEntry, handParkedQ, None, None, None, None, None]
    wppoxy = [[containerEntry[0], containerEntry[1], entryHeight], handParkedQ, None, None, None, None, None]
    wpezpo = [[handParkedP[0], handParkedP[1], entryHeight], handParkedQ, None, None, None, None, None]
    waypoints = [wphppo, wppoxy, wpezpo]
    tolerances = [[tolhp, tolpo], [tolpo, tolxy], [tolez, tolpo]]
    entities = {'handP': handP, 'handQ': handQ, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight], 'containerEntry': containerEntry}
    return [{"type": "G", "description": {"goal": "near", "relatum": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]
            
def _getOpeningConditions(w, name, description, node, predCache):
    hand = getDoorManipulationHand(w, name, description.get('hand', None), predCache)
    container = description.get('container', None)
    return [{"type": "G", "description": {"goal": "near", "relatum": container}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "armTriggeredPortalHandle", "hand": hand, "component": description.get("component"), "action": "open", "container": container}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "parkedArm", "hand": hand}, "children": [], "previousStatus": None, "numerics": {}}]

def _getClosingConditions(w, name, description, node, predCache):
    hand = getDoorManipulationHand(w, name, description.get('hand', None), predCache)
    container = description.get('container', None)
    return [{"type": "G", "description": {"goal": "near", "relatum": container}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "armTriggeredPortalHandle", "container": container, "component": description.get("component"), "hand": hand, "action": "close"}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "parkedArm", "hand": hand}, "children": [], "previousStatus": None, "numerics": {}}]

def _getBakingConditions(w, name, description, node, predCache):
    item = description.get('item', None)
    hand = description.get('hand', None)
    oven = description.get('oven', None)
    destination = description.get('destination', None)
    bakedType = description.get('bakedType', None)
    processDisposition = description.get('processDisposition', 'bakable')
    timeAmount = description.get("timeAmount")
    timeUnit = description.get("timeUnit")
    source = node.get('sourceContainer')
    sourcePart = node.get('sourceComponent')
    if checkBakedContents(w, item, bakedType, processDisposition, predCache):
        return [{'type': 'G', 'description': {'goal': 'placedItem', 'item': item, 'hand': hand, 'container': destination}}]
    retq = [{'type': 'G', 'description': {'goal': 'placedItem', 'item': item, 'hand': hand, 'container': oven}}]
    if (timeAmount is not None) and (checkItemInContainer(w, name, item, oven, predCache)) and (not checkGrasped(w, name, None, item, predCache)):
        if timeUnit is None:
            timeUnit = 1.0
        timeAmount = timeAmount*timeUnit
        previousStartTime = node.get("previousStartTime")
        if (previousStartTime is None):
            previousStartTime = w._kinematicTrees[agentName].get("customStateVariables", {}).get("timing", {}).get("timer",0)
        retq.append({'type': 'G', 'description': {'goal': 'waited', 'timeEnd': timeAmount + previousStartTime}})
        node["previousStartTime"] = previousStartTime
    return retq

def _getSprinklingConditions(w, name, description, node, predCache):
    item = description.get('item', None)
    hand = description.get('hand', None)
    shaker = description.get('shaker', None)
    storage = description.get('storage', None)
    sprinkledType = description.get('sprinkledType', None)
    parked = node.get('parked', False)
    parked = checkParked(w, name, hand, parked, predCache)
    node['parked'] = parked
    allSprinkled, toSprinkle = checkSprinkled(w, name, item, sprinkledType, predCache)
    toSprinkle = sorted(toSprinkle)
    down = w.getDown()
    handLink = getHandLink(w, name, hand, predCache)
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
    shakerP, shakerQ, _, _ = getKinematicData(w, (shaker,), predCache)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': item}, {}, predCache)
    shakerFn = getContainmentFn(w, shaker, predCache).get("pouring", {}).get("outof", {})
    if allSprinkled:
        containerEntryInContainer = w._kinematicTrees[item].get("fn", {}).get("containment", {}).get("pouring", {}).get("into", {}).get("point") or [0,0,0]
        containerEntry, _ = w.objectPoseRelativeToWorld(itemP, itemQ, containerEntryInContainer, [0,0,0,1])
    else:
        containerEntry, _, _, _ = getKinematicData(w, (toSprinkle[0],), predCache)
    pourPointInShaker = shakerFn.get("point") or [0,0,0]
    pourAxisInShaker = shakerFn.get("axis") or [1,0,0]
    pourPoint, _ = w.objectPoseRelativeToWorld(shakerP, shakerQ, pourPointInShaker, [0,0,0,1])
    pourAxis = stubbornTry(lambda : pybullet.rotateVector(shakerQ, pourAxisInShaker))
    pourDown = pourAxis[0]*down[0] + pourAxis[1]*down[1] + pourAxis[2]*down[2]
    pouring = (0.8 < abs(pourDown))
    upright = node.get('upright', False)
    uTh = 0.2
    if upright:
        uTh = 0.4
    upright = (uTh > abs(pourDown))
    node['upright'] = upright
    pourPointInHand, shakerOrientationInHand = w.objectPoseRelativeToObject(handP, handQ, pourPoint, shakerQ)
    pourAxisInHand = stubbornTry(lambda : pybullet.rotateVector(shakerOrientationInHand, pourAxisInShaker))
    entryHeightPourPoint = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, pourPointInHand))[2]
    if shaker in getHeld(w, name, hand, predCache):
        fwdInHand = [1,0,0]
        facingYaw = getFacingYaw(w, item, itemP, predCache)
        fwd = [math.cos(facingYaw), math.sin(facingYaw),0]
        #axis = ([mixAxisInHand[1]*down[2]-mixAxisInHand[2]*down[1], mixAxisInHand[2]*down[0]-mixAxisInHand[0]*down[2], mixAxisInHand[0]*down[1]-mixAxisInHand[1]*down[0]])
        #angle = math.acos((mixAxisInHand[0]*down[0] + mixAxisInHand[1]*down[1] + mixAxisInHand[2]*down[2]))
        tippedQ = quatFromVecPairs((fwdInHand, fwd), (pourAxisInHand, down))
    else:
        tippedQ = [0,0,0,1]

    _, shakerParkedQ = w.objectPoseRelativeToWorld([0,0,0], handParkedQ, [0,0,0], shakerOrientationInHand)
    handHoverP = handParkedP
    handHoverQ = handParkedQ
    if pouring:
        handHoverP, handHoverQ = w.handPoseToBringObjectToPose(pourPointInHand, shakerOrientationInHand, [containerEntry[0], containerEntry[1], entryHeightPourPoint], tippedQ)
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
        tolez = [0.01, 0.05]
        conpo = ['equalq', 'handQ', 'handParkedQ']
        tolpo = [0.99, 0.9]
        constraintConjunctions = [[conad, conxy, conez], [conxy, conez], [conez]]
        #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
        wpadxyez = [[containerEntry[0], containerEntry[1], entryHeightPourPoint], tippedQ, None, pourPointInHand, shakerOrientationInHand, None, None]
        wpxyez = [[containerEntry[0], containerEntry[1], entryHeightPourPoint], handHoverQ, None, pourPointInHand, None, None, None]
        wpez = [[handHoverP[0], handHoverP[1], entryHeight], handHoverQ, None, None, None, None, None]
        waypoints = [wpadxyez, wpxyez, wpez]
        tolerances = [[tolad, tolxy, tolez], [tolxy, tolez], [tolez]]
        entities = {'down': down, 'handP': handP, 'handQ': handQ, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight], 'pourPoint': pourPoint, 'containerEntry': containerEntry, 'pourAxis': pourAxis}
        return [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": shaker}, "children": [], "previousStatus": None, "numerics": {}},
                {"type": "G", "description": {"goal": "near", "relatum": item}, "children": [], "previousStatus": None, "numerics": {}},
                {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]
    elif allSprinkled and checkGrasped(w, name, hand, shaker, predCache) and not upright:
        #For uprighting:
        #  we want po/xy/ez: handOrientation at parkedOrientation and handXY at container entryXY and handZ at entryHeight
        #  we can achieve po/xy/ez by an uprighting process
        ### TODO: a more robust uprighting: take shakerOrientationInHand into account rather than assume hand pitch 0 will do it
        #roll, _, yaw = stubbornTry(lambda : pybullet.getEulerFromQuaternion(handQ))
        handUprightQ = handParkedQ#stubbornTry(lambda : pybullet.getQuaternionFromEuler((roll, 0, yaw)))
        conpo = ['equalq', 'handQ', 'handUprightQ']
        ##conpo = ['equalq', 'handQ', 'handParkedQ']
        tolpo = [0.99, 0.9]
        conxy = ['equalxy', 'handP', 'containerEntry']
        tolxy = [0.0001, 0.0004]
        conez = ['equalz', 'handP', 'entryHeight']
        tolez = [0.01, 0.05]
        constraintConjunctions = [[conpo, conxy, conez]]
        #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
        ##wppoxyez = [[containerEntry[0], containerEntry[1], entryHeight], shakerParkedQ, None, pourPointInHand, shakerOrientationInHand, None, None]
        ##waypoints = [wppoxyez]
        ##tolerances = [[tolpo, tolxy, tolez]]
        wppoez = [[handP[0], handP[1], entryHeight], handUprightQ, None, None, None, None, None]
        waypoints = [wppoez]
        tolerances = [[tolpo, tolez]]
        entities = {'handP': handP, 'handQ': handQ, 'handParkedQ': handParkedQ, 'entryHeight': [0,0,entryHeight], 'containerEntry': containerEntry, 'handUprightQ': handUprightQ}
        return [{"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]
    elif (checkGrasped(w, name, hand, shaker, predCache)) or (not checkItemInContainer(w, name, shaker, storage, predCache)):
        return [{"type": "G", "description": {"goal": "placedItem", "hand": hand, "item": shaker, "container": storage}, "children": [], "previousStatus": None, "numerics": {}}]
    else:
        return [{"type": "G", "description": {"goal": "parkedArm", "hand": hand}, "children": [], "previousStatus": None, "numerics": {}}]
    return []

def _getCuttingItemConditions(w, name, description, node, predCache):
    item = description.get('item', None)
    hand = description.get('hand', None)
    tool = description.get('tool', None)
    storage = description.get('storage', None)
    down = w.getDown()
    handLink = getHandLink(w, name, hand, predCache)
    cuttingFn = w._kinematicTrees[tool].get("fn", {}).get("cutting", {})
    blade = cuttingFn.get("links")[0]
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
    toolP, toolQ, _, _ = getKinematicData(w, (tool, blade), predCache)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': item}, {}, predCache)
    toolPositionInHand, toolOrientationInHand = w.objectPoseRelativeToObject(handP, handQ, toolP, toolQ)
    bladeAxisInTool = cuttingFn.get("axis", {}).get(blade) or [0,0,-1]
    bladeAxis = stubbornTry(lambda : pybullet.rotateVector(toolQ, bladeAxisInTool))
    bladeAxisInHand = stubbornTry(lambda : pybullet.rotateVector(toolOrientationInHand, bladeAxisInTool))
    facingYaw = getFacingYaw(w, item, itemP, predCache)
    fwdTool = cuttingFn.get("length", {}).get(blade) or [1,0,0]
    fwdInHand = stubbornTry(lambda : pybullet.rotateVector(toolOrientationInHand, fwdTool))
    fwd = [math.cos(facingYaw), math.sin(facingYaw),0]
    tippedOrientation = quatFromVecPairs((fwdInHand, fwd), (bladeAxisInHand, down))
    entryHeightBlade = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, toolPositionInHand))[2]
    overItem = w.getAABB((item,))[1][2] + (toolP[2] - w.getAABB((tool, blade))[0][2])
    #For cutting:
    #  we want xy/ad/bz: bladeXY at itemXY and blade axis down and bladeZ over item
    #  we can achieve xy/ad/bz by a lowering process that maintains xy/ad: bladeXY at itemXY and blade axis down
    #  we can achieve xy/ad by a tipping process that maintains xy/ez: bladeXY at itemXY, handZ at entryHeight
    #  we can achieve xy/ez by a bringing process that maintains ez: handZ at entryHeight
    #  we can achieve ez by a lifting process
    conad = ['aligned', 'bladeAxis', 'down']
    tolad = [0.99, 0.95]
    conbz = ['equalz', 'bladePoint', 'overItem']
    tolbz = [0.01, 0.05]
    conxy = ['equalxy', 'bladePoint', 'itemP']
    tolxy = [0.0001, 0.0004]
    conez = ['equalz', 'handP', 'entryHeight']
    tolez = [0.01, 0.05]
    constraintConjunctions = [[conad, conxy, conbz], [conad, conxy], [conxy, conez], [conez]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpadxybz = [[itemP[0], itemP[1], overItem], tippedOrientation, None, toolPositionInHand, None, None, None]
    wpadxy = [[itemP[0], itemP[1], entryHeightBlade], tippedOrientation, None, toolPositionInHand, None, None, None]
    wpxyez = [[itemP[0], itemP[1], entryHeightBlade], handQ, None, toolPositionInHand, None, None, None]
    wpez = [[handP[0], handP[1], entryHeight], handQ, None, None, None, None, None]
    waypoints = [wpadxybz, wpadxy, wpxyez, wpez]
    tolerances = [[tolad, tolxy, tolbz], [tolad, tolxy], [tolxy, tolez], [tolez]]
    entities = {'down': down, 'handP': handP, 'handQ': handQ, 'entryHeight': [0,0,entryHeight], 'bladePoint': toolP, 'itemP': itemP, 'bladeAxis': bladeAxis, 'overItem': [0,0,overItem]}
    return [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": tool}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getPeelingItemConditions(w, name, description, node, predCache): # container, containerForPeels, hand, item, otherHand, storage, tool
    item = description.get('item', None)
    hand = description.get('hand', None)
    otherHand = description.get('otherHand', None)
    tool = description.get('tool', None)
    disposition = description.get("disposition", None)
    containerForPeels = description.get("containerForPeels", None)
    storage = description.get('storage', None)
    down = w.getDown()
    handLink = getHandLink(w, name, hand, predCache)
    disp = {"peelable": "peeling", "seedable": "seeding"}.get(disposition)
    dispFn = w._kinematicTrees[tool].get("fn", {}).get(disp, {})
    blade = dispFn.get("links")[0]
    handP, handQ, _, _ = getKinematicData(w, (name, handLink), predCache)
    itemP, itemQ, _, _ = getKinematicData(w, (item,), predCache)
    toolP, toolQ, _, _ = getKinematicData(w, (tool, blade), predCache)
    handParkedP, handParkedQ, _ = getParkedPose(w, name, hand, predCache)
    entryHeight = getEntryHeight(w, name, {'hand': hand, 'item': containerForPeels}, {}, predCache)
    toolPositionInHand, toolOrientationInHand = w.objectPoseRelativeToObject(handP, handQ, toolP, toolQ)
    bladeAxisInTool = dispFn.get("axis", {}).get(blade) or [0,0,-1]
    bladeAxis = stubbornTry(lambda : pybullet.rotateVector(toolQ, bladeAxisInTool))
    bladeAxisInHand = stubbornTry(lambda : pybullet.rotateVector(toolOrientationInHand, bladeAxisInTool))
    facingYaw = getFacingYaw(w, item, itemP, predCache)
    fwdTool = dispFn.get("length", {}).get(blade) or [1,0,0]
    fwdInHand = stubbornTry(lambda : pybullet.rotateVector(toolOrientationInHand, fwdTool))
    fwd = [math.cos(facingYaw), math.sin(facingYaw),0]
    tippedOrientation = quatFromVecPairs((fwdInHand, fwd), (bladeAxisInHand, down))
    entryHeightBlade = entryHeight + stubbornTry(lambda : pybullet.rotateVector(handQ, toolPositionInHand))[2]
    adjLen = 0.85*w._kinematicTrees[item].get("fn", {}).get(disp, {}).get("radius", 0.8)
    if "hand_left" == otherHand:
        adj = [adjLen*math.cos(facingYaw + 0.5*math.pi), adjLen*math.sin(facingYaw + 0.5*math.pi)]
    elif "hand_right" == otherHand:
        adj = [adjLen*math.cos(facingYaw - 0.5*math.pi), adjLen*math.sin(facingYaw - 0.5*math.pi)]
    itemPAdj = [itemP[0] + adj[0], itemP[1] + adj[1], itemP[2]]
    itemBottom = w.getAABB((item,))[0][2]
    #For peeling:
    #  we want xy/ad/bz: bladeXY at adjustedItemXY and blade axis down and bladeZ at item bottom
    #  we can achieve xy/ad/bz by a lowering process that maintains xy/ad: bladeXY at adjustedItemXY and blade axis down
    #  we can achieve xy/ad by a tipping process that maintains xy/ez: bladeXY at adjustedItemXY, handZ at entryHeight
    #  we can achieve xy/ez by a bringing process that maintains ez: handZ at entryHeight
    #  we can achieve ez by a lifting process
    conad = ['aligned', 'bladeAxis', 'down']
    tolad = [0.99, 0.95]
    conbz = ['equalz', 'bladePoint', 'itemBottom']
    tolbz = [0.01, 0.05]
    conxy = ['equalxy', 'bladePoint', 'itemPAdj']
    tolxy = [0.0001, 0.0004]
    conez = ['equalz', 'handP', 'entryHeight']
    tolez = [0.01, 0.05]
    constraintConjunctions = [[conad, conxy, conbz], [conad, conxy], [conxy, conez], [conez]]
    #positionA, orientation, positionB, positionInLink, orientationInLink, positionCr, linVelCr; positionB, positionCr, linVelCr must be None if the waypoint is not oscillant
    wpadxybz = [[itemPAdj[0], itemPAdj[1], itemBottom], tippedOrientation, None, toolPositionInHand, None, None, None]
    wpadxy = [[itemPAdj[0], itemPAdj[1], entryHeightBlade], tippedOrientation, None, toolPositionInHand, None, None, None]
    wpxyez = [[itemPAdj[0], itemPAdj[1], entryHeightBlade], handQ, None, toolPositionInHand, None, None, None]
    wpez = [[handP[0], handP[1], entryHeight], handQ, None, None, None, None, None]
    waypoints = [wpadxybz, wpadxy, wpxyez, wpez]
    tolerances = [[tolad, tolxy, tolbz], [tolad, tolxy], [tolxy, tolez], [tolez]]
    entities = {'down': down, 'handP': handP, 'handQ': handQ, 'entryHeight': [0,0,entryHeight], 'bladePoint': toolP, 'itemPAdj': itemPAdj, 'bladeAxis': bladeAxis, 'itemBottom': [0,0,itemBottom]}
    return [{"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": tool}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "pickedItem", "hand": otherHand, "item": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "near", "relatum": item}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "itemAboveContainer", "item": item, "hand": otherHand, "container": containerForPeels, "sideHold": True}, "children": [], "previousStatus": None, "numerics": {}},
            {"type": "G", "description": {"goal": "constraintFollowed", "hand": hand, "constraintConjunctions": constraintConjunctions, "isTop": True}, "children": [], "previousStatus": None, "numerics": {"waypoints": waypoints, "tolerances": tolerances, "entities": entities}}]

def _getBringingNearConditions(w, name, description, node, predCache):
    trajector = description.get('trajector', None)
    hand = description.get('hand', None)
    relatum = description.get('relatum', None)
    source = description.get('sourceContainer', None)
    sourcePart = description.get('sourceComponent', None)
    picked = {'type': 'G', 'description': {'goal': 'pickedItem', 'item': trajector, 'hand': hand}}
    freeHand = getFreeHand(w, name, hand, predCache)
    closed = {'type': 'G', 'description': {'goal': 'closed', 'container': source, 'component': sourcePart, 'hand': freeHand}}
    near = {'type': 'G', 'description': {'goal': 'near', 'item': relatum}}
    return [picked, closed, near]

goalCheckers = {
    'done': _alwaysTrue,
    'noped': _alwaysFalse,
    'waited': _checkWaited, # timeEnd
    'notifiedCompletion': _alwaysFalse,
    'stoppedHands': _checkStoppedHands,
    'near': _checkNear, # relatum | position
    'parkedArm': _checkParkedArm, # hand |
    'armNearItemHandle': _checkArmNearItemHandle, # hand, item | 
    'grasped': _checkGrasped, # hand, item | 
    'ungrasped': _checkUngrasped, # hand, item |
    'pickedItem': _checkPickedItem, # hand, item | 
    'placedItem': _checkPlacedItem, # container, hand, item[, allowedComponents] | 
    'loweredItemAt': _checkLoweredItemAt, # container, hand, item[, allowedComponents] | 
    "itemAboveContainer": _checkItemAboveContainer, # item, hand, container[, sideHold]
    "turnedControlAndParked": _checkTurnedControlAndParked, # item, hand, link, setting
    "turnedHand": _checkTurnedHand, # item, hand, link, setting
    'transferredAndStored': _checkTransferredAndStored, # amount, container, hand, item, pouredType, storage |
    'transferredAndUprighted': _checkTransferredAndUprighted, # amount, container, hand, item, pouredType |
    'transferredContents': _checkTransferredContents, # amount, container, hand, item, pouredType |
    'constraintFollowed': _checkConstraintFollowed, # constraintConjunctions, hand, isTop | waypoints, tolerances, entities
    'mixedAndStored': _checkMixedAndStored, # container, hand, mixedType, storage, tool, toolLink |
    'mixedAndUprighted': _checkMixedAndUprighted, # container, hand, mixedType, storage, tool, toolLink |
    'mixed': _checkMixed, # container, hand, mixedType, storage, tool, toolLink |
    'mashedAndStored': _checkMashedAndStored, # container, hand, storage, tool, toolLink |
    'mashedAndUprighted': _checkMashedAndUprighted, # container, hand, storage, tool, toolLink |
    'mashed': _checkMashed, # container, hand, storage, tool, toolLink |
    'linedAndParked': _checkLinedAndParked, # hand, item, lining |
    'lined': _checkLined, # hand, item, lining |
    'coveredAndParked': _checkCoveredAndParked, # cover, hand, item |
    'covered': _checkCovered, # cover, hand, item |
    'shapedAndParked': _checkShapedAndParked, # destination, hand, ingredientTypes, item, shapedType |
    'sprinkled': _checkSprinkled, # hand, item, shaker, sprinkledType, storage |
    'bakedItem': _checkBaked,
    'cutItem': _checkCutItem, # hand, item, storage, tool |
    'peeledAndStored': _checkPeeledAndStored, # container, containerForPeels, disposition, hand, item, otherHand, storage, tool
    'armTriggeredPortalHandle': _checkArmTriggeredPortalHandle,
    'stoppedOpening': _checkStoppedOpening,
    'stoppedClosing': _checkStoppedClosing,
    'opened': _checkOpened,
    'closed': _checkClosed,
    'broughtNear': _checkBroughtNear
    }
processSuggesters = {
    'notifiedCompletion': _suggestNotifiedCompletion,
    'noped': _suggestNoped,
    'waited': _suggestWaited, # timeEnd
    'stoppedHands': _suggestStoppedHands,
    'near': _suggestNearA, # relatum | position
    'parkedArm': _suggestParkedArm, # hand | 
    'armNearItemHandle': _suggestArmNearItemHandle, # hand, item | 
    'grasped': _suggestGrasped, # hand, item | 
    'ungrasped': _suggestUngrasped, # hand, item | 
    'pickedItem': _suggestPickedItem, # hand, item | 
    'placedItem': _suggestPlacedItem, # container, hand, item[, allowedComponents] | 
    'loweredItemAt': _suggestLoweredItemAt, # container, hand, item[, allowedComponents] |
    "itemAboveContainer": _suggestItemAboveContainer, # item, hand, container[, sideHold]
    'transferredAndStored': _suggestTransferredAndStored, # amount, container, hand, item, pouredType, storage |
    'transferredAndUprighted': _suggestTransferredAndUprighted, # amount, container, hand, item, pouredType |
    'transferredContents': _suggestTransferredContents, # amount, container, hand, item, pouredType |
    "turnedControlAndParked": _suggestTurnedControlAndParked, # item, hand, link, setting
    "turnedHand": _suggestTurnedHand, # item, hand, link, setting
    'constraintFollowed': _suggestConstraintFollowed, # constraintConjunctions, hand, isTop | waypoints, tolerances, entities
    'mixedAndStored': _suggestMixedAndStored, # container, hand, mixedType, storage, tool, toolLink |
    'mixedAndUprighted': _suggestMixedAndUprighted, # container, hand, mixedType, tool, toolLink |
    'mixed': _suggestMixed, # container, hand, mixedType, tool, toolLink |
    'mashedAndStored': _suggestMashedAndStored, # container, hand, storage, tool, toolLink |
    'mashedAndUprighted': _suggestMashedAndUprighted, # container, hand, tool, toolLink |
    'mashed': _suggestMashed, # container, hand, tool, toolLink |
    'linedAndParked': _suggestLinedAndParked, # hand, item, lining |
    'lined': _suggestLined, # hand, item, lining |
    'coveredAndParked': _suggestCoveredAndParked, # hand, item, cover |
    'covered': _suggestCovered, # hand, item, cover |
    'shapedAndParked': _suggestShapedAndParked, # destination, hand, ingredientTypes, item, shapedType |
    'sprinkled': _suggestSprinkled, # hand, item, shaker, sprinkledType, storage |
    'cutItem': _suggestCutItem, # hand, item, storage, tool |
    'peeledAndStored': _suggestPeeledAndStored, # container, containerForPeels, disposition, hand, item, otherHand, storage, tool
    'opened': _suggestOpened,
    'closed': _suggestClosed,
    'stoppedOpening': _suggestStoppedOpening,
    'stoppedClosing': _suggestStoppedClosing,
    'bakedItem': _suggestBakedItem,
    'broughtNear': _suggestBroughtNear,
    'armTriggeredPortalHandle': _suggestArmTriggeredPortalHandle
    
    }
# TODO: rewrite/redesign: complicated setups of constraintFollowed goals are only needed if previous coherence conditions are met
conditionListers = {
    'notifyingCompletion': _emptyList,
    'waiting': _emptyList, # 
    'nearing': _getNearingConditionsA, # relatum | 
    'stoppingHands': _emptyList,
    'parkingArm': _getParkingArmConditions, # hand |
    'armNearingItemHandle': _getArmNearingItemHandleConditions, # hand, item |
    'grasping': _getGraspingConditions, # hand, item |
    'ungrasping': _emptyList, # hand, item |
    'pickingItem': _getPickingItemConditions, # hand, item |
    'placingItem': _getPlacingItemConditions, # container, hand, item[, allowedComponents] | 
    'loweringItem': _getLoweringItemConditions, # container, hand, item[, allowedComponents] |
    "holdingItemAbove": _getHoldingItemAboveConditions, # container, hand, item, sideHold[, allowedComponents] |
    'transferringAndStoring': _getTransferringAndStoringConditions, # amount, container, hand, item, pouredType, storage |
    'transferringAndUprighting': _getTransferringAndUprightingConditions, # amount, container, hand, item, pouredType, storage | positionInHand, orientationInHand
    'transferringContents': _getTransferringContentsConditions, # container, hand, item | positionInHand, orientationInHand
    "turningControl": _getTurningControlConditions, # item, hand, link, setting
    "turningHand": _getTurningHandConditions, # item, hand, link, setting
    'constraintFollowing': _getConstraintFollowingConditions, # constraintConjunctions, hand, isTop | waypoints, tolerances, entities
    'mixingAndStoring': _getMixingAndStoringConditions, # container, hand, mixedType, storage, tool, toolLink |
    'mixingAndUprighting': _getMixingAndUprightingConditions, # container, hand, mixedType, tool, toolLink |
    'mixing': _getMixingConditions, # container, hand, mixedType, tool, toolLink |
    'mashingAndStoring': _getMashingAndStoringConditions, # container, hand, storage, tool, toolLink |
    'mashingAndUprighting': _getMashingAndUprightingConditions, # container, hand, tool, toolLink |
    'mashing': _getMashingConditions, # container, hand, tool, toolLink |
    'liningAndParking': _getLiningAndParkingConditions, # hand, item, lining |
    'lining': _getLiningConditions, # hand, item, lining |
    'coveringAndParking': _getCoveringAndParkingConditions, # hand, item, cover |
    'covering': _getCoveringConditions, # hand, item, cover |
    'shaping': _getShapingConditions, # destination, hand, ingredientTypes, item, shapedType |
    'sprinkling': _getSprinklingConditions, # hand, item, shaker, sprinkledType, storage |
    'cuttingItem': _getCuttingItemConditions, # hand, item, storage, tool |
    'peelingItem': _getPeelingItemConditions, # container, containerForPeels, disposition, hand, item, otherHand, storage, tool
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

def checkGoal(w, name, node, predCache):
    description = node.get('description', {})
    goal = description.get('goal', 'notifiedCompletion')
    fn = goalCheckers[goal]
    return fn(w, name, description, node, predCache)

def checkProcess(w, name, node, predCache):
    ### TODO
    return True
def checkNoal(w, name, node, predCache):
    ### TODO
    return True
def checkThreat(w, name, node, predCache):
    ### TODO
    return True

def getThreats(w, name, description, predCache):
    return [] ### TODO; for now leave as empty list

def getProcesses(w, name, description, node, predCache):
    goal = description.get('goal', 'notifiedCompletion')
    return processSuggesters[goal](w, name, description, node, predCache)

def getCoherenceConditions(w, name, description, node, predCache):
    process = description.get('process', 'notifyingCompletion')
    return conditionListers[process](w, name, description, node, predCache)
    
def poseXYYawDistance(positionA, yawA, positionB, yawB, posThreshold, yawThreshold):
    d = [x-y for x,y in zip(positionA[:2], positionB[:2])]
    if (posThreshold*posThreshold < (d[0]*d[0] + d[1]*d[1])):
        return False
    aV = stubbornTry(lambda : pybullet.rotateVector(pybullet.getQuaternionFromEuler((0,0,yawA)), (1, 0, 0)))
    bV = stubbornTry(lambda : pybullet.rotateVector(pybullet.getQuaternionFromEuler((0,0,yawB)), (1, 0, 0)))
    if (yawThreshold >= (aV[0]*bV[0] + aV[1]*bV[1] + aV[2]*aV[2])):
        return False
    return True

def pose6DDistance(positionA, orientationA, positionB, orientationB, posThreshold, ornThreshold):
    d = [x-y for x,y in zip(positionA, positionB)]
    if (posThreshold*posThreshold < (d[0]*d[0] + d[1]*d[1] + d[2]*d[2])):
        return False
    aX = stubbornTry(lambda : pybullet.rotateVector(orientationA, (1, 0, 0)))
    bX = stubbornTry(lambda : pybullet.rotateVector(orientationB, (1, 0, 0)))
    aY = stubbornTry(lambda : pybullet.rotateVector(orientationA, (0,1,0)))
    bY = stubbornTry(lambda : pybullet.rotateVector(orientationB, (0,1,0)))
    if (ornThreshold >= (aX[0]*bX[0] + aX[1]*bX[1] + aX[2]*aX[2])) or (ornThreshold >= (aY[0]*bY[0] + aY[1]*bY[1] + aY[2]*bY[2])):
        return False
    return True

def toOriginAABB(aabb, pos):
    return [[x-y for x,y in zip(aabb[0],pos)], [x-y for x,y in zip(aabb[1],pos)]]

#def getCausalConditions(w, name, description, predCache):
#    return []

def checkStatus(w, name, node, predCache):
    nodeType = node['type']
    status = {'G': checkGoal, 'P': checkProcess, 'N': checkNoal, 'T': checkThreat}[nodeType](w, name, node, predCache)
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

# TODO: rewrite/redesign to make more efficient
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
    w = customDynamicsAPI["leetHAXXOR"]()
    startD = time.perf_counter()
    csvProcessGardening = w._kinematicTrees[name].get("customStateVariables", {}).get("processGardening") or {}
    fnProcessGardening = w._kinematicTrees[name].get("fn", {}).get("processGardening") or {}
    garden = csvProcessGardening.get('garden', {})
    if 0 not in garden:
        return
    toVisit = [0]
    predCache = {}
    bodyProcesses = {}
    goalStatuses = {}
    siblings = {}
    startG = time.perf_counter()
    profileUP = 0
    profileCG = 0
    while toVisit:
        cr = toVisit.pop()
        #sUP = time.perf_counter()
        nodeType, status = checkStatus(w, name, garden[cr], predCache)
        #eUP = time.perf_counter()
        #print("    CG %d %s | %s %f" % (cr, garden[cr]["description"].get("goal"), garden[cr]["description"].get("process"), eUP-sUP))
        #profileCG += eUP-sUP
        sibling = siblings.get(cr, None)
        goalStatuses[cr] = status
        #print('POP', cr, garden[cr]['description'], garden[cr].get('children'), status)
        if 'G' == nodeType:
            garden[cr]['previousStatus'] = status
            if status:
                popSubforest(garden, cr, 'P')
                #if isinstance(garden[cr]['description'], list):
                #    crGoal = (garden[cr].get('currentGoal', 0) + 1)%len(garden[cr]['description'])
                #    garden[cr]['currentGoal'] = crGoal
                #    sUP = time.perf_counter()
                #    updateSubforest(garden, cr, 'P', getProcesses(w, name, garden[cr]['description'][crGoal], garden[cr], predCache))
                #    profileUP += time.perf_counter()-sUP
                #else:
                if True:
                    sUP = time.perf_counter()
                    updateSubforest(garden, cr, 'T', getThreats(w, name, garden[cr]['description'], predCache)) ### TODO; for now will just always return an empty list
                    profileUP += time.perf_counter()-sUP
            else:
                #if isinstance(garden[cr]['description'], list):
                #    crGoal = garden[cr].get('currentGoal', 0)
                #    sUP = time.perf_counter()
                #    updateSubforest(garden, cr, 'P', getProcesses(w, name, garden[cr]['description'][crGoal], garden[cr], predCache))
                #    profileUP += time.perf_counter()-sUP
                #else:
                if True:
                    #sUP = time.perf_counter()
                    updateSubforest(garden, cr, 'P', getProcesses(w, name, garden[cr]['description'], garden[cr], predCache))
                    #eUP = time.perf_counter()
                    #print("    SP %d %s %f" % (cr, garden[cr]['description']['goal'], eUP-sUP))
                    #profileUP += eUP-sUP
        elif 'P' == nodeType:
            #sUP = time.perf_counter()
            updateSubforest(garden, cr, 'G', getCoherenceConditions(w, name, garden[cr]['description'], garden[cr], predCache))
            #eUP = time.perf_counter()
            #print("    CC %d %s %f" % (cr, garden[cr]['description']['process'], eUP-sUP))
            #profileUP += eUP-sUP
            #if status:
            #    updateSubforest(garden, cr, 'G', getCoherenceConditions(w, name, garden[cr]['description'], garden[cr]))
            #else:
            #    updateSubforest(garden, cr, 'G', getCausalConditions(w, name, garden[cr]['description']))
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
            #print('    PUSH first ', fc, garden[fc]['description'])
        if status and (sibling is not None):
            toVisit.append(sibling)
            #print('    PUSH sibling ', sibling, garden[sibling]['description'])
    startC = time.perf_counter()
    toKeep = set([])
    toVisit = [0]
    while toVisit:
        cr = toVisit.pop()
        toKeep.add(cr)
        toVisit = toVisit + garden[cr].get('children', [])
    for nodeIdx in list(garden.keys()):
        if nodeIdx not in toKeep:
            garden.pop(nodeIdx)
        elif "error" in garden[nodeIdx]:
            garden[0]["error"] = garden[nodeIdx]["error"]

    if "error" in garden[0]:
        return
    startB = time.perf_counter()
    #for k in sorted(garden.keys()):
    #    print(k, garden[k]['description'], garden[k].get('children'), garden[k].get('previousStatus'))

    # customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'processGardening', 'garden'), garden)
    startA = time.perf_counter()
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
                #print("    >>> ", actuator, variable, getDictionaryEntry(fnProcessGardening, ('actuator', 'variablePath'), None))
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
    endD = time.perf_counter()
    print("    garden %f\t(GRD %f\tUP %f\tCG %f\tAC %f)" % (endD-startD, startC-startG, profileUP, profileCG, endD-startA))


'''
OBSERVATION VERY IMPORTANT:
    IK will not precisely impose constraints based on waypoints. E.g., if need to go from p,q1 to p,q2, there may be times of significant deviations from p along the way.
    CONSEQUENCES:
        ALWAYS COMPUTE WAYPOINTS "ABSOLUTELY": the positions etc. of the trajectors cannot themselves be used as waypoints
        use Schmitt-Trigger pattern: precise thresholds when going False->True, tolerant thresholds once True.
'''

