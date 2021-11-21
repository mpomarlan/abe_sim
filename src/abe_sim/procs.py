import math
import random
import numpy as np
import pybullet as p
from abe_sim.garden import Goal, Process, BodyProcess
import abe_sim.pobject as pobject

from abe_sim.utils import stubbornTry
from abe_sim.geom import quaternionProduct, overlappingObjects, extrudeBox, overlappingObjectNames, translateVector, vectorNorm, vectorNormalize, scaleVector, vectorDifference, distance, interpolatePoint, angleDifference
from abe_sim.motplan import validExtrusion, allowCollisionByWhitelist, Corridor, planCorridor

def closestPointOnCuboid(aabbMin, aabbMax, position):
    x, y, z = position
    xm, ym, zm = aabbMin
    xM, yM, zM = aabbMax
    return (min(xM, max(xm, x)), min(yM, max(ym, y)), min(zM, max(zm, z)))

def closestFreePointOnCuboid(world, name, whitelist, suppMin, suppMax, position, dims):
    initialPos = closestPointOnCuboid(suppMin, suppMax, position)
    auxMin = [a-b for a,b in zip(initialPos,dims)]
    auxMax = [a+b for a,b in zip(initialPos,dims)]
    dimsAdj = [0.15,0.15,0.05]
    blockNames = set(overlappingObjectNames(auxMin, auxMax, world)).difference(whitelist)
    if 0 == len(blockNames):
        return initialPos
    kMax = [round((a - b)/(c)) for a,b,c in zip(suppMax,suppMin,dimsAdj)]
    initialPosK = [round((a-b)/(c)) for a,b,c in zip(initialPos,suppMin,dimsAdj)]
    cells = []
    for kX in range(kMax[0]):
        for kY in range(kMax[1]):
            cells.append((abs(kX - initialPosK[0]) + abs(kY - initialPosK[1]), kX, kY))
    for c in sorted(cells):
        if 1>=c[0]:
            continue
        auxMin = [suppMin[0]+dimsAdj[0]*c[1]-dims[0], suppMin[1]+dimsAdj[1]*c[2]-dims[1], suppMax[2]-dims[2]+0.01]
        auxMax = [suppMin[0]+dimsAdj[0]*c[1]+dims[0], suppMin[1]+dimsAdj[1]*c[2]+dims[1], suppMax[2]+dims[2]+0.01]
        if 0 == len(set(overlappingObjectNames(auxMin, auxMax, world)).difference(whitelist)):
            px = [suppMin[0] + c[1]*dimsAdj[0], suppMin[1] + c[2]*dimsAdj[1], suppMax[2]+0.01]
            return px
    return None

def closestPointOnSphere(center, radius, position, onUpper=True):
    d = [a-b for a,b in zip(position, center)]
    nd = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
    d = [x*radius/nd for x in d]
    if onUpper:
        if 0 > d[2]:
            d[2] = 0
            nD = math.sqrt(d[0]*d[0] + d[1]*d[1])
            if 0.001 > nD:
                d = [0,0,1]
            else:
                d[0] = radius*d[0]/nD
                d[1] = radius*d[1]/nD
    return [a+b for a,b in zip(center, d)]

def getHandLinearJointControls(agent, hand, controls, posHTarget, velocity=[0,0,0]):
    positionB = agent.getBodyProperty(("base_yaw",), "position")
    orientationB = agent.getBodyProperty(("base_yaw",), "orientation")
    handBase = {"right": [0, -0.4, 0.95], "left": [0, 0.4, 0.95]}[hand]
    positionB = [a+b for a,b in zip(positionB, p.rotateVector(orientationB, handBase))]
    ### T hand in world; need T hand in arm base; have T arm in world
    ### Thiw = Taiw*Thia => Thia = inv(Taiw)*Thiw
    ### for a transform T and its inverse iT: (iT*T).t = 0: -iT.q*T.t = iT.t
    position = [a-b for a,b in zip(p.rotateVector((orientationB[0], orientationB[1], orientationB[2], -orientationB[3]), posHTarget), p.rotateVector((orientationB[0], orientationB[1], orientationB[2], -orientationB[3]), positionB))]
    joints = {"left": ("hand_left_base_to_hand_left_x", "hand_left_x_to_hand_left_y", "hand_left_y_to_hand_left_z"), "right": ("hand_right_base_to_hand_right_x", "hand_right_x_to_hand_right_y", "hand_right_y_to_hand_right_z")}[hand]
    controls["jointTargets"].update({joints[k]: (position[k], velocity[k], 1.0) for k in [0,1,2]})
    return controls

def getHandAngularJointControls(agent, hand, controls, orHTarget, velocity=[0,0,0]):
    ### Have target orientation in world, orientation of arm base; need orientation in arm base
    ### Thiw = Tbiw*Thib => Thib = inv(Tbiw)*Thiw
    orientationB = agent.getBodyProperty(("base_yaw",), "orientation")
    orTH = quaternionProduct((orientationB[0], orientationB[1], orientationB[2], -orientationB[3]), orHTarget)
    ### Need also current orientation in arm base, from the orientation in the world
    orientationH = agent.getBodyProperty(({"left": "hand_left_roll", "right": "hand_right_roll"}[hand],), "orientation")
    orCH = quaternionProduct((orientationB[0], orientationB[1], orientationB[2], -orientationB[3]), orientationH)
    ### Need a difference quaternion: orTH = dq*orCH => dq = orTH*inv(orCH)
    dq = quaternionProduct(orTH, (orCH[0], orCH[1], orCH[2], -orCH[3]))
    axis,angle = p.getAxisAngleFromQuaternion(dq)
    angle = angleRemap(angle)
    euler = p.getEulerFromQuaternion(orTH)
    if 0.01 < abs(angle):
        aux = 0.01
        if 0 > angle:
            aux = -aux
        euler = p.getEulerFromQuaternion(quaternionProduct(p.getQuaternionFromAxisAngle(axis, aux), orCH))
    joints = {"left": ("hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"), "right": ("hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll")}[hand]
    controls["jointTargets"].update({joints[k]: (euler[k], velocity[k], 1.0) for k in [0,1,2]})
    return controls

def getTrajectorBoxes(agent, handLink, tolerance=0.0):
    tolerance = abs(tolerance)
    world = agent._world
    position = agent.getBodyProperty((handLink,), "position")
    graspedNames = agent.getBodyProperty((handLink,), "grasping")
    pulledNames = [x.split(':')[0] for x in agent.getBodyProperty((handLink,), "pulling")]
    whitelist = ["abe"] + graspedNames + pulledNames
    boxes = [agent.getAABB((handLink,))] + [world._pobjects[x].getAABB(None) for x in graspedNames]
    def aux(box):
        aabbMin, aabbMax = box
        aabbMin = translateVector(vectorDifference(aabbMin, position), [-tolerance]*3)
        aabbMax = translateVector(vectorDifference(aabbMax, position), [tolerance]*3)
        return aabbMin, aabbMax
    boxes = [aux(x) for x in boxes]
    allowableCollisionFn = lambda box, collidingObjects: allowCollisionByWhitelist(box, collidingObjects, whitelistNames=whitelist, whitelistTypes=w._particleTypes.keys())
    return boxes, allowableCollisionFn

def isGraspValid(agent, hand):
    handLink = {"right": "hand_right_roll", "left": "hand_left_roll"}[hand]
    handPos = agent.getBodyProperty((handLink,), "position")
    trajectorBoxes, allowableCollisionFn = getTrajectorBoxes(agent, handLink, tolerance=0.04)
    return validExtrusion(trajectorBoxes, handPos, handPos, allowableCollisionFn, agent._world) and (1.2 < handPos[2])

def handSampleBox(agent, hand, bMin, bMax):
    pB = agent.getBodyProperty(("base_yaw",), "position")
    oB = agent.getBodyProperty(("base_yaw",), "orientation")
    hB = translateVector(pB, p.rotateVector(oB, {"right": (0,-0.4,0.95), "left": (0,0.4,0.95)}[hand]))
    sampleBox = [translateVector(hB, bMin), translateVector(hB, bMax)]
    return sampleBox

def validateEndpoint(boxes, position, suggestedPosition, allowableCollisionFn, world):
    toAppend = False
    toReplace = False
    isValid = validExtrusion(boxes, position, position, allowableCollisionFn, world)
    isClose = False
    if (None != suggestedPosition) and (0.1 > distance(position, suggestedPosition)):
        isClose = True
    if isClose:
        return suggestedPosition, False, False
    if isValid:
        if (None == suggestedPosition) or not validExtrusion(boxes, position, suggestedPosition, allowableCollisionFn, world):
            return position, True, True
        else:
            return position, False, False
    else:
        for k in range(30):
            sample = translateVector(position, [random.uniform(-0.1,0.1) for x in range(3)])
            if validExtrusion(boxes, sample, suggestedPosition, allowableCollisionFn, world) and validExtrusion(boxes, sample, sample, allowableCollisionFn, world):
                return sample, True, False
        while True:
            sample = translateVector(position, [random.uniform(-0.1,0.1) for x in range(3)])
            if validExtrusion(boxes, sample, sample, allowableCollisionFn, world):
                if validExtrusion(boxes, sample, suggestedPosition, allowableCollisionFn, world):
                    return sample, True, False
                else:
                    return sample, True, True
    return validPosition, toAppend, toReplace

def updateCorridor(trajectorBoxes, corridor, connected, position, allowableCollisionFn, world, atEnd=True):
    idx = -1
    if not atEnd:
        idx = 0
    suggestedPosition = None
    for k,box in enumerate(trajectorBoxes):
        bm, bM = extrudeBox(box,position,position)
        ovs = overlappingObjects(bm,bM,world)
    if (None != corridor) and ([]!=corridor.waypoints):
        suggestedPosition = corridor.waypoints[idx]
    validPosition, toAppend, toReplace = validateEndpoint(trajectorBoxes, position, suggestedPosition, allowableCollisionFn, world)
    if toAppend:
        if (None == corridor) or ([]==corridor.waypoints):
            corridor = Corridor([validPosition])
            connected = False
        else:
            if toReplace:
                corridor = Corridor([validPosition])
                connected = False
            else:
                if atEnd:
                    corridor.waypoints.append(validPosition)
                else:
                    corridor.waypoints = [validPosition] + corridor.waypoints
    return connected, corridor

def planArmMotion(agent, hand, targetPos, corridor, controls,maxspeed=0.1):
    world = agent._world
    handLink = {"right": "hand_right_roll", "left": "hand_left_roll"}[hand]
    boxes, allowableCollisionFn = getTrajectorBoxes(agent, handLink, tolerance=0)
    connected = False
    corridors = [None, None]
    if (None != corridor) and ([] == corridor.waypoints):
        corridor = None
    if None != corridor:
        if 0.005 < distance(targetPos, corridor.waypoints[-1]):
            corridor.waypoints.append(targetPos)
        corridors = corridor.validCorridors(boxes, allowableCollisionFn, world)
        if (None != corridors[0]) and (corridors[0] == corridors[1]):
            connected = True
    if None == corridors[0]:
        corridors[0] = Corridor(waypoints=[targetPos])
    position = agent.getBodyProperty((handLink,), "position")
    if connected:
        corridor = corridors[0]
    else:
        sampleBox = {"left": [[-0.85,-1.4,-0.9], [2,1.85,2]], "right": [[-0.85,-1.85,-0.9], [2,1.4,2]]}[hand]
        armBase = {"left": (0,0.4,0.95), "right": (0,-0.4,0.95)}[hand]
        positionBase = agent.getBodyProperty(("base_yaw",), "position")
        orientationBase = agent.getBodyProperty(("base_yaw",), "orientation")
        positionArmBase = translateVector(positionBase, p.rotateVector(orientationBase, armBase))
        corridor = planCorridor(sampleBox, boxes, allowableCollisionFn, world, corridors[0].waypoints[0], position, positionArmBase, orientationBase)
        if None != corridor:
            corridor = Corridor(waypoints=corridor.waypoints+corridors[0].waypoints)
    if None != corridor:
        nextPosition, k = corridor.nextAlong(position, boxes, allowableCollisionFn, world)
        if None != nextPosition:
            d = distance(nextPosition, position)
            diff = vectorDifference(nextPosition, position)
            if maxspeed < d:
                nextPosition = translateVector(position, scaleVector(diff, maxspeed/d))
            elif (0.005 > d) and (1 < len(corridor.waypoints)):
                corridor.waypoints = corridor.waypoints[k+1:]
            controls = getHandLinearJointControls(agent, hand, controls, nextPosition, velocity=[0,0,0])
    return corridor, controls

def getFreeTargetAroundAgent(agent, hand, corridor, samples=100):
    world = agent._world
    handLink = {"right": "hand_right_roll", "left": "hand_left_roll"}[hand]
    refPos = [0, {"left": 0.4, "right": -0.4}[hand], 0.95]
    boxes, allowableCollisionFn = getTrajectorBoxes(agent, handLink, tolerance=0)
    positionBase = agent.getBodyProperty(("base_yaw",), "position")
    orientationBase = agent.getBodyProperty((), "orientation")
    points = []
    staggered = [x[1] for x in sorted([(abs(x), x) for x in list(np.arange(0.4,-0.45,-0.05))])]
    for z in list(np.arange(0.5,-0.45,-0.05)):
        for y in staggered:
            for x in staggered:
                pointA = translateVector(positionBase, p.rotateVector(orientationBase, translateVector(refPos,[x,y,z])))
                if validExtrusion(boxes, pointA, pointA, allowableCollisionFn, world):
                    return pointA
    return translateVector(positionBase, p.rotateVector(orientationBase, refPos))

def stopHand(agent, hand):
    js = agent.getJointStates()
    controls = {"+constraints": [], "-constraints": [], "jointTargets": {}}
    for jn in {"left": ["hand_left_base_to_hand_left_x", "hand_left_x_to_hand_left_y", "hand_left_y_to_hand_left_z", "hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"], "right": ["hand_right_base_to_hand_right_x", "hand_right_x_to_hand_right_y", "hand_right_y_to_hand_right_z", "hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll"]}[hand]:
        controls["jointTargets"][jn] = (js[jn][0],0,1)
    agent.applyRigidBodyControls([controls])

def getContents(item,brds=[0,0,0]):
    aabbMin, aabbMax = item.getAABB(None)
    aabbMin = translateVector(aabbMin,[-brds[0], -brds[1], -brds[2]])
    aabbMax = translateVector(aabbMax,brds)
    def isIngredient(x):
        return x.getBodyProperty("fn", "ingredient")
    return [x for x in closeObjects(item, radius=0, filterFn=isIngredient) if aabbContainment(x, aabbMin, aabbMax)]

def getSubstance(item):
    substancelink = item.getBodyProperty("fn", "substancelink")
    return item.getBodyProperty((substancelink,), "substance") 

def findByType(world, typeT,listAll=False):
    retq = None
    if listAll:
        retq = []
    if typeT in world._ontoTypes:
        retq = [world._pobjects[name] for name in world._ontoTypes[typeT]]
        if not listAll:
            retq = retq[0]
    return retq

def aabbContainment(item, aabbMin, aabbMax):
    iaabbMin, iaabbMax = item.getAABB(None)
    mins = [x<=y for x,y in zip(aabbMin, iaabbMin)]
    maxs = [x>=y for x,y in zip(aabbMax, iaabbMax)]
    return (all(mins) and all(maxs))

def closeObjects(item,radius=0.25, filterFn=None):
    aabbMin, aabbMax = item.getAABB(None)
    minC = [a - radius for a in aabbMin]
    maxC = [a + radius for a in aabbMax]
    retq = overlappingObjects(minC, maxC, item._world)
    if None != filterFn:
        retq = [x for x in retq if filterFn(x)]
    return retq

def isSubclass(cs, cS):
    subclassOf = {"fridge": ["fridge", "container"], "oven": ["oven", "container"], "fridgeDoor": ["pullableDoor"], "freezerDoor": ["pullableDoor"]}
    return (cs in subclassOf) and (cS in subclassOf[cs])

def findContainerHandleAround(item, findalways=False):
    objs = closeObjects(item)
    world = item._world
    retq = None
    for o in objs:
        ot = o.getBodyProperty("", "type")
        if isSubclass(ot, "container"):
            door = o.getBodyProperty("fn", "door")
            if door and (door in world._pobjects):
                door = world._pobjects[door]
                fn = door.getBodyProperty("fn", "doorhandlelink")
                fnj = door.getBodyProperty("fn", "doorhandlejoint")
                if fn:
                    opMin = door.getBodyProperty("fn", "openmin")
                    opMax = door.getBodyProperty("fn", "openmax")
                    doing = True
                    a = stubbornTry(lambda : p.getJointState(door._id, door._jointName2Id[fnj], door._world.getSimConnection()))[0]
                    if findalways or (a < opMin) or (opMax < a):
                        retq = pobject.PObjectWrapper(door, fn, fnj)
                        break
    return retq

def isGrasping(agent, handLink, item, radius):
    grasping = agent.getBodyProperty((handLink,), "grasping")
    if None == grasping:
        return False
    if item.getName() in grasping:
        posI = item.getBodyProperty((), "position")
        handle = item.getBodyProperty("fn", "handle")
        if handle:
            posI = translateVector(posI, p.rotateVector(item.getBodyProperty((), "orientation"), handle))
        posH = agent.getBodyProperty((handLink,), "position")
        return distance(posI, posH) < radius
    return False


class Location:
    def __init__(self, relatum=None):
        self._relatum = relatum
    def isThere(self, trajector):
        return False
    def __str__(self):
        return type(self).__name__ + "(" + str(self._relatum) + ")"
    def suggestPose(self, trajector):
        return (0,0,0), (0,0,0,1)

class LocationUpright(Location):
    def isThere(self, trajector):
        orientation = trajector.getBodyProperty((), "orientation")
        vertical = p.rotateVector(orientation, [0, 0, 1])
        return 0.98 < vertical[2]
    def suggestPose(self, trajector):
        position = trajector.getBodyProperty((), "position")
        orientation = trajector.getBodyProperty((), "orientation")
        adjVert = p.rotateVector(orientation, [0,0,1])
        if 0.98 > adjVert[2]:
            axis = [adjVert[1],-adjVert[0],0]
            nA = math.sqrt(axis[0]*axis[0] + axis[1]*axis[1])
            axis = [x/nA for x in axis]
            angle = math.acos(adjVert[2])
            dq = p.getQuaternionFromAxisAngle(axis, angle)
            orientation = quaternionProduct(dq, orientation)
        return position, orientation

class LocationAt(Location):
    def isThere(self, trajector):
        aabbMin, aabbMax = trajector.getAABB(None)
        dims = [(a-b)/2.0 for a,b in zip(aabbMax,aabbMin)]
        dims = [dims[0]+0.015, dims[1]+0.015, dims[2]]
        relatum = self._relatum
        if isinstance(relatum, pobject.PObjectWrapper):
            relatum = relatum._pobject
        supportBBs = self._relatum.getBodyProperty("fn", "supportbbs")
        if None == supportBBs:
            supportBBs = [self._relatum.getAABB(None)]
        pR = self._relatum.getBodyProperty((), "position")
        oR = self._relatum.getBodyProperty((), "orientation")
        aux = []
        for s in supportBBs:
            rm = [a+b for a,b in zip(pR, p.rotateVector(oR, s[0]))]
            rM = [a+b for a,b in zip(pR, p.rotateVector(oR, s[1]))]
            aux.append([[min(a,b) for a,b in zip(rm,rM)], [max(a,b) for a,b in zip(rm,rM)]])
        supportBBs = aux
        for sbb in supportBBs:
            suppMin = [sbb[0][0]+dims[0], sbb[0][1]+dims[1], sbb[1][2]+dims[2]+0.01]
            suppMax = [sbb[1][0]-dims[0], sbb[1][1]-dims[1], sbb[1][2]+dims[2]+0.02]
            sbba = [suppMin, suppMax]
            if trajector in overlappingObjects(sbba[0], sbba[1], trajector._world):
                return True
        return False
    def suggestPose(self, trajector):
        aabbMin, aabbMax = trajector.getAABB(None)
        dims = [(a-b)/2.0 for a,b in zip(aabbMax,aabbMin)]
        dims = [dims[0]+0.015, dims[1]+0.015, dims[2]]
        position = trajector.getBodyProperty((), "position")
        _, orientation = LocationUpright(trajector).suggestPose(trajector)
        supportBBs = self._relatum.getBodyProperty("fn", "supportbbs")
        shelved = True
        if None == supportBBs:
            supportBBs = [self._relatum.getAABB(None)]
            shelved = False
        else:
            pR = self._relatum.getBodyProperty((), "position")
            oR = self._relatum.getBodyProperty((), "orientation")
            aux = []
            for s in supportBBs:
                rm = [a+b for a,b in zip(pR, p.rotateVector(oR, s[0]))]
                rM = [a+b for a,b in zip(pR, p.rotateVector(oR, s[1]))]
                aux.append([[min(a,b) for a,b in zip(rm,rM)], [max(a,b) for a,b in zip(rm,rM)]])
            supportBBs = aux
        minD = None
        bestPos = position
        whitelist = ["abe",trajector.getName()]
        if shelved:
            whitelist.append(self._relatum.getName())
        for supp in supportBBs:
            suppMin = [supp[0][0]+dims[0], supp[0][1]+dims[1], supp[1][2]+dims[2]+0.01]
            suppMax = [supp[1][0]-dims[0], supp[1][1]-dims[1], supp[1][2]+dims[2]+0.02]
            candidate = closestFreePointOnCuboid(self._relatum._world, trajector.getName(), whitelist, suppMin, suppMax, position, dims)
            if None == candidate:
                continue
            d = distance(candidate, position)
            if (None == minD) or (d < minD):
                minD = d
                bestPos = candidate
        bestPos = list(bestPos)
        return bestPos, orientation

class LocationIn(LocationAt):
    def isThere(self, trajector):
        visited = {"floor": True}
        toVisit = [trajector]
        relatum = self._relatum
        if isinstance(relatum, pobject.PObjectWrapper):
            relatum = relatum._pobject
        while toVisit:
            cr = toVisit.pop()
            if cr.getName() in visited:
                continue
            visited[cr.getName()] = True
            aabbMin, aabbMax = cr.getAABB(None)
            aabbMin = tuple([aabbMin[0], aabbMin[1], aabbMin[2]-0.01])
            overlaps = overlappingObjects(aabbMin, aabbMax, trajector._world)
            if relatum in overlaps:
                return True
            [toVisit.append(x) for x in overlaps]
        return False

class LocationOver(Location):
    def isThere(self, trajector):
        position = trajector.getBodyProperty((), "position")
        orientation = trajector.getBodyProperty((), "orientation")
        refPt = trajector.getBodyProperty("fn", "refpt")
        if None == refPt:
            refPt = position
        else:
            refPt = [a+b for a,b in zip(position, p.rotateVector(orientation, refPt))]
        refPos = self._relatum.getBodyProperty((), "position")
        d = [a-b for a,b in zip(refPos, refPt)]
        d = math.sqrt(d[0]*d[0] + d[1]*d[1])
        return 0.03 > d
    def suggestPose(self, trajector):
        aabbMin, aabbMax = trajector.getAABB(None)
        aabbMinR, aabbMaxR = self._relatum.getAABB(None)
        dims = [(a-b)/2.0 for a,b in zip(aabbMax,aabbMin)]
        dimsR = [(a-b)/2.0 for a,b in zip(aabbMaxR,aabbMinR)]
        positionT = trajector.getBodyProperty((), "position")
        orientationT = trajector.getBodyProperty((), "orientation")
        refPt = trajector.getBodyProperty("fn", "refpt")
        if None == refPt:
            refPt = [0,0,0]
        refPt = p.rotateVector(orientationT, refPt)
        positionR = self._relatum.getBodyProperty((), "position")
        refPtTarget = list(positionR)
        refPtTarget[2] = refPtTarget[2] + dimsR[2] + 0.1 + (refPt[2]+dims[2])
        bestPos = [a-b for a,b in zip(refPtTarget, refPt)]
        return bestPos, orientationT

class LocationTippedOver(Location):
    def isThere(self, trajector):
        orientation = trajector.getBodyProperty((), "orientation")
        dripAxis = trajector.getBodyProperty("fn", "dripaxis")
        if None == dripAxis:
            dripAxis = [0,1,0]
        tipped = -0.99 > p.rotateVector(orientation, dripAxis)[2]
        return LocationOver(self._relatum).isThere(trajector) and tipped
    def suggestPose(self, trajector):
        positionO, orientationO = LocationOver(self._relatum).suggestPose(trajector)
        dripAxis = trajector.getBodyProperty("fn", "dripaxis")
        if None == dripAxis:
            dripAxis = [0,1,0]
        dripAdj = p.rotateVector(orientationO, dripAxis)
        # dripAdj crossproduct -z
        crossP = [-dripAdj[1], dripAdj[0], 0]
        cD = math.sqrt(crossP[0]*crossP[0] + crossP[1]*crossP[1])
        if 0.001 > cD:
            bestPos = positionO
            orientation = orientationO
        else:
            refPt = trajector.getBodyProperty("fn", "refpt")
            if None == refPt:
                refPt = [0,0,0]
            q = p.getQuaternionFromAxisAngle([x/cD for x in crossP], math.acos(-dripAdj[2]))
            orientation = quaternionProduct(q, orientationO)
            positionR = [a+b for a,b in zip(positionO, p.rotateVector(orientationO, refPt))]
            bestPos = p.rotateVector(q, [a-b for a,b in zip(positionO,positionR)])
            bestPos = [a+b for a,b in zip(bestPos,positionR)]
        return bestPos, orientation

class Unwill(Goal):
    def _isFulfilled(self):
        return False

class ParkedArms(Goal):
    def __init__(self,agent):
        super().__init__()
        self._agent = agent
        self._joints = {"right": [["hand_right_base_to_hand_right_x", "hand_right_x_to_hand_right_y", "hand_right_y_to_hand_right_z"], ["hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll"]], "left": [["hand_left_base_to_hand_left_x", "hand_left_x_to_hand_left_y", "hand_left_y_to_hand_left_z"], ["hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"]]}
        self._handLinks = {"left": "hand_left_roll", "right": "hand_right_roll"}
        self._armBases = {"left": (0,0.4,1.35), "right": (0,-0.4,1.35)}
    def _strVarPart(self):
        return str(self._agent)
    def _armParked(self, arm):
        joints = self._joints[arm]
        velT = all([0.001 > abs(p.getJointState(self._agent._id, self._agent._jointName2Id[j],self._agent._world.getSimConnection())[1]) for j in joints[0]])
        velQ = all([0.001 > abs(p.getJointState(self._agent._id, self._agent._jointName2Id[j],self._agent._world.getSimConnection())[1]) for j in joints[1]])
        atT = all([l > abs(p.getJointState(self._agent._id, self._agent._jointName2Id[j],self._agent._world.getSimConnection())[0]) for l,j in zip([0.15,0.15,0.5], joints[0])])
        atQ = all([0.15 > abs(p.getJointState(self._agent._id, self._agent._jointName2Id[j],self._agent._world.getSimConnection())[0]) for j in joints[1]])
        pB = self._agent.getBodyProperty(("base_yaw",), "position")
        oB = self._agent.getBodyProperty(("base_yaw",), "orientation")
        aB = translateVector(pB, p.rotateVector(oB,self._armBases[arm]))
        closeOb = overlappingObjects(translateVector(aB,[-0.15,-0.15,-0.15]), translateVector(aB,[0.15,0.15,0.15]), self._agent._world)
        def filterFn(x):
            return "abe" != x.getName() and (not x.getBodyProperty("fn", "ingredient")) and (x.getName() not in self._agent.getBodyProperty((self._handLinks[arm],), "grasping"))
        if None != filterFn:
            closeOb = [x for x in closeOb if filterFn(x)]
        if closeOb:
            atT = True
        return atT and atQ and velT and velQ
    def _isFulfilled(self):
        return all([self._armParked(x) for x in ["left", "right"]])
    def suggestProcess(self):
        return ParkingArms(self._agent)

class ParkingArms(BodyProcess):
    def __init__(self,agent):
        super().__init__(coherence=[])
        self._agent = agent
        self._joints = ["hand_right_base_to_hand_right_x", "hand_right_x_to_hand_right_y", "hand_right_y_to_hand_right_z", "hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll", "hand_left_base_to_hand_left_x", "hand_left_x_to_hand_left_y", "hand_left_y_to_hand_left_z", "hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"]
        self._tjoints = ["hand_right_base_to_hand_right_x", "hand_right_x_to_hand_right_y", "hand_right_y_to_hand_right_z", "hand_left_base_to_hand_left_x", "hand_left_x_to_hand_left_y", "hand_left_y_to_hand_left_z"]
        self._rjoints = ["hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll", "hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"]
        self._corridors = {"left": None, "right": None}
    def _markForDeletion(replacement=None):
        self._coherence.append(Unwill())
        return None
    def bodyAction(self):
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {}}
        positionBase = self._agent.getBodyProperty(("base_yaw",), "position")
        orientationBase = self._agent.getBodyProperty(("base_yaw",), "orientation")
        self._corridors["right"], controls = planArmMotion(self._agent, "right", translateVector(positionBase, p.rotateVector(orientationBase, [0,-0.4,1.35])), self._corridors["right"], controls)
        self._corridors["left"], controls = planArmMotion(self._agent, "left", translateVector(positionBase, p.rotateVector(orientationBase, [0,0.4,1.35])), self._corridors["left"], controls)
        js = self._agent.getJointStates()
        sumJ = 0.0
        tjs = [0.15, 0.15, 0.5, 0.15, 0.15, 0.5]
        for k,j in enumerate(self._tjoints):
            d = (abs(js[j][0])-tjs[k])
            if 0 < d:
                sumJ = 1.0
                break
        if 0.2 > sumJ:
            for j in self._rjoints:
                controls["jointTargets"][j] = (0,0,1)
        self._agent.applyRigidBodyControls([controls])
        return

class ItemOnCounter(Goal):
    def __init__(self, item):
        super().__init__()
        self._item = item
        self._counter = findByType(self._item._world, "countertop")
    def _strVarPart(self):
        return str(self._item) + "," + str(self._counter)
    def _isFulfilled(self):
        return LocationAt(self._counter).isThere(self._item)
    def getThreats(self, processes):
        grabbedLeft = SwitchingOnGrasping(self._item,"left")
        grabbedRight = SwitchingOnGrasping(self._item,"right")
        retq = [x for x in [grabbedLeft, grabbedRight] if str(x) in processes]
        return retq
    def suggestProcess(self):
        hand = "right"
        if Grasped(self._item,"left").isFulfilled():
            hand = "left"
        return PlacingGraspedItem(self._item, LocationAt(self._counter), hand)

class ItemOnLocation(Goal):
    def __init__(self, item, destination):
        super().__init__()
        self._item = item
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._destination)
    def _isFulfilled(self):
        return LocationAt(self._destination).isThere(self._item)
    def getThreats(self, processes):
        grabbedLeft = SwitchingOnGrasping(self._item,"left")
        grabbedRight = SwitchingOnGrasping(self._item,"right")
        retq = [x for x in [grabbedLeft, grabbedRight] if str(x) in processes]
        return retq
    def suggestProcess(self):
        hand = "right"
        if Grasped(self._item,"left").isFulfilled():
            hand = "left"
        return PlacingGraspedItem(self._item, LocationAt(self._destination), hand)

class PlacingGraspedItem(Process):
    def __init__(self,item,location,hand):
        super().__init__(coherence=[Grasped(item,hand), BaseNear(location), ItemNear(item,location)])
        self._item = item
        self._location = location
        self._hand = hand
    def _strVarPart(self):
        return str(self._item) + "," + str(self._location) + "," + str(self._hand)

class SwitchedOnGrasping(Goal):
    def __init__(self,item,hand):
        super().__init__()
        self._item = item
        self._hand = hand
        self._agent = findByType(self._item._world, "agent")
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[hand]
        self._radius = item.getBodyProperty("fn", "graspingradius")
        if None == self._radius:
            self._radius = 0.2
    def _strVarPart(self):
        return str(self._item) + "," + str(self._hand)
    def _isFulfilled(self):
        return isGrasping(self._agent, self._handLink, self._item, self._radius)
    def suggestProcess(self):
        return SwitchingOnGrasping(self._item, self._hand)

class SwitchingOnGrasping(BodyProcess):
    def __init__(self,item,hand):
        super().__init__(coherence=[BaseNear(item), HandNear(item,hand)])
        self._item = item
        self._hand = hand
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[hand]
        self._agent = findByType(self._item._world, "agent")
    def _strVarPart(self):
        return str(self._item) + "," + str(self._hand)
    def _markForDeletionInternal(self, replacement=None):
        grasping = self._agent.getBodyProperty((self._handLink,), "grasping")
        uprighting = self._agent.getBodyProperty((self._handLink,), "uprighting")
        if self._item.getName() in grasping:
            grasping.remove(self._item.getName())
        if self._item.getName() in uprighting:
            uprighting.remove(self._item.getName())
        self._agent.setBodyProperty((self._handLink,), "grasping", grasping)
        self._agent.setBodyProperty((self._handLink,), "uprighting", uprighting)
        self._coherence.append(Unwill())
        return None
    def bodyAction(self):
        grasping = self._agent.getBodyProperty((self._handLink,), "grasping")
        if self._item.getName() not in grasping:
            grasping.append(self._item.getName())
        self._agent.setBodyProperty((self._handLink,), "grasping", grasping)

class Grasped(Goal):
    def __init__(self,item,hand):
        super().__init__()
        self._item = item
        self._hand = hand
        self._agent = findByType(self._item._world, "agent")
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[hand]
        self._radius = item.getBodyProperty("fn", "graspingradius")
        if None == self._radius:
            self._radius = 0.2
    def _strVarPart(self):
        return str(self._item) + "," + str(self._hand)
    def _isFulfilled(self):
        return isGrasping(self._agent, self._handLink, self._item, self._radius)
    def suggestProcess(self):
        return GraspingItem(self._item, self._hand)

class GraspingItem(Process):
    def __init__(self,item,hand):
        super().__init__(coherence=[Accessible(item),BaseNear(item),HandNear(item,hand),SwitchedOnGrasping(item,hand)])
        self._item = item
        self._hand = hand
    def _strVarPart(self):
        return str(self._item) + "," + str(self._hand)
    def _markForDeletionInternal(self, replacement=None):
        return ItemOnLocation(self._item, self._item._world._pobjects["kitchenCabinet"])

class BaseNear(Goal):
    def __init__(self,location):
        super().__init__()
        self._location = location
        if isinstance(self._location, Location):
            self._item = self._location._relatum
        else:
            self._item = self._location
        self._agent = findByType(self._item._world, "agent")
    def _strVarPart(self):
        return str(self._location)
    def _isFulfilled(self):
        refPO = self._location
        if isinstance(refPO, Location):
            if isinstance(refPO, LocationUpright):
                return True
            refPO = refPO._relatum
        aabbMin, aabbMax = refPO.getAABB(None)
        aabbMin = tuple([aabbMin[0] - 1.3, aabbMin[1] - 1.3, -1])
        aabbMax = tuple([aabbMax[0] + 1.3, aabbMax[1] + 1.3, 1])
        positionR = refPO.getBodyProperty((), "position")
        position = self._agent.getBodyProperty(("base_yaw",), "position")
        positionR = (positionR[0], positionR[1], 0)
        position = (position[0], position[1], 0)
        orientation = self._agent.getBodyProperty(("base_yaw",), "orientation")
        velocityR = refPO.getBodyProperty((), "linearVelocity")
        velocity = self._agent.getBodyProperty(("base_yaw",), "linearVelocity")
        nVR = math.sqrt(velocityR[0]*velocityR[0] + velocityR[1]*velocityR[1] + velocityR[2]*velocityR[2])
        nVO = math.sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2])
        tracking = False
        if (0.01 > nVR) and (0.01 > nVO):
            tracking = True
        elif (0.01 <= nVR) and (0.01 <= nVO):
            vP = [a*b/(nVR*nVO) for a,b in zip(velocityR,velocity)]
            tracking = 0.95 < (vP[0]+vP[1]+vP[2])
        close = all([((pMin < p) and (p < pMax)) for pMin,p,pMax in zip(aabbMin, position, aabbMax)])
        fwd = p.rotateVector(orientation,[1,0,0])
        nd = math.sqrt(fwd[0]*fwd[0] + fwd[1]*fwd[1] + fwd[2]*fwd[2])
        fwd = (fwd[0]/nd, fwd[1]/nd, 0)
        d = [a-b for a,b in zip(positionR, position)]
        nd = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        d = [x/nd for x in  d]
        oriented = 0.9 < (d[0]*fwd[0] + d[1]*fwd[1])
        return oriented and close and tracking
    def suggestProcess(self):
        return NavigateTo(self._location)

class NavigateTo(BodyProcess):
    def __init__(self,location):
        super().__init__(coherence=[])
        self._location = location
        if isinstance(self._location, Location):
            self._item = self._location._relatum
        else:
            self._item = self._location
        self._agent = findByType(self._item._world, "agent")
        self._corridors = {"right": None, "left": None}
    def _strVarPart(self):
        return str(self._location)
    def _markForDeletionInternal(self, replacement=None):
        self._coherence.append(Unwill())
        return None
    def _max(self, a, b):
        if None == a:
            return b
        if None == b:
            return a
        return max(a,b)
    def _carryingRadius(self,hand):
        handLink = {"right": "hand_right_roll", "left": "hand_left_roll"}[hand]
        basePos = self._agent.getBodyProperty(("base_yaw",), "position")
        grasped = self._agent.getBodyProperty((handLink,), "grasping")
        retq = None
        for g in grasped:
            o = self._agent._world._pobjects[g]
            if 0 < len(getContents(o)):
                pg = o.getBodyProperty((), "position")
                c = distance([pg[0],pg[1],0], [basePos[0],basePos[1],0])
                if (None==retq) or (retq < c):
                    retq = c
        return retq
    def bodyAction(self):
        position = self._agent.getBodyProperty(("base_yaw",), "position")
        position = (position[0], position[1], 0)
        yaw = p.getJointState(self._agent._id, self._agent._jointName2Id["base_y_to_base_yaw"],self._agent._world.getSimConnection())[0]
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {}}
        needPark = False
        for hand in ["right", "left"]:
            if not isGraspValid(self._agent, hand):
                needPark = True
                freePos = getFreeTargetAroundAgent(self._agent, hand, self._corridors[hand])
                self._corridors[hand], controls = planArmMotion(self._agent, hand, freePos, self._corridors[hand], controls, maxspeed=0.015)
        if needPark:
            controls.update({"world_to_base_x": (position[0], 0, 1.0), "base_x_to_base_y": (position[1], 0, 1.0), "base_y_to_base_yaw": (yaw, 0, 1.0)})
            self._agent.applyRigidBodyControls([controls])
            return
        stillUprighting = False
        for hl, ups in [("hand_left_roll", self._agent.getBodyProperty(("hand_left_roll",), "uprighting")), ("hand_right_roll", self._agent.getBodyProperty(("hand_right_roll",), "uprighting"))]:
            if None == ups:
                continue
            aux = []
            for trajectorName in ups:
                if not LocationUpright(None).isThere(self._agent._world._pobjects[trajectorName]):
                    aux.append(trajectorName)
                    stillUprighting = True
            self._agent.setBodyProperty((hl,), "uprighting", aux)
        if stillUprighting:
            controls.update({"world_to_base_x": (position[0], 0, 1.0), "base_x_to_base_y": (position[1], 0, 1.0), "base_y_to_base_yaw": (yaw, 0, 1.0)})
            self._agent.applyRigidBodyControls([controls])
            return
        carryStuff = self._max(self._carryingRadius("left"), self._carryingRadius("right"))
        refPO = self._location
        if isinstance(refPO, Location):
            refPO = refPO._relatum
        aabbMin, aabbMax = refPO.getAABB(None)
        aabbMin = tuple([aabbMin[0] - 0.5, aabbMin[1] - 0.5, aabbMin[2]])
        aabbMax = tuple([aabbMax[0] + 0.5, aabbMax[1] + 0.5, aabbMax[2]])
        positionR = refPO.getBodyProperty((), "position")
        positionR = (positionR[0], positionR[1], 0)
        position = self._agent.getBodyProperty(("base_yaw",), "position")
        position = (position[0], position[1], 0)
        yaw = p.getJointState(self._agent._id, self._agent._jointName2Id["base_y_to_base_yaw"],self._agent._world.getSimConnection())[0]
        positionT = closestPointOnCuboid(aabbMin, aabbMax, position)
        d = vectorNormalize(vectorDifference(positionR, position))
        yawT = math.atan2(d[1], d[0])
        if None != carryStuff:
            positionD = vectorDifference(positionT, position)
            yawD = angleDifference(yawT, yaw)
            if 0.015 < vectorNorm(positionD):
                positionD = scaleVector(vectorNormalize(positionD), 0.015)
                positionT = translateVector(positionD,position)
            thresh = 0.015/carryStuff
            if thresh < abs(yawD):
                if 0 > yawD:
                    yawD = -thresh
                else:
                    yawD = thresh
                yawT = yaw + yawD
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {"world_to_base_x": (positionT[0], 0, 1.0), "base_x_to_base_y": (positionT[1], 0, 1.0), "base_y_to_base_yaw": (yawT, 0, 1.0)}}
        self._agent.applyRigidBodyControls([controls])
        return

class HandNear(Goal):
    def __init__(self,item,hand):
        super().__init__()
        self._item = item
        self._hand = hand
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[hand]
        self._agent = findByType(self._item._world, "agent")
        self._radius = self._item.getBodyProperty("fn", "graspingradius")*0.9
    def _strVarPart(self):
        return str(self._item) + ","+ str(self._hand)
    def _isFulfilled(self):
        posI = self._item.getBodyProperty((), "position")
        handle = self._item.getBodyProperty("fn", "handle")
        posH = self._agent.getBodyProperty((self._handLink,), "position")
        velocityR = self._item.getBodyProperty((), "linearVelocity")
        velocity = self._agent.getBodyProperty((self._handLink,), "linearVelocity")
        if self._item.getBodyProperty("fn", "pullabledoor"):
            handlePoint = self._item.getBodyProperty("fn", "handlepoint")
            orientationI = self._item.getBodyProperty((), "orientation")
            pullAxis = [self._radius*x for x in self._item.getBodyProperty("fn", "openingaxis")]
            posI = translateVector(posI, p.rotateVector(orientationI, translateVector(pullAxis, handlePoint)))
            ## TODO: recompute velocity too
        elif handle:
            posI = translateVector(posI, p.rotateVector(orientationI, handle))
        tracking = distance(velocityR, velocity) < 0.01
        ok = distance(posI, posH) < self._radius
        if not ok:
            grasping = self._agent.getBodyProperty((self._handLink,), "grasping")
            if self._item.getName() in grasping:
                grasping.remove(self._item.getName())
                self._agent.setBodyProperty((self._handLink,), "grasping", grasping)
        return ok and tracking
    def getThreats(self, processes):
        return [x for x in [HandReaching(self._item,self._hand)] if str(x) in processes]
    def suggestProcess(self):
        return HandReaching(self._item, self._hand)

class StoppedHand(Goal):
    def __init__(self, agent, hand):
        super().__init__()
        self._agent = agent
        self._hand = hand
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[hand]
    def _strVarPart(self):
        return str(self._agent) + "," + str(self._hand)
    def isFulfilled(self):
        linVel = self._agent.getBodyProperty((self._handLink,), "linearVelocity")
        angVel = self._agent.getBodyProperty((self._handLink,), "angularVelocity")
        d = math.sqrt(linVel[0]*linVel[0]+linVel[1]*linVel[1]+linVel[2]*linVel[2])
        noLin = 0.01 > d
        d = math.sqrt(angVel[0]*angVel[0]+angVel[1]*angVel[1]+angVel[2]*angVel[2])
        noAng = 0.01 > d
        return noLin and noAng
    def getThreats(self, processes):
        return [x for x in [StoppingHand(self._agent,self._hand)] if str(x) in processes]
    def suggestProcess(self):
        return StoppingHand(self._agent,self._hand)

class StoppingHand(BodyProcess):
    def __init__(self,agent,hand):
        super().__init__(coherence=[])
        self._agent = agent
        self._hand = hand
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[hand]
    def _strVarPart(self):
        return str(self._agent) + "," + str(self._hand)
    def _markForDeletionInternal(self, replacement=None):
        self._coherence.append(Unwill())
        return None
    def bodyAction(self):
        position = self._agent.getBodyProperty((self._handLink,), "position")
        orientation = self._agent.getBodyProperty((self._handLink,), "orientation")
        controls = getHandLinearJointControls(self._agent, self._hand, {"+constraints": [], "-constraints": [], "jointTargets": {}}, position, velocity=[0,0,0])
        controls = getHandAngularJointControls(self._agent, self._hand, controls, orientation, velocity=[0,0,0])
        self._agent.applyRigidBodyControls([controls])

class HandReaching(BodyProcess):
    def __init__(self, item, hand):
        super().__init__(coherence=[])
        self._item = item
        self._hand = hand
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[hand]
        self._agent = findByType(self._item._world, "agent")
        self._radius = self._item.getBodyProperty("fn", "graspingradius")*0.89
        self._corridor = None
    def _strVarPart(self):
        return str(self._item) + "," + str(self._hand)
    def _markForDeletionInternal(self, replacement=None):
        self._coherence.append(Unwill())
        stopHand(self._agent, self._hand)
        return None
    def bodyAction(self):
        boxes, allowableCollisionFn = getTrajectorBoxes(self._agent, self._handLink, tolerance=0)
        if (None == self._corridor) or ([] == self._corridor.waypoints) or (([] != self._corridor.waypoints) and (not validExtrusion(boxes, self._corridor.waypoints[-1], self._corridor.waypoints[-1], allowableCollisionFn, self._agent._world))):
            refPO = self._item
            if isinstance(refPO, Location):
                refPO = refPO._relatum
            whitelist = [refPO.getName()] + [x.getName() for x in closeObjects(refPO)]
            positionR = refPO.getBodyProperty((), "position")
            position = translateVector(positionR,[0,0,self._radius])
            handle = refPO.getBodyProperty("fn", "handle")
            if refPO.getBodyProperty("fn", "pullabledoor"):
                handlePoint = refPO.getBodyProperty("fn", "handlepoint")
                orientationR = refPO.getBodyProperty((), "orientation")
                pullAxis = [0.8*self._radius*x for x in refPO.getBodyProperty("fn", "openingaxis")]
                position = translateVector(positionR, p.rotateVector(orientationR, translateVector(pullAxis, handlePoint)))
            elif handle:
                orientationR = refPO.getBodyProperty((), "orientation")
                position = translateVector(positionR, p.rotateVector(orientationR, handle))
        else:
            position = self._corridor.waypoints[-1]
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {}}
        if 0 < len(self._agent.getBodyProperty((self._handLink,), "grasping")):
            self._corridor, controls = planArmMotion(self._agent, self._hand, position, self._corridor, controls,maxspeed=0.015)
        else:
            self._corridor, controls = planArmMotion(self._agent, self._hand, position, self._corridor, controls)
        self._agent.setBodyProperty((self._handLink,), "pullingopen", False)
        self._agent.applyRigidBodyControls([controls])
        return

class ItemNear(Goal):
    def __init__(self, item, location, hand=None):
        super().__init__()
        self._item = item
        self._location = location
        if None == hand:
            hand = "right"
        if Grasped(item,"left").isFulfilled():
            hand = "left"
        self._hand = hand
    def _strVarPart(self):
        return str(self._item) + "," + str(self._location) + "," + str(self._hand)
    def _isFulfilled(self):
        return self._location.isThere(self._item)
    def suggestProcess(self):
        return PlacingItem(self._item, self._location, self._hand)

def getHandTargetPose(targetPos, targetOr, positionO, orientationO, positionH, orientationH):
    ### Have (cr.) object in world, hand in world; need object in hand
    ### Toiw = Thiw*Toih => Toih = inv(Thiw)*Toiw
    posOiH = [a-b for a,b in zip(p.rotateVector((orientationH[0], orientationH[1], orientationH[2], -orientationH[3]), positionO), p.rotateVector((orientationH[0], orientationH[1], orientationH[2], -orientationH[3]), positionH))]
    orOiH = quaternionProduct((orientationH[0], orientationH[1], orientationH[2], -orientationH[3]), orientationO)
    ### have (targ.) object in world, object in hand; need (targ.) hand in world
    ### Toiw = Thiw*Toih => Thiw = Toiw*inv(Toih)
    ### inv(T)*T has 0 translation so -iT.q*T.t = iT.t
    posOiH = p.rotateVector((orOiH[0], orOiH[1], orOiH[2], -orOiH[3]), posOiH)
    posTH = [a+b for a,b in zip(targetPos, p.rotateVector(targetOr, (-posOiH[0], -posOiH[1], -posOiH[2])))]
    orTH = quaternionProduct(targetOr, (orOiH[0], orOiH[1], orOiH[2], -orOiH[3]))
    return posTH, orTH

def angleRemap(angle):
    while math.pi < angle:
        angle = angle - 2*math.pi
    while -math.pi > angle:
        angle = angle + 2*math.pi
    return angle

class PlacingItem(BodyProcess):
    def __init__(self, item, location, hand):
        super().__init__(coherence=[Grasped(item,hand), BaseNear(location)])
        self._item = item
        self._location = location
        self._hand = hand
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[self._hand]
        self._agent = findByType(self._item._world, "agent")
        self._corridor = None
    def _strVarPart(self):
        return str(self._item) + "," + str(self._location) + "," + str(self._hand)
    def _markForDeletionInternal(self, replacement=None):
        self._coherence.append(Unwill())
        stopHand(self._agent, self._hand)
        return None
    def bodyAction(self):
        ### location: locationAt, locationIn, locationUpright, locationOver, locationTippedOver
        location = self._location
        if isinstance(location, pobject.PObject) or isinstance(location, pobject.PObjectWrapper):
            location = LocationAt(location)
        targetPos, targetOr = location.suggestPose(self._item)
        positionO = self._item.getBodyProperty((), "position")
        orientationO = self._item.getBodyProperty((), "orientation")
        need2Upright = False
        if isinstance(location, LocationUpright):
            dq = quaternionProduct(targetOr, [orientationO[0], orientationO[1], orientationO[2], -orientationO[3]])
            _, angle = p.getAxisAngleFromQuaternion(dq)
            angle = angleRemap(angle)
            uprighting = self._agent.getBodyProperty((self._handLink,), "uprighting")
            if 0.1 < abs(angle):
                if self._item.getName() not in uprighting:
                    uprighting.append(self._item.getName())
                    self._agent.setBodyProperty((self._handLink,), "uprighting", uprighting)
                    need2Upright = True
            else:
                if self._item.getName() in uprighting:
                    uprighting.remove(self._item.getName())
                    self._agent.setBodyProperty((self._handLink,), "uprighting", uprighting)
        positionH = self._agent.getBodyProperty((self._handLink,), "position")
        orientationH = self._agent.getBodyProperty((self._handLink,), "orientation")
        ### Have (cr.) object in world, hand in world; need object in hand
        ### Toiw = Thiw*Toih => Toih = inv(Thiw)*Toiw
        posOiH = [a-b for a,b in zip(p.rotateVector((orientationH[0], orientationH[1], orientationH[2], -orientationH[3]), positionO), p.rotateVector((orientationH[0], orientationH[1], orientationH[2], -orientationH[3]), positionH))]
        orOiH = quaternionProduct((orientationH[0], orientationH[1], orientationH[2], -orientationH[3]), orientationO)
        ### have (targ.) object in world, object in hand; need (targ.) hand in world
        ### Toiw = Thiw*Toih => Thiw = Toiw*inv(Toih)
        ### inv(T)*T has 0 translation so -iT.q*T.t = iT.t
        posOiH = p.rotateVector((orOiH[0], orOiH[1], orOiH[2], -orOiH[3]), posOiH)
        posTH = [a+b for a,b in zip(targetPos, p.rotateVector(targetOr, (-posOiH[0], -posOiH[1], -posOiH[2])))]
        orTH = quaternionProduct(targetOr, (orOiH[0], orOiH[1], orOiH[2], -orOiH[3]))
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {}}
        _, angle = p.getAxisAngleFromQuaternion(quaternionProduct(orientationH, [orTH[0], orTH[1], orTH[2], -orTH[3]]))
        if 0.005 > abs(angleRemap(angle)):
            self._corridor, controls = planArmMotion(self._agent, self._hand, posTH, self._corridor, controls,maxspeed=0.015)
        if need2Upright:
            controls = getHandAngularJointControls(self._agent, self._hand, controls, orTH, velocity=[5,5,5])
        else:
            controls = getHandAngularJointControls(self._agent, self._hand, controls, orTH, velocity=[0,0,0])
        self._agent.applyRigidBodyControls([controls])
        return

class Accessible(Goal):
    def __init__(self, item):
        super().__init__()
        self._item = item
        self._handle = findContainerHandleAround(self._item, findalways=True)
    def _strVarPart(self):
        return str(self._item)
    def _isFulfilled(self):
        return (None == findContainerHandleAround(self._item))
    def getThreats(self, processes):
        if None == self._handle:
            return []
        return [x for x in [PullingOpen(self._handle,"left"), PullingOpen(self._handle,"right")] if str(x) in processes]
    def suggestProcess(self):
        return OpeningContainerAround(self._item)

class OpeningContainerAround(Process):
    def __init__(self, item):
        handle = findContainerHandleAround(item)
        super().__init__(coherence=[BaseNear(handle), HandNear(handle,"left"), PulledOpen(handle,"left")])
        self._item = item
    def _strVarPart(self):
        return str(self._item)

class PulledOpen(Goal):
    def __init__(self, handle, hand):
        super().__init__()
        self._handle = handle
        self._hand = hand
        self._agent = findByType(self._handle._world, "agent")
    def _strVarPart(self):
        return str(self._handle) + "," + str(self._hand)
    def _isFulfilled(self):
        pob = self._handle
        if isinstance(pob, pobject.PObjectWrapper):
            pob = pob._pobject
        ot = pob.getBodyProperty("", "type")
        if isSubclass(ot, "pullableDoor"):
            fn = pob.getBodyProperty("fn", "doorhandlelink")
            fnj = pob.getBodyProperty("fn", "doorhandlejoint")
            if fn:
                opMin = pob.getBodyProperty("fn", "openmin")
                opMax = pob.getBodyProperty("fn", "openmax")
                a = stubbornTry(lambda : p.getJointState(pob._id, pob._jointName2Id[fnj], pob._world.getSimConnection()))[0]
                if (a >= opMin) and (opMax >= a):
                    return True
                return False
        return True
    def suggestProcess(self):
        return PullingOpen(self._handle, self._hand)


class PullingOpen(BodyProcess):
    def __init__(self, handle, hand):
        super().__init__(coherence=[BaseNear(handle), HandNear(handle,hand)])
        self._handle = handle
        self._hand = hand
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[hand]
        self._agent = findByType(self._handle._world, "agent")
    def _strVarPart(self):
        return str(self._handle) + "," + str(self._hand)
    def _markForDeletionInternal(self, replacement=None):
        pulling = self._agent.getBodyProperty((self._handLink,), "pulling")
        if self._handle.getName() in pulling:
            pulling.remove(self._handle.getName())
        self._agent.setBodyProperty((self._handLink,), "pulling", pulling)
        self._agent.setBodyProperty((self._handLink,), "pullingopen", False)
        self._coherence.append(Unwill())
        return None
    def bodyAction(self):
        ### TODO
        pulling = self._agent.getBodyProperty((self._handLink,), "pulling")
        if self._handle.getName() not in pulling:
            pulling.append(self._handle.getName())
        self._agent.setBodyProperty((self._handLink,), "pulling", pulling)
        self._agent.setBodyProperty((self._handLink,), "pullingopen", True)
        positionH = self._handle.getBodyProperty((), "position")
        orientationH = self._handle.getBodyProperty((), "orientation")
        pullDirection = self._handle.getBodyProperty("fn", "openingaxis")
        graspRadius = self._handle.getBodyProperty("fn", "graspingradius")
        handlePoint = self._handle.getBodyProperty("fn", "handlepoint")
        refPoint = translateVector([graspRadius*x for x in pullDirection], handlePoint)
        positionTarget = translateVector(positionH, p.rotateVector(orientationH, refPoint))
        controls = getHandLinearJointControls(self._agent, self._hand, {"+constraints": [], "-constraints": [], "jointTargets": {}}, positionTarget, velocity=[0,0,0])
        self._agent.applyRigidBodyControls([controls])
        return None


class ProportionedItem(Goal):
    def __init__(self, item, quantity, destination):
        super().__init__()
        self._item = item
        self._portionTypeName = item.getBodyProperty("fn", "portiontype")
        self._portionType = item._world._particleTypes[self._portionTypeName]
        self._quantity = quantity
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._quantity) + "," + str(self._destination)
    def _isFulfilled(self):
        portions = findByType(self._item._world, self._portionTypeName, listAll=True)
        required = int(self._quantity/20)
        if 0 == required:
            required = 1
        return required <= len([x for x in portions if LocationIn(self._destination).isThere(x)])
    def getThreats(self, processes):
        if self._item.getBodyProperty("", "type") in ["butter"]:
            tool = findByType(self._item._world, "Knife",True)
            retq = [x for x in [GraspingItem(tool,"left"), GraspingItem(tool,"right")] if str(x) in processes]
            return retq
        return [x for x in [PouringInto(self._item,self._destination)] if str(x) in processes]
    def suggestProcess(self):
        if self._item.getBodyProperty("", "type") in ["butterS"]:
            tool = findByType(self._item._world, "Knife")
            retq = ProportioningByChopping(self._item, tool, self._quantity, self._destination)
        else:
            retq = ProportioningByPouring(self._item, self._quantity, self._destination)
        return retq

class ProportioningByChopping(Process):
    def __init__(self, item, tool, quantity, destination):
        super().__init__(coherence=[ItemOnCounter(item), ItemOnCounter(destination), Grasped(tool, "right"), BaseNear(item), ChoppedPortionExists(item, tool, quantity,destination), PreviousPortionAtDest(item, destination)])
        self._item = item
        self._tool = tool
        self._quantity = quantity
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._tool) + "," + str(self._quantity) + "," + str(self._destination)

class ChoppedPortionExists(Goal):
    def __init__(self, item, tool, quantity, destination):
        super().__init__()
        self._item = item
        self._portionTypeName = item.getBodyProperty("fn", "portiontype")
        self._portionType = item._world._particleTypes[self._portionTypeName]
        self._tool = tool
        self._quantity = quantity
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._quantity) + "," + str(self._destination)
    def _isFulfilled(self):
        portions = findByType(self._item._world, self._portionTypeName, listAll=True)
        required = int(self._quantity/20)
        if 0 == required:
            required = 1
        return required <= len(portions)
    def suggestProcess(self):
        return Chopping(self._item, self._tool, self._quantity, self._destination)

class Chopping(BodyProcess):
    def __init__(self, item, tool, quantity, destination):
        super().__init__(coherence=[PreviousPortionAtDestination(item,destination)])
        self._item = item
        self._tool = tool
        self._quantity = quantity
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._tool) + "," + str(self._quantity) + "," + str(self._destination)
    def bodyAction(self):
        ### TODO
        return

class PreviousPortionAtDest(Goal):
    def __init__(self, item, destination):
        super().__init__()
        self._item = item
        self._portionTypeName = item.getBodyProperty("fn", "portiontype")
        self._portionType = item._world._particleTypes[self._portionTypeName]
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._destination)
    def _isFulfilled(self):
        portions = findByType(self._item._world, self._portionTypeName, listAll=True)
        return all([LocationIn(self._destination).isThere(x) for x in portions])
    def suggestProcess(self):
        portions = [x for x in findByType(self._item._world, self._portionTypeName, listAll=True) if not LocationIn(self._destination).isThere(x)]
        if 0 == len(portions):
            return None
        minD = None
        minK = -1
        posD = self._destination.getBodyProperty((), "position")
        for k, portion in enumerate(portions):
            pos = portion.getBodyProperty((), "position")
            d = [a-b for a,b in zip(posD,pos)]
            d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
            if (None == minD) or (d < minD):
                minD = d
                minK = k
        return ItemNear(portions[minK],LocationIn(self._destination),"left")

class ProportioningByPouring(Process):
    def __init__(self,item, quantity, destination):
        super().__init__(coherence=[ItemOnCounter(destination), Grasped(item,"right"), PouredPortionExists(item,quantity,destination)])
        self._item = item
        self._quantity = quantity
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._quantity) + "," + str(self._destination)
        

class PouredPortionExists(Goal):
    def __init__(self, item, quantity, destination):
        super().__init__()
        self._item = item
        self._portionTypeName = item.getBodyProperty("fn", "portiontype")
        self._portionType = item._world._particleTypes[self._portionTypeName]
        self._quantity = quantity
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._quantity) + "," + str(self._destination)
    def _isFulfilled(self):
        portions = findByType(self._item._world, self._portionTypeName, listAll=True)
        required = int(self._quantity/20)
        if 0 == required:
            required = 1
        return required <= len([x for x in portions if LocationIn(self._destination).isThere(x)])
    def suggestProcess(self):
        return PouringInto(self._item, self._destination)

class PouringInto(Process):
    def __init__(self,item, destination):
        super().__init__(coherence=[Grasped(item,"right"),BaseNear(destination),ItemNear(item,LocationOver(destination)),ItemNear(item,LocationTippedOver(destination))])
        self._item = item
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._destination)
    def _markForDeletionInternal(self, replacement=None):
        return StoppedPouring(self._item)

class StoppedPouring(Goal):
    def __init__(self, item):
        super().__init__()
        self._item = item
    def _strVarPart(self):
        return str(self._item)
    def _isFulfilled(self):
        orientation = self._item.getBodyProperty((), "orientation")
        vertical = p.rotateVector(orientation, [0,0,1])
        return 0.98 < vertical[2]
    def suggestProcess(self):
        return StoppingPouring(self._item)

class StoppingPouring(Process):
    def __init__(self, item):
        super().__init__(coherence=[ItemNear(item, LocationUpright(item)), ItemOnLocation(item,item._world._pobjects['kitchenCabinet'])])
        self._item = item
    def _strVarPart(self):
        return str(self._item)

class TransferredContents(Goal):
    def __init__(self, item, destination):
        super().__init__()
        self._item = item
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._destination)
    def _isFulfilled(self):
        return 0 == len(getContents(self._item,brds=[0.03,0.03,0.03]))
    def getThreats(self, processes):
        return [x for x in [PouringContents(self._item,self._destination)] if str(x) in processes]
    def suggestProcess(self):
        return TransferByPouring(self._item,self._destination)

class TransferByPouring(Process):
    def __init__(self, item, destination):
        super().__init__(coherence=[ItemOnCounter(destination), Grasped(item,"right"), PouredContents(item, destination)])
        self._item = item
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._destination)

class PouredContents(Goal):
    def __init__(self, item, destination):
        super().__init__()
        self._item = item
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._destination)
    def _isFulfilled(self):
        return 0 == len(getContents(self._item,brds=[0.03,0.03,0.03]))
    def getThreats(self, processes):
        return [x for x in [PouringContents(self._item,self._destination)] if str(x) in processes]
    def suggestProcess(self):
        return PouringContents(self._item, self._destination)

class PouringContents(Process):
    def __init__(self, item, destination):
        super().__init__(coherence=[ItemOnCounter(destination), Grasped(item,"right"), BaseNear(destination), ItemNear(item,LocationOver(destination)), ItemNear(item,LocationTippedOver(destination))])
        self._item = item
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._destination)
    def _markForDeletionInternal(self, replacement=None):
        return StoppedPouring(self._item)

def getMixedSubstance(item):
    substances = set([getSubstance(x) for x in getContents(item)])
    sweetbutter = set(['sugar', 'butter'])
    if substances == sweetbutter:
        return "sweetbutter"
    return ""

class MixedContents(Goal):
    def __init__(self, item, tool, substance):
        super().__init__()
        self._item = item
        self._substance = substance
        self._tool = tool
    def _strVarPart(self):
        return str(self._item) + "," + str(self._substance)
    def _isFulfilled(self):
        return all([(self._substance == getSubstance(x)) for x in getContents(self._item)])
    def getThreats(self, processes):
        return [x for x in [MixingContents(self._item, self._tool, self._substance)] if str(x) in processes]
    def suggestProcess(self):
        return MixingContents(self._item, self._tool, self._substance)

class MixingContents(Process):
    def __init__(self, item, tool, substance):
        super().__init__(coherence=[Grasped(tool,"right"), BaseNear(item), MixedStuff(item, tool, substance)])
        self._item = item
        self._tool = tool
        self._substance = substance
    def _strVarPart(self):
        return str(self._item) + "," + str(self._tool) + "," + str(self._substance)

class MixedStuff(Goal):
    def __init__(self, item, tool, substance):
        super().__init__()
        self._item = item
        self._tool = tool
        self._substance = substance
    def _strVarPart(self):
        return str(self._item) + "," + str(self._tool) + "," + str(self._substance)
    def _isFulfilled(self):
        return all([(self._substance == getSubstance(x)) for x in getContents(self._item)])
    def suggestProcess(self):
        return MixingStuff(self._item, self._tool, "right")

class MixingStuff(BodyProcess):
    def __init__(self, item, tool, hand):
        super().__init__(coherence=[Grasped(tool), BaseNear(item)])
        self._item = item
        self._tool = tool
        self._hand = hand
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[hand]
    def _strVarPart(self):
        return str(self._item) + "," + str(self._tool)
    def bodyAction(self):
        ### TODO
        refpt = self._tool.getBodyProperty("fn", "refpt")
        refaxis = self._tool.getBodyProperty("fn", "refaxis")
        position = self._tool.getBodyProperty((), "position")
        orientation = self._tool.getBodyProperty((), "orientation")
        positionI = self._item.getBodyProperty((), "position")
        positionH = self._agent.getBodyProperty((self._handLink,), "position")
        orientationH = self._agent.getBodyProperty((self._handLink,), "orientation")
        refpt = translateVector(position, p.rotateVector(orientation, refpt))
        refaxis = p.rotateVector(orientation, refaxis)
        dist = distance(refpt, positionI)
        refptXY = [refpt[0],refpt[1],0]
        positionIXY = [positionI[0],positionI[1],0]
        xydist = distance(refptXY,positionIXY)
        _, targetOr = LocationUpright().suggestPose(self._tool)
        if -0.98 < refaxis[2]:
            targetPos = [position[0], position[1], 1]
        elif 0.01 < xydist:
            targetPos = [positionI[0], positionI[1], position[2]]
        elif 0.01 < dist:
            inc = -0.002
            targetPos = [positionI[0], positionI[1], position[2]+inc]
        else:
            targetPos = [positionI[0], positionI[1], position[2]]
            targetOr = quaternionProduct(p.getQuaternionFromAxisAngle([0,0,1], math.pi/180.0), orientation)
        positionH, orientationH = getHandTargetPose(targetPos, targetOr, position, orientation, positionH, orientationH)
        controls = getHandLinearJointControls(self._agent, self._hand, {"+constraints": [], "-constraints": [], "jointTargets": {}}, positionH, velocity=[0,0,0])
        controls = getHandAngularJointControls(self._agent, self._hand, controls, orientationH, velocity=[0,0,0])
        self._agent.applyRigidBodyControls([controls])
        return

class CombinedStuffInto(Goal):
    def __init__(self, itema, itemb, destination, substance):
        super().__init__()
        self._itema = itema
        self._itemb = itemb
        self._destination = destination
        self._substance = substance
    def _strVarPart(self):
        return str(self._itema) + "," + str(self._itemb) + "," + str(self._destination) + "," + str(self._substance)
    def _isFulfilled(self):
        return (0 == len(getContents(self._itema))) and (0 == len(getContents(self._itemb))) and (all([(self._substance == getSubstance(x)) for x in getContents(self._destination)]))
    def suggestProcess(self):
        return CombiningStuffInto(self._itema, self._itemb, self._destination, self._substance)

class CombiningStuffInto(Goal):
    def __init__(self, itema, itemb, destination, substance):
        super().__init__(coherence=[TransferredContents(itema, destination), TransferredContents(itemb, destination), MixedContents(destination, substance)])
        self._itema = itema
        self._itemb = itemb
        self._destination = destination
        self._substance = substance
    def _strVarPart(self):
        return str(self._itema) + "," + str(self._itemb) + "," + str(self._destination) + "," + str(self._substance)

