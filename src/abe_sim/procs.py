import math
import random
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
    dimsAdj = [0.05,0.05,0.05]
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
        auxMin = [suppMin[0]+dimsAdj[0]*c[1]-dims[0], suppMin[1]+dimsAdj[1]*c[2]-dims[1], suppMax[2]-dims[2]+0.02]
        auxMax = [suppMin[0]+dimsAdj[0]*c[1]+dims[0], suppMin[1]+dimsAdj[1]*c[2]+dims[1], suppMax[2]+dims[2]+0.02]
        if 0 == len(set(overlappingObjectNames(auxMin, auxMax, world)).difference(whitelist)):
            px = [suppMin[0] + c[1]*dimsAdj[0], suppMin[1] + c[2]*dimsAdj[1], suppMin[2]]
            #print("cfpc ret", c, "%0.3f %0.3f %0.3f"%(px[0],px[1],px[2]))
            #print("    inipos", "%0.3f %0.3f %0.3f"%(initialPos[0],initialPos[1],initialPos[2]), initialPosK)
            #rint("    auxcnd", "%0.3f %0.3f %0.3f | %0.3f %0.3f %0.3f"%(auxMin[0],auxMin[1],auxMin[2], auxMax[0],auxMax[1],auxMax[2]))
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
    orH = quaternionProduct((orientationB[0], orientationB[1], orientationB[2], -orientationB[3]), orHTarget)
    euler = p.getEulerFromQuaternion(orH)
    joints = {"left": ("hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"), "right": ("hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll")}[hand]
    controls["jointTargets"].update({joints[k]: (euler[k], velocity[k], 1.0) for k in [0,1,2]})
    return controls

def getTrajectorBoxes(agent, handLink, tolerance=0.01):
    tolerance = abs(tolerance)
    world = agent._world
    position = agent.getBodyProperty((handLink,), "position")
    graspedNames = agent.getBodyProperty((handLink,), "grasping")
    whitelist = ["abe"] + graspedNames
    boxes = [agent.getAABB((handLink,))] + [world._pobjects[x].getAABB(None) for x in graspedNames]
    def aux(box):
        aabbMin, aabbMax = box
        aabbMin = translateVector(vectorDifference(aabbMin, position), [-tolerance]*3)
        aabbMax = translateVector(vectorDifference(aabbMax, position), [tolerance]*3)
        return aabbMin, aabbMax
    boxes = [aux(x) for x in boxes]
    allowableCollisionFn = lambda box, collidingObjects: allowCollisionByWhitelist(box, collidingObjects, whitelistNames=whitelist, whitelistTypes=["sugarparticle", "butterparticle", "particle"])
    return boxes, allowableCollisionFn

def isGraspValid(agent, hand):
    handLink = {"right": "hand_right_roll", "left": "hand_left_roll"}[hand]
    handPos = agent.getBodyProperty((handLink,), "position")
    trajectorBoxes, allowableCollisionFn = getTrajectorBoxes(agent, handLink, tolerance=0.04)
    return validExtrusion(trajectorBoxes, handPos, handPos, allowableCollisionFn, agent._world)

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
        #print("blockage", k, bm, bM, [(o.getName(),o.getAABB(None)) for o in ovs])
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
    if None != corridor:
        corridors = corridor.validCorridors(boxes, allowableCollisionFn, world)
        if (None != corridors[0]) and (corridors[0] == corridors[1]):
            connected = True
    #print("    pam ue")
    connected, corridors[0] = updateCorridor(boxes, corridors[0], connected, targetPos, allowableCollisionFn, world, atEnd=True)
    position = agent.getBodyProperty((handLink,), "position")
    #print("    pam us")
    connected, corridors[1] = updateCorridor(boxes, corridors[1], connected, position, allowableCollisionFn, world, atEnd=False)
    #print("    pam on")
    if connected:
        corridor = corridors[0]
    else:
        sampleBox = {"left": [[-0.85,-1.4,-0.9], [2,1.85,2]], "right": [[-0.85,-1.85,-0.9], [2,1.4,2]]}[hand]
        armBase = {"left": (0,0.4,0.95), "right": (0,-0.4,0.95)}[hand]
        positionBase = agent.getBodyProperty(("base_yaw",), "position")
        orientationBase = agent.getBodyProperty(("base_yaw",), "orientation")
        positionArmBase = translateVector(positionBase, p.rotateVector(orientationBase, armBase))
        #print(corridors[0].waypoints, corridors[1].waypoints)
        corridor = planCorridor(sampleBox, boxes, allowableCollisionFn, world, corridors[0], corridors[1], positionArmBase, orientationBase)
    if None != corridor:
        nextPosition, _ = corridor.nextAlong(position)
        #if None == nextPosition:
        #    #print("HUHUHU")
        if None != nextPosition:
            diff = vectorDifference(nextPosition, position)
            if maxspeed < vectorNorm(diff):
                nextPosition = translateVector(position, scaleVector(vectorNormalize(diff), maxspeed))
            controls = getHandLinearJointControls(agent, hand, controls, nextPosition, velocity=[0,0,0])
    return corridor, controls

def getFreeTargetAroundAgent(agent, hand, corridor, samples=100):
    world = agent._world
    handLink = {"right": "hand_right_roll", "left": "hand_left_roll"}[hand]
    boxes, allowableCollisionFn = getTrajectorBoxes(agent, handLink, tolerance=0)
    if None != corridor:
        if validExtrusion(boxes, corridor.waypoints[-1], corridor.waypoints[-1], allowableCollisionFn, world):
            return corridor.waypoints[-1]
    sampleBox = handSampleBox(agent, hand, (-0.35, -0.35, -0.35), (0.35, 0.35, 0.35))
    while 0 < samples:
        sample = [random.uniform(x,y) for x,y in zip(sampleBox[0], sampleBox[1])]
        if validExtrusion(boxes, sample, sample, allowableCollisionFn, world):
            return sample
        samples = samples - 1
    return None

def stopHand(agent, hand):
    js = agent.getJointStates()
    controls = {"+constraints": [], "-constraints": [], "jointTargets": {}}
    for jn in {"left": ["hand_left_base_to_hand_left_x", "hand_left_x_to_hand_left_y", "hand_left_y_to_hand_left_z", "hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"], "right": ["hand_right_base_to_hand_right_x", "hand_right_x_to_hand_right_y", "hand_right_y_to_hand_right_z", "hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll"]}[hand]:
        controls["jointTargets"][jn] = (js[jn][0],0,1)
    agent.applyRigidBodyControls([controls])

def getContents(item):
    aabbMin, aabbMax = item.getAABB(None)
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
    subclassOf = {"fridge": ["fridge", "container"], "oven": ["oven", "container"]}
    return (cs in subclassOf) and (cS in subclassOf[cs])

def findContainerHandleAround(item):
    objs = closeObjects(item)
    retq = None
    for o in objs:
        ot = o.getBodyProperty("", "type")
        if isSubclass(ot, "container"):
            fn = o.getBodyProperty("fn", "doorhandlelink")
            fnj = o.getBodyProperty("fn", "doorhandlejoint")
            if fn:
                opMin = o.getBodyProperty("fn", "openmin")
                opMax = o.getBodyProperty("fn", "openmax")
                doing = True
                a = stubbornTry(lambda : p.getJointState(o._id, o._jointName2Id[fnj], o._world.getSimConnection()))[0]
                if (a < opMin) or (opMax < a):
                    retq = pobject.PObjectWrapper(o, fn, fnj)
                    break
    return retq

def isGrasping(agent, handLink, item, radius):
    grasping = agent.getBodyProperty((handLink,), "grasping")
    if None == grasping:
        return False
    if item.getName() in grasping:
        posI = item.getBodyProperty((), "position")
        posH = agent.getBodyProperty((handLink,), "position")
        d = [a-b for a,b in zip(posI,posH)]
        d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        return d < radius
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
        return 0.95 < vertical[2]
    def suggestPose(self, trajector):
        position = trajector.getBodyProperty((), "position")
        orientation = trajector.getBodyProperty((), "orientation")
        adjVert = p.rotateVector(orientation, [0,0,1])
        if 0.95 > adjVert[2]:
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
        aabbMin = tuple([aabbMin[0], aabbMin[1], aabbMin[2]-0.05])
        retq = overlappingObjects(aabbMin, aabbMax, trajector._world)
        if isinstance(self._relatum, pobject.PObjectWrapper):
            return self._relatum._pobject in retq
        return self._relatum in retq
    def suggestPose(self, trajector):
        aabbMin, aabbMax = trajector.getAABB(None)
        dims = [(a-b)/2.0 for a,b in zip(aabbMax,aabbMin)]
        position = trajector.getBodyProperty((), "position")
        _, orientation = LocationUpright(trajector).suggestPose(trajector)
        supportBBs = self._relatum.getBodyProperty("fn", "supportbbs")
        if None == supportBBs:
            supportBBs = [self._relatum.getAABB(None)]
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
        for supp in supportBBs:
            suppMin = [supp[0][0]+dims[0], supp[0][1]+dims[1], supp[1][2]+dims[2]]
            suppMax = [supp[1][0]-dims[0], supp[1][1]-dims[1], supp[1][2]+dims[2]+0.02]
            candidate = closestFreePointOnCuboid(self._relatum._world, trajector.getName(), ["abe",trajector.getName()], suppMin, suppMax, position, dims)
            if None == candidate:
                continue
            d = distance(candidate, position)
            if (None == minD) or (d < minD):
                minD = d
                bestPos = candidate
        bestPos = list(bestPos)
        return bestPos, orientation

class LocationIn(LocationAt):
    def __dummy__(self):
        return None

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
        offset = dimsR[2]+0.4
        positionR = self._relatum.getBodyProperty((), "position")
        refPtTarget = list(positionR)
        refPtTarget[2] = refPtTarget[2] + offset
        positionT = trajector.getBodyProperty((), "position")
        orientationT = trajector.getBodyProperty((), "orientation")
        refPt = trajector.getBodyProperty("fn", "refpt")
        if None == refPt:
            refPt = [0,0,0]
        #if not self.isThere(trajector):
        #    _, orientation = LocationUpright(trajector).suggestPose(trajector)
        #    dq = quaternionProduct(orientation,[orientationT[0], orientationT[1], orientationT[2], -orientationT[3]])
        #    rT = p.rotateVector(orientationT, refPt)[2]
        #    rO = p.rotateVector(orientation, refPt)[2]
        #    if rT < rO:
        #        offset = offset + rO - rT
        #else:
        #    orientation = orientationT
        orientation = orientationT
        # Have refPt in object, refPt in world; need object in world
        # Triw = Toiw*Trio => Toiw = Triw*inv(Trio)
        bestPos = [a-b for a,b in zip(refPtTarget, p.rotateVector(orientation, refPt))]
        #print("SUGGESTED vs ACTUAL", bestPos, positionT)
        return bestPos, orientation

class LocationTippedOver(Location):
    def isThere(self, trajector):
        orientation = trajector.getBodyProperty((), "orientation")
        dripAxis = trajector.getBodyProperty("fn", "dripaxis")
        if None == dripAxis:
            dripAxis = [0,1,0]
        tipped = -0.95 > p.rotateVector(orientation, dripAxis)[2]
        return LocationOver(self._relatum).isThere and tipped
    def suggestPose(self, trajector):
        positionO, orientationO = LocationOver(self._relatum).suggestPose(trajector)
        dripAxis = trajector.getBodyProperty("fn", "dripaxis")
        if None == dripAxis:
            dripAxis = [0,1,0]
        dripAdj = p.rotateVector(orientationO, dripAxis)
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
        self._armBases = {"left": (0,0.4,0.95), "right": (0,-0.4,0.95)}
    def _strVarPart(self):
        return str(self._agent)
    def _armParked(self, arm):
        joints = self._joints[arm]
        velT = all([0.001 > abs(p.getJointState(self._agent._id, self._agent._jointName2Id[j],self._agent._world.getSimConnection())[1]) for j in joints[0]])
        velQ = all([0.001 > abs(p.getJointState(self._agent._id, self._agent._jointName2Id[j],self._agent._world.getSimConnection())[1]) for j in joints[1]])
        atT = all([0.15 > abs(p.getJointState(self._agent._id, self._agent._jointName2Id[j],self._agent._world.getSimConnection())[0]) for j in joints[0]])
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
        #print([self._armParked(x) for x in ["left", "right"]])
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
        self._corridors["right"], controls = planArmMotion(self._agent, "right", translateVector(positionBase, p.rotateVector(orientationBase, [0,-0.4,0.95])), self._corridors["right"], controls)
        self._corridors["left"], controls = planArmMotion(self._agent, "left", translateVector(positionBase, p.rotateVector(orientationBase, [0,0.4,0.95])), self._corridors["left"], controls)
        js = self._agent.getJointStates()
        sumJ = 0.0
        for j in self._tjoints:
            sumJ = sumJ + abs(js[j][0])
        if 0.1 > sumJ:
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
        if self._item.getName() in grasping:
            grasping.remove(self._item.getName())
        self._agent.setBodyProperty((self._handLink,), "grasping", grasping)
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
        return ItemOnCounter(self._item)

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
            refPO = refPO._relatum
        aabbMin, aabbMax = refPO.getAABB(None)
        aabbMin = tuple([aabbMin[0] - 1, aabbMin[1] - 1, -1])
        aabbMax = tuple([aabbMax[0] + 1, aabbMax[1] + 1, 1])
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
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {}}
        for hand in ["right", "left"]:
            if not isGraspValid(self._agent, hand):
                self._corridors[hand], controls = planArmMotion(self._agent, hand, getFreeTargetAroundAgent(self._agent, hand, self._corridors[hand]), self._corridors[hand], controls, maxspeed=0.015)
        if controls["jointTargets"]:
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
        position = self._agent.getBodyProperty(("base_yaw",), "position")
        positionR = (positionR[0], positionR[1], 0)
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
        posH = self._agent.getBodyProperty((self._handLink,), "position")
        velocityR = self._item.getBodyProperty((), "linearVelocity")
        velocity = self._agent.getBodyProperty((self._handLink,), "linearVelocity")
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
        #print("BEF",self._corridor)
        boxes, allowableCollisionFn = getTrajectorBoxes(self._agent, self._handLink, tolerance=0)
        if (None == self._corridor) or (([] != self._corridor.waypoints) and (not validExtrusion(boxes, self._corridor.waypoints[-1], self._corridor.waypoints[-1], allowableCollisionFn, self._agent._world))):
            refPO = self._item
            if isinstance(refPO, Location):
                refPO = refPO._relatum
            whitelist = [refPO.getName()] + [x.getName() for x in closeObjects(refPO)]
            positionR = refPO.getBodyProperty((), "position")
            position = translateVector(positionR,[0,0,self._radius])
            #print("HUH", validExtrusion(boxes, position, position, allowableCollisionFn, self._agent._world,debug=True))
        else:
            position = self._corridor.waypoints[-1]
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {}}
        if 0 < len(self._agent.getBodyProperty((self._handLink,), "grasping")):
            self._corridor, controls = planArmMotion(self._agent, self._hand, position, self._corridor, controls,maxspeed=0.015)
        else:
            self._corridor, controls = planArmMotion(self._agent, self._hand, position, self._corridor, controls)
        #print("AFT",self._corridor.waypoints)
        #if 0 < len(self._agent.getBodyProperty((self._handLink,), "grasping")):
        #    print("HRVpick", vectorNorm(self._agent.getBodyProperty(("hand_right_roll",),"linearVelocity")))
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
        if 0.01 > abs(angleRemap(angle)):
            #if None != self._corridor:
            #    print("pal", str(self), self._hand, distance(posTH, positionH), self._corridor.waypoints)
            #else:
            #    print("pal", str(self), self._hand, distance(posTH, positionH), None)
            #print("(%0.3f %0.3f %0.3f) (%0.3f %0.3f %0.3f) (%0.3f %0.3f %0.3f) (%0.3f %0.3f %0.3f)" % (targetPos[0], targetPos[1], targetPos[2], positionO[0], positionO[1], positionO[2], posTH[0], posTH[1], posTH[2], positionH[0], positionH[1], positionH[2]))
            #print(str(location))
            self._corridor, controls = planArmMotion(self._agent, self._hand, posTH, self._corridor, controls,maxspeed=0.015)
            #print("HRVplace", vectorNorm(self._agent.getBodyProperty(("hand_right_roll",),"linearVelocity")))
            #print(self._corridor, controls)
        #else:
        #    #print("ANGLE",angle)
        controls = getHandAngularJointControls(self._agent, self._hand, controls, orTH, velocity=[0,0,0])
        self._agent.applyRigidBodyControls([controls])
        return

class Accessible(Goal):
    def __init__(self, item):
        super().__init__()
        self._item = item
    def _strVarPart(self):
        return str(self._item)
    def _isFulfilled(self):
        return (None == findContainerHandleAround(self._item))
    def getThreats(self, processes):
        return [x for x in [PullingOpen(self._item,"left"), PullingOpen(self._item,"right")] if str(x) in processes]
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
        self._agent = findByType(self._item._world, "agent")
    def _strVarPart(self):
        return str(self._handle) + "," + str(self._hand)
    def _isFulfilled(self):
        pob = self._handle
        if isinstance(pob, pobject.PObjectWrapper):
            pob = pob._pobject
        ot = pob.getBodyProperty("", "type")
        if isSubclass(ot, "container"):
            fn = pob.getBodyProperty("fn", "doorhandlelink")
            fnj = pob.getBodyProperty("fn", "doorhandlejoint")
            if fn:
                opMin = pob.getBodyProperty("fn", "openmin")
                opMax = pob.getBodyProperty("fn", "openmax")
                a = stubbornTry(lambda : p.getJointState(pob._id, pob._jointName2Id[fnj], pob._world.getSimConnection()))[0]
                if (a >= opMin) or (opMax >= a):
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
        self._agent = findByType(self._handle._world, "agent")
    def _strVarPart(self):
        return str(self._handle) + "," + str(self._hand)
    def _markForDeletionInternal(self, replacement=None):
        self._coherence.append(Unwill())
        return None
    def bodyAction(self):
        ### TODO
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
        #print("POURINGPROP threats", [str(x) for x in [PouringInto(self._item,self._destination)] if str(x) in processes])
        #print("    ", processes)
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
        super().__init__(coherence=[ItemOnCounter(destination), Grasped(item,"right"), BaseNear(destination), PouredPortionExists(item,quantity,destination)])
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
        #return required <= len(portions)
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
        return 0.95 < vertical[2]
    def suggestProcess(self):
        return StoppingPouring(self._item)

class StoppingPouring(Process):
    def __init__(self, item):
        super().__init__(coherence=[ItemNear(item, LocationUpright(item)), ItemOnCounter(item)])
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
        return 0 == len(getContents(self._item))
    def getThreats(self, processes):
        return [x for x in [PouringContents(self._item,self._destination)] if str(x) in processes]
    def suggestProcess(self):
        return TransferByPouring(self._item,self._destination)

class TransferByPouring(Process):
    def __init__(self, item, destination):
        super().__init__(coherence=[ItemOnCounter(destination), Grasped(item,"right"), BaseNear(destination), PouredContents(item, destination)])
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
        return 0 == len(getContents(self._item))
    def getThreats(self, processes):
        return [x for x in [PouringContents(self._item,self._destination)] if str(x) in processes]
    def suggestProcess(self):
        return PouringContents(self._item, self._destination)

class PouringContents(Process):
    def __init__(self, item, destination):
        super().__init__(coherence=[ItemOnCounter(destination), Grasped(item,"right"), ItemNear(item,LocationOver(destination)), ItemNear(item,LocationTippedOver(destination))])
        self._item = item
        self._destination = destination
    def _strVarPart(self):
        return str(self._item) + "," + str(self._destination)
    def _markForDeletionInternal(self, replacement=None):
        return StoppedPouring(self._item)

class MixedContents(Goal):
    def __init__(self, item, substance):
        super().__init__()
        self._item = item
        self._substance = substance
        self._tool = findByType(self._item._world, "Whisk")
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
        super().__init__(coherence=[Grasped(tool), BaseNear(item), MixedStuff(item, tool, substance)])
        self._item = item
        self._tool = tool
        self._substance = substance
    def _strVarPart(self):
        return str(self._item) + "," + str(self._tool) + "," + str(self._substance)
    def _markForDeletionInternal(self, replacement=None):
        return ItemOnCounter(self._tool)

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
        return MixingStuff(self._item, self._tool)

class MixingStuff(BodyProcess):
    def __init__(self, item, tool):
        super().__init__(coherence=[Grasped(tool), BaseNear(item)])
        self._item = item
        self._tool = tool
    def _strVarPart(self):
        return str(self._item) + "," + str(self._tool)
    def bodyAction(self):
        ### TODO
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

