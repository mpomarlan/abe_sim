import math
import pybullet as p
from abe_sim.garden import Goal, Process, BodyProcess
import abe_sim.pobject as pobject

locationKnowledge = {("countertop","mediumbowl"): [[0,4.561,0.76], [-0.55,4.561,0.76], [-1.1,4.561,0.76], [0.55,4.561,0.76]], ("countertop","sugarbag"): [[0,5.055,0.76], [-0.55,5.055,0.76], [-1.1,5.055,0.76], [0.55,5.055,0.76]], ("countertop","butterbag"): [[0,5.055,0.76], [-0.55,5.055,0.76], [-1.1,5.055,0.76], [0.55,5.055,0.76]]}

def overlappingObjects(aabbMin, aabbMax, pybulletConnection):
    retq = p.getOverlappingObjects(aabbMin, aabbMax, pybulletConnection)
    if None == retq:
        retq = []
    return retq

def closestPointOnCuboid(aabbMin, aabbMax, position):
    x, y, z = position
    xm, ym, zm = aabbMin
    xM, yM, zM = aabbMax
    return (min(xM, max(xm, x)), min(yM, max(ym, y)), min(zM, max(zm, z)))

def pickKnownLocation(relatum, trajector):
    position = trajector.getBodyProperty((), "position")
    aabbMin, aabbMax = trajector.getAABB(None)
    dims = [(a-b)/(2.0) for a,b in zip(aabbMax, aabbMin)]
    relT = relatum.getBodyProperty("", "type")
    trajT = trajector.getBodyProperty("", "type")
    if (relT,trajT) in locationKnowledge:
        candidates = locationKnowledge[(relT,trajT)]
        minD = None
        retq = None
        for c in candidates:
            aabbMinC = [a-b for a,b in zip(c,dims)]
            aabbMaxC = [a+b for a,b in zip(c,dims)]
            if 0 < len(set([trajector._world.getPObjectById(x[0]).getName() for x in overlappingObjects(aabbMinC, aabbMaxC, trajector._world.getSimConnection())]).difference(["abe",trajector.getName(),relatum.getName()])):
                continue
            d = [a-b for a,b in zip(c,position)]
            d = math.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2])
            if (None == minD) or (d < minD):
                minD = d
                retq = list(c)
        d = [a-b for a,b in zip(retq,position)]
        d = math.sqrt(d[0]*d[0]+d[1]*d[1])
        if 1.5*min(dims[0:2]) < d:
            #print("WOOOOOO",("%.3f"%d),("[%.3f, %.3f, %.3f]"%(retq[0],retq[1],retq[2])),("[%.3f, %.3f, %.3f]"%(position[0],position[1],position[2])))
            retq[2] = position[2]
        #print("SELECTED",("[%.3f, %.3f, %.3f]"%(retq[0],retq[1],retq[2])),("[%.3f, %.3f, %.3f]"%(position[0],position[1],position[2])))
        return retq
    else:
        aabbMinR, aabbMaxR = relatum.getAABB(None)
        return closestPointOnCuboid(aabbMinAdj, aabbMaxAdj, position)
    

def closestFreePointOnCuboid(world, name, whitelist, suppMin, suppMax, position, dims):
    #dims = [0.15,0.15,0.15]
    initialPos = closestPointOnCuboid(suppMin, suppMax, position)
    auxMin = [a-b for a,b in zip(initialPos,dims)]
    auxMax = [a+b for a,b in zip(initialPos,dims)]
    dimsAdj = [0.15,0.15,0.15]
    blockNames = set([world.getPObjectById(x[0]).getName() for x in overlappingObjects(auxMin, auxMax, world.getSimConnection())]).difference(whitelist)
    if 0 == len(blockNames):
        return initialPos
    #else:
    #    print("BLOCKNAMES", blockNames, whitelist)
    kMax = [round((a - b)/(c)) for a,b,c in zip(suppMax,suppMin,dimsAdj)]
    initialPosK = [round((a-b)/(c)) for a,b,c in zip(initialPos,suppMin,dimsAdj)]
    cells = []
    for kX in range(kMax[0]):
        for kY in range(kMax[1]):
            cells.append((abs(kX - initialPosK[0]) + abs(kY - initialPosK[1]), 0, kY, -kX))
    for c in sorted(cells):
        if 2>=c[0]:
            continue
        c = [c[0],c[1],c[3],-c[2]]
        auxMin = [suppMin[0]+dimsAdj[0]*c[2]-dims[0], suppMin[1]+dimsAdj[1]*c[3]-dims[1], suppMax[2]-dims[2]+0.02]
        auxMax = [suppMin[0]+dimsAdj[0]*c[2]+dims[0], suppMin[1]+dimsAdj[1]*c[3]+dims[1], suppMax[2]+dims[2]+0.02]
        if 0 == len(set([world.getPObjectById(x[0]).getName() for x in overlappingObjects(auxMin, auxMax, world.getSimConnection())]).difference(whitelist)):
            #print("CFPOC", initialPosK,c,[suppMin[0] + c[2]*dimsAdj[0], suppMin[1] + c[3]*dimsAdj[1], suppMin[2]],world._pobjects["mediumBowl1"].getBodyProperty((),"position"))
            return [suppMin[0] + c[2]*dimsAdj[0], suppMin[1] + c[3]*dimsAdj[1], suppMin[2]]
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

def velocityAdjustAABB(aabbMin, aabbMax, velocity, dt):
    displacement = [x*dt for x in velocity]
    displacedMin = [a+b for a,b in zip(aabbMin,displacement)]
    displacedMax = [a+b for a,b in zip(aabbMax,displacement)]
    aabbMin = [min(a,b) for a,b in zip(aabbMin,displacedMin)]
    aabbMax = [max(a,b) for a,b in zip(aabbMax,displacedMax)]
    return (aabbMin, aabbMax)

def getImpendingCollisions(agent, handLink, handVelocity, boxes, whitelist, backupPoint, goingDown=False):
    if agent.getName() not in whitelist:
        whitelist.append(agent.getName())
    close = []
    for box in boxes:
        aabbMin, aabbMax = box
        close = close + [agent._world.getPObjectById(x[0]) for x in overlappingObjects(aabbMin, aabbMax, agent._world.getSimConnection())]
    close = list(set(close))
    close = [x for x in close if x.getName() not in whitelist]
    needAvoidance = False
    retq = [0,0,0]
    posH = agent.getBodyProperty((handLink,), "position")
    backupDirection = [a-b for a,b in zip(backupPoint, posH)]
    nB = math.sqrt(backupDirection[0]*backupDirection[0] + backupDirection[1]*backupDirection[1] + backupDirection[2]*backupDirection[2])
    if 0.001 < nB:
        backupDirection = [x/nB for x in backupDirection]
    else:
        posB = list(agent.getBodyProperty(("base_yaw",), "position"))
        posB[2] = 0.95
        backupDirection = [a-b for a,b in zip(posB, posH)]
        nB = math.sqrt(backupDirection[0]*backupDirection[0] + backupDirection[1]*backupDirection[1] + backupDirection[2]*backupDirection[2])
        if 0.001 < nB:
            backupDirection = [x/nB for x in backupDirection]
        else:
            s = 1
            if "hand_left_roll" == handLink:
                s = -1
            backupDirection = [0,s,0]
    for o in close:
        if not needAvoidance:
            needAvoidance = True
        retq = handVelocity
        #nnV = math.sqrt(retq[0]*retq[0] + retq[1]*retq[1] + retq[2]*retq[2])
        #if 0.001 < nnV:
        #    nV = [x/nnV for x in retq]
        #    goingDown = True
        #else:
        #    nV = retq
        #goingDown = (-0.8 > nV[2])
        #print("handvel", nV, retq, goingDown, backupDirection)
        exitDirection = o.getBodyProperty("fn", "exitdirection")
        if None == exitDirection:
            exitDirection = [0,0,1]
        if goingDown and (0.8 < exitDirection[2]):
            exitDirection = backupDirection
        print("NEEDAVOID",exitDirection, goingDown, backupDirection)
        retq = [a+b for a,b in zip(retq, exitDirection)]
    return needAvoidance, retq

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

def quaternionProduct(qa, qb):
    b1,c1,d1,a1 = qa
    b2,c2,d2,a2 = qb
    return (a1*b2+b1*a2+c1*d2-d1*c2, a1*c2-b1*d2+c1*a2+d1*b2, a1*d2+b1*c2-c1*b2+d1*a2,a1*a2-b1*b2-c1*c2-d1*d2)

def getHandAngularJointControls(agent, hand, controls, orHTarget, velocity=[0,0,0]):
    ### Have target orientation in world, orientation of arm base; need orientation in arm base
    ### Thiw = Tbiw*Thib => Thib = inv(Tbiw)*Thiw
    orientationB = agent.getBodyProperty(("base_yaw",), "orientation")
    orH = quaternionProduct((orientationB[0], orientationB[1], orientationB[2], -orientationB[3]), orHTarget)
    euler = p.getEulerFromQuaternion(orH)
    joints = {"left": ("hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"), "right": ("hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll")}[hand]
    controls["jointTargets"].update({joints[k]: (euler[k], velocity[k], 1.0) for k in [0,1,2]})
    return controls

def graspedCollisionAvoidanceForHand(agent, hand, handLink, controls, whitelist=[], dt=1.0/240.0, backupPoint=None, goingDown=False):
    posH = agent.getBodyProperty((handLink,), "position")
    velH = agent.getBodyProperty((handLink,), "linearVelocity")
    boxesH = [velocityAdjustAABB([a-b for a,b in zip(posH,[0.1]*3)], [a+b for a,b in zip(posH,[0.1]*3)], velH, dt)]
    graspedHNames = agent.getBodyProperty((handLink,), "grasping")
    graspedH = [agent._world._pobjects[x] for x in graspedHNames]
    for g in graspedH:
        aabbMin, aabbMax = g.getAABB(None)
        boxesH.append(velocityAdjustAABB(aabbMin, aabbMax, g.getBodyProperty((), "linearVelocity"), dt))
    needAvoid, avoidingV = getImpendingCollisions(agent, handLink, velH, boxesH, whitelist+graspedHNames, backupPoint, goingDown=goingDown)
    if needAvoid:
        avoidingH = [x*dt for x in avoidingV]
        posHTarget = [a+b for a,b in zip(posH, avoidingH)]
        controls = getHandLinearJointControls(agent, hand, controls, posHTarget, velocity=avoidingV)
    joints = {"left": ("hand_left_base_to_hand_left_x", "hand_left_x_to_hand_left_y", "hand_left_y_to_hand_left_z"), "right": ("hand_right_base_to_hand_right_x", "hand_right_x_to_hand_right_y", "hand_right_y_to_hand_right_z")}
    #for name in joints[hand]:
    #    if name not in controls["jointTargets"]:
    #        continue
    #    #inc = -0.002
    #    #if 0 > controls["jointTargets"][name][0]:
    #    #    inc = 0.002
    #    #aux = list(controls["jointTargets"][name])
    #    #aux[0] = controls["jointTargets"][name][0] + inc - 0.002*controls["jointTargets"][name][0]
    #    #controls["jointTargets"][name] = tuple(aux)
    return controls

def graspedCollisionAvoidance(agent, controls, whitelist=[], backupPoint=None, goingDownRight=False, goingDownLeft=False):
    if None == backupPoint:
        backupPoint = list(agent.getBodyProperty(("base_yaw",), "position"))
        backupPoint[2] = 0.95
    dt = 1.0/240.0
    controls = graspedCollisionAvoidanceForHand(agent, "left", "hand_left_roll", controls, whitelist=whitelist, dt=dt, backupPoint=backupPoint, goingDown=goingDownLeft)
    controls = graspedCollisionAvoidanceForHand(agent, "right", "hand_right_roll", controls, whitelist=whitelist, dt=dt, backupPoint=backupPoint, goingDown=goingDownRight)
    return controls

def getContents(item):
    aabbMin, aabbMax = item.getAABB(None)
    return [x for x in closeObjects(item, radius=0) if aabbContainment(x, aabbMin, aabbMax)]

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

def closeObjects(item,radius=0.25):
    aabbMin, aabbMax = item.getAABB(None)
    minC = [a - radius for a in aabbMin]
    maxC = [a + radius for a in aabbMax]
    retq = list(set([item._world.getPObjectById(x[0]) for x in overlappingObjects(minC, maxC, item._world.getSimConnection())]))
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
                a = p.getJointState(o._id, o._jointName2Id[fnj], o._world.getSimConnection())[0]
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
    def suggestBackup(self, trajector):
        return None

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
    def suggestBackup(self, trajector):
        return None

class LocationAt(Location):
    def isThere(self, trajector):
        aabbMin, aabbMax = trajector.getAABB(None)
        aabbMin = tuple([aabbMin[0], aabbMin[1], aabbMin[2]-0.02])
        retq = list(set([trajector._world.getPObjectById(x[0]) for x in overlappingObjects(aabbMin, aabbMax, trajector._world.getSimConnection())]))
        if isinstance(self._relatum, pobject.PObjectWrapper):
            return self._relatum._pobject in retq
        return self._relatum in retq
    def suggestPose(self, trajector):
        aabbMin, aabbMax = trajector.getAABB(None)
        dims = [(a-b)/2.0 for a,b in zip(aabbMax,aabbMin)]
        position = trajector.getBodyProperty((), "position")
        _, orientation = LocationUpright(trajector).suggestPose(trajector)
        velocity = trajector.getBodyProperty((), "linearVelocity")
        #supportBBs = self._relatum.getBodyProperty("fn", "supportbbs")
        #if None == supportBBs:
        #    supportBBs = [self._relatum.getAABB(None)]
        #else:
        #    pR = self._relatum.getBodyProperty((), "position")
        #    oR = self._relatum.getBodyProperty((), "orientation")
        #    aux = []
        #    for s in supportBBs:
        #        rm = [a+b for a,b in zip(pR, p.rotateVector(oR, s[0]))]
        #        rM = [a+b for a,b in zip(pR, p.rotateVector(oR, s[1]))]
        #        aux.append([[min(a,b) for a,b in zip(rm,rM)], [max(a,b) for a,b in zip(rm,rM)]])
        #    supportBBs = aux
        #minD = None
        #bestPos = position
        #for supp in supportBBs:
        #    suppMin = [supp[0][0]+dims[0], supp[0][1]+dims[1], supp[1][2]+dims[2]]
        #    suppMax = [supp[1][0]-dims[0], supp[1][1]-dims[1], supp[1][2]+dims[2]+0.05]
        #    #candidate = closestFreePointOnCuboid(self._relatum._world, trajector.getName(), ["abe",trajector.getName(), self._relatum.getName()], suppMin, suppMax, position, dims)
        #    if None == candidate:
        #        continue
        #    d = [a-b for a,b in zip(candidate, position)]
        #    nD = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        #    if (None == minD) or (d < minD):
        #        minD = d
        #        bestPos = candidate
        #bestPos = list(bestPos)
        bestPos = list(pickKnownLocation(self._relatum, trajector))
        targMin = [a-b for a,b in zip(bestPos,dims)]
        targMax = [a+b for a,b in zip(bestPos,dims)]
        wayMin = [min(a,b) for a,b in zip(targMin,aabbMin)]
        wayMax = [max(a,b) for a,b in zip(targMax,aabbMax)]
        d = [a-b for a,b in zip(bestPos, position)]
        #hD = math.sqrt(d[0]*d[0]+d[1]*d[1])
        vD = math.sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1])
        onTheWay = set([trajector._world.getPObjectById(x[0]).getName() for x in overlappingObjects(wayMin, wayMax, trajector._world.getSimConnection())])
        if (0 != len(onTheWay.difference([trajector.getName(),self._relatum.getName(),"abe"]))):
            #print("BLABLABLA", onTheWay)
            bestPos[2] = position[2]
        #elif (0.02 < vD):
        #    bestPos[2] = position[2]
        print("SUGGESTED",("[%.3f,%.3f,%.3f]"%(bestPos[0],bestPos[1],bestPos[2])),("[%.3f,%.3f,%.3f]"%(position[0],position[1],position[2])))
        return bestPos, orientation
    def suggestBackup(self, trajector):
        #aabbMin, aabbMax = trajector.getAABB(None)
        #dims = [(a-b)/2.0 for a,b in zip(aabbMax,aabbMin)]
        #position = trajector.getBodyProperty((), "position")
        #supportBBs = self._relatum.getBodyProperty("fn", "supportbbs")
        #if None == supportBBs:
        #    supportBBs = [self._relatum.getAABB(None)]
        #minD = None
        #bestPos = position
        #for supp in supportBBs:
        #    suppMin = [supp[0][0]+dims[0], supp[0][1]+dims[1], supp[1][2]+dims[2]]
        #    suppMax = [supp[1][0]-dims[0], supp[1][1]-dims[1], supp[1][2]+dims[2]+0.05]
        #    candidate = [(a+b)/2 for a,b in zip(suppMin,suppMax)]
        #    d = [a-b for a,b in zip(candidate, position)]
        #    nD = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        #    if (None == minD) or (d < minD):
        #        minD = d
        #        bestPos = candidate
        #return bestPos
        return self.suggestPose(trajector)[0]

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
    def suggestBackup(self, trajector):
        return None

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
    def suggestBackup(self, trajector):
        return None

class Unwill(Goal):
    def _isFulfilled(self):
        return False

class ParkedArms(Goal):
    def __init__(self,agent):
        super().__init__()
        self._agent = agent
        self._joints = ["hand_right_base_to_hand_right_x", "hand_right_x_to_hand_right_y", "hand_right_y_to_hand_right_z", "hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll","hand_left_base_to_hand_left_x", "hand_left_x_to_hand_left_y", "hand_left_y_to_hand_left_z", "hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"]
    def _strVarPart(self):
        return str(self._agent)
    def _isFulfilled(self):
        retq = []
        for j in self._joints:
            s = p.getJointState(self._agent._id, self._agent._jointName2Id[j],self._agent._world.getSimConnection())
            retq.append((0.01>abs(s[0]))and(0.001>abs(s[1])))
        return all(retq)
    def suggestProcess(self):
        return ParkingArms(self._agent)

class ParkingArms(BodyProcess):
    def __init__(self,agent):
        super().__init__(coherence=[])
        self._agent = agent
        self._joints = ["hand_right_base_to_hand_right_x", "hand_right_x_to_hand_right_y", "hand_right_y_to_hand_right_z", "hand_right_z_to_hand_right_yaw", "hand_right_yaw_to_hand_right_pitch", "hand_right_pitch_to_hand_right_roll","hand_left_base_to_hand_left_x", "hand_left_x_to_hand_left_y", "hand_left_y_to_hand_left_z", "hand_left_z_to_hand_left_yaw", "hand_left_yaw_to_hand_left_pitch", "hand_left_pitch_to_hand_left_roll"]
    def _markForDeletion(replacement=None):
        self._coherence.append(Unwill())
        return None
    def bodyAction(self):
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {x: (0,0,1) for x in self._joints}}
        self._agent.applyRigidBodyControls([graspedCollisionAvoidance(self._agent, controls)])
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
    def _strVarPart(self):
        return str(self._location)
    def _markForDeletionInternal(self, replacement=None):
        self._coherence.append(Unwill())
        return None
    def _bodyAction(self):
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {}}
        graspL = self._agent.getBodyProperty(("hand_left_roll",), "grasping")
        graspR = self._agent.getBodyProperty(("hand_right_roll",), "grasping")
        handL = self._agent.getBodyProperty(("hand_left_roll",), "position")
        handR = self._agent.getBodyProperty(("hand_right_roll",), "position")
        retq = False
        for hand, grasping, handPos in [("left", graspL, handL), ("right", graspR, handR)]:
            if not grasping:
                continue
            exitDirections = []
            aabbs = []
            for g in grasping:
                aabbMin,aabbMax = self._agent._world._pobjects[g].getAABB(None)
                aabbs.append((aabbMin,aabbMax))
            aabbMin = [a-b for a,b in zip(handPos,[0.2]*3)]
            aabbMax = [a+b for a,b in zip(handPos,[0.2]*3)]
            #aabbs.append((aabbMin,aabbMax))
            for aabbMin,aabbMax in aabbs:
                overlaps = [self._agent._world.getPObjectById(x[0]) for x in overlappingObjects(aabbMin, aabbMax, self._item._world.getSimConnection())]
                exitDirections = exitDirections + [x.getBodyProperty("fn", "exitdirection") for x in overlaps]
            exitDirections = [x for x in exitDirections if None!=x]
            if not exitDirections:
                if 0.95 > handPos[2]:
                    exitDirections = [[0,0,1-handPos[2]]]
            if not exitDirections:
                continue
            retq = True
            inc = [0,0,0]
            for e in exitDirections:
                inc[0] = inc[0] + e[0]
                inc[1] = inc[1] + e[1]
                inc[2] = inc[2] + e[2]
            pT = [0.1*x+y for x,y in zip(inc,handPos)]
            controls = getHandLinearJointControls(self._agent, hand, controls, pT)
        if retq:
            #print("EXITCONTROL")
            self._agent.applyRigidBodyControls([controls])
        return retq
    def bodyAction(self):
        if self._bodyAction():
            return
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
        position = closestPointOnCuboid(aabbMin, aabbMax, position)
        d = [a-b for a,b in zip(positionR, position)]
        nd = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        d = [x/nd for x in d]
        yaw = math.atan2(d[1], d[0])
        controls = {"+constraints": [], "-constraints": [], "jointTargets": {"world_to_base_x": (position[0], 0, 1.0), "base_x_to_base_y": (position[1], 0, 1.0), "base_y_to_base_yaw": (yaw, 0, 1.0)}}
        #posL = self._agent.getBodyProperty(("hand_left_roll",), "position")
        #posR = self._agent.getBodyProperty(("hand_right_roll",), "position")
        #orL = self._agent.getBodyProperty(("hand_left_roll",), "orientation")
        #orR = self._agent.getBodyProperty(("hand_right_roll",), "orientation")
        #controls = getHandLinearJointControls(self._agent, "left", controls, posL)
        #controls = getHandLinearJointControls(self._agent, "right", controls, posR)
        #controls = getHandAngularJointControls(self._agent, "left", controls, orL)
        #controls = getHandAngularJointControls(self._agent, "right", controls, orR)
        self._agent.applyRigidBodyControls([graspedCollisionAvoidance(self._agent, controls)])
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
        nVR = math.sqrt(velocityR[0]*velocityR[0] + velocityR[1]*velocityR[1] + velocityR[2]*velocityR[2])
        nVO = math.sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1] + velocity[2]*velocity[2])
        tracking = False
        if (0.01 > nVR) and (0.01 > nVO):
            tracking = True
        elif (0.01 <= nVR) and (0.01 <= nVO):
            vP = [a*b/(nVR*nVO) for a,b in zip(velocityR,velocity)]
            tracking = 0.95 < (vP[0]+vP[1]+vP[2])
        d = [a-b for a,b in zip(posI, posH)]
        ok = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]) < self._radius
        if not ok:
            grasping = self._agent.getBodyProperty((self._handLink,), "grasping")
            if self._item.getName() in grasping:
                grasping.remove(self._item.getName())
                self._agent.setBodyProperty((self._handLink,), "grasping", grasping)
        
        return ok #and tracking
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
    def _strVarPart(self):
        return str(self._item) + "," + str(self._hand)
    def _markForDeletionInternal(self, replacement=None):
        self._coherence.append(Unwill())
        return None#StoppedHand(self._agent, self._hand)
    def bodyAction(self):
        refPO = self._item
        if isinstance(refPO, Location):
            refPO = refPO._relatum
        whitelist = [refPO.getName()] + [x.getName() for x in closeObjects(refPO)]
        positionR = refPO.getBodyProperty((), "position")
        position = closestPointOnSphere(positionR, self._radius, self._agent.getBodyProperty((self._handLink,), "position"))
        controls = getHandLinearJointControls(self._agent, self._hand, {"+constraints": [], "-constraints": [], "jointTargets": {}}, position, velocity=[0,0,0])
        self._agent.applyRigidBodyControls([graspedCollisionAvoidance(self._agent, controls, whitelist=whitelist, backupPoint=positionR)])
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

class PlacingItem(BodyProcess):
    def __init__(self, item, location, hand):
        super().__init__(coherence=[Grasped(item,hand), BaseNear(location)])
        self._item = item
        self._location = location
        self._hand = hand
        self._handLink = {"left": "hand_left_roll", "right": "hand_right_roll"}[self._hand]
        self._agent = findByType(self._item._world, "agent")
        self._halt = None
    def _strVarPart(self):
        return str(self._item) + "," + str(self._location) + "," + str(self._hand)
    def _markForDeletionInternal(self, replacement=None):
        self._coherence.append(Unwill())
        return None
    def bodyAction(self):
        ### location: locationAt, locationIn, locationUpright, locationOver, locationTippedOver
        location = self._location
        if isinstance(location, pobject.PObject) or isinstance(location, pobject.PObjectWrapper):
            location = LocationAt(location)
        targetPos, targetOr = location.suggestPose(self._item)
        positionO = self._item.getBodyProperty((), "position")
        vv = self._item.getBodyProperty((), "linearVelocity")
        v = math.sqrt(vv[0]*vv[0]+vv[1]*vv[1]+5*vv[2]*vv[2])
        if 0.1 < v:
            if None == self._halt:
                self._halt = list(positionO)
            targetPos = self._halt
            #print("SLOWDOWN",targetPos,positionO)
        else:
            self._halt = None
            d = [a-b for a,b in zip(targetPos,positionO)]
            nD = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
            if 0.05 < nD:
                d = [0.05*x/nD for x in d]
                targetPos = [a+b for a,b in zip(d,positionO)]
        print("TRAJECTOR",("%.3f"%v),("[%.3f,%.3f,%.3f]"%(vv[0],vv[1],vv[2])),("[%.3f,%.3f,%.3f]"%(positionO[0],positionO[1],positionO[2])))
        print("____",("[%.3f,%.3f,%.3f]"%(targetPos[0],targetPos[1],targetPos[2])))
        orientationO = self._item.getBodyProperty((), "orientation")
        positionH = self._agent.getBodyProperty((self._handLink,), "position")
        orientationH = self._agent.getBodyProperty((self._handLink,), "orientation")
        ### Have (cr.) object in world, hand in world; need object in hand
        ### Toiw = Thiw*Toih => Toih = inv(Thiw)*Toiw
        posOiH = [a-b for a,b in zip(p.rotateVector((orientationH[0], orientationH[1], orientationH[2], -orientationH[3]), positionO), p.rotateVector((orientationH[0], orientationH[1], orientationH[2], -orientationH[3]), positionH))]
        orOiH = quaternionProduct((orientationH[0], orientationH[1], orientationH[2], -orientationH[3]), orientationO)
        #print("CrW", positionH, positionO)
        #print("ObjInHand", posOiH, orOiH)
        ### have (targ.) object in world, object in hand; need (targ.) hand in world
        ### Toiw = Thiw*Toih => Thiw = Toiw*inv(Toih)
        ### inv(T)*T has 0 translation so -iT.q*T.t = iT.t
        posOiH = p.rotateVector((orOiH[0], orOiH[1], orOiH[2], -orOiH[3]), posOiH)
        posTH = [a+b for a,b in zip(targetPos, p.rotateVector(targetOr, (-posOiH[0], -posOiH[1], -posOiH[2])))]
        orTH = quaternionProduct(targetOr, (orOiH[0], orOiH[1], orOiH[2], -orOiH[3]))
        ##
        #dTH = [a-b for a,b in zip(posTH,positionH)]
        #dOS = [a-b for a,b in zip(location.suggestPose(self._item)[0],self._item.getBodyProperty((), "position"))]
        #dST = [a-b for a,b in zip(location.suggestPose(self._item)[0],targetPos)]
        #print("DIFFS", math.sqrt(dTH[0]*dTH[0] + dTH[1]*dTH[1] + dTH[2]*dTH[2]), math.sqrt(dOS[0]*dOS[0] + dOS[1]*dOS[1] + dOS[2]*dOS[2]), math.sqrt(dST[0]*dST[0] + dST[1]*dST[1] + dST[2]*dST[2]))
        #print("    ", location.suggestPose(self._item)[0], targetPos, self._item.getBodyProperty((), "position"))
        ##
        controls = getHandLinearJointControls(self._agent, self._hand, {"+constraints": [], "-constraints": [], "jointTargets": {}}, posTH, velocity=[0,0,0])
        controls = getHandAngularJointControls(self._agent, self._hand, controls, orTH, velocity=[0,0,0])
        gDR = False
        gDL = False
        if ("right" == self._hand) and isinstance(self._location, LocationAt):
            gDR = True
        if ("left" == self._hand) and isinstance(self._location, LocationAt):
            gDL = True
        controls = graspedCollisionAvoidance(self._agent, controls, whitelist=[location._relatum.getName()], backupPoint=location.suggestBackup(self._item), goingDownRight=gDR, goingDownLeft=gDL)
        #print("CONTROLS", controls["jointTargets"]["hand_right_y_to_hand_right_z"])
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
                a = p.getJointState(pob._id, pob._jointName2Id[fnj], pob._world.getSimConnection())[0]
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
        super().__init__(coherence=[ItemOnCounter(destination), Grasped(item), BaseNear(destination), PouredContents(item, destination)])
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
        super().__init__(coherence=[ItemOnCounter(destination), Grasped(item), ItemNear(item,LocationOver(destination)), ItemNear(item,LocationTippedOver(destination))])
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

