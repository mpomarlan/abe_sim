import heapq
import math
import numpy

import pybullet
from world import stubbornTry, getDictionaryEntry

step = 0.05

def vertexInDictionary(vertex, d):
    for e in vertex:
        if e not in d:
            return False
        d = d[e]
    return True

def vertexDataFromDictionary(vertex, d):
    for e in vertex[:-1]:
        d = d[e]
    return d[vertex[-1]]

def addVertexToDictionary(vertex, d, value):
    for e in vertex[:-1]:
        if e not in d:
            d[e] = {}
        d = d[e]
    d[vertex[-1]] = value

def getPath(partialGoal, startDistance, currentLocation):
    if partialGoal is None:
        return []
    _, _, vertex = partialGoal
    retq = []
    while vertex is not None:
        if vertex == currentLocation:
            break
        retq.append(vertex)
        vertex = vertexDataFromDictionary(vertex, startDistance)[1]
    retq = list(reversed(retq))
    mink = sorted([(numpy.linalg.norm([a-b for a,b in zip(e,currentLocation)]), k) for k, e in enumerate(retq)])[0][1]
    return retq[mink:]
'''
    # if previous partial goal was a dead end, must backtrack to rejoin path
    adj = []
    vertex = vertexDataFromDictionary(currentLocation, startDistance)[1]
    while vertex is not None:
        try:
            where = retq.index(vertex)
            return adj + retq[where:]
        except ValueError:
            pass
        adj.append(vertex)
        vertex = vertexDataFromDictionary(vertex, startDistance)[1]
    return adj + retq
'''

def heuristic(a,b):
    return numpy.linalg.norm([x-y for x,y in zip(a,b)])

def getSuccessors(collisionFn, goal, cr, visitedVertices, obstacles, capLimit):
    def _getSuccessorsBasic(p):
        return [(tuple([a + b for a,b in zip(p, inc)]), step) for inc in [(step,0,0), (-step,0,0), (0,step,0), (0,-step,0), (0,0,step), (0,0,-step)]]
    def _validSuccessor(collisionFn, x, visitedVertices, obstacles, capLimit):
        succ, cost = x
        if vertexInDictionary(succ, visitedVertices) or vertexInDictionary(succ, obstacles):
            return False
        inCollision = collisionFn(succ)
        if inCollision:
            addVertexToDictionary(succ, obstacles, True)
            return False
        return all([(cl <= xV) and (xV <= cL) for cl, cL, xV in zip(capLimit[0], capLimit[1], succ)])
    successors = _getSuccessorsBasic(cr)
    goalCost = heuristic(cr, goal)
    if goalCost < step*2:
        successors.append((goal, goalCost))
    return [x for x in successors if _validSuccessor(collisionFn, x, visitedVertices, obstacles, capLimit)]

# A* for some number of iterations
# select a "partial goal" to move towards
def updatePath(collisionFn, currentLocation, goal, partialGoal, openVertices, visitedVertices, obstacles, startDistance, capLimit):
    k = 5
    while openVertices and (0 < k):
        k -= 1
        f, h, cr = heapq.heappop(openVertices)
        cr = tuple(cr)
        if vertexInDictionary(cr, visitedVertices):
            continue
        addVertexToDictionary(cr, visitedVertices, True)
        if (partialGoal is None) or ((h, f) < partialGoal[:2]):
            partialGoal = (h, f, cr)
            if 0.005 > h:
                break
        for s in getSuccessors(collisionFn, goal, cr, visitedVertices, obstacles, capLimit):
            succ, cost = s
            newStartDistance = (vertexDataFromDictionary(cr, startDistance)[0] + cost, cr)
            if (not vertexInDictionary(succ, startDistance)) or (newStartDistance[0] < vertexDataFromDictionary(succ, startDistance)[0]):
                addVertexToDictionary(succ, startDistance, newStartDistance)
                hsucc = heuristic(succ, goal)
                heapq.heappush(openVertices, (hsucc + vertexDataFromDictionary(succ, startDistance)[0], hsucc, succ))
    return partialGoal, getPath(partialGoal, startDistance, currentLocation)

def coarseCollisionCheck(customDynamicsAPI, position, allowableProximities, ignorableObjects):
    overlaps = customDynamicsAPI['checkOverlap']([[x-0.025 for x in position], [x+0.025 for x in position]])
    for e in overlaps:
        if e in ignorableObjects:
            continue
        closePoints = customDynamicsAPI['probeClosestPoints']((e,), position, 0.03)
        for close in closePoints:
            identifier, pos, normal, distance = close
            if (identifier[0] not in allowableProximities) or (identifier[1] not in allowableProximities[identifier[0]]):
                return True
            position = customDynamicsAPI['getObjectProperty'](identifier, 'position')
            orientation = customDynamicsAPI['getObjectProperty'](identifier, 'orientation')
            if not any([(((0.8 < numpy.dot(normal, customDynamicsAPI['objectPoseRelativeToWorld'](position, orientation, n[0], (0,0,0,1))[0])) and (distance >= n[1])) or (distance >= n[2])) for n in allowableProximities[identifier[0]][identifier[1]]]):
                return True
    return False

def fineCollisionCheck(customDynamicsAPI, identifiers, allowableProximities, ignorableObjects):
    def incoming(identifier, position, orientation, normal, distance, allowableProximities):
        if (identifier[0] in allowableProximities) and (identifier[1] in allowableProximities[identifier[0]]):
            direction, minOnDirection, distanceCutOff = allowableProximities[identifier[0]]
            if distanceCutOff <= distance:
                return False
            directionWorld, _ = customDynamicsAPI['objectPoseRelativeToWorld'](position, orientation, direction, (0,0,0,1))
            if (0.8 < numpy.dot(directionWorld, normal)) and (minOnDirection <= distance):
                return False
        return True
    incomingVelocity = numpy.array([0,0,0])
    for identifier in identifiers:
        aabb = customDynamicsAPI['getObjectProperty'](identifier, 'aabb')
        aabbAdj = customDynamicsAPI['addAABBRadius'](aabb, 0.05)
        overlaps = customDynamicsAPI['checkOverlap'](aabbAdj)
        for e in overlaps:
            if e in ignorableObjects:
                continue
            closePoints = customDynamicsAPI['checkClosestPoints'](identifier, (e,), maxDistance=0.20)
            for close in closePoints:
                identifierA, identifierB, posA, posB, normal, distance = close
                positionB = customDynamicsAPI['getObjectProperty'](identifierB, 'position')
                orientationB = customDynamicsAPI['getObjectProperty'](identifierB, 'orientation')
                if incoming(identifierB, positionB, orientationB, normal, distance, allowableProximities):
                    positionA = customDynamicsAPI['getObjectProperty'](identifierA, 'position')
                    orientationA = customDynamicsAPI['getObjectProperty'](identifierA, 'orientation')
                    dpB, _ = customDynamicsAPI['objectPoseRelativeToObject'](positionB, orientationB, posB, (0,0,0,1))
                    dpA, _ = customDynamicsAPI['objectPoseRelativeToObject'](positionA, orientationA, posA, (0,0,0,1))
                    velocityB = numpy.array(customDynamicsAPI['getObjectProperty'](identifierB, 'linearVelocity'))
                    omegaB = numpy.array(customDynamicsAPI['getObjectProperty'](identifierB, 'angularVelocity'))
                    velocityA = numpy.array(customDynamicsAPI['getObjectProperty'](identifierA, 'linearVelocity'))
                    omegaA = numpy.array(customDynamicsAPI['getObjectProperty'](identifierA, 'angularVelocity'))
                    relativeVelocity = (velocityB + numpy.cross(omegaB, dpB)) - (velocityA + numpy.cross(omegaA, dpA))
                    relativeDisplacement = numpy.array(posB) - numpy.array(posA)
                    displacementNorm = numpy.linalg.norm(relativeDisplacement)
                    displacementDirection = relativeDisplacement/displacementNorm
                    coefficient = numpy.dot(displacementDirection, relativeVelocity)
                    if 0 > coefficient:
                        incomingVelocity = incomingVelocity + coefficient*displacementDirection*min(10, 0.1/(0.01 + displacementNorm))
    return incomingVelocity

def updateMotionPlanning(name, customDynamicsAPI):
    fnMotionPlanning = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'motionPlanning'), {})
    csvMotionPlanning = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'motionPlanning'), {})
    fnKinematicControl = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl'), {})
    csvKinematicControl = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'kinematicControl'), {})
    for efName in fnMotionPlanning.get('endEffectors', []):
        ef = getDictionaryEntry(fnKinematicControl, ('efLink', efName), '')
        allowableProximities = getDictionaryEntry(csvMotionPlanning, ('allowableProximities', ef), {})
        ignorableObjects = getDictionaryEntry(csvMotionPlanning, ('ignorableObjects', ef), {})
        if name not in ignorableObjects:
            ignorableObjects[name] = True
        identifiers = [(name, ef)] + [(x[0][0],) for x in customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'grasping', 'actuallyGrasping', ef), [])]
        incomingVelocity = fineCollisionCheck(customDynamicsAPI, identifiers, allowableProximities, ignorableObjects)
        incomingNorm = numpy.linalg.norm(incomingVelocity)
        if 0.001 < incomingNorm:
            positionLink = customDynamicsAPI['getObjectProperty']((name, ef), 'position')
            orientationLink = customDynamicsAPI['getObjectProperty']((name, ef), 'orientation')
            positionEF = getDictionaryEntry(csvKinematicControl, ('positionInLink', ef), (0,0,0))
            orientationEF = getDictionaryEntry(csvKinematicControl, ('orientationInLink', ef), (0,0,0,1))
            positionEFWorld, _ = customDynamicsAPI['objectPoseRelativeToWorld'](positionLink, orientationLink, positionEF, orientationEF)
            baseLinkEF = getDictionaryEntry(fnKinematicControl, ('baseLink', ef), '')
            baseLinkPosition = customDynamicsAPI['getObjectProperty']((name, baseLinkEF), 'position')
            baseLinkOrientation = customDynamicsAPI['getObjectProperty']((name, baseLinkEF), 'orientation')
            parkedPosition = getDictionaryEntry(fnKinematicControl, ('parkedPosition', ef), (0,0,0))
            parkedPositionInWorld, _ = customDynamicsAPI['objectPoseRelativeToWorld'](baseLinkPosition, baseLinkOrientation, parkedPosition, (0,0,0,1))
            incomingDirection = incomingVelocity/incomingNorm
            parkingDirection = incomingDirection
            parkingDisplacement = numpy.array(parkedPositionInWorld) - numpy.array(positionLink)
            parkingNorm = numpy.linalg.norm(parkingDisplacement)
            if 0.01 < parkingNorm:
                parkingDisplacement = parkingDisplacement/parkingNorm
                parkingDisplacement = parkingDisplacement - numpy.dot(parkingDisplacement, incomingVelocity)*incomingVelocity
                parkingNorm = numpy.linalg.norm(parkingDisplacement)
                if 0.01 < parkingNorm:
                    parkingDirection = parkingDisplacement/parkingNorm
            targetPosition = list(positionEFWorld + parkingDirection*0.05)
            targetOrientation = customDynamicsAPI['objectPoseRelativeToWorld'](positionLink, orientationLink, positionEF, orientationEF)
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'motionPlanning', 'openVertices', ef), [])
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'motionPlanning', 'visitedVertices', ef), {})
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'motionPlanning', 'startDistance', ef), {})
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'motionPlanning', 'partialGoal', ef), None)
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'kinematicControl', 'target', ef), [targetPosition, targetOrientation])
        else:
            goalPose = getDictionaryEntry(csvMotionPlanning, ('goal', ef), None)
            if goalPose is not None:
                goalPosition, goalOrientation = goalPose
                openVertices = getDictionaryEntry(csvMotionPlanning, ('openVertices', ef), [])
                visitedVertices = getDictionaryEntry(csvMotionPlanning, ('visitedVertices', ef), {})
                obstacles = getDictionaryEntry(csvMotionPlanning, ('obstacles', ef), {})
                startDistance = getDictionaryEntry(csvMotionPlanning, ('startDistance', ef), {})
                partialGoal = getDictionaryEntry(csvMotionPlanning, ('partialGoal', ef), None)
                positionLink = customDynamicsAPI['getObjectProperty']((name, ef), 'position')
                orientationLink = customDynamicsAPI['getObjectProperty']((name, ef), 'orientation')
                positionEF = getDictionaryEntry(csvKinematicControl, ('positionInLink', ef), (0,0,0))
                orientationEF = getDictionaryEntry(csvKinematicControl, ('orientationInLink', ef), (0,0,0,1))
                baseLinkEF = getDictionaryEntry(fnKinematicControl, ('baseLink', ef), '')
                baseLinkPosition = customDynamicsAPI['getObjectProperty']((name, baseLinkEF), 'position')
                baseLinkOrientation = customDynamicsAPI['getObjectProperty']((name, baseLinkEF), 'orientation')
                parkedPosition = getDictionaryEntry(fnKinematicControl, ('parkedPosition', ef), (0,0,0))
                parkedPositionInWorld, parkedOrientationInWorld = customDynamicsAPI['objectPoseRelativeToWorld'](baseLinkPosition, baseLinkOrientation, parkedPosition, (0,0,0,1))
                parkedPositionEFInWorld, _ = customDynamicsAPI['objectPoseRelativeToWorld'](parkedPositionInWorld, parkedOrientationInWorld, positionEF, (0,0,0,1))
                currentPosition, currentOrientation = customDynamicsAPI['objectPoseRelativeToWorld'](positionLink, orientationLink, positionEF, orientationEF)
                currentLocation = [round((x*100)/5)*0.05 for x in currentPosition]
                parkedPositionEFInWorld = [round((x*100)/5)*0.05 for x in parkedPositionEFInWorld]
                h = heuristic(currentLocation, goalPosition)
                if [] == openVertices:
                    openVertices = [[h, h, currentLocation]]
                if {} == startDistance:
                    addVertexToDictionary(currentLocation, startDistance, [0, None])
                elif not vertexInDictionary(currentLocation, startDistance):
                    startDistance = {}
                    addVertexToDictionary(currentLocation, startDistance, [0, None])
                    openVertices = [[h, h, currentLocation]]
                    visitedVertices = {}
                    partialGoal = None
                limits = getDictionaryEntry(fnMotionPlanning, ('limits', ef), [1,1,1])
                capLimit = [[x-y for x,y in zip(parkedPositionEFInWorld, limits)], [x+y for x,y in zip(parkedPositionEFInWorld, limits)]]
                partialGoal, path = updatePath((lambda x : coarseCollisionCheck(customDynamicsAPI, x, allowableProximities, ignorableObjects)), currentLocation, goalPosition, partialGoal, openVertices, visitedVertices, obstacles, startDistance, capLimit)
                currentOrientationInverse = list(currentOrientation)
                currentOrientationInverse[3] = -currentOrientationInverse[3]
                _, quatDifference = stubbornTry(lambda : pybullet.multiplyTransforms((0,0,0), goalOrientation, (0,0,0), currentOrientationInverse))
                differenceAxis, differenceAngle = stubbornTry(lambda : pybullet.getAxisAngleFromQuaternion(quatDifference))
                if 0.0001 < differenceAngle:
                    differenceAngle = min(differenceAngle, 0.05)
                    increment = stubbornTry(lambda : pybullet.getQuaternionFromAxisAngle(differenceAxis, differenceAngle))
                    _, nextOrientation = stubbornTry(lambda : pybullet.multiplyTransforms((0, 0, 0), increment, (0, 0, 0), currentOrientation))
                else:
                    nextOrientation = currentOrientation
                customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'motionPlanning', 'partialGoal', ef), partialGoal)
                customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'motionPlanning', 'startDistance', ef), startDistance)
                customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'motionPlanning', 'obstacles', ef), obstacles)
                customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'motionPlanning', 'openVertices', ef), openVertices)
                customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'motionPlanning', 'visitedVertices', ef), visitedVertices)
                index = 0
                while index < len(path):
                    if 0.15 > numpy.linalg.norm([a-b for a,b in zip(currentPosition, path[index])]):
                        if index + 1 < len(path):
                            index = index + 1
                        else:
                            break
                    else:
                        break
                print(index, len(path), currentPosition, path[index])
                customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'kinematicControl', 'target', efName), [path[index], nextOrientation])

