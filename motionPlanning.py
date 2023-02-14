import pybullet
import math
import heapq
import random

import world

from abe_sim.utils import stubbornTry
from abe_sim.geom import quaternionProduct, overlappingObjects, translateVector, scaleVector, vectorNorm, vectorNormalize, extrudeBox, distance, interpolatePoint, boxHasPoint, splitBox

def getDistances(customDynamicsAPI, a, b):
    d = customDynamicsAPI['distance'](a[0], b[0])
    dq = abs(customDynamicsAPI['orientationDifferenceAA'](a[1], b[1])[1])
    return d, dq

def nearEnough(d, dq):
    return (0.005 > d) and (0.005 > dq)

def estimateTotalCost(customDynamicsAPI, end, start):
    d, dq = getDistances(customDynamicsAPI, end, start)
    return dq + d

def getIncoming(customDynamicsAPI, trajectors, allowedApproacherNames, allowedApproacherTypes):
    return None
def getMovementCost(customDynamicsAPI, trajectors, startPose, nextPose, incoming, allowedColliderNames, allowedColliderTypes, allowedApproacherNames, allowedApproacherTypes, preferredPose):
    return None

def arrived(customDynamicsAPI, pose, goal, trajectorRels, incoming, allowedColliderNames, allowedColliderTypes, allowedApproacherNames, allowedApproacherTypes):
    isSafe = True
    isAtGoal = True
    if incoming is not None:
        isSafe = False ####
        restore = []
        for identifier, p, q in restore:

    if goal is not None:
        d, dq = getDistances(customDynamicsAPI, goal, pose)
        isAtGoal = nearEnough(d, dq)
    return isSafe and isAtGoal

def planMotion(customDynamicsAPI, trajectors, startPose, endPose, incoming, allowedColliderNames, allowedColliderTypes, allowedApproacherNames, allowedApproacherTypes, preferredPose):
    trajectorRels = []
    for t in trajectors:
        p = customDynamicsAPI['getObjectProperty'](t, 'position')
        q = customDynamicsAPI['getObjectProperty'](t, 'orientation')
        trajectorRels.append((customDynamicsAPI['objectPoseRelativeToObject'](startPose[0], startPose[1], p, q), p, q))
    retq = []
    visited = {}
    startPosition, startOrientation = startPose
    goal = endPose
    if endPose is None:
        endPose = startPose
    endPosition, endOrientation = endPose
    toVisit = [(estimateTotalCost(customDynamicsAPI, endPose, startPose), 0, (startPosition, startOrientation)), None]
    stoppedAt = None
    while toVisit:
        estTotalCost, cost, pose, prev = heapq.heappop(toVisit)
        if pose in visited:
            continue
        visited[pose] = prev
        if arrived(customDynamicsAPI, pose, goal, trajectorRels, incoming, allowedColliderNames, allowedColliderTypes, allowedApproacherNames, allowedApproacherTypes):
            stoppedAt = pose
            break
        neighbors = getNeighbors(pose, start, end, incoming, allowedApproacherNames, allowedApproacherTypes, preferredPose)
        for neighbor in neighbors:
            if neighbor in visited:
                continue
            costMovement = getMovementCost(customDynamicsAPI, trajectors, pose, neighbor, incoming, allowedColliderNames, allowedColliderTypes, allowedApproacherNames, allowedApproacherTypes, preferredPose)
            if (costMovement is not None):
                costBase = cost + costMovement
                estimatedNew = costBase + estimateTotalCost(customDynamicsAPI, end, neighbor)
                heapq.heappush(toVisit, [estimatedNew, costBase, neighbor, pose])
    while stoppedAt is not None:
        retq.append(stoppedAt)
        stoppedAt = visited[stoppedAt]
    return list(reversed(retq))
        

def updateMotionPlan(name, customDynamicsAPI):
    for ef in customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'endEffectors'), []):
        endPosition, endOrientation = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'motionPlanning', 'targetPose', ef), (None, None))
        allowedApproacherNames =  customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'motionPlanning', 'allowedApproacherNames', ef), [])
        allowedApproacherTypes =  customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'motionPlanning', 'allowedApproacherTypes', ef), [])
        grasped = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'grasping', 'actuallyGrasped', ef), [])
        trajectors = [(name, ef)] + grasped
        incoming = getIncoming(customDynamicsAPI, trajectors, allowedApproacherNames, allowedApproacherTypes)
        if (endPosition is None) and (incoming is None):
            continue
        allowedColliderNames =  customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'motionPlanning', 'allowedColliderNames', ef), [])
        allowedColliderTypes =  customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'motionPlanning', 'allowedColliderTypes', ef), [])
        preferredPosition, preferredOrientation = (None, None)
        refLink = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'motionPlanning', 'preferredPose', 'refLink', ef), None)
        if refLink is not None:
            preferredPosition = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'motionPlanning', 'preferredPose', 'preferredPosition', ef), None)
            preferredOrientation = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'motionPlanning', 'preferredPose', 'preferredOrientation', ef), None)
            if (preferredPosition is None) or (preferredOrientation is None):
                preferredPosition, preferredOrientation = (None, None)
            else:
                positionRef = customDynamicsAPI['getObjectProperty']((name, refLink), 'position')
                orientationRef = customDynamicsAPI['getObjectProperty']((name, refLink), 'orientation')
                preferredPosition, preferredOrientation = customDynamicsAPI['objectPoseRelativeToWorld'](positionRef, orientationRef, preferredPosition, preferredOrientation)
                if ef != trajectors[-1][0]:
                    startPositionEF = customDynamicsAPI['getObjectProperty']((name, ef), 'position')
                    startOrientationEF = customDynamicsAPI['getObjectProperty']((name, ef), 'orientation')
                    relPose = customDynamicsAPI['objectPoseRelativeToObject'](startPositionEF, startOrientationEF, startPosition, startOrientation)
                    preferredPosition, preferredOrientation = customDynamicsAPI['objectPoseRelativeToWorld'](preferredPosition, preferredOrientation, relPose[0], relPose[1])
        startPosition = customDynamicsAPI['getObjectProperty'](trajectors[-1], 'position')
        startOrientation = customDynamicsAPI['getObjectProperty'](trajectors[-1], 'orientation')
        if (incoming is None) and (endPosition is None):
            continue
        if (endPosition is not None):
            d, dq = getDistances(customDynamicsAPI, (startPosition, startOrientation), (endPosition, endOrientation))
            if nearEnough(d, dq):
                continue
        currentPlan = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'motionPlanning', 'currentPlan', ef), [])
        if 0 < len(currentPlan):
            next = currentPlan[0]
            if 0.005 > customDynamicsAPI['distance'](next, startPosition):
                currentPlan = currentPlan[1:]
        if (0 == len(currentPlan)) or (getMovementCost(customDynamicsAPI, trajectors, (startPosition, startOrientation), currentPlan[0], incoming, allowedColliderNames, allowedColliderTypes, allowedApproacherNames, allowedApproacherTypes, None) is None):
            currentPlan = planMotion(customDynamicsAPI, trajectors, (startPosition, startOrientation), (endPosition, endOrientation), incoming, allowedColliderNames, allowedColliderTypes, allowedApproacherNames, allowedApproacherTypes, (preferredPosition, preferredOrientation))
            customDynamicsAPI['setObjectProperty'](('customStateVariables', 'motionPlanning', 'currentPlan', ef), currentPlan)
            if 0 < len(currentPlan):
                customDynamicsAPI['setObjectProperty'](('customStateVariables', 'kinematicControl', 'target', ef), currentPlan[0])
                if ef != trajectors[-1][0]:
                    customDynamicsAPI['setObjectProperty'](('customStateVariables', 'kinematicControl', 'positionInLink', ef), relPose[0])
                    customDynamicsAPI['setObjectProperty'](('customStateVariables', 'kinematicControl', 'orientationInLink', ef), relPose[1])
            else:
                customDynamicsAPI['setObjectProperty'](('customStateVariables', 'kinematicControl', 'target', ef), None)

