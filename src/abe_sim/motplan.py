import pybullet as p
import math

import heapq

import random

from abe_sim.utils import stubbornTry
from abe_sim.geom import quaternionProduct, overlappingObjects, translateVector, scaleVector, vectorNorm, vectorNormalize, extrudeBox, distance, interpolatePoint, boxHasPoint, splitBox

def validExtrusion(boxes, start, end, allowableCollisionFn, world, debug=False):
    extrudedBoxes = [extrudeBox(x, start, end) for x in boxes]
    retq = [allowableCollisionFn(x, overlappingObjects(*(list(x) + [world]))) for x in extrudedBoxes]
    if debug:
        print(retq, [(x, overlappingObjects(*(list(x) + [world]))) for x in extrudedBoxes])
    return all(retq)

def adjustBox(box, pos, q):
    aabbmin, aabbmax = box
    aabbmin = translateVector(pos,p.rotateVector(q,aabbmin))
    aabbmax = translateVector(pos,p.rotateVector(q,aabbmax))
    c1 = [aabbmin[0],aabbmin[1],aabbmax[2]]
    c2 = [aabbmin[0],aabbmax[1],aabbmin[2]]
    c3 = [aabbmin[0],aabbmax[1],aabbmax[2]]
    c4 = [aabbmax[0],aabbmin[1],aabbmax[2]]
    c5 = [aabbmax[0],aabbmax[1],aabbmin[2]]
    c6 = [aabbmax[0],aabbmax[1],aabbmax[2]]
    aabbmin = [min(a,b,c,d,e,f,g,h) for a,b,c,d,e,f,g,h in zip(aabbmin,aabbmax,c1,c2,c3,c4,c5,c6)]
    aabbmax = [max(a,b,c,d,e,f,g,h) for a,b,c,d,e,f,g,h in zip(aabbmin,aabbmax,c1,c2,c3,c4,c5,c6)]
    return aabbmin, aabbmax

def overlappingBoxes(a,b):
    def fb(xm, x, xM):
        return (xm <= x) and (x <= xM)
    def fn(xm1, xM1, xm2, xM2):
        return fb(xm1, xm2, xM1) or fb(xm1, xM2, xM1) or fb(xm2, xm1, xM2) or fb(xm2, xM1, xM2)
    return all([fn(x,y,z,w) for x,y,z,w in zip(a[0],a[1],b[0],b[1])])

def detailedTestValid(box, obj, debug=False):
    details = obj.getBodyProperty("fn", "boards")
    if None == details:
        if debug:
            print("    No detailed test because no detailed boxes, and the overall box is in collision.")
        return False
    position = obj.getBodyProperty((), "position")
    orientation = obj.getBodyProperty((), "orientation")
    details = [adjustBox(x, position, orientation) for x in details]
    for d in details:
        if overlappingBoxes(d,box):
            if debug:
                print("    detailed collision", d)
            return False
    return True

def allowCollisionByWhitelist(box, collidingObjects, whitelistNames=[], whitelistTypes=[], debug=False):
    if debug:
        print("aCBWB", collidingObjects, whitelistNames, whitelistTypes)
    for o in collidingObjects:
        if (o.getName() not in whitelistNames) and (o.getBodyProperty("", "type") not in whitelistTypes):
            if debug:
                print("    ?", o.getName())
            if not detailedTestValid(box, o, debug=debug):
                if debug:
                    print("        colliding", box, o.getAABB(None))
                return False
    return True

class Corridor:
    def __init__(self, waypoints=[]):
        self.waypoints = list(waypoints)
    def validCorridors(self, trajectorBoxes, allowableCollisionFn, world,debug=False):
        if 0 == len(self.waypoints):
            return [None, None]
        fromEnd = []
        fromStart = []
        e = None
        for s in reversed(self.waypoints):
            if validExtrusion(trajectorBoxes, s, e, allowableCollisionFn, world, debug=debug):
                fromEnd.append(s)
                e = s
            else:
                break
        if len(fromEnd) == len(self.waypoints):
            return [self, self]
        fromEnd = list(reversed(fromEnd))
        e = None
        for s in self.waypoints:
            if validExtrusion(trajectorBoxes, s, e, allowableCollisionFn, world, debug=debug):
                fromStart.append(s)
                e = s
            else:
                break
        retqFromEnd = None
        retqFromStart = None
        if (0 < len(fromEnd)):
            retqFromEnd = Corridor(fromEnd)
        if (0 < len(fromStart)):
            retqFromStart = Corridor(fromStart)
        return [retqFromEnd, retqFromStart]
    def nextAlong(self, point, boxes, allowableCollisionFn, world):
        if 0 == len(self.waypoints):
            return None, None
        retq = (self.waypoints[0], 0)
        for k, s in enumerate(self.waypoints):
            if validExtrusion(boxes, point, s, allowableCollisionFn, world):
                retq = (s, k)
        return retq

def planCorridor(sampleBox, trajectorBoxes, allowableCollisionFn, world, end, start, position, orientation, debug=False):
    start = tuple(start)
    end = tuple(end)
    iOrientation = [orientation[0], orientation[1], orientation[2], -orientation[3]]
    iPosition = scaleVector(p.rotateVector(iOrientation, position), -1)
    visited = {}
    toVisit = [[distance(start,end), 0, start, None]]
    if debug:
        print(start, end, trajectorBoxes, sampleBox, position, orientation)
    if not validExtrusion(trajectorBoxes, start, start, allowableCollisionFn, world, debug=debug):
        print("START NOT VALID!!!!!", end, world._pobjects["abe"].getBodyProperty(("hand_right_roll",), "position"))
        extrudedBoxes = [extrudeBox(x, start, start) for x in trajectorBoxes]
        collidingObjects = [overlappingObjects(*(list(x) + [world])) for x in extrudedBoxes][0]
        print("    ", collidingObjects)
        for box in extrudedBoxes:
            allowCollisionByWhitelist(box, collidingObjects, whitelistNames=["abe"], whitelistTypes=[], debug=True)
    if not validExtrusion(trajectorBoxes, end, end, allowableCollisionFn, world, debug=True):
        print("GOAL NOT VALID!!!!!", end, world._pobjects["abe"].getBodyProperty(("hand_right_roll",), "position"))
        extrudedBoxes = [extrudeBox(x, end, end) for x in trajectorBoxes]
        collidingObjects = [overlappingObjects(*(list(x) + [world])) for x in extrudedBoxes][0]
        print("    ", collidingObjects)
        for box in extrudedBoxes:
            allowCollisionByWhitelist(box, collidingObjects, whitelistNames=["abe"], whitelistTypes=[], debug=True)
    while toVisit:
        estTotalCost, cost, pos, prev = heapq.heappop(toVisit)
        if pos in visited:
            continue
        visited[pos] = prev
        if not validExtrusion(trajectorBoxes, pos, pos, allowableCollisionFn, world):
            cost = cost + 1000
        if validExtrusion(trajectorBoxes, pos, end, allowableCollisionFn, world):
            if 0.001 > distance(pos,end):
                visited[end] = prev
            else:
                visited[end] = pos
            break
        eCost = 0.1
        if 0.1 > distance(pos, end):
            if not validExtrusion(trajectorBoxes, pos, end, allowableCollisionFn, world):
                eCost = 1000
            heapq.heappush(toVisit, [eCost+cost, eCost+cost, end, pos])
        for ix, iy, iz in [[0.1, 0, 0], [-0.1, 0, 0], [0, 0.1, 0], [0, -0.1, 0], [0, 0, 0.1], [0, 0, -0.1]]:
            posAdj = tuple(translateVector(pos, [ix, iy, iz]))
            if not boxHasPoint(sampleBox, translateVector(iPosition, p.rotateVector(iOrientation, posAdj))):
                continue
            hCost = distance(posAdj, end)
            if not validExtrusion(trajectorBoxes, pos, posAdj, allowableCollisionFn, world):
                eCost = 1000
            heapq.heappush(toVisit, [hCost+eCost+cost, eCost+cost, posAdj, pos])
    if end not in visited:
        return None
    retq = [end]
    while True:
        if None == visited[retq[-1]]:
            break
        retq.append(visited[retq[-1]])
    retq = list(reversed(retq))
    return Corridor(waypoints=retq)

