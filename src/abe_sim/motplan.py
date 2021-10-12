import pybullet as p
import math

import random

from abe_sim.utils import stubbornTry
from abe_sim.geom import quaternionProduct, overlappingObjects, translateVector, scaleVector, vectorNorm, vectorNormalize, extrudeBox, distance, interpolatePoint, boxHasPoint, splitBox

def validExtrusion(boxes, start, end, allowableCollisionFn, world, debug=False):
    extrudedBoxes = [extrudeBox(x, start, end) for x in boxes]
    retq = [allowableCollisionFn(x, overlappingObjects(*(list(x) + [world]))) for x in extrudedBoxes]
    if debug:
        print(retq)
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

def detailedTestValid(box, obj):
    details = obj.getBodyProperty("fn", "boards")
    if None == details:
        return False
    position = obj.getBodyProperty((), "position")
    orientation = obj.getBodyProperty((), "orientation")
    details = [adjustBox(x, position, orientation) for x in details]
    for d in details:
        if overlappingBoxes(d,box):
            return False
    return True

def allowCollisionByWhitelist(box, collidingObjects, whitelistNames=[], whitelistTypes=[]):
    for o in collidingObjects:
        if (o.getName() not in whitelistNames) and (o.getBodyProperty("", "type") not in whitelistTypes):
            if not detailedTestValid(box, o):
                return False
    return True

class Corridor:
    def __init__(self, waypoints=[]):
        self.waypoints = list(waypoints)
    def validCorridors(self, trajectorBoxes, allowableCollisionFn, world):
        if 0 == len(self.waypoints):
            return [None, None]
        fromEnd = []
        fromStart = []
        e = None
        for s in reversed(self.waypoints):
            if validExtrusion(trajectorBoxes, s, e, allowableCollisionFn, world):
                fromEnd.append(s)
                e = s
            else:
                break
        if len(fromEnd) == len(self.waypoints):
            return [self, self]
        fromEnd = list(reversed(fromEnd))
        e = None
        for s in self.waypoints:
            if validExtrusion(trajectorBoxes, s, e, allowableCollisionFn, world):
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
    def nextAlong(self, point, tolerance=0.1):
        tolerance = abs(tolerance)
        if 0 == len(self.waypoints):
            return None, None
        e = None
        retq = (self.waypoints[0], 0)
        for k, s in enumerate(self.waypoints):
            if boxHasPoint(extrudeBox([[-tolerance]*3, [tolerance]*3], s, e), point):
                retq = (s, k)
            e = s
        return retq

class CorridorTree:
    def __init__(self, corridor=None, reverse=False):
        self.waypoints = {}
        wps = []
        if None != corridor:
            wps = corridor.waypoints
        if reverse:
            wps = list(reversed(wps))
        iK = None
        for k, p in enumerate(wps):
            self.waypoints[k] = (p, iK)
            iK = k
    def expandTowards(self, point, trajectorBoxes, allowableCollisionFn, world):
        minD = None
        minK = None
        for k, p in self.waypoints.items():
            d = distance(p[0], point)
            if (None == minD) or (d < minD):
                minD = d
                minK = k
        f = 1.0
        p = self.waypoints[minK][0]
        while True:
            pI = interpolatePoint(p, point, f)
            if validExtrusion(trajectorBoxes, p, pI, allowableCollisionFn, world):
                self.waypoints[len(self.waypoints)] = (pI, minK)
                break
            f = f*0.75

def samplePoint(sampleBox, position, orientation):
    sampleMin, sampleMax = sampleBox
    return translateVector(position, p.rotateVector(orientation, [random.uniform(a,b) for a,b in zip(sampleMin, sampleMax)]))

def connectTrees(endTree, startTree, boxes, allowableCollisionFn, world, tryAll=False):
    tryingPairs = []
    if not tryAll:
        tryingPairs = [(len(endTree.waypoints)-1, len(startTree.waypoints)-1)]
    else:
        for ke in endTree.waypoints.keys():
            for ks in startTree.waypoints.keys():
                tryingPairs.append((ke, ks))
    minD = None
    minP = None
    retq = None
    for tp in tryingPairs:
        pa = endTree.waypoints[tp[0]][0]
        pb = startTree.waypoints[tp[1]][0]
        if validExtrusion(boxes, pa, pb, allowableCollisionFn, world):
            d = distance(pa, pb)
            if (None == minD) or (d < minD):
                minD = d
                minP = tp
    if None != minD:
        waypoints = []
        pe = minP[0]
        ps = minP[1]
        while None != ps:
            waypoints.append(startTree.waypoints[ps][0])
            ps = startTree.waypoints[ps][1]
        waypoints = list(reversed(waypoints))
        while None != pe:
            waypoints.append(endTree.waypoints[pe][0])
            pe = endTree.waypoints[pe][1]
        retq = Corridor(waypoints)
    return retq

def planCorridor(sampleBox, trajectorBoxes, allowableCollisionFn, world, endCorridor, startCorridor, position, orientation, samples=1000):
    print("PLANCORRIDOR", endCorridor, startCorridor)
    endTree = CorridorTree(endCorridor, True)
    startTree = CorridorTree(startCorridor, False)
    connectedCorridor = None
    tryAll = True
    print("    SAMPLING")
    while (0 < samples):
        samples = samples - 1
        print("        ", samples)
        connectedCorridor = connectTrees(endTree, startTree, trajectorBoxes, allowableCollisionFn, world, tryAll=tryAll)
        print("        cc", samples)
        tryAll = False
        if None != connectedCorridor:
            print("DONE")
            return connectedCorridor
        sample = samplePoint(sampleBox, position, orientation)
        print("        sp", samples)
        endTree.expandTowards(sample, trajectorBoxes, allowableCollisionFn, world)
        print("        ee", samples)
        startTree.expandTowards(sample, trajectorBoxes, allowableCollisionFn, world)
    return None

def getCleanStart(position, trajectorBoxes, allowableCollisionFn, world, samples=1000):
    splitBoxes = [splitBox(x) for x in trajectorBoxes]
    cleanSplits = [[validExtrusion(x, position, position, allowableCollisionFn, world) for x in sb] for sb in splitBoxes]
    dirChecks = [((0,1,2,3), (0,0,-1)), ((4,5,6,7), (0,0,1)), ((0,2,4,6), (-1,0,0)), ((1,3,5,7), (1,0,0)), ((0,1,4,5), (0,-1,0)), ((2,3,6,7), (0,1,0))]
    free = {0: True, 1: True, 2: True, 3: True, 4: True, 5: True, 6: True, 7: True}
    for cs in cleanSplits:
        for k in range(8):
            if not cs[k]:
                free[k] = False
    available = []
    step = []
    for dc in dirChecks:
        flags, direction = dc
        if all([free[x] for x in flags]):
            available.append(direction)
            step.append(0.05)
    if [] == available:
        ### TODO: warning? return None? search farther?
        available = [(0,0,1)]
        step.append(0.05)
    k = 0
    while (0 < samples):
        samples = samples - 1
        pAdj = translateVector(position, scaleVector(available[k], step[k]))
        step[k] = step[k]+0.01
        k = k + 1
        if k >= len(step):
            k = 0
        if validExtrusion(trajectorBoxes, pAdj, pAdj, allowableCollisionFn, world):
            return Corridor(waypoints=[pAdj])
    return None

