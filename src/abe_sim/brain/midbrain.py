import os
import sys
import math
import numpy as np

import abe_sim.brain.geom as geom
from abe_sim.brain.cerebellum import Cerebellum

import math
import heapq

import ast
import json

# A grid of cells which stores whether each cell is passable or not, plus some scaling info to relate the grid to a Cartesian space.
# Assumption: the grid is aligned to the axes of the Cartesian space.
class CellMap:
    def __init__(self, resolution, passables, xMin, yMin):
        self.resolution = resolution
        self.passables = passables
        self.xMin = xMin
        self.yMin = yMin
    def getGridCoordinates(self, cartesianCoordinates):
        return (round((cartesianCoordinates[0] - self.xMin)/self.resolution), round((cartesianCoordinates[1] - self.yMin)/self.resolution))
    def getCartesianCoordinates(self, gridCoordinates):
        return (self.xMin + self.resolution*gridCoordinates[0], self.yMin + self.resolution*gridCoordinates[1])

# A cell containing information about a placement in Cartesian space (which includes the yaw of an agent when they are "at" this cell) and the time to reach.
# Time is initialized as None because originally no information is known about reachability of this cell. Indeed, it may be impossible to reach from some start location.
class TimedMapCell:
    def __init__(self, x, y, yaw):
        self.neighbors = {}
        self.incomingNeighbors={}
        self.x = x
        self.y = y
        self.yaw = yaw
        self.time = None

# Given a map of (im)passable cells and an initial location plus velocities, compute a map of timed cells that can then be used for navigation.
def computeTimedMap(cellMap, pos, velocity, angularVelocity):
    x0 = pos["x"]
    y0 = pos["y"]
    yaw0 = pos["yaw"]
    yaws = [0, 1, 2, 3, 4, 5, 6, 7]
    yaw2Dir = {0: (1, 0), 1: (1, 1), 2: (0, 1), 3: (-1, 1), 4: (-1, 0), 5: (-1, -1), 6: (0, -1), 7: (1, -1)}
    rotationBaseTime = (math.pi/4)/angularVelocity
    retq = []
    # initialize timed cells
    for ky, line in enumerate(cellMap.passables):
        auxL = []
        for kx, cell in enumerate(line):
            auxR = {}
            for kyaw in yaws:
                x, y = cellMap.getCartesianCoordinates((kx, ky))
                auxR[kyaw] = TimedMapCell(x, y, kyaw*math.pi/4)
            auxL.append(auxR)
        retq.append(auxL)
    # create their neighboring relations
    for ky, line in enumerate(retq):
        for kx, cell in enumerate(line):
            if not cellMap.passables[ky][kx]:
                continue
            for kyaw in cell.keys():
                # create relationships for neighboring yaws at the same (x, y) location:
                kyawPlus = (kyaw + 1)%8
                kyawMinus = (kyaw - 1)%8
                cell[kyaw].neighbors[(kx, ky, kyawPlus)] = rotationBaseTime
                cell[kyaw].neighbors[(kx, ky, kyawMinus)] = rotationBaseTime
                cell[kyaw].incomingNeighbors[(kx, ky, kyawPlus)] = rotationBaseTime
                cell[kyaw].incomingNeighbors[(kx, ky, kyawMinus)] = rotationBaseTime
                # create relationships for neighboring (x,y) locations but at the same yaw:
                pX = [0]
                if 0 < kx:
                    pX.append(-1)
                if kx < len(line)-1:
                    pX.append(1)
                pY = [0]
                if 0 < ky:
                    pY.append(-1)
                if ky < len(retq)-1:
                    pY.append(1)
                iX = yaw2Dir[kyaw][0]
                iY = yaw2Dir[kyaw][1]
                if (iX in pX) and (iY in pY):
                    if cellMap.passables[ky+iY][kx+iX]:
                        cell[kyaw].neighbors[(kx+iX, ky+iY, kyaw)] = math.sqrt(iX*iX + iY*iY)*cellMap.resolution/velocity
                iX = -iX
                iY = -iY
                if (iX in pX) and (iY in pY):
                    if cellMap.passables[ky+iY][kx+iX]:
                        cell[kyaw].incomingNeighbors[(kx+iX, ky+iY, kyaw)] = math.sqrt(iX*iX + iY*iY)*cellMap.resolution/velocity
    # run Dijkstra algorithm
    #  identify start cell. We'll assume the start in Cartesian space is on the map!
    toVisit = []
    kx, ky = cellMap.getGridCoordinates((x0, y0))
    pX = [0]
    pY = [0]
    if 0 < kx:
        pX.append(-1)
    if len(cellMap.passables[0]) - 1 > kx:
        pX.append(1)
    if 0 < ky:
        pY.append(-1)
    if len(cellMap.passables[0]) - 1 > ky:
        pY.append(1)
    for iY in pY:
        for iX in pX:
            for kyaw in range(8):
                cx, cy = cellMap.getCartesianCoordinates((kx + iX, ky + iY))
                cYaw = kyaw*math.pi/4
                dx = cx-x0
                dy = cy-y0
                d = math.sqrt(dx*dx+dy*dy)
                t = d/velocity
                if 0.001 < d:
                    wYaw = math.atan2(dy, dx)
                    t = t + (abs(geom.angle_diff(wYaw, yaw0)) + abs(geom.angle_diff(cYaw, wYaw)))/angularVelocity
                else:
                    t = t + abs(geom.angle_diff(cYaw, yaw0))/angularVelocity
                heapq.heappush(toVisit, (t, kx + iX, ky + iY, kyaw))
    #  run the algorithm proper
    while toVisit:
        time, kx, ky, kyaw = heapq.heappop(toVisit)
        oldTime = retq[ky][kx][kyaw].time
        if (None == oldTime) or (time < oldTime):
            retq[ky][kx][kyaw].time = time
            for idx, stepTime in retq[ky][kx][kyaw].neighbors.items():
                heapq.heappush(toVisit, (time+stepTime, idx[0], idx[1], idx[2]))
    return retq

def simplifyPath(path):
    retq = []
    if path:
        ops = []
        cX = path[0]["x"]
        cY = path[0]["y"]
        cA = path[0]["yaw"]
        for wp in path[1:]:
            dx = cX - wp["x"]
            dy = cY - wp["y"]
            d = math.sqrt(dx*dx + dy*dy)
            da = geom.angle_diff(cA, wp["yaw"])
            if 0.001 < d:
                ops.append("fwd")
            elif 0.001 < da:
                ops.append("a+")
            elif -0.001 > da:
                ops.append("a-")
            cX = wp["x"]
            cY = wp["y"]
            cA = wp["yaw"]
        ops.append("end")
        cOp = None
        for k, op in enumerate(ops):
            if None == cOp:
                cOp = op
            elif "end" == cOp:
                retq.append(path[k])
            elif cOp != op:
                retq.append(path[k])
                cOp = op
    return retq

# Uses a collection of timed cells to produce a path from a start location to a destination one.
def computePath(cellMap, timedMap, pos0, posF, velocity=1, angularVelocity=1):
    angularVelocity = angularVelocity
    x0 = pos0["x"]
    y0 = pos0["y"]
    yaw0 = pos0["yaw"]
    xf = posF["x"]
    yf = posF["y"]
    yawf = posF["yaw"]
    retq = [{"x": xf, "y": yf, "yaw": yawf}]
    kx, ky = cellMap.getGridCoordinates((xf, yf))
    kyaw = round(yawf/(math.pi/4))%8
    # what if (closest) cell is unreachable? Then return no path
    if None == timedMap[ky][kx][kyaw].time:
        return []
    pX = [0]
    pY = [0]
    if 0 < kx:
        pX.append(-1)
    if len(cellMap.passables[0]) - 1 > kx:
        pX.append(1)
    if 0 < ky:
        pY.append(-1)
    if len(cellMap.passables[0]) - 1 > ky:
        pY.append(1)
    minT = None
    minkX = None
    minkY = None
    minkYaw = None
    for iY in pY:
        for iX in pX:
            for kyaw in range(8):
                if None == timedMap[ky+iY][kx+iX][kyaw].time:
                    continue
                cx, cy = cellMap.getCartesianCoordinates((kx + iX, ky + iY))
                cYaw = kyaw*math.pi/4
                dx = xf-cx
                dy = yf-cy
                d = math.sqrt(dx*dx+dy*dy)
                t = d/velocity
                if 0.001 < d:
                    wYaw = math.atan2(dy, dx)
                    t = t + (abs(geom.angle_diff(yawf, wYaw)) + abs(geom.angle_diff(wYaw, cYaw)))/angularVelocity
                else:
                    t = t + abs(geom.angle_diff(yawf, cYaw))/angularVelocity
                if (None == minT) or (t+timedMap[ky+iY][kx+iX][kyaw].time < minT):
                    minT = t+timedMap[ky+iY][kx+iX][kyaw].time
                    minkX = kx+iX
                    minkY = ky+iY
                    minkYaw = kyaw
    kx = minkX
    ky = minkY
    kyaw = minkYaw
    cx, cy = cellMap.getCartesianCoordinates((kx, ky))
    retq.append({"x": cx, "y": cy, "yaw": kyaw*math.pi/4})
    onPath = True
    while onPath:
        onPath = False
        minTime = timedMap[ky][kx][kyaw].time
        for idx in timedMap[ky][kx][kyaw].incomingNeighbors.keys():
            if (None != timedMap[idx[1]][idx[0]][idx[2]].time) and (timedMap[idx[1]][idx[0]][idx[2]].time < minTime):
                onPath = True
                minTime = timedMap[idx[1]][idx[0]][idx[2]].time
                kx, ky, kyaw = idx
        if onPath:
            cx, cy = cellMap.getCartesianCoordinates((kx, ky))
            retq.append({"x": cx, "y": cy, "yaw": kyaw*math.pi/4})
    # No need to append x0, y0, yaw0 as a waypoint since agent should already be there
    retq.reverse()
    return simplifyPath(retq)

def simpleOnNavigationDoneCallback(x):
    print("Arrived at %s" % x)

class Midbrain:
    def __init__(self, headActuator, handsActuator, baseActuator, poseSensor, worldDump, simu):
        self.cerebellum = Cerebellum(headActuator, handsActuator, baseActuator, poseSensor)
        self.cerebellum.initializePosition("hands/left", {"x": 0, "y": 0.4, "z": 0.96, "roll": 0, "pitch": 0, "yaw": 0})
        self.cerebellum.initializePosition("hands/right", {"x": 0, "y": -0.4, "z": 0.96, "roll": 0, "pitch": 0, "yaw": 0})
        self.cerebellum.initializePosition("head", {"pan": 0, "tilt": 0})
        self.worldDump = worldDump
        self.cellMap = None
        self.collisionManager = geom.BoxCollisionManager()
        self.simu = simu
    def _retrieveObjects(self):
        haveObjs = False
        while not haveObjs:
            try:
                objects = json.loads(self.simu.rpc('robot.worlddump', 'world_dump'))
                haveObjs = True
            except OSError:
                continue
        return objects
    def listObjects(self):
        objects = self._retrieveObjects()
        for k in sorted(objects.keys()):
            otype = "\t(untyped)\n"
            if "type" in objects[k]:
                otype = "\t"+objects[k]["type"]+"\n"
            description = "\n"
            if "description" in objects[k]:
                description = "\t"+objects[k]["description"]+"\n"
            graspable = "\t(not graspable)"
            if "graspable" in objects[k]:
                if objects[k]["graspable"]:
                    graspable = "\tgraspable\n"
                else:
                    graspable = "\tnot graspable\n"
            furniture = "\t(not furniture)\n"
            if "furniture" in objects[k]:
                if objects[k]["furniture"]:
                    furniture = "\tfurniture\n"
                else:
                    furniture = "\tnot furniture\n"
            meshfile = "\n"
            if "mesh" in objects[k]:
                meshfile = "\t"+objects[k]["mesh"]+"\n"
            position = "\t(x: %f; y: %f; z: %f)\n" % (objects[k]["position"]["x"], objects[k]["position"]["y"], objects[k]["position"]["z"])
            orientation = "\t(x: %f; y: %f; z: %f; w: %f)\n" % (objects[k]["orientation"]["x"], objects[k]["orientation"]["y"], objects[k]["orientation"]["z"], objects[k]["orientation"]["w"])
            s = k+"\n"+otype+description+graspable+furniture+meshfile+position+orientation
            print(s)
    def updateNavigationMap(self):
        objects = self._retrieveObjects()
        self.collisionManager.clear_objects()
        for k in objects.keys():
            if ("furniture" in objects[k]) and objects[k]["furniture"]:
                box = geom.boxFromPath(objects[k]["mesh"])
                if box:
                    self.collisionManager.add_object(k, box, ((objects[k]["position"]["x"], objects[k]["position"]["y"], objects[k]["position"]["z"]), (objects[k]["orientation"]["x"], objects[k]["orientation"]["y"], objects[k]["orientation"]["z"], objects[k]["orientation"]["w"])))
        testBox = geom.Box()
        testBox.vertices = [[-0.5, -0.5, 0], [0.5, -0.5, 0], [-0.5, 0.5, 0], [0.5, 0.5, 0], [-0.5, -0.5, 1], [0.5, -0.5, 1], [-0.5, 0.5, 1], [0.5, 0.5, 1]]
        passables = []
        for k in range(10):
            passables.append([False]*10)
        xMin = -4.5
        yMin = -4.5
        resolution = 1
        for k in range(10):
            for j in range(10):
                if not self.collisionManager.in_collision_single(testBox, ((xMin + j*resolution, yMin + k*resolution, 0), (0, 0, 0, 1))):
                    passables[k][j] = True
        self.cellMap = CellMap(resolution, passables, xMin, yMin)
    def startOperations(self, onNavigationDoneCallback=simpleOnNavigationDoneCallback):
        self.updateNavigationMap()
        self.cerebellum.setCallback("base", onNavigationDoneCallback)
        self.cerebellum.startMonitoring()
    def navigateToPosition(self, x, y, yaw):
        if not self.cellMap:
            print("Don't have a navigation map: perhaps startOperation has not been called?")
            return False
        crPos = self.cerebellum.currentPosition("base")
        finPos = {"x": x, "y": y, "yaw": yaw}
        timedMap = computeTimedMap(self.cellMap, crPos, 3, 3)
        waypoints = computePath(self.cellMap, timedMap, crPos, finPos, 3, 3)
        self.cerebellum.clearWaypoints("base")
        if waypoints:
            for wp in waypoints:
                self.cerebellum.pushWaypoint("base", wp)
            print("On our way!")
            return True
        print("Target unreachable: out of map, or no clear paths to it.")
        return False

