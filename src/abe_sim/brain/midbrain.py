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

import schemasim.space.space2D as space2D
import schemasim.space.space as space

def simpleOnNavigationDoneCallback(x):
    print("Arrived at %s" % x)

class Validator2DVW:
    def __init__(self, collisionManager, trajector):
        self.collisionManager = collisionManager
        self.trajector = trajector
        return
    def isValid(self, coordinates):
        return not self.collisionManager.in_collision_single(self.trajector, ((coordinates[0], coordinates[1], 0), (0, 0, 0, 1)))

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
    def _simplifyWaypoints(self, waypoints):
        retq = []
        if waypoints:
            ops = []
            cX = waypoints[0][0]
            cY = waypoints[0][1]
            cA = waypoints[0][2]
            for wp in waypoints[1:]:
                dx = cX - wp[0]
                dy = cY - wp[1]
                d = math.sqrt(dx*dx + dy*dy)
                da = geom.angle_diff(cA, wp[2])
                if 0.001 < d:
                    ops.append("fwd")
                elif 0.001 < da:
                    ops.append("a+")
                elif -0.001 > da:
                    ops.append("a-")
                cX = wp[0]
                cY = wp[1]
                cA = wp[2]
            ops.append("end")
            cOp = None
            for k, op in enumerate(ops):
                if None == cOp:
                    cOp = op
                elif "end" == cOp:
                    coords = self.cellMap.pointId2EmbeddingCoordinates(waypoints[k])
                    retq.append({"x": coords[0], "y": coords[1], "yaw": coords[2]})
                elif cOp != op:
                    coords = self.cellMap.pointId2EmbeddingCoordinates(waypoints[k])
                    retq.append({"x": coords[0], "y": coords[1], "yaw": coords[2]})
                    cOp = op
        return retq
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
        self.cellMap = space2D.Grid2DVW8(lines=10, cols=10, resolution=1, xLeft=-4.5, yDown=-4.5, gridYaw=0, validator=Validator2DVW(self.collisionManager, testBox), velocity=3, angularVelocity=3)
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
        timedMap = space.TimedPointGraph(self.cellMap, self.cellMap.graphIngressPoints((crPos["x"], crPos["y"], crPos["yaw"])))
        pId = None
        pT = None
        for p, t in self.cellMap.graphEgressPoints((x, y, yaw)).items():
            if None == t:
                continue
            if None == pT:
                pT = t + timedMap.pointTime(p)
                pId = p
            else:
                t = t + timedMap.pointTime(p)
                if pT > t:
                    pT = t
                    pId = p
        waypoints = []
        if None != pId:
            waypoints = timedMap.generatePath(pId)
            waypoints = self._simplifyWaypoints(waypoints)
            waypoints.append({"x": x, "y": y, "yaw": yaw})
        self.cerebellum.clearWaypoints("base")
        if waypoints:
            for wp in waypoints:
                self.cerebellum.pushWaypoint("base", wp)
            print("On our way!")
            return True
        print("Target unreachable: out of map, or no clear paths to it.")
        return False

