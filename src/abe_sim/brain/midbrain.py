import os
import sys
import math
import time
import numpy as np

import abe_sim.brain.geom as geom
from abe_sim.brain.cerebellum import Cerebellum

import math
import heapq

import ast
import json

import schemasim.space.space3D as space3D
import schemasim.space.space2D as space2D
import schemasim.space.space as space

import schemasim.schemas.l0_schema_templates as st
import schemasim.schemas.l1_geometric_primitives as gp
import schemasim.schemas.l2_geometric_primitive_relations as gpr

import schemasim.objects.example_objects as eo

import schemasim.simulators.physics_simulator_2D as ps2D
import schemasim.simulators.physics_simulator_3D as ps3D

import schemasim.scene_generator as sg

from schemasim.util.geometry import fibonacci_sphere

def simpleOnNavigationDoneCallback(x):
    print("Base arrived at %s" % x)

def simpleHandsLeftPositioningDoneCallback(x):
    print("Left hand arrived at %s" % x)

def simpleHandsRightPositioningDoneCallback(x):
    print("Right hand arrived at %s" % x)

class Validator2DVW:
    def __init__(self, collisionManager, trajector):
        self.collisionManager = collisionManager
        self.trajector = trajector
        return
    def isValid(self, coordinates):
        return not self.collisionManager.in_collision_single(self.trajector, ((coordinates[0], coordinates[1], 0), (0, 0, 0, 1)))

class Validator3D:
    def __init__(self, collisionManager, trajectors, space):
        self.collisionManager = collisionManager
        self.trajectors = trajectors
        self.space = space
        return
    def isValid(self, coordinates):
        for trajector, transform in self.trajectors:
            pose = self.space.poseFromTR((transform[0][0]+coordinates[0], transform[0][1]+coordinates[1], transform[0][2]+coordinates[2]), transform[1])
            if self.collisionManager.in_collision_single(trajector, pose):
                return False
        return True

class Midbrain:
    def __init__(self, headActuator, handsActuator, baseActuator, poseSensor, worldDump, simu):
        self.cerebellum = Cerebellum(headActuator, handsActuator, baseActuator, poseSensor, simu, worldDump)
        self.cerebellum.initializePosition("hands/left", {"x": 0, "y": 0.4, "z": 0.96, "roll": 0, "pitch": 0, "yaw": 0})
        self.cerebellum.initializePosition("hands/right", {"x": 0, "y": -0.4, "z": 0.96, "roll": 0, "pitch": 0, "yaw": 0})
        self.cerebellum.initializePosition("head", {"pan": 0, "tilt": 0})
        self.worldDump = worldDump
        self.cellMap = None
        self.collisionManager = geom.BoxCollisionManager()
        self.simu = simu
        self.sim2D = ps2D.PhysicsSimulator2D(particleSamplingResolution=0.1, translationSamplingResolution=1.0, rotationSamplingResolution=math.pi/4, speedSamplingResolution=1.0)
        self.sim3D = ps3D.PhysicsSimulator3D(particleSamplingResolution=0.01, translationSamplingResolution=0.1, rotationSamplingResolution=0.1, speedSamplingResolution=0.1, sampleValidationStrictness=0.005, collisionPadding=0.005)
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
    def getObjectSchemas(self):
        pathPrefix = os.path.join(os.path.dirname(__file__), "../meshes")
        objects = self.cerebellum._retrieveObjects()
        retq = {}
        for k,o in objects.items():
            retq[k] = eo.MiscellaneousRigidObject(name=k, object_type=o["type"], mesh=os.path.join(pathPrefix, o["mesh"]))
            retq[k]._parameters["tx"] = o["position"]["x"]
            retq[k]._parameters["ty"] = o["position"]["y"]
            retq[k]._parameters["tz"] = o["position"]["z"]
            retq[k]._parameters["rx"] = o["orientation"]["x"]
            retq[k]._parameters["ry"] = o["orientation"]["y"]
            retq[k]._parameters["rz"] = o["orientation"]["z"]
            retq[k]._parameters["rw"] = o["orientation"]["w"]
            retq[k]._parameters["vx"] = 0.0
            retq[k]._parameters["vy"] = 0.0
            retq[k]._parameters["vz"] = 0.0
            retq[k]._parameters["wx"] = 0.0
            retq[k]._parameters["wy"] = 0.0
            retq[k]._parameters["wz"] = 0.0
        return retq
    def listObjects(self):
        objects = self.cerebellum._retrieveObjects()
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
        objects = self.cerebellum._retrieveObjects()
        self.collisionManager.clear_objects()
        for k in objects.keys():
            if ("furniture" in objects[k]) and objects[k]["furniture"]:
                box = geom.boxFromPath(objects[k]["mesh"])
                if box:
                    self.collisionManager.add_object(k, box, ((objects[k]["position"]["x"], objects[k]["position"]["y"], objects[k]["position"]["z"]), (objects[k]["orientation"]["x"], objects[k]["orientation"]["y"], objects[k]["orientation"]["z"], objects[k]["orientation"]["w"])))
        testBox = geom.Box()
        testBox.vertices = [[-0.5, -0.5, 0], [0.5, -0.5, 0], [-0.5, 0.5, 0], [0.5, 0.5, 0], [-0.5, -0.5, 1], [0.5, -0.5, 1], [-0.5, 0.5, 1], [0.5, 0.5, 1]]
        self.cellMap = space2D.Grid2DVW8(lines=10, cols=10, resolution=1, xLeft=-4.5, yDown=-4.5, gridYaw=0, validator=Validator2DVW(self.collisionManager, testBox), velocity=3, angularVelocity=3)
    def startOperations(self, onNavigationDoneCallback=simpleOnNavigationDoneCallback, onHandsLeftPositioningDoneCallback=simpleHandsLeftPositioningDoneCallback, onHandsRightPositioningDoneCallback=simpleHandsRightPositioningDoneCallback):
        self.updateNavigationMap()
        self.cerebellum.setCallback("base", onNavigationDoneCallback)
        self.cerebellum.setCallback("hands/left", onHandsLeftPositioningDoneCallback)
        self.cerebellum.setCallback("hands/right", onHandsRightPositioningDoneCallback)
        self.cerebellum.startMonitoring()
    def navigateToPosition(self, x, y, yaw):
        if not self.cellMap:
            print("Don't have a navigation map: perhaps startOperation has not been called?")
            return False
        crPos = self.cerebellum.currentPosition("base")
        finPos = {"x": x, "y": y, "yaw": yaw}
        timedMap = space.TimedPointGraph(self.cellMap, self.cellMap.graphIngressPoints((crPos["x"], crPos["y"], crPos["yaw"])))
        waypoints = timedMap.generatePath((x, y, yaw))
        self.cerebellum.clearWaypoints("base")
        if waypoints:
            waypoints = self._simplifyWaypoints(waypoints)
            waypoints.append({"x": x, "y": y, "yaw": yaw})
            for wp in waypoints:
                self.cerebellum.pushWaypoint("base", wp)
            print("On our way!")
            return True
        print("Target unreachable: out of map, or no clear paths to it.")
        return False
    def _makeValidator(self, objects, blacklistNames, trajectors):
        pathPrefix = os.path.join(os.path.dirname(__file__), "../meshes")
        collisionManager = self.sim3D.space().makeCollisionManager()
        for oname in objects.keys():
            if oname in blacklistNames:
                continue
            mesh = self.sim3D.space().loadVolume(os.path.join(pathPrefix, objects[oname]["mesh"]))
            oC = objects[oname]
            oCP = [oC["position"]["x"], oC["position"]["y"], oC["position"]["z"]]
            oCQ = [oC["orientation"]["x"], oC["orientation"]["y"], oC["orientation"]["z"], oC["orientation"]["w"]]
            rP, rQ = self.cerebellum.robotTransform()
            invRobT = self.sim3D.space().invertTransform((rP, rQ))
            oCP, oCQ = self.sim3D.space().transformTransform(invRobT, (oCP, oCQ))
            collisionManager.add_object(oname, mesh, self.sim3D.space().poseFromTR(oCP, oCQ))
        return Validator3D(collisionManager, trajectors, self.sim3D.space())
    def _interpretDestSpec(self, trajectorName, destSpec):
        enet = sg.explicateSchemas(destSpec, self.sim3D)
        p = None
        q = None
        for s in enet.schemas():
            if isinstance(s, st.ParameterizedSchema) and ("name" in s._parameters) and (trajectorName == s._parameters["name"]):
                p = [0,0,0]
                q = [0,0,0,1]
                p[0] = s._parameters["tx"]
                p[1] = s._parameters["ty"]
                p[2] = s._parameters["tz"]
                q[0] = s._parameters["rx"]
                q[1] = s._parameters["ry"]
                q[2] = s._parameters["rz"]
                q[3] = s._parameters["rw"]
                break
        return p, q
    def navigateToDestSpec(self, trajectorName, destSpec):
        pos, q = self._interpretDestSpec(trajectorName, destSpec)
        if not pos:
            return False
        pathPrefix = os.path.join(os.path.dirname(__file__), "../meshes")
        pathAbe = os.path.join(pathPrefix, "abe.2d")
        return self.navigateToObject({"name": "aux", "type": "aux", "mesh": pathAbe, "position": {"x": pos[0], "y": pos[1], "z": pos[2]}, "orientation": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]}}, fwd_align=False)
    def navigateToObject(self, objectD, fwd_align=True):
        pathPrefix = os.path.join(os.path.dirname(__file__), "../meshes")
        pathAbe = os.path.join(pathPrefix, "abe.2d")
        abe = eo.MiscellaneousRigidObject(name="abe", object_type="agent", mesh=pathAbe)
        objectName = ""
        if isinstance(objectD,str):
            objectName = str(objectD)
            objects = self.cerebellum._retrieveObjects()
            if not objects or objectD not in objects:
                print("Either couldn't retrieve objects, or didn't find %s among them." % objectD)
                return False
            objectD = objects[objectD]
        else:
            objectName = objectD["name"]
        pathRel = os.path.join(pathPrefix, "%s.2d" % objectD["mesh"][:objectD["mesh"].rfind(".")])
        rel = eo.MiscellaneousRigidObject(name=objectName, object_type=objectD["type"], mesh=pathRel)
        rel._parameters["tx"] = objectD["position"]["x"]
        rel._parameters["ty"] = objectD["position"]["y"]
        rel._parameters["yaw"] = geom.quaternion_to_euler([objectD["orientation"]["x"], objectD["orientation"]["y"], objectD["orientation"]["z"], objectD["orientation"]["w"]])[0]
        rel._parameters["vx"] = 0.0
        rel._parameters["vy"] = 0.0
        rel._parameters["w"] = 0.0
        fdRel = gp.ForwardDirection(obj=rel)
        fdAbe = gp.ForwardDirection(obj=abe)
        schemas = [(gpr.PointProximity(a=abe, b=rel),0.005), (gpr.AxisPointingTo(axis=fdAbe,point=rel),-math.pi/(4*math.log(0.1)))]
        if fwd_align and fdRel.getAxis(self.sim2D):
            schemas.append((gpr.AxisCounterAlignment(a=fdAbe, b=fdRel),0.005))
        rpd = []
        for k in self.cellMap._points.keys():
            if self.cellMap._points[k].valid:
                x = self.cellMap.pointId2EmbeddingCoordinates(k)
                rpd.append([1.0, (x[0], x[1]), (x[2],)])
        crPos = self.cerebellum.currentPosition("base")
        rpd.append([1.0, (crPos["x"], crPos["y"]), (crPos["yaw"],)])
        for s,strictness in schemas:
            rpd = s.filterPD(rpd, self.sim2D,strictness=strictness)
        maxE = rpd[0]
        for e in rpd[1:]:
            if maxE[0] < e[0]:
                maxE = e
        return self.navigateToPosition(maxE[1][0], maxE[1][1], maxE[2][0])
    def bringHandToPosition(self, hand, x, y, z, roll, pitch, yaw, objects={}):
        if hand not in ["hands/left", "hands/right"]:
            print("Only have a left and a right hand.")
            return False
        if not self.cellMap:
            print("Don't have a navigation map: perhaps startOperation has not been called?")
            return False
        ### Function assumes the target position to be given in the robot's local coordinate frame. Therefore, we will need to compute poses
        ### of world objects in this frame.
        crPos = self.cerebellum.currentPosition("base")
        crHandPos = self.cerebellum.currentPosition(hand)
        finHandPos = {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}
        pathPrefix = os.path.join(os.path.dirname(__file__), "../meshes")
        handVolume = self.sim3D.space().loadVolume(os.path.join(pathPrefix, "Hand.stl"))
        blacklistNames = []
        trajectors = [[handVolume, [[0,0,0], [0,0,0,1]]]]#geom.euler_to_quaternion([yaw, pitch, roll])]]]
        oIHName, oIHTr, oVolume = self.cerebellum.getObjectInHand(hand)
        if oIHName:
            blacklistNames.append(oIHName)
            trajectors.append([oVolume, oIHTr])
        validator = self._makeValidator(objects, blacklistNames, trajectors)
        resolution = 0.2
        xBack = {"hands/left": -0.2, "hands/right": -0.2}
        yRight = {"hands/left": -0.4, "hands/right": -1.0}
        zDown = {"hands/left": 0, "hands/right": 0}
        cellMap = space3D.Grid3D(planes=round(2/resolution), lines=round(2/resolution), cols=round(2.5/resolution), resolution=resolution, xBack=xBack[hand], yRight=yRight[hand], zDown=zDown[hand], gridQ=(0, 0, 0, 1), validator=validator, velocity=1)
        timedMap = space.TimedPointGraph(cellMap, cellMap.graphIngressPoints((crHandPos["x"], crHandPos["y"], crHandPos["z"])))
        pidDBG = timedMap._pointGraph.embeddingCoordinates2PointId((x,y,z))
        waypoints = timedMap.generatePath((x, y, z))
        self.cerebellum.clearWaypoints(hand)
        if waypoints:
            waypoints = [cellMap.pointId2EmbeddingCoordinates(wp) for wp in waypoints]
            for wp in waypoints:
                self.cerebellum.pushWaypoint(hand, {"x": wp[0], "y": wp[1], "z": wp[2], "roll": roll, "pitch": pitch, "yaw": yaw})
            self.cerebellum.pushWaypoint(hand, finHandPos)
            print("On our way!")
            return True
        print("Target unreachable: either too far or no clear paths to it.")
        return False
    def pickObject(self, objectName):
        parkY = {"hands/left": 0.4, "hands/right": -0.4}
        hand = self.cerebellum.getFreeHand()
        objects = self.cerebellum._retrieveObjects()
        if not hand:
            print("Both hands busy, can't pick up anything more")
            return False
        if objectName not in objects:
            print("Either couldn't retrieve objects, or didn't find %s among them." % objectD)
            return False
        if not objects[objectName]["graspable"]:
            print("Object not graspable.")
            return False
        rP, rQ = self.cerebellum.robotTransform()
        o = objects[objectName]
        oPW = [o["position"]["x"], o["position"]["y"], o["position"]["z"]]
        oQW = [o["orientation"]["x"], o["orientation"]["y"], o["orientation"]["z"], o["orientation"]["w"]]
        invRobT = self.sim3D.space().invertTransform((rP, rQ))
        oP, oQ = self.sim3D.space().transformTransform(invRobT, (oPW, oQW))
        pathPrefix = os.path.join(os.path.dirname(__file__), "../meshes")
        volume = self.sim3D.space().loadVolume(os.path.join(pathPrefix, objects[objectName]["mesh"]))
        handVolume = self.sim3D.space().loadVolume(os.path.join(pathPrefix, "Hand.stl"))
        validator = self._makeValidator(objects, [], [[handVolume, [[0,0,0], [0,0,0,1]]]])
        radius = self.sim3D.space().boundaryBoxDiameter(self.sim3D.space().volumeBounds(volume))/2.0
        if objects[objectName]["particle"]:
            radius = radius + 0.1
        candidates = []
        for x in fibonacci_sphere(samples=40, only_positive_quadrant=True):
            c = self.sim3D.space().vectorSum(oP, self.sim3D.space().vectorScale(radius, x))
            if validator.isValid(c):
                candidates.append(c)
        minD = None
        minC = None
        for c in candidates:
            d = self.sim3D.space().vectorNorm(self.sim3D.space().vectorDifference(c, [0, parkY[hand], 0.9]))
            if (None == minD) or (minD > d):
                minD = d
                minC = c
        if None == minC:
            print("Huh. Couldn't seem to find any point to grasp from.")
            return False
        x, y, z = minC
        def retractFn():
            if self.bringHandToPosition(hand, 0, parkY[hand], 0.9, 0, 0, 0, objects=objects):
                while True:
                    if self.cerebellum.haveNoMoreWaypoints(hand):
                        break
                    time.sleep(0.05)
                return True
            return False
        def graspFn():
            if self.bringHandToPosition(hand, x, y, z, 0, 0, 0, objects=objects):
                while True:
                    if self.cerebellum.haveNoMoreWaypoints(hand):
                        break
                    time.sleep(0.05)
                self.cerebellum.grabObject(hand, objectName, volume, self.sim3D.space(), [oPW, oQW], [rP, rQ])
                return True
            return False
        tasks = self.cerebellum._tasks
        tasks.pushTask(retractFn)
        tasks.pushTask(graspFn)
        return True
    def placeObject(self, trajectorName, destinationSpec, hand):
        parkY = {"hands/left": 0.4, "hands/right": -0.4}
        objects = self.cerebellum._retrieveObjects()
        if (hand not in self.cerebellum._handItems) or (trajectorName != self.cerebellum._handItems[hand]):
            print("Hand/Object Error: either the hand is not recognized, or it holds another object.")
            return False
        p, q = self._interpretDestSpec(trajectorName, destinationSpec)
        if not p:
            print("Could not interpret destspec")
            return False
        hHP, hHQ = self.sim3D.space().transformTransform((p, q), self.sim3D.space().invertTransform(self.cerebellum._objectInHandTransforms[hand]))
        rP, rQ = self.cerebellum.robotTransform()
        hP, hQ = self.sim3D.space().transformTransform(self.sim3D.space().invertTransform((rP, rQ)), (hHP, hHQ))
        x, y, z = hP
        yaw, pitch, roll = geom.quaternion_to_euler(hQ)
        def retractFn():
            if self.bringHandToPosition(hand, 0, parkY[hand], 0.9, 0, 0, 0, objects=objects):
                while True:
                    if self.cerebellum.haveNoMoreWaypoints(hand):
                        break
                    time.sleep(0.05)
                print("DONE RETRACT")
                return True
            return False
        def releaseFn():
            if self.bringHandToPosition(hand, x, y, z, roll, pitch, yaw, objects=objects):
                while True:
                    if self.cerebellum.haveNoMoreWaypoints(hand):
                        break
                    time.sleep(0.05)
                self.cerebellum.releaseObject(hand)
                print("DONE RELEASE")
                return True
            return False
        tasks = self.cerebellum._tasks
        tasks.pushTask(retractFn)
        tasks.pushTask(releaseFn)
        return True
    def carryObject(self, trajectorName, destinationSpec):
        tasks = self.cerebellum._tasks
        tasks.clearTasks()
        def navFn():
            if self.navigateToObject(trajectorName,fwd_align=False):
                while True:
                    if self.cerebellum.haveNoMoreWaypoints("base"):
                        break
                    time.sleep(0.05)
                return True
            return False
        def graspFn():
            hand = self.pickObject(trajectorName)
            if hand:
                while True:
                    if self.cerebellum.haveNoMoreWaypoints(hand):
                        break
                    time.sleep(0.05)
                return True
            return False
        def nav2Fn():
            if self.navigateToDestSpec(trajectorName, destinationSpec):
                while True:
                    if self.cerebellum.haveNoMoreWaypoints("base"):
                        break
                    time.sleep(0.05)
                return True
            return False
        def placeFn():
            hand = self.cerebellum.getItemHand(trajectorName)
            if hand and self.placeObject(trajectorName, destinationSpec, hand):
                while True:
                    if self.cerebellum.haveNoMoreWaypoints(hand):
                        break
                    time.sleep(0.05)
                return True
            return False
        tasks.appendTask(navFn)
        tasks.appendTask(graspFn)
        tasks.appendTask(nav2Fn)
        tasks.appendTask(placeFn)
        with tasks._lock:
            if None == tasks._currentTask:
                tasks.needsSwitch = True
        return True
    ##### Delete the fns below
    def _toWeightedExtension(self, cellMap, location):
        retq = {"operation": "weighted-extension", "spec": {}}
        if isinstance(location, tuple):
            retq["spec"] = {location: 1.0}
        elif isinstance(location, list):
            retq["spec"] = {l: 1.0 for l in location}
        elif isinstance(location, str):
            retq["spec"] = self.interpretLocation(cellMap, location)["spec"]
        elif isinstance(location, dict):
            if "extension" == location["operation"]:
                retq["spec"] = {l: 1.0 for l in location["spec"]}
            elif "weighted-extension" == location["operation"]:
                retq["spec"] = location["spec"]
            else:
                retq = self.interpretLocation(cellMap, location)
        return retq
    def interpretLocation(self, cellMap, location):
        if isinstance(location, tuple) or isinstance(location, list):
            return location
        if isinstance(location, str):
            objects = self.cerebellum._retrieveObjects()
            if location in objects:
                return {"operation": "weighted-extension", "spec": {(objects[location]["position"]["x"], objects[location]["position"]["y"], yawId*math.pi/4): 1.0 for yawId in range(8)}}
        elif isinstance(location, dict):
            if "extension" == location["operation"]:
                return location["spec"]
            elif "weighted-extension" == location["operation"]:
                retq = []
                cMax = None
                for p, w in location["spec"].items():
                    if (None == cMax) or (cMax < w):
                        cMax = w
                        retq = [p]
                    elif cMax == w:
                        retq.append(p)
                return retq
            elif "near-to" == location["operation"]:
                location = self._toWeightedExtension(cellMap, location["spec"])
                retq = {"operation": "weighted-extension", "spec": {}}
                for g, v in cellMap._points.items():
                    if v.valid:
                        gE = tuple(cellMap.pointId2EmbeddingCoordinates(g))
                        retq["spec"][gE] = 0
                        for p, w in location["spec"].items():
                            dx = (p[0] - gE[0])
                            dy = (p[1] - gE[1])
                            d = math.sqrt(dx*dx + dy*dy)
                            if 0.01 < d:
                                tYaw = math.atan2(dy, dx)
                                dyaw = 0.1*geom.angle_diff(tYaw, gE[2])
                                d = math.sqrt(dx*dx + dy*dy + dyaw*dyaw)
                            retq["spec"][gE] = max(retq["spec"][gE], w*(1.0/(1.0 + d*d)))
                return retq
            elif "conjunction" == location["operation"]:
                locations = [self._toWeightedExtension(cellMap, l) for l in location["spec"]]
                retq = {"operation": "weighted-extension", "spec": {}}
                if locations:
                    for p, w in locations[0]["spec"].items():
                        if 0 == w:
                            continue
                        for loc in locations[1:]:
                            if p in loc["spec"]:
                                w = min(w, loc["spec"][p])
                            else:
                                w = 0
                                break
                        if 0 < w:
                            retq["spec"][p] = w
                return retq
            elif "disjunction" == location["operation"]:
                locations = [self._toWeightedExtension(cellMap, l) for l in location["spec"]]
                retq = {"operation": "weighted-extension", "spec": {}}
                for l in locations:
                    for p, w in l["spec"].items():
                        if p not in retq["spec"]:
                            retq["spec"][p] = 0
                        retq["spec"][p] = max(retq["spec"][p], w)
                return retq
        return None

