import os
import sys

import json

from threading import Lock

from abe_sim.brain.geom import angle_diff, euler_to_quaternion, euler_diff_to_angvel, invert_quaternion, quaternion_product, quaternion_to_euler

import math
import time
import numpy as np

import _thread

class TaskList:
    def __init__(self):
        self._lock = Lock()
        self._currentTask = None
        self._tasks = []
        self.needsSwitch = False
    def isEmpty(self):
        return (None == self._currentTask) and ([] == self._tasks)
    def isLocked(self):
        return self._lock.locked()
    def pushTask(self, task):
        with self._lock:
            self._tasks = [task] + self._tasks
    def appendTask(self, task):
        with self._lock:
            self._tasks.append(task)
    def clearTasks(self):
        with self._lock:
            self._tasks = []
            self._currentTask = None
    def popTask(self):
        with self._lock:
            peekCrWP = self._currentWP
            peekNextWP = None
            if 0 < len(self._tasks):
                peekNextWP = self._tasks[0]
            d = 0
            if None != peekCrWP:
                d = self.distance(peekCrWP, crPos)
            if (d < tolerance):
                if (None != peekNextWP):
                    change = True
                    self._currentWP = peekNextWP
                    self._tasks = self._tasks[1:]
                else:
                    if None != self._currentWP:
                        change = True
                    self._currentWP = None
            retq = self._currentWP
        return change, retq

class WaypointsList:
    def __init__(self):
        self._lock = Lock()
        self._currentWP = None
        self._waypoints = []
    def isAppropriateWP(self, wp):
        return False
    def distance(self, wpA, wpB):
        return 0
    def isEmpty(self):
        return None == self._currentWP
    def isLocked(self):
        return self._lock.locked()
    def pushWP(self, wp):
        if self.isAppropriateWP(wp):
            with self._lock:
                self._waypoints.append(wp)
                if None == self._currentWP:
                    self._currentWP = wp
    def clearWPs(self):
        with self._lock:
            self._waypoints = []
            self._currentWP = None
    def popWP(self, crPos, tolerance):
        change = False
        retq = None
        with self._lock:
            peekCrWP = self._currentWP
            peekNextWP = None
            if 0 < len(self._waypoints):
                peekNextWP = self._waypoints[0]
            d = 0
            if None != peekCrWP:
                d = self.distance(peekCrWP, crPos)
            if (d < tolerance):
                if (None != peekNextWP):
                    change = True
                    self._currentWP = peekNextWP
                    self._waypoints = self._waypoints[1:]
                else:
                    if None != self._currentWP:
                        change = True
                    self._currentWP = None
            retq = self._currentWP
        return change, retq

class Waypoints2DR(WaypointsList):
    def __init__(self):
        super().__init__()
    def isAppropriateWP(self, wp):
        if isinstance(wp,dict) and ('x' in wp) and ('y' in wp) and ('yaw' in wp):
            return True
        return False
    def distance(self, wpA, wpB):
        dx = wpA['x'] - wpB['x']
        dy = wpA['y'] - wpB['y']
        dz = angle_diff(wpA['yaw'], wpB['yaw'])
        return math.sqrt(dx*dx + dy*dy + dz*dz)

class Waypoints3DdR(WaypointsList):
    def __init__(self):
        super().__init__()
    def isAppropriateWP(self, wp):
        if isinstance(wp,dict) and ('x' in wp) and ('y' in wp) and ('z' in wp) and ('yaw' in wp) and ('pitch' in wp) and ('roll' in wp):
            return True
        return False
    def distance(self, wpA, wpB):
        dx = wpA['x'] - wpB['x']
        dy = wpA['y'] - wpB['y']
        dz = wpA['z'] - wpB['z']
        dr = wpA['roll'] - wpB['roll']
        dp = wpA['pitch'] - wpB['pitch']
        dw = wpA['yaw'] - wpB['yaw']
        return math.sqrt(dx*dx + dy*dy + dz*dz + dr*dr + dp*dp + dw*dw)

class WaypointsPT(WaypointsList):
    def __init__(self):
        super().__init__()
    def distance(self, wpA, wpB):
        dx = angle_diff(wpA['pan'], wpB['pan'])
        dy = angle_diff(wpA['tilt'], wpB['tilt'])
        return math.sqrt(dx*dx + dy*dy)

class Vel2DR:
    def __init__(self, vel=3, angvel=3, tolerance=0.05, angleTolerance=0.01, acc=1.0/60, angacc=1.0/60):
        self._velocity = vel
        self._angularVelocity = angvel
        self._tolerance = tolerance
        self._angleTolerance = angleTolerance
        self._accv = acc
        self._accw = angacc
    def actuation(self, target, current, dt, currentV):
        cv, cw = [currentV["v"], currentV["w"]]
        retq = {"v": cv, "w": 0}
        if None != target:
            dx = target["x"] - current["x"]
            dy = target["y"] - current["y"]
            d = math.sqrt(dx*dx + dy*dy)
            if self._tolerance < d:
                tYaw = math.atan2(dy, dx)
                alpha  = angle_diff(tYaw, current["yaw"])
                if self._angleTolerance < alpha:
                    retq["w"] = min(cw + self._accw, min(self._angularVelocity, self._angularVelocity*alpha))
                    retq["v"] = max(0, cv - 2*self._accv)
                elif -self._angleTolerance > alpha:
                    retq["w"] = max(cw - self._accw, max(-self._angularVelocity, self._angularVelocity*alpha))
                    retq["v"] = max(0, cv - 2*self._accv)
                else:
                    retq["v"] = min(cv + self._accv, min(self._velocity, self._velocity*d))
            else:
                alpha = angle_diff(target["yaw"], current["yaw"])
                if self._angleTolerance < alpha:
                    retq["w"] = min(cw + self._accw, min(self._angularVelocity, self._angularVelocity*alpha))
                elif -self._angleTolerance > alpha:
                    retq["w"] = max(cw - self._accw, max(-self._angularVelocity, self._angularVelocity*alpha))
                retq["v"] = max(0, cv - 2*self._accv)
        else:
            retq["v"] = max(0, retq["v"] - 2*self._accv)
        return retq

class Pos3D:
    def __init__(self, vel=1, tolerance=0.01):
        self._velocity = vel
        self._tolerance = tolerance
    def actuation(self, target, current, dt, currentV):
        retq = {"x": current["x"], "y": current["y"], "z": current["z"], "roll": current["roll"], "pitch": current["pitch"], "yaw": current["yaw"]}
        v = math.sqrt(currentV["x"]*currentV["x"]+currentV["y"]*currentV["y"]+currentV["z"]*currentV["z"])
        w = math.sqrt(currentV["rx"]*currentV["rx"]+currentV["ry"]*currentV["ry"]+currentV["rz"]*currentV["rz"])
        accV = 1/60.0
        accW = 1/60.0
        if None != target:
            dx = target["x"] - current["x"]
            dy = target["y"] - current["y"]
            dz = target["z"] - current["z"]
            d = math.sqrt(dx*dx + dy*dy + dz*dz)
            if self._tolerance < d:
                travelD = 0.03 #self._velocity*dt
                if d < travelD:
                    retq["x"] = target["x"]
                    retq["y"] = target["y"]
                    retq["z"] = target["z"]
                else:
                    tV = max(min(3*d, v + accV), v - accV)
                    travelFactor = dt*tV/d
                    retq["x"] = retq["x"] + dx*travelFactor
                    retq["y"] = retq["y"] + dy*travelFactor
                    retq["z"] = retq["z"] + dz*travelFactor
            else:
                qC = euler_to_quaternion([current['yaw'], current['pitch'], current['roll']])
                qT = euler_to_quaternion([target['yaw'], target['pitch'], target['roll']])
                qD = quaternion_product(qT, invert_quaternion(qC))
                halfAlpha = math.acos(qD[3])
                u = [0,0,0]
                if 0.005 < halfAlpha:
                    s = math.sin(halfAlpha)
                    u = [qD[0]/s, qD[1]/s, qD[2]/s]
                else:
                    halfAlpha = 0
                if 0 == halfAlpha:
                    retq["roll"] = target["roll"]
                    retq["pitch"] = target["pitch"]
                    retq["yaw"] = target["yaw"]
                else:
                    w = max(min(3*halfAlpha, w + accW), w - accW)
                    halfAlpha = dt*w
                    s = math.sin(halfAlpha)
                    qD = [u[0]*s, u[1]*s, u[2]*s, math.cos(halfAlpha)]
                    retq["yaw"], retq["pitch"], retq["roll"] = quaternion_to_euler(quaternion_product(qD, qC))
        return retq

class PosPT:
    def __init__(self, vel=2, tolerance=0.01):
        self._velocity = vel
        self._tolerance = tolerance
    def actuation(self, target, current, dt, currentV):
        retq = {"pan": current["pan"], "tilt": current["tilt"]}
        if None != target:
            dx = angle_diff(target["pan"], current["pan"])
            dy = angle_diff(target["tilt"], current["tilt"])
            d = math.sqrt(dx*dx + dy*dy)
            if self._tolerance < d:
                travelD = self._velocity*dt
                if d < travelD:
                    retq["pan"] = target["pan"]
                    retq["tilt"] = target["tilt"]
                else:
                    travelFactor = travelD/d
                    retq["pan"] = retq["pan"] + dx*travelFactor
                    retq["tilt"] = retq["tilt"] + dy*travelFactor
        return retq

class Cerebellum:
    def __init__(self, headActuator, handsActuator, baseActuator, poseSensor, simu, worldDump):
        self._handItems = {"hands/left": None, "hands/right": None}
        self._objectInHandTransforms = {"hands/left": None, "hands/right": None}
        self._objectInHandMesh = {"hands/left": None, "hands/right": None}
        self._tasks = TaskList()
        self._objects = {}
        self._dt = 0.001
        self._previousT = None
        self._simu = simu
        self._worldDump = worldDump
        self._headActuator = headActuator
        self._handsActuator = handsActuator
        self._baseActuator = baseActuator
        self._poseSensor = poseSensor
        self._callbacks = {"base": None, "hands/left": None, "hands/right": None, "head": None}
        self._waypoints = {"base": Waypoints2DR(), "hands/left": Waypoints3DdR(), "hands/right": Waypoints3DdR(), "head": WaypointsPT()}
        self._controllers = {"base": Vel2DR(), "hands/left": Pos3D(), "hands/right": Pos3D(), "head": PosPT()}
        self._positions = {"base": {"x": 0, "y": 0, "yaw": 0}, "hands/left": {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}, "hands/right": {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}, "head": {"pan": 0, "tilt": 0}}
        self._velocities = {"base": {"v": 0, "w": 0}, "hands/left": {"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0}, "hands/right": {"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0}, "head": {"pan": 0, "tilt": 0}}
    def _retrieveObjects(self):
        haveObjs = False
        while not haveObjs:
            try:
                self._objects = json.loads(self._simu.rpc('robot.worlddump', 'world_dump'))
                haveObjs = True
            except OSError:
                continue
        return self._objects
    def robotTransform(self):
        x = self._positions["base"]["x"]
        y = self._positions["base"]["y"]
        z = 0
        halfYaw = self._positions["base"]["yaw"]/2
        return [x, y, z], [0, 0, math.sin(halfYaw), math.cos(halfYaw)]
    def getItemHand(self, objectName):
        if objectName == self._handItems["hands/right"]:
            return "hands/right"
        elif objectName == self._handItems["hands/left"]:
            return "hands/left"
        return None
    def getObjectInHand(self, hand):
        return self._handItems[hand], self._objectInHandTransforms[hand], self._objectInHandMesh[hand]
    def getFreeHand(self):
        return self.getItemHand(None)
    def grabObject(self, hand, objectName, mesh, space, objInWorldTransform, robInWorldTransform):
        handActuation = {"lx": self._positions["hands/left"]["x"], "rx": self._positions["hands/right"]["x"], "ly": self._positions["hands/left"]["y"], "ry": self._positions["hands/right"]["y"], "lz": self._positions["hands/left"]["z"], "rz": self._positions["hands/right"]["z"], "lrx": self._positions["hands/left"]["roll"], "rrx": self._positions["hands/right"]["roll"], "lry": self._positions["hands/left"]["pitch"], "rry": self._positions["hands/right"]["pitch"], "lrz": self._positions["hands/left"]["yaw"], "rrz": self._positions["hands/right"]["yaw"], "lgrab": "", "rgrab": "", "lrelease": False, "rrelease": False}
        if "hands/left" == hand:
            handActuation["lgrab"] = objectName
        elif "hands/right" == hand:
            handActuation["rgrab"] = objectName
        parent = {"hands/left": "LeftHand", "hands/right": "RightHand"}[hand]
        while True:
            self._handsActuator.publish(handActuation)
            if parent != self._retrieveObjects()[objectName]["parent"]:
                time.sleep(0.01)
            else:
                break
        self._handItems[hand] = objectName
        pHR = [self._positions[hand]["x"], self._positions[hand]["y"], self._positions[hand]["z"]]
        qHR = euler_to_quaternion([self._positions[hand]["yaw"], self._positions[hand]["pitch"], self._positions[hand]["roll"]])
        oP, oQ = space.transformTransform(space.invertTransform(space.transformTransform(robInWorldTransform, [pHR, qHR])), objInWorldTransform)
        self._objectInHandTransforms[hand] = [oP, oQ]
        self._objectInHandMesh[hand] = mesh
        return
    def releaseObject(self, hand):
        handActuation = {"lx": self._positions["hands/left"]["x"], "rx": self._positions["hands/right"]["x"], "ly": self._positions["hands/left"]["y"], "ry": self._positions["hands/right"]["y"], "lz": self._positions["hands/left"]["z"], "rz": self._positions["hands/right"]["z"], "lrx": self._positions["hands/left"]["roll"], "rrx": self._positions["hands/right"]["roll"], "lry": self._positions["hands/left"]["pitch"], "rry": self._positions["hands/right"]["pitch"], "lrz": self._positions["hands/left"]["yaw"], "rrz": self._positions["hands/right"]["yaw"], "lgrab": "", "rgrab": "", "lrelease": False, "rrelease": False}
        if "hands/left" == hand:
            handActuation["lrelease"] = True
        elif "hands/right" == hand:
            handActuation["rrelease"] = True
        while True:
            self._handsActuator.publish(handActuation)
            if "" != self._retrieveObjects()[self._handItems[hand]]["parent"]:
                time.sleep(0.01)
            else:
                break
        self._handItems[hand] = None
        self._objectInHandTransforms[hand] = None
        self._objectInHandMesh[hand] = None
    def initializePosition(self, waypointType, pos):
        if waypointType in self._positions:
            for k in self._positions[waypointType].keys():
                if k in pos:
                    self._positions[waypointType][k] = pos[k]
    def currentPosition(self, waypointType):
        if waypointType in self._positions:
            return self._positions[waypointType]
        return None
    def setCallback(self, waypointType, callback):
        if (waypointType in self._callbacks) and (callable(callback) or (None == callback)):
            self._callbacks[waypointType] = callback
            return True
        return False
    def clearWaypoints(self, waypointType):
        if waypointType in self._waypoints:
            self._waypoints[waypointType].clearWPs()
            return True
        return False
    def haveNoMoreWaypoints(self, waypointType):
        if waypointType in self._waypoints:
            return self._waypoints[waypointType].isEmpty()
        return True
    def pushWaypoint(self, waypointType, wp):
        if waypointType in self._waypoints:
            self._waypoints[waypointType].pushWP(wp)
            return True
        return False
    def startMonitoring(self):
        self._previousT = time.time()
        self._poseSensor.subscribe(lambda x: self._followWaypoints(x))
    def _followWaypoints(self, x):
        self.initializePosition("base", x)
        ct = time.time()
        if None != self._previousT:
            self._dt = min(1.0/60, ct - self._previousT)
        self._previousT = ct
        actuations = {}
        toCall = []
        for k in self._waypoints.keys():
            if self._waypoints[k].isLocked():
                continue
            change, crWP = self._waypoints[k].popWP(self._positions[k], 0.05+0.01)
            if change and (None == crWP) and (None != self._callbacks[k]):
                toCall.append((self._callbacks[k], self._positions[k]))
            actuations[k] = self._controllers[k].actuation(crWP, self._positions[k], self._dt, self._velocities[k])
        if "base" in actuations:
            self._baseActuator.publish(actuations["base"])
            self._velocities["base"] = actuations["base"].copy()
        if "head" in actuations:
            self._headActuator.publish(actuations["head"])
            self._positions["head"]["pan"] = actuations["head"]["pan"]
            self._positions["head"]["tilt"] = actuations["head"]["tilt"]
        handActuation = {"lx": self._positions["hands/left"]["x"], "rx": self._positions["hands/right"]["x"], "ly": self._positions["hands/left"]["y"], "ry": self._positions["hands/right"]["y"], "lz": self._positions["hands/left"]["z"], "rz": self._positions["hands/right"]["z"], "lrx": self._positions["hands/left"]["roll"], "rrx": self._positions["hands/right"]["roll"], "lry": self._positions["hands/left"]["pitch"], "rry": self._positions["hands/right"]["pitch"], "lrz": self._positions["hands/left"]["yaw"], "rrz": self._positions["hands/right"]["yaw"], "lgrab": "", "rgrab": "", "lrelease": False, "rrelease": False}
        if "hands/left" in actuations:
            handActuation["lx"] = actuations["hands/left"]["x"]
            handActuation["ly"] = actuations["hands/left"]["y"]
            handActuation["lz"] = actuations["hands/left"]["z"]
            handActuation["lrx"] = actuations["hands/left"]["roll"]
            handActuation["lry"] = actuations["hands/left"]["pitch"]
            handActuation["lrz"] = actuations["hands/left"]["yaw"]
            self._velocities["hands/left"]["x"] = (actuations["hands/left"]["x"]-self._positions["hands/left"]["x"])/self._dt
            self._velocities["hands/left"]["y"] = (actuations["hands/left"]["y"]-self._positions["hands/left"]["y"])/self._dt
            self._velocities["hands/left"]["z"] = (actuations["hands/left"]["z"]-self._positions["hands/left"]["z"])/self._dt
            self._velocities["hands/left"]["rx"], self._velocities["hands/left"]["ry"], self._velocities["hands/left"]["rz"] = euler_diff_to_angvel([self._positions["hands/left"]["yaw"], self._positions["hands/left"]["pitch"], self._positions["hands/left"]["roll"]], [actuations["hands/left"]["yaw"], actuations["hands/left"]["pitch"], actuations["hands/left"]["roll"]], self._dt)
            self._positions["hands/left"]["x"] = actuations["hands/left"]["x"]
            self._positions["hands/left"]["y"] = actuations["hands/left"]["y"]
            self._positions["hands/left"]["z"] = actuations["hands/left"]["z"]
            self._positions["hands/left"]["roll"] = actuations["hands/left"]["roll"]
            self._positions["hands/left"]["pitch"] = actuations["hands/left"]["pitch"]
            self._positions["hands/left"]["yaw"] = actuations["hands/left"]["yaw"]
        if "hands/right" in actuations:
            handActuation["rx"] = actuations["hands/right"]["x"]
            handActuation["ry"] = actuations["hands/right"]["y"]
            handActuation["rz"] = actuations["hands/right"]["z"]
            handActuation["rrx"] = actuations["hands/right"]["roll"]
            handActuation["rry"] = actuations["hands/right"]["pitch"]
            handActuation["rrz"] = actuations["hands/right"]["yaw"]
            self._velocities["hands/right"]["x"] = (actuations["hands/right"]["x"]-self._positions["hands/right"]["x"])/self._dt
            self._velocities["hands/right"]["y"] = (actuations["hands/right"]["y"]-self._positions["hands/right"]["y"])/self._dt
            self._velocities["hands/right"]["z"] = (actuations["hands/right"]["z"]-self._positions["hands/right"]["z"])/self._dt
            self._velocities["hands/right"]["rx"], self._velocities["hands/right"]["ry"], self._velocities["hands/right"]["rz"] = euler_diff_to_angvel([self._positions["hands/right"]["yaw"], self._positions["hands/right"]["pitch"], self._positions["hands/right"]["roll"]], [actuations["hands/right"]["yaw"], actuations["hands/right"]["pitch"], actuations["hands/right"]["roll"]], self._dt)
            self._positions["hands/right"]["x"] = actuations["hands/right"]["x"]
            self._positions["hands/right"]["y"] = actuations["hands/right"]["y"]
            self._positions["hands/right"]["z"] = actuations["hands/right"]["z"]
            self._positions["hands/right"]["roll"] = actuations["hands/right"]["roll"]
            self._positions["hands/right"]["pitch"] = actuations["hands/right"]["pitch"]
            self._positions["hands/right"]["yaw"] = actuations["hands/right"]["yaw"]
        self._handsActuator.publish(handActuation)
        for cb, x in toCall:
            cb(x)
        if not self._tasks.isLocked():
            with self._tasks._lock:
                if self._tasks.needsSwitch:
                    self._tasks.needsSwitch = False
                    self._tasks._currentTask = None
                    if [] != self._tasks._tasks:
                        fn = self._tasks._tasks[0]
                        self._tasks._tasks = self._tasks._tasks[1:]
                        def tfn(taskList):
                            if fn():
                                with taskList._lock:
                                    taskList.needsSwitch = True
                            else:
                                taskList.clearTasks()
                        self._tasks._currentTask = _thread.start_new_thread(tfn, (self._tasks,))

