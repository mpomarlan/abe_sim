import os
import sys

import copy
import json

from threading import Lock

from abe_sim.brain.geom import angle_diff, euler_to_quaternion, euler_diff_to_angvel, invert_quaternion, quaternion_product, quaternion_to_euler, poseFromTQ

import math
import time
import random
import numpy as np

import schemasim.simulators.physics_simulator_2D as ps2D
import schemasim.simulators.physics_simulator_3D as ps3D

import _thread

import trimesh

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
                if 0 > qD[3]:
                    qD = [-qD[0], -qD[1], -qD[2], -qD[3]]
                halfAlpha = math.acos(qD[3])
                u = [0,0,0]
                s = math.sin(halfAlpha)
                if 1e-6 < s:
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
        self._HAXX_at = {}
        self._tasks = TaskList()
        self._objects = {}
        self._volumes = {}
        self._locations = {}
        self._preferredLocations = {}
        self._sortedLocations = []
        self._sortedPreferredLocations = []
        self._transformedPreferredLocations = {}
        self._dt = 0.001
        self._previousT = None
        self._simu = simu
        self._sim2D = ps2D.PhysicsSimulator2D(particleSamplingResolution=0.1, translationSamplingResolution=1.0, rotationSamplingResolution=math.pi/4, speedSamplingResolution=1.0)
        self._sim3D = ps3D.PhysicsSimulator3D(particleSamplingResolution=0.01, translationSamplingResolution=0.1, rotationSamplingResolution=0.1, speedSamplingResolution=0.1, sampleValidationStrictness=0.005, collisionPadding=0.005)
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
    def _loadVolume(self, name, path):
        pathPrefix = os.path.join(os.path.dirname(__file__), "../meshes")
        noext = path[:path.rfind('.')]
        volumePath = os.path.join(pathPrefix, noext + ".stl")
        locationPath = os.path.join(pathPrefix, noext + "_location.stl")
        preferredLocationPath = os.path.join(pathPrefix, noext + "_preferredLocation.stl")
        if os.path.isfile(volumePath):
            self._volumes[name] = trimesh.load(volumePath)
        else:
            self._volumes[name] = None
        if os.path.isfile(locationPath):
            self._locations[name] = trimesh.load(locationPath)
        else:
            self._locations[name] = None
        if os.path.isfile(preferredLocationPath):
            self._preferredLocations[name] = trimesh.load(preferredLocationPath)
        elif os.path.isfile(locationPath):
            self._preferredLocations[name] = trimesh.load(locationPath)
        else:
            self._preferredLocations[name] = None
    def _transformVolume(self, vol, name, objs):
        if None == vol:
            return None
        dp = objs[name]['position']
        dr = objs[name]['orientation']
        return vol.copy().apply_transform(poseFromTQ([dp['x'], dp['y'], dp['z']], [dr['x'], dr['y'], dr['z'], dr['w']]))
    def _retrieveObjects(self, fullDump=False):
        haveObjs = False
        while not haveObjs:
            try:
                self._objects = json.loads(self._simu.rpc('robot.worlddump', 'world_dump', fullDump))
                if fullDump:
                    for n in self._objects.keys():
                        if n not in self._volumes:
                            if 'meshfile' in self._objects[n]['props']:
                                self._loadVolume(n, self._objects[n]['props']['meshfile'])
                    if not self._sortedLocations:
                        self._sortedLocations = sorted([(self._locations[n].volume, n) for n in self._locations.keys() if None != self._locations[n]])
                    if not self._sortedPreferredLocations:
                        self._sortedPreferredLocations = sorted([(self._preferredLocations[n].volume, n) for n in self._preferredLocations.keys() if None != self._preferredLocations[n]])
                    self._transformedPreferredLocations = {n: self._transformVolume(v, n, self._objects) for n, v in self._preferredLocations.items()}
                    transformedLocations = {n: self._transformVolume(v, n, self._objects) for n, v in self._locations.items()}
                    transformedVolumes = {n: self._transformVolume(v, n, self._objects) for n, v in self._volumes.items()}
                    for n in transformedVolumes.keys():
                        self._objects[n]['at'] = None
                        for v, c in self._sortedLocations:
                            if transformedLocations[c].contains(transformedVolumes[n].vertices).all():
                                self._objects[n]['at'] = c
                                break
                haveObjs = True
            except OSError:
                continue
        return self._objects
    def _getReqLocation(self, objName, reqLocations, currentLocations):
        retq = {}
        if (objName in reqLocations) and ("position" in reqLocations[objName]): 
            retq["position"] = reqLocations[objName]["position"]
        else:
            retq["position"] = currentLocations[objName]["position"]
        if (objName in reqLocations) and ("orientation" in reqLocations[objName]): 
            retq["orientation"] = reqLocations[objName]["orientation"]
        else:
            retq["orientation"] = currentLocations[objName]["orientation"]
        return retq
    def _setObjectLocations(self, data):
        data = copy.deepcopy(data)
        sceneObjects = self._retrieveObjects(fullDump=True)
        ## TODO: this method of targetting objects will not notice aggregates; for those we may need special handling
        objectsToSet = {k: True for k in data.keys() if k in sceneObjects.keys()}
        ## build a dependency graph for object placements
        ## objects with specified coordinates or placed at "", or at "kitchen[State_]", or an object in objectIsSet, or without an at key, do not depend on anyone else
        for k in objectsToSet.keys():
            if (("position" in data[k]) and ("orientation" in data[k])) or ("at" not in data[k]):
                objectsToSet[k] = False
        objectsToSet = {k: True for k in objectsToSet.keys() if objectsToSet[k]}
        objectIsSet = {k: True for k in sceneObjects.keys() if (k not in objectsToSet.keys())}
        objectLocations = {k: self._getReqLocation(k, data, sceneObjects) for k in objectIsSet}
        collisionManager = self._sim3D.space().makeCollisionManager()
        for k, v in objectLocations.items():
            pose = self._sim3D.space().poseFromTR([objectLocations[k]["position"]["x"], objectLocations[k]["position"]["y"], objectLocations[k]["position"]["z"]], [objectLocations[k]["orientation"]["x"], objectLocations[k]["orientation"]["y"], objectLocations[k]["orientation"]["z"], objectLocations[k]["orientation"]["w"]])
            if k in self._volumes.keys():
                collisionManager.add_object(k, self._volumes[k], np.array(pose,dtype=np.double))
        dependents = {"floor": []}
        topoSortTiers = {0: list(set([k for k in objectIsSet.keys() if objectIsSet[k]] + ["floor"])), 1: []}
        self._HAXX_at = {}
        for k in objectsToSet.keys():
            if (data[k]["at"] in ["", "floor", "kitchen"]) or ("kitchenState" == data[k]["at"][:len("kitchenState")]):
                if ('furniture' in sceneObjects[k]['props']) and (sceneObjects[k]['props']['furniture']):
                    objectsToSet[k] = False
                    data[k]['position'] = sceneObjects[k]['position']
                    data[k]['orientation'] = sceneObjects[k]['orientation']
                    objectLocations[k] = {'position': sceneObjects[k]['position'], 'orientation': sceneObjects[k]['orientation']}
                    topoSortTiers[0].append(k)
                else:
                    data[k]["at"] = "floor"
                    topoSortTiers[1].append(k)
                    dependents["floor"].append(k)
            elif (data[k]["at"] in objectIsSet) and (objectIsSet[data[k]["at"]]):
                topoSortTiers[1].append(k)
                if ('furniture' not in sceneObjects[data[k]["at"]]['props']) or (not sceneObjects[data[k]["at"]]['props']['furniture']):
                    if data[k]["at"] not in self._HAXX_at:
                        self._HAXX_at[data[k]["at"]] = []
                    self._HAXX_at[data[k]["at"]].append(k)
            else:
                if data[k]["at"] not in dependents:
                    dependents[data[k]["at"]] = []
                dependents[data[k]["at"]].append(k)
                if ('furniture' not in sceneObjects[data[k]["at"]]['props']) or (not sceneObjects[data[k]["at"]]['props']['furniture']):
                    if data[k]["at"] not in self._HAXX_at:
                        self._HAXX_at[data[k]["at"]] = []
                    self._HAXX_at[data[k]["at"]].append(k)
        objectsToSet = {k:True for k in objectsToSet.keys() if objectsToSet[k]}
        ## toposort it (yay...)
        cTL = 0
        k = 0
        while k < len(objectsToSet):
            nTL = cTL + 1
            topoSortTiers[nTL] = []
            for j in topoSortTiers[cTL]:
                if j not in dependents:
                    continue
                for d in dependents[j]:
                    topoSortTiers[nTL].append(d)
                    k = k + 1
            cTL = nTL
        ## in toposorted order, place each object
        for tl in sorted(topoSortTiers.keys())[1:]:
            for name in topoSortTiers[tl]:
                dp = objectLocations[data[name]['at']]['position']
                dr = objectLocations[data[name]['at']]['orientation']
                arrangment = 'unorderedHeap'
                if (data[name]['at'] in sceneObjects) and ('arrangement' in sceneObjects[data[name]['at']]['props']):
                    arrangement = sceneObjects[data[name]['at']]['props']['arrangement']
                elif (data[name]['at'] in data) and ('arrangement' in data[data[name]['at']]['props']):
                    arrangement = data[data[name]['at']]['props']['arrangement']
                if arrangement not in ['shelved']:
                    arrangement = 'unorderedHeap'
                targetRegion = self._preferredLocations[data[name]['at']].copy().apply_transform(poseFromTQ([dp['x'], dp['y'], dp['z']], [dr['x'], dr['y'], dr['z'], dr['w']]))
                trajector = self._volumes[name]
                tBox = self._sim3D.space().volumeBounds(trajector)
                if 'shelved' == arrangement:
                    shelves = trimesh.graph.split(targetRegion)
                    found = False
                    for k in range(35):
                        shelf = shelves[random.randrange(len(shelves))]
                        bBox = self._sim3D.space().volumeBounds(shelf)
                        tv = [random.uniform(bBox[i][0] - tBox[i][0], bBox[i][1] - tBox[i][1]) for i in range(2)] + [bBox[2][0] + 0.005-tBox[2][0]]
                        tTrajector = trajector.copy().apply_transform(poseFromTQ(tv, [dr['x'], dr['y'], dr['z'], dr['w']]))
                        if (not collisionManager.in_collision_single(tTrajector, poseFromTQ([0,0,0], [0,0,0,1]))) and (all(targetRegion.contains(tTrajector.vertices))):
                            data[name]["position"] = {"x": tv[0], "y": tv[1], "z": tv[2]}
                            data[name]["orientation"] = {"x": dr['x'], "y": dr['y'], "z": dr['z'], "w": dr['w']}
                            found = True
                            break
                elif 'unorderedHeap' == arrangement:
                    bBox = self._sim3D.space().volumeBounds(targetRegion)
                    found = False
                    for k in range(35):
                        tv = [random.uniform(bBox[i][0] - tBox[i][0], bBox[i][1] - tBox[i][1]) for i in range(3)]
                        tTrajector = trajector.copy().apply_transform(poseFromTQ(tv, [dr['x'], dr['y'], dr['z'], dr['w']]))
                        if (not collisionManager.in_collision_single(tTrajector, poseFromTQ([0,0,0], [0,0,0,1]))) and (all(targetRegion.contains(tTrajector.vertices))):
                            data[name]["position"] = {"x": tv[0], "y": tv[1], "z": tv[2]}
                            data[name]["orientation"] = {"x": dr['x'], "y": dr['y'], "z": dr['z'], "w": dr['w']}
                            found = True
                            break
                if (not found):
                    if name in sceneObjects:
                        data[name]["position"] = sceneObjects[name]["position"]
                        data[name]["orientation"] = sceneObjects[name]["orientation"]
                    else:
                        data[name]["position"] = {"x": 0, "y": 0, "z": -5}
                        data[name]["orientation"] = {"x": 0, "y": 0, "z": 0, "w": 1}
                objectLocations[name] = {"position": data[name]["position"], "orientation": data[name]["orientation"]}
                pose = self._sim3D.space().poseFromTR([data[name]["position"]["x"], data[name]["position"]["y"], data[name]["position"]["z"]], [data[name]["orientation"]["x"], data[name]["orientation"]["y"], data[name]["orientation"]["z"], data[name]["orientation"]["w"]])
                collisionManager.add_object(name, trajector, np.array(pose,dtype=np.double))
        return data
        ## TODO: fix the code below
#        retq = {}
#        clumps = {}
#        for k in data.keys():
#            if (not (("props" in retq[k]) and ("aggregate" in retq[k]['props']) and (retq[k]['props']['aggregate']))) or ("particles" in retq[k]):
#                retq[k] = data[k]
#            else:
#                location = "floor"
#                if "at" in data[k]:
#                    location = data[k]['at']
#                substance = "Ether"
#                if ("props" in data[k]) and ("substance" in data[k]['props']):
#                    substance = data[k]['props']['substance']
#                amount = 10
#                ## TODO: get amount from props
#                if substance not in clumps:
#                    clumps[substance] = {"": 0}
#                clumps[substance][k] = {"amount": amount, "location": location}
#                clumps[substance][""] = clumps[substance][""] + amount
#        if clumps:
#            objs = self._retrieveObjects(fullDump=True)
#            availables = {}
#            for k in objs.keys():
#                if ("props" in objs[k]) and ("particle" in objs[k]['props']) and (objs[k]['props']['particle']) and ("active" in objs[k]['props']) and (not objs[k]['props']['active']):
#                    substance = "Ether"
#                    if ("substance" in objs[k]['props']):
#                        substance = objs[k]['props']['substance']
#                    if substance not in availables:
#                        availables[substance] = {}
#                    availables[substance][k] = True
#            for subst in availables.keys():
#                totalNeeded = clumps[subst][""]
#                totalAvailable = len(availables[subst])
#                if totalAvailable < totalNeeded:
#                    fac = totalAvailable/(1.0*totalNeeded)
#                    for k in clumps[subst].keys():
#                        if "" != k:
#                            clumps[subst][k]['amount'] = int(fac*clumps[subst][k]['amount'])
#            ## TODO: for each clump, sample positions
#        return retq
    def _setObjects(self, data):
        tfile = '/tmp/.tmp_abe_sim'
        with open(tfile, 'w') as outfile:
            outfile.write(json.dumps(self._setObjectLocations(data)))
        haveObjs = False
        while not haveObjs:
            try:
                self._simu.rpc('robot.greatreset', 'great_reset', tfile)
                haveObjs = True
            except OSError:
                continue
        os.remove(tfile)
    def _retrieveWorldState(self, forJSON=False):
        if forJSON:
            return {"robotState": {"handItems": self._handItems, "objectInHandTransforms": self._objectInHandTransforms}, "worldState": self._retrieveObjects(fullDump=True)}
        return {"robotState": {"handItems": self._handItems, "objectInHandTransforms": self._objectInHandTransforms, "objectInHandMesh": self._objectInHandMesh}, "worldState": self._retrieveObjects(fullDump=True)}
    def _setWorldState(self, state):
        self._handItems = state["robotState"]["handItems"]
        self._objectInHandTransforms = state["robotState"]["objectInHandTransforms"]
        self._objectInHandMesh = state["robotState"]["objectInHandMesh"]
        self._setObjects(state["worldState"])
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
        grabbedObjects = ""
        toHAXX = [objectName]
        while toHAXX:
            oName = toHAXX.pop()
            sep = ""
            if "" != grabbedObjects:
                sep = ";"
            grabbedObjects = grabbedObjects + sep + oName
            if oName in self._HAXX_at:
                toHAXX = toHAXX + self._HAXX_at[oName]
        if "hands/left" == hand:
            handActuation["lgrab"] = grabbedObjects
        elif "hands/right" == hand:
            handActuation["rgrab"] = grabbedObjects
        parent = {"hands/left": "LeftHand", "hands/right": "RightHand"}[hand]
        while True:
            self._handsActuator.publish(handActuation)
            sc = self._retrieveObjects()[objectName]["parent"]
            if parent != sc:
                print(parent, sc, "...")
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

