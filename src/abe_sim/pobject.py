import pybullet as p
import copy

import math

from abe_sim.geom import quaternionProduct, overlappingObjectNames
from abe_sim.utils import stubbornTry

class DebugObject:
    def __init__(self, pobject):
        self._parent = pobject
        self._world = pobject._world
        pobject._debugObjects.append(self)
        self._id = -1
        self._world._debugObjects.append(self)
    def remove(self):
        self._clearSelf()
        self._parent._debugObjects.remove(self)
        self._world._debugObjects.remove(self)
    def _addSelf(self, pos, quat):
        return
    def _clearanceLambda(self):
        return
    def _clearSelf(self):
        if -1 != self._id:
            stubbornTry(self._clearanceLambda())
        self._id = -1
    def _moveSelf(self, pos, quat):
        return
    def _positionSelf(self):
        return (0,0,0), (0,0,0,1)
    def updateLocation(self):
        if not self._world._debugVisualizationsEnabled:
            return
        if -1 == self._id:
            self._addSelf(*self._positionSelf())
        else:
            self._moveSelf(*self._positionSelf())

class DebugText(DebugObject):
    def __init__(self, pobject, text):
        super().__init__(pobject)
        self._text = str(text)
    def _addSelf(self, pos, quat):
        self._id = stubbornTry(lambda : p.addUserDebugText(self._text, pos, textColorRGB=[0,0,0], textSize=1, lifeTime=0))
    def _clearanceLambda(self):
        return lambda : p.removeUserDebugItem(self._id, self._world.getSimConnection())
    def _moveSelf(self, pos, quat):
        self._addSelf(pos, quat)
    def _positionSelf(self):
        pos = list(self._parent.getBodyProperty((), "position"))
        aabbMin, aabbMax = self._parent.getAABB(None)
        zExt = aabbMax[2] - aabbMin[2]
        pos[2] = pos[2] + 0.5*zExt + 0.005
        return tuple(pos), (0,0,0,1)

class DebugCapsule(DebugObject):
    def __init__(self, pobject, label):
        super().__init__(pobject)
        self._visualShape = -1
        if label in pobject._highlightCapsules:
            self._visualShape = pobject._highlightCapsules[label]
        elif '' in pobject._highlightCapsules:
            self._visualShape = pobject._highlightCapsules['']
    def _addSelf(self, pos, quat):
        self._id = stubbornTry(lambda : p.createMultiBody(baseCollisionShapeIndex=-1,baseVisualShapeIndex=self._visualShape,basePosition=[0,0,1],baseOrientation=[0,0,0,1],physicsClientId=self._world.getSimConnection()))
    def _clearanceLambda(self):
        return lambda : p.removeBody(self._id, self._world.getSimConnection())
    def _moveSelf(self, pos, quat):
        stubbornTry(lambda : p.resetBasePositionAndOrientation(self._id, pos, quat, self._world.getSimConnection()))
    def _positionSelf(self):
        return self._parent.getBodyProperty((), "position"), self._parent.getBodyProperty((), "orientation")
    
class PObjectWrapper:
    def __init__(self, pobject, bodyName, parentJointName):
        self._pobject = pobject
        self._bodyName = bodyName
        self._parentJointName = parentJointName
        self._world = self._pobject._world
    def __str__(self):
        return str(self._pobject) + ":" + str(self._bodyName)
    def getJointId(self, name):
        return self._pobject._jointName2Id[name]
    def getLinkId(self, name):
        return self._pobject._linkName2Id[name]
    def getId(self):
        return self._pobject._id
    def getBodyIdentifiers(self):
        return tuple((x,) for x in [self._bodyName])
    def getBodyProperty(self, identifier, propertyId):
        if (None == identifier) or (() == identifier):
            return self._pobject.getBodyProperty((self._bodyName,), propertyId)
        return self._pobject.getBodyProperty(identifier, propertyId)
    def setBodyProperty(self, identifier, propertyId, value):
        if (None == identifier) or (() == identifier):
            return self._pobject.setBodyProperty((self._bodyName,), propertyId, value)
        return self._pobject.setBodyProperty(identifier, propertyId, value)
    def getName(self):
        return str(self._pobject) + ":" + str(self._bodyName)
    def at(self):
        return self._pobject.at()
    def getAABB(self, identifier):
        if isinstance(identifier, tuple) and (() != identifier):
            linkId = self._pobject._linkName2Id[identifier[0]]
        else:
            linkId = self._pobject._linkName2Id[self._bodyName]
        return stubbornTry(lambda : p.getAABB(self._pobject._id, linkId, self._pobject._world.getSimConnection()))

class PObject():
    def _customInitPreLoad(self, *args, **kwargs):
        return
    def _customInitDynamicModels(self):
        return
    def __str__(self):
        return self._name
    def __init__(self, world, name, initialBasePosition, initialBaseOrientation, *args, **kwargs):
        super().__init__()
        self._debugObjects = []
        self._highlightCapsules = {}
        self._args = args
        self._kwargs = kwargs
        # Prepare some member variables, and put some defaults in them. These defaults can, and in some cases must, be overwritten by more specialized classes of pobject.
        #     _urdf: path to a urdf file which describes how the object is constructed out of joints and links; see the URDF documentation for more details
        self._urdf = None
        # "Functional" meshes: define regions which, if they collide, indicate various conditions in the
        #     world trigger.
        #     _sem: path to a semantics file which contains miscellaneous information about an object such as preferred axes, semantically labelled regions and paths to their shapes etc.
        self._sem = None
        #     _dynamics: stores objects defining procedures to update state variables from one simulation step to another;
        self._dynamics = {}
        #    _customStateVariables: stores values that can change under custom dynamics (i.e., dynamics other than what pybullet simulates, such as temperature). Indexed by body (a pobject may have several bodies) then by variable name.
        self._customStateVariables = {}
        #    _initialBasePosition: 3D vector for the initial location of the pobject 
        self._initialBasePosition = initialBasePosition###[0,0,0]
        #    _initialBaseOrientation: quaternion for the initial orientation of the pobject
        self._initialBaseOrientation = initialBaseOrientation###p.getQuaternionFromEuler([0,0,0])
        #    _useMaximalCoordinates: if true, bodies in the pobject are modelled as 6DoF rigid bodies, upon which constraints define joints; if false, special solvers will work with reduced spaces for the bodies in the pobject. True is recommended for most objects, but active kinematic chains like robots should overwrite this and use false.
        self._useMaximalCoordinates = True
        #    _useFixedBase: force the base link of the pobject to never move in the world.
        self._useFixedBase = True
        #    _urdfFlags: see pybullet doc. for loadURDF.
        self._urdfFlags = 0
        #    _globalScaling: a scale factor to apply to all the bodies in a pobject.
        self._globalScaling = 1.0
        self._maxForce = {}
        self._positionGain = {}
        self._velocityGain = {}
        self._maxVelocity = {}
        # Each pobject class will have its own initialization steps as well.
        self._name = name
        self._world = world
        self._id = None
        self._customInitPreLoad(*args, **kwargs)
        # Some post initialization steps to ensure general logical constraints on pobjects.
        #     all pobjects have the rigid_body dynamics: the pobject moves through 3D space and has position, orientation, linear velocity and angular velocity; updated by the (py)bullet physics engine rather than a custom dynamics model.
        # A PObject only exists embedded in a world under a unique name.
        #     TODO: somewhat redundant as this is also ensured in the world addPObjectByType function.
        if name in world._pobjects:
            world.removePObject(name)
        world._pobjects[name] = self
        self._childOfConstraints = {}
        self._parentOfConstraints = {}
        self.reloadObject(self._initialBasePosition, self._initialBaseOrientation)
        self._lastAppliedJointControls={}
    def at(self):
        aabbMin, aabbMax = self.getAABB(None)
        center = [(a+b)*0.5 for a,b in zip(aabbMin,aabbMax)]
        aabbMin = [center[0]-0.01, center[1]-0.01, aabbMin[2]-0.05]
        aabbMax = [center[0]+0.01, center[1]+0.01, aabbMin[2]+0.01]
        aabbMin[2] = aabbMin[2] - 0.04
        closeObjects = overlappingObjectNames(aabbMin, aabbMax, self._world)
        retq = None
        minD = None
        pos = self.getBodyProperty((), "position")
        for o in closeObjects:
            if (o == self.getName()) or (not self._world._pobjects[o].getBodyProperty("fn", "cancontain")):
                continue
            d = [a-b for a,b in zip(pos,self._world._pobjects[o].getBodyProperty((), "position"))]
            d = math.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2])
            if (None==minD) or (d < minD):
                minD = d
                retq = o
        return retq
    def getAABB(self, identifier):
        linkId = -1
        if isinstance(identifier, tuple) and (() != identifier):
            linkId = self._linkName2Id[identifier[0]]
        ### TODO: if identifier is None, get an AABB around the entire obj, not just the base
        return stubbornTry(lambda : p.getAABB(self._id, linkId, self._world.getSimConnection()))
    def reloadObject(self, position, orientation):
        ccons = copy.deepcopy(self._childOfConstraints)
        pcons = copy.deepcopy(self._parentOfConstraints)
        for cN, cI in self._childOfConstraints.items():
            self.removeRigidBodyConstraint(cN[0], cN[1], cN[2])
        for cN, cI in self._parentOfConstraints.items():
            self._world._pobjects[cN[1]].removeRigidBodyConstraint(cN[0], self._name, cN[2])
        if None != self._id:
            stubbornTry(lambda : p.removeBody(self._id, self._world.getSimConnection()))
        self._id = stubbornTry(lambda : p.loadURDF(self._urdf, [0,0,0], [0,0,0,1], self._useMaximalCoordinates, self._useFixedBase, self._urdfFlags, self._globalScaling, self._world.getSimConnection()))
        aabbMin, aabbMax = stubbornTry(lambda : p.getAABB(self._id, -1, self._world.getSimConnection()))
        zExt = aabbMax[2] - aabbMin[2]
        dx = 0.5*(aabbMax[0] - aabbMin[0])
        dy = 0.5*(aabbMax[1] - aabbMin[1])
        xyExt = math.sqrt(dx*dx + dy*dy)
        self._highlightCapsules = {
            '': p.createVisualShape(p.GEOM_CAPSULE,radius=xyExt*1.1,length=zExt*1.1,rgbaColor=[0.5,0.5,0.5,0.5],specularColor=[0,0,0]),
            'danger': p.createVisualShape(p.GEOM_CAPSULE,radius=xyExt*1.1,length=zExt*1.1,rgbaColor=[1,0,0,0.5],specularColor=[0,0,0]),
            'warning': p.createVisualShape(p.GEOM_CAPSULE,radius=xyExt*1.1,length=zExt*1.1,rgbaColor=[1,0.5,0,0.5],specularColor=[0,0,0]),
            'selected': p.createVisualShape(p.GEOM_CAPSULE,radius=xyExt*1.1,length=zExt*1.1,rgbaColor=[0,1,0,0.5],specularColor=[0,0,0]),
            'instrumental': p.createVisualShape(p.GEOM_CAPSULE,radius=xyExt*1.1,length=zExt*1.1,rgbaColor=[0,0,1,0.5],specularColor=[0,0,0]),
            'instrumentalBackup': p.createVisualShape(p.GEOM_CAPSULE,radius=xyExt*1.1,length=zExt*1.1,rgbaColor=[0.2,1,1,0.5],specularColor=[0,0,0]),
        }
        stubbornTry(lambda: p.resetBasePositionAndOrientation(self._id, position, orientation, self._world.getSimConnection()))
        self._jointName2Id = {}
        baseLinkIdx = 0
        if self._useMaximalCoordinates:
            baseLinkIdx = 1
        self._linkName2Id = {stubbornTry(lambda : p.getBodyInfo(self._id, self._world.getSimConnection()))[baseLinkIdx].decode('ascii'): -1}
        for k in range(stubbornTry(lambda : p.getNumJoints(self._id, self._world.getSimConnection()))):
            info = stubbornTry(lambda : p.getJointInfo(self._id, k, self._world.getSimConnection()))
            if info[2] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                self._jointName2Id[info[1].decode('ascii')] = k
            self._linkName2Id[info[12].decode('ascii')] = k
        self._customInitDynamicModels()
        self._dynamics["rigid_body"] = None
        for cN, cI in ccons.items():
            self.addRigidBodyConstraint(cN[0], cN[1], cN[2])
        for cN, cI in pcons.items():
            self._world._pobjects[cN[1]].addRigidBodyConstraint(cN[0], self._name, cN[2])
    def getJointStates(self):
        retq = {}
        for n,k in self._jointName2Id.items():
            retq[n] = stubbornTry(lambda : p.getJointState(self._id, k, self._world.getSimConnection()))[0:2]
        return retq
    def setJointStates(self, states):
        for n,s in states.items():
            stubbornTry(lambda : p.resetJointState(self._id, self._jointName2Id[n], s[0], 0, self._world.getSimConnection()))
        return
    def getJointId(self, name):
        return self._jointName2Id[name]
    def getLinkId(self, name):
        return self._linkName2Id[name]
    def getId(self):
        return self._id
    def getBodyIdentifiers(self):
        return tuple((x,) for x in self._linkName2Id.keys())
    def getBodyProperty(self, identifier, propertyId):
        if propertyId in ["position", "orientation", "linearVelocity", "angularVelocity"]:
            linkId = -1
            if isinstance(identifier, tuple) and (() != identifier):
                linkId = self._linkName2Id[identifier[0]]
            if 0 <= linkId:
                _, _, _, _, position, orientation, linearVelocity, angularVelocity = stubbornTry(lambda : p.getLinkState(self._id, linkId, 1, False, self._world.getSimConnection()))
            else:
                position, orientation = stubbornTry(lambda : p.getBasePositionAndOrientation(self._id, self._world.getSimConnection()))
                linearVelocity, angularVelocity = stubbornTry(lambda : p.getBaseVelocity(self._id, self._world.getSimConnection()))
            return {"position": position, "orientation": orientation, "linearVelocity": linearVelocity, "angularVelocity": angularVelocity}[propertyId]
        if ("" == identifier) and ("type" == propertyId):
            return self._customStateVariables.get("type", None)
        if not (isinstance(identifier, tuple) and (() != identifier)):
            identifier = [identifier]
        if identifier[0] not in self._customStateVariables:
            return None
        if propertyId not in self._customStateVariables[identifier[0]]:
            return None
        return self._customStateVariables[identifier[0]][propertyId]
    def setBodyProperty(self, identifier, propertyId, value):
        if propertyId in ["position", "orientation", "linearVelocity", "angularVelocity"]:
            linkId = -1
            if isinstance(identifier, tuple) and (() != identifier):
                linkId = self._linkName2Id[identifier[0]]
            if -1 == linkId:
                position, orientation = stubbornTry(lambda : p.getBasePositionAndOrientation(self._id, self._world.getSimConnection()))
                linearVelocity, angularVelocity = stubbornTry(lambda : p.getBaseVelocity(self._id, self._world.getSimConnection()))
                aux = {"position": position, "orientation": orientation, "linearVelocity": linearVelocity, "angularVelocity": angularVelocity}
                aux[propertyId] = value
                stubbornTry(lambda : p.resetBasePositionAndOrientation(self._id, aux["position"], aux["orientation"], self._world.getSimConnection()))
                stubbornTry(lambda : p.resetBaseVelocity(self._id, aux["linearVelocity"], aux["angularVelocity"], self._world.getSimConnection()))
                return True
            return False
        if "" == identifier:
            return None
        if not (isinstance(identifier, tuple) and (() != identifier)):
            identifier = [identifier]
        if identifier[0] not in self._customStateVariables:
            self._customStateVariables[identifier[0]] = {}
        self._customStateVariables[identifier[0]][propertyId] = value
        return True
    def getName(self):
        return str(self._name)
    def remove(self):
        for dob in list(self._debugObjects):
            dob.remove()
        self._world.removePObject(self._name)
    def getCustomDynamicModels(self):
        # Returns a list of functions which, when called, compute new values for custom state variables and then generate functions that update the variables with those new values.
        retq = []
        for k, model in self._dynamics.items():
            if None != model:
                ###retq.append(lambda: model.update(self._world, self))
                retq.append(model.update)
        return retq
    def getCustomDynamicsLocalInputVariables(self, dynamicsName):
        # Returns the custom state variables belonging to this pobject that are relevant for updating with the given custom dynamics.
        if (dynamicsName not in self._dynamics) or ('rigid_body' == dynamicsName):
            return {}
        return {body: self._dynamics[dynamicsName].selectInputVariables(customStateVariables) for body, customStateVariables in self._customStateVariables.items()}
    def getCustomDynamicsOutputVariables(self, dynamicsName):
        # Returns the custom state variables belonging to this pobject that are updated by the given custom dynamics.
        if (dynamicsName not in self._dynamics) or ('rigid_body' == dynamicsName):
            return {}
        return {body: self._dynamics[dynamicsName].selectOutputVariables(customStateVariables) for body, customStateVariables in self._customStateVariables.items()}
    def getRigidBodyVariables(self):
        # Produces a dictionary with keys x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz, joints, links, constraints. 
        # All values are floats, except the value for 
        #     joints which is a dictionary where the keys are joint names and the values are pairs of floats giving that joint's position and velocity;
        #     links which is a dictionary where the keys are link names and the values are dictionaries with keys x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz
        #     constraints which is a list of tuples of form (linkName, parentName, parentLinkName)
        position, orientation = stubbornTry(lambda : p.getBasePositionAndOrientation(self._id, self._world.getSimConnection()))
        linearVelocity, angularVelocity = stubbornTry(lambda : p.getBaseVelocity(self._id, self._world.getSimConnection()))
        retq = {"joints": {}, "links": {}, "constraints": [], "x": position[0], "y": position[1], "z": position[2], "qx": orientation[0], "qy": orientation[1], "qz": orientation[2], "qw": orientation[3], "vx": linearVelocity[0], "vy": linearVelocity[1], "vz": linearVelocity[2], "wx": angularVelocity[0], "wy": angularVelocity[1], "wz": angularVelocity[2]}
        for j, k in self._jointName2Id.items():
            # TODO: ignore joint reaction force and applied force/torque for now, since these are not used for the WorldDump
            retq["joints"][j] = stubbornTry(lambda : p.getJointState(self._id, k, self._world.getSimConnection()))[0:2]
        for l, k in self._linkName2Id.items():
            _, _, _, _, position, orientation, linearVelocity, angularVelocity = stubbornTry(lambda : p.getLinkState(self._id, k, 1, False, self._world.getSimConnection()))
            retq["links"][l] = {"x": position[0], "y": position[1], "z": position[2], "qx": orientation[0], "qy": orientation[1], "qz": orientation[2], "qw": orientation[3], "vx": linearVelocity[0], "vy": linearVelocity[1], "vz": linearVelocity[2], "wx": angularVelocity[0], "wy": angularVelocity[1], "wz": angularVelocity[2]}
        for constraint in self._childOfConstraints.keys():
            retq["constraints"].append(constraint)
        return retq
    def addRigidBodyConstraint(self, linkName, parentName, parentLinkName):
        if (linkName, parentName, parentLinkName) not in self._childOfConstraints:
            parent = self._world._pobjects[parentName]
            childPosition = self.getBodyProperty((linkName,), "position")
            parentPosition = parent.getBodyProperty((parentLinkName,), "position")
            childOrientation = self.getBodyProperty((linkName,), "orientation")
            parentOrientation = parent.getBodyProperty((parentLinkName,), "orientation")
            parentIOrientation = list(parentOrientation)
            parentIOrientation[3] = -parentOrientation[3]
            # Have obj in world, hand in world. Need obj in hand:
            # Toiw = Thiw*Toih => Toih = inv(Thiw)*Toiw
            # Also, iT.t = -iT.q*T.t
            relOr = quaternionProduct(parentIOrientation, childOrientation)
            relPos = [x-y for x,y in zip(p.rotateVector(parentIOrientation,childPosition), p.rotateVector(parentIOrientation, parentPosition))]
            self._childOfConstraints[(linkName, parentName, parentLinkName)] = stubbornTry(lambda : p.createConstraint(parent._id, parent._linkName2Id[parentLinkName], self._id, self._linkName2Id[linkName], p.JOINT_FIXED, [1,0,0], relPos, [0,0,0], relOr, [0,0,0,1], physicsClientId=self._world.getSimConnection()))
            parent._parentOfConstraints[(linkName, self._name, parentLinkName)] = self._childOfConstraints[(linkName, parentName, parentLinkName)]
    def removeRigidBodyConstraint(self, linkName, parentName, parentLinkName):
        stubbornTry(lambda : p.removeConstraint(self._childOfConstraints[(linkName, parentName, parentLinkName)], self._world.getSimConnection()))
        self._world._pobjects[parentName]._parentOfConstraints.pop((linkName, self._name, parentLinkName))
        self._childOfConstraints.pop((linkName, parentName, parentLinkName))
    def resetRigidBodyVariables(self, data):
        # Expects a dictionary with keys x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz, joints, and constraints. 
        # All values are floats, except the value for joints which is a dictionary where the keys are joint names and the values are pairs of floats giving that joint's position and velocity.
        position = [data['x'], data['y'], data['z']]
        orientation = [data['qx'], data['qy'], data['qz'], data['qw']]
        linearVelocity = [data['vx'], data['vy'], data['vz']]
        angularVelocity = [data['wx'], data['wy'], data['wz']]
        stubbornTry(lambda : p.resetBasePositionAndOrientation(self._id, position, orientation, self._world.getSimConnection()))
        stubbornTry(lambda : p.resetBaseVelocity(self._id, linearVelocity, angularVelocity, self._world.getSimConnection()))
        for j, state in data['joints'].items():
            stubbornTry(lambda : p.resetJointState(self._id, self._jointName2Id[j], state[0], state[1], self._world.getSimConnection()))
        for linkName, parentName, parentLinkName in self._childOfConstraints.keys():
            self.removeRigidBodyConstraint(linkName, parentName, parentLinkName)
        for linkName, parentName, parentLinkName in data['constraints']:
            self.addRigidBodyConstraint(linkName, parentName, parentLinkName)
    def applyRigidBodyControls(self, controls):
        # controls is a list where every item is a dictionary with keys jointTargets, +constraints, -constraints.
        #     jointTargets is a dictionary where the keys are joint names and the values are tuples of form (position, velocity, weight); several joint targets may be defined for the same joint, and their weighted average is used.
        #     +constraints is a list of tuples of form (link name, parent pobject name, parent pobject link name); describes a fixed joint constraint that will be added with a link of this pobject as a child of another pobject link.
        #     -constraints is a list of tuples of form (link name, parent pobject name, parent pobject link name). If an item appears both on the +constraints and -constraints lists, then nothing is changed (if already present, is not removed; if not present, it is not added).
        jointTargets = {}
        constraintsAdd = set()
        constraintsDelete = set()
        for control in controls:
            for jointName, jointTarget in control['jointTargets'].items():
                if jointName not in jointTargets:
                    jointTargets[jointName] = [0,0,0]
                jointTargets[jointName] = [x+y*z for x,y,z in zip(jointTargets[jointName], jointTarget, [jointTarget[2], jointTarget[2], 1])]
            for constraint in control['+constraints']:
                constraintsAdd.add(constraint)
            for constraint in control['-constraints']:
                constraintsDelete.add(constraint)
        constraintsWillAdd = constraintsAdd.difference(constraintsDelete)
        constraintsWillDelete = constraintsDelete.difference(constraintsAdd)
        for jointName, jointTarget in jointTargets.items():
            targetPosition = jointTarget[0]/jointTarget[2]
            targetVelocity = jointTarget[1]/jointTarget[2]
            self._lastAppliedJointControls[jointName] = [targetPosition,targetVelocity]
            stubbornTry(lambda : p.setJointMotorControl2(self._id, self._jointName2Id[jointName], p.POSITION_CONTROL, targetPosition=targetPosition, targetVelocity=targetVelocity,force=self._getMaxForce(jointName), positionGain=self._getPositionGain(jointName), maxVelocity=self._getMaxVelocity(jointName), physicsClientId=self._world.getSimConnection()))
        for linkName, parentName, parentLinkName in constraintsWillDelete:
            self.removeRigidBodyConstraint(linkName, parentName, parentLinkName)
        for linkName, parentName, parentLinkName in constraintsWillAdd:
            self.addRigidBodyConstraint(linkName, parentName, parentLinkName)
    def _getJointControlParameter(self, jointName, parameterStore, default):
        if jointName in parameterStore:
            return parameterStore[jointName]
        return default
    def _getMaxForce(self, jointName):
        return self._getJointControlParameter(jointName, self._maxForce, 500)
    def _getPositionGain(self, jointName):
        return self._getJointControlParameter(jointName, self._positionGain, 0.1)
    def _getVelocityGain(self, jointName):
        return self._getJointControlParameter(jointName, self._velocityGain, 1)
    def _getMaxVelocity(self, jointName):
        return self._getJointControlParameter(jointName, self._maxVelocity, 3)

