import pybullet as p
import copy

class PObject():
    def _customInitPreLoad(self, *args, **kwargs):
        return
    def _customInitDynamicModels(self):
        return
    def __init__(self, world, name, initialBasePosition, initialBaseOrientation, *args, **kwargs):
        super().__init__()
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
    def reloadObject(self, position, orientation):
        ccons = copy.deepcopy(self._childOfConstraints)
        pcons = copy.deepcopy(self._parentOfConstraints)
        for cN, cI in self._childOfConstraints.items():
            self.removeRigidBodyConstraint(cN[0], cN[1], cN[2])
        print("remcs")
        for cN, cI in self._parentOfConstraints.items():
            self._world._pobjects[cN[1]].removeRigidBodyConstraint(cN[0], self._name, cN[2])
        print("remce", self._id, {k:v._id for k, v in self._world._pobjects.items()})
        if None != self._id:
            p.removeBody(self._id, self._world.getSimConnection())
        print("remoe")
        self._id = p.loadURDF(self._urdf, position, orientation, self._useMaximalCoordinates, self._useFixedBase, self._urdfFlags, self._globalScaling, self._world.getSimConnection())
        print("load")
        self._jointName2Id = {}
        baseLinkIdx = 0
        if self._useMaximalCoordinates:
            baseLinkIdx = 1
        self._linkName2Id = {p.getBodyInfo(self._id, self._world.getSimConnection())[baseLinkIdx].decode('ascii'): -1}
        for k in range(p.getNumJoints(self._id, self._world.getSimConnection())):
            info = p.getJointInfo(self._id, k, self._world.getSimConnection())
            if info[2] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                self._jointName2Id[info[1].decode('ascii')] = k
            self._linkName2Id[info[12].decode('ascii')] = k
        self._customInitDynamicModels()
        self._dynamics["rigid_body"] = None
        for cN, cI in ccons.items():
            self.addRigidBodyConstraint(cN[0], cN[1], cN[2])
        for cN, cI in pcons.items():
            self._world._pobjects[cN[1]].addRigidBodyConstraint(cN[0], self._name, cN[2])
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
            linkId = self._linkName2Id[identifier[0]]
            if 0 <= linkId:
                _, _, _, _, position, orientation, linearVelocity, angularVelocity = p.getLinkState(self._id, linkId, 1, False, self._world.getSimConnection())
            else:
                position, orientation = p.getBasePositionAndOrientation(self._id, self._world.getSimConnection())
                linearVelocity, angularVelocity = p.getBaseVelocity(self._id, self._world.getSimConnection())
            return {"position": position, "orientation": orientation, "linearVelocity": linearVelocity, "angularVelocity": angularVelocity}[propertyId]
        if identifier[0] not in self._customStateVariables:
            return None
        if propertyId not in self._customStateVariables[identifier[0]]:
            return None
        return self._customStateVariables[identifier[0]][propertyId]
    def setBodyProperty(self, identifier, propertyId, value):
        if propertyId in ["position", "orientation", "linearVelocity", "angularVelocity"]:
            if -1 == self._linkName2Id[identifier[0]]:
                position, orientation = p.getBasePositionAndOrientation(self._id, self._world.getSimConnection())
                linearVelocity, angularVelocity = p.getBaseVelocity(self._id, self._world.getSimConnection())
                aux = {"position": position, "orientation": orientation, "linearVelocity": linearVelocity, "angularVelocity": angularVelocity}
                aux[propertyId] = value
                p.resetBasePositionAndOrientation(self._id, aux["position"], aux["orientation"], self._world.getSimConnection())
                p.resetBaseVelocity(self._id, aux["linearVelocity"], aux["angularVelocity"], self._world.getSimConnection())
                return True
            return False
        self._customStateVariables[identifier[0]][propertyId] = value
        return True
    def getName(self):
        return str(self._name)
    def remove(self):
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
        position, orientation = p.getBasePositionAndOrientation(self._id, self._world.getSimConnection())
        linearVelocity, angularVelocity = p.getBaseVelocity(self._id, self._world.getSimConnection())
        retq = {"joints": {}, "links": {}, "constraints": [], "x": position[0], "y": position[1], "z": position[2], "qx": orientation[0], "qy": orientation[1], "qz": orientation[2], "qw": orientation[3], "vx": linearVelocity[0], "vy": linearVelocity[1], "vz": linearVelocity[2], "wx": angularVelocity[0], "wy": angularVelocity[1], "wz": angularVelocity[2]}
        for j, k in self._jointName2Id.items():
            # TODO: ignore joint reaction force and applied force/torque for now, since these are not used for the WorldDump
            retq["joints"][j] = p.getJointState(self._id, k, self._world.getSimConnection())[0:2]
        for l, k in self._linkName2Id.items():
            _, _, _, _, position, orientation, linearVelocity, angularVelocity = p.getLinkState(self._id, k, 1, False, self._world.getSimConnection())
            retq["links"][l] = {"x": position[0], "y": position[1], "z": position[2], "qx": orientation[0], "qy": orientation[1], "qz": orientation[2], "qw": orientation[3], "vx": linearVelocity[0], "vy": linearVelocity[1], "vz": linearVelocity[2], "wx": angularVelocity[0], "wy": angularVelocity[1], "wz": angularVelocity[2]}
        for constraint in self._childOfConstraints.keys():
            retq["constraints"].append(constraint)
        return retq
    def addRigidBodyConstraint(self, linkName, parentName, parentLinkName):
        if (linkName, parentName, parentLinkName) not in self._childOfConstraints:
            parent = self._world._pobjects[parentName]
            childPosition = self.getBodyProperty((linkName,), "position")
            parentPosition = parent.getBodyProperty((parentLinkName,), "position")
            relPos = [x-y for x,y in zip(childPosition, parentPosition)]
            self._childOfConstraints[(linkName, parentName, parentLinkName)] = p.createConstraint(parent._id, parent._linkName2Id[parentLinkName], self._id, self._linkName2Id[linkName], p.JOINT_FIXED, [1,0,0], relPos, [0,0,0], physicsClientId=self._world.getSimConnection())
            parent._parentOfConstraints[(linkName, self._name, parentLinkName)] = self._childOfConstraints[(linkName, parentName, parentLinkName)]
    def removeRigidBodyConstraint(self, linkName, parentName, parentLinkName):
        p.removeConstraint(self._childOfConstraints[(linkName, parentName, parentLinkName)], self._world.getSimConnection())
        self._world._pobjects[parentName]._parentOfConstraints.pop((linkName, self._name, parentLinkName))
        self._childOfConstraints.pop((linkName, parentName, parentLinkName))
    def resetRigidBodyVariables(self, data):
        # Expects a dictionary with keys x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz, joints, and constraints. 
        # All values are floats, except the value for joints which is a dictionary where the keys are joint names and the values are pairs of floats giving that joint's position and velocity.
        position = [data['x'], data['y'], data['z']]
        orientation = [data['qx'], data['qy'], data['qz'], data['qw']]
        linearVelocity = [data['vx'], data['vy'], data['vz']]
        angularVelocity = [data['wx'], data['wy'], data['wz']]
        p.resetBasePositionAndOrientation(self._id, position, orientation, self._world.getSimConnection())
        p.resetBaseVelocity(self._id, linearVelocity, angularVelocity, self._world.getSimConnection())
        for j, state in data['joints'].items():
            p.resetJointState(self._id, self._jointName2Id[j], state[0], state[1], self._world.getSimConnection())
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
            p.setJointMotorControl2(self._id, self._jointName2Id[jointName], p.POSITION_CONTROL, targetPosition=targetPosition, targetVelocity=targetVelocity,force=self._getMaxForce(jointName), positionGain=self._getPositionGain(jointName), maxVelocity=self._getMaxVelocity(jointName), physicsClientId=self._world.getSimConnection())
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

