import os
import sys
import json
import math
import numpy as np
import platform
import pybullet
import pybullet_data
import copy
import time

#### The World class
#
#    Acts as a wrapper around pybullet functionality. The wrapper uses human-readable strings for object identification, as opposed to pybullet's
#    integer ids. It makes it possible to access functions relating to links, joints, and kinematic constraints with the same functions as for
#    whole kinematic trees, streamlining the interfaces for getting/setting properties. Further, it integrates custom dynamics in the simulation
#    step; custom dynamics are scripts that check on relevant object properties and update various state variables, both custom and physical.
#    Custom dynamics are useful to approximate phenomena outside of the scope of the rigid body simulator, such as baking, or to simplify some
#    mechanical interactions that would otherwise be hard to model robustly, such as grasping, or introduce other force fields than gravity,
#    such as the forces between a collection of particles that model a fluid.

#### World Object types -- basic
#
#    (Afford: addition to/removal from world, reportable by worldDump, resetable by greatReset, can retrieve/set property values)
#
#    Kinematic tree
#    Kinematic Constraint
#    Marker
#
#### World Object types -- derived (parts of basic objects, exist only as attached to these basic objects)
#
#    (Afford: can retrieve/set property values)
#
#    Kinematic Link
#    Kinematic Joint
#    Geometric Feature
#
#### Object groups
#
#    Aggregates: only reported by worldDump, usable in greatResets. Not stored as such in a World instance.
#
#### Special properties
#
#      Common to all basic objects
#
#    name: string, used in identifying an object; not writeable
#    simtype: string, used to identify what kind of basic world object this is; can be one of ktree, kcon, marker; not writable
#    idx: integer, used by pybullet to identify the object; not reported by worldDump, instead a new idx is generated whenever an object is loaded or a greatReset happens; invisible to
#                getObjectProperty, not writable
#    type: string, used to identify an ontological type; ontological types may be supplied by an ontology and serve as a shorthand for a bundle of expectations about an object; not writable
#    customStateVariables: dictionary containing key/value pairs used by custom dynamics; the values are expected to change during simulation
#    fn: dictionary containing key/value pairs where the values are constants describing the object, e.g. a path to its mesh, body-relevant points for geometric features etc.
#
#      Common to Kinematic trees, links, and markers
#
#    position: (x,y,z) tuple describing position in world frame; writable only for base link
#    orientation: (x,y,z,w) tuple describing orientation in world frame; writable only for base link
#    linearVelocity: (x,y,z) tuple describing linear velocity in world frame; writable only for base link
#    angularVelocity: (x,y,z) tuple describing angular velocity in world frame; writable only for base link
#    aabb: ((xmin,ymin,zmin), (xmax,ymax,zmax)) tuple describing the axis-aligned bounding box in world frame; not reported by WorldDump, not writable
#    localAABB: ((xmin,ymin,zmin), (xmax,ymax,zmax)) tuple describing the axis-aligned bounding box in object base local frame; not reported by WorldDump, not writable
#
#      Common to Kinematic trees, links
#
#    mass: float
#    lateralFriction: float
#    localInertiaDiagonal: (xx,yy,zz) vector
#    localInertiaPosition: (x,y,z vector); not writable
#    localInertiaOrientation: (x,y,z,w) vector; not writable
#    restitution: float
#    rollingFriction: float
#    spinningFriction: float
#    contactDamping: float
#    contactStiffness: float
#    frictionAnchor: boolean; TODO retrieve value from pybullet
#    linearDamping: float; TODO retrieve value from pybullet 
#    angularDamping: float; TODO retrieve value from pybullet
#    upperLimit: float; not writable
#    lowerLimit: float; not writable
#    TODO: jointDamping, jointFriction
#
#      Kinematic trees
#
#    links: dictionary containing key-value pairs of variables associated to links: idx; getObjectProperty only returns key list, not writable by setObjectProperty
#    joints: dictionary containing key-value pairs of variables associated to joints: idx; getObjectProperty only returns key list, not writable by setObjectProperty
#    idx2Link: dictionary mapping pybullet link ids to link names; not reported by worldDump, instead is generated whenever an object is loaded or a greatReset happens, invisible to getObjectProperty
#    idx2Joint: dictionary mapping pybullet link ids to link names; not reported by worldDump, instead is generated whenever an object is loaded or a greatReset happens, invisible to getObjectProperty
#    baseLinkName: name of the base link; not reported by worldDump, not writable
#    dofList: a list of (joint name, joint dof count) in the order in which they appear in a list of degrees of freedom, e.g. as used by calculateInverseDynamics; not reported by worldDump, 
#                instead is generated whenever an object is loaded or a greatReset happens; not writable
#    jointPositions: a dictionary mapping joint names to position values
#    jointVelocities: a dictionary mapping joint names to velocity values
#    jointReactionForces: a dictionary mapping joint names to vectors of 6 components giving forces and torques on the joint
#    jointAppliedTorques: a dictionary mapping joint names to the last applied torque
#    at: None or a string identifying a kinematic tree that is construed as a reference for location for this kinematic tree; not writable
#    atComponent: None or a pair of strings identifying a kinematic tree and its link which is construed as a reference for location for this kinematic tree; not writable
#    parentTo: a list of names of constraints that this ktree is a parent of; not writable
#    childOf: a list of names of constraints that this ktree is a child of; not writable
#    filename: path to the file describing the kinematic tree
#    immobile: boolean indicating whether to use a fixed base; not writeable
#
#      Links
#
#    parent: name of the parent joint, or None for the base link; not reported by worldDump, not writable
#    fn/maxForce/<link name>: maximum force that parent joint can apply
#
#      Constraints:
#
#    force: a vector of forces last applied to constrained degrees of freedom; not writable
#    parent: name of parent kinematic tree
#    child: name of child kinematic tree
#    parentLink: name of parent link
#    childLink: name of child link
#    jointType: one of fixed, prismatic, revolute
#    jointAxis: an (x,y,z) tuple, joint axis is in child link frame
#    childFramePosition: an (x,y,z) tuple, position of joint in child link frame
#    childFrameOrientation: an (x,y,z,w) tuple, orientation of joint in child link center of mass frame
#    maxForce: a float giving the maximum force that the constraint can apply

dirPath = os.path.dirname(os.path.realpath(__file__))

class World():
    def __init__(self, pybulletOptions="", useGUI=True, name="pybulletWorld", gravity=None, backgroundTemperature=None, heatExchangeCoefficient=None, worldSize=1000000.0, objectKnowledge=None, processKnowledge=None, customDynamics=None, simFrameRate=None):
        self._kincompDyn = None
        self._name = name
        self._limbo = []
        self._ylem = {}
        self._kinematicTrees = {}
        self._kinematicConstraints = {}
        self._markers = {}
        self._idx2KinematicTree = {}
        self._customDynamicsAPI = {'ktree': {}, 'kcon': {}, 'marker': {}}
        self._customDynamics = []
        self._lastProfile = {"stepSimulation":0, "customDynamics": {}}
        if customDynamics is not None:
            self._customDynamics = list(customDynamics)
        self._profile = 0
        self._profileCLS = 0
        self._profileCON = 0
        self._profileOVR = 0
        self._profileGET = 0
        self._profileSET = 0
        self._detProfGet = {}
        self._cacheTime = 0
        self._getObjectPropertyCache = {}
        self._sfr = 240
        self._frameStepCount = 1
        if simFrameRate is not None:
            self._sfr = simFrameRate
        self._customDynamicsUpdaters = {'ktree': {}, 'kcon': {}, 'marker': {}}
        self._debugVisualizationsEnabled = True ### TODO: infer from pybulletOptions
        self._collisionShapes = {}
        self._computedCollisions = False
        self._collisions = None
        self._worldSize = worldSize
        self._pybulletOptions = pybulletOptions
        if useGUI:
            self._pybulletConnection = stubbornTry(lambda : pybullet.connect(pybullet.GUI, options=pybulletOptions))
        else:
            self._pybulletConnection = stubbornTry(lambda : pybullet.connect(pybullet.DIRECT, options=pybulletOptions))
        # TODO: record a list of search paths, coz pybullet will only remember one otherwise.
        stubbornTry(lambda : pybullet.setTimeStep(1.0/(1.0*self._sfr), self._pybulletConnection))
        stubbornTry(lambda : pybullet.setAdditionalSearchPath(dirPath))
        self._byssos = stubbornTry(lambda : pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(), 'plane.urdf'), (0,0,-100), (0,0,0,1)))
        self._colliderProbe = stubbornTry(lambda : pybullet.loadURDF(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'collider_probe.urdf'), (0,0,-110), (0,0,0,1)))
        if gravity is None:
            gravity = (0, 0, 0)
        self._down = (0,0,-1)
        self._up = (0,0,1)
        if backgroundTemperature is None:
            backgroundTemperature = 20.0
        if heatExchangeCoefficient is None:
            heatExchangeCoefficient = 0.05
        self._backgroundTemperature = backgroundTemperature
        self._heatExchangeCoefficient = heatExchangeCoefficient
        self.setGravity(gravity)
        self._newObjectCounter = 0
        self._processKnowledge = {}
        if processKnowledge is not None:
            self._processKnowledge = copy.deepcopy(processKnowledge)
        self._objectKnowledge = {}
        if objectKnowledge is not None:
            self._objectKnowledge = copy.deepcopy(objectKnowledge)
        self._customDynamicsAPIBase = {
          'leetHAXXOR': (lambda : self),
          'getFrameStepCount': (lambda : self.getFrameStepCount()),
          'setFrameStepCount': (lambda x : self.setFrameStepCount()),
          'getSFR' : (lambda : self.getSFR()),
          'addObject': (lambda x : self.addNewObject(x)), 
          'getObjectProperty': (lambda x,y,defaultValue=None: self.PgetObjectProperty(x,y,defaultValue=defaultValue)),
          'getDistance': (lambda x,y,z: self.getDistance(x,y,z)), 
          'probeClosestPoints': (lambda x, pos, maxDistance=None: self.probeClosestPoints(x, pos, maxDistance=maxDistance)),
          'checkCollision': (lambda x, identifierB=None : self.checkCollision(x, identifierB=identifierB)), 
          'checkOverlap': (lambda x, : self.checkOverlap(x)), 
          'checkClosestPoints': (lambda x,y, maxDistance=None : self.checkClosestPoints(x, y, maxDistance=maxDistance)), 
          'getCameraImage': (lambda cameraPosition=None, cameraTarget=None, cameraUpAxis=None, fovInDegrees=60.0, aspectRatio=1.0, near=0.1, far=15, xSize=240, ySize=240: self.getCameraImage(cameraPosition=cameraPosition, cameraTarget=cameraTarget, cameraUpAxis=cameraUpAxis, fovInDegrees=fovInDegrees, aspectRatio=aspectRatio, near=near, far=far, xSize=xSize, ySize=ySize)), 
          'adjustAABBRadius': (lambda x,y: self.adjustAABBRadius(x, y)), 
          'addAABBRadius': (lambda x,y: self.addAABBRadius(x, y)), 
          'objectPoseRelativeToObject': (lambda x, y, z, w: self.objectPoseRelativeToObject(x, y, z, w)), 
          'objectPoseRelativeToWorld': (lambda x, y, z, w: self.objectPoseRelativeToWorld(x, y, z, w)), 
          'orientationDifferenceAA': (lambda x, y: self.orientationDifferenceAA(x, y)), 
          'handPoseToBringObjectToPose': (lambda x, y, z, w: self.handPoseToBringObjectToPose(x, y, z, w)), 
          'distance': (lambda x,y: self.distance(x, y)), 
          'getNewObjectCounter': (lambda : self.getNewObjectCounter()), 
          'getGravity': (lambda : self.getGravity()), 
          'getDown': (lambda : self.getDown()), 
          'getUp': (lambda : self.getUp()), 
          'getBackgroundTemperature': (lambda : self.getBackgroundTemperature()), 
          'getObjectTypeKnowledge': (lambda x : self.getObjectTypeKnowledge(x)),
          'getProcessOutcome': (lambda x : self.getProcessOutcome(x)), 
          'getProcessResource': (lambda x: self.getProcessResource(x))}
    def getLastProfile(self):
        return self._lastProfile
    def getObjectTypeKnowledge(self, otype):
        return copy.deepcopy(self._objectKnowledge.get(otype, None))
    def copy(self):
        worldState = self.worldDump()
        retq = World(pybulletOptions=self._pybulletOptions, useGUI=False, name=("copy(%s)" % self._name), worldSize=self._worldSize)
        retq.greatReset(worldState)
        return retq
    def getName(self):
        return self._name
    def getSimulatorConnection(self):
        return self._pybulletConnection
    def getSFR(self):
        return self._sfr
    def getFrameStepCount(self):
        return self._frameStepCount
    def setFrameStepCount(self, x):
        self._frameStepCount = x
    def adjustAABBRadius(self, aabb, radius):
        center = [(x+y)/2 for x,y in zip(aabb[0], aabb[1])]
        return tuple([tuple([x-radius for x in center]), tuple([x+radius for x in center])])
    def addAABBRadius(self, aabb, radius):
        return tuple([tuple([x-radius for x in aabb[0]]), tuple([x+radius for x in aabb[1]])])
    def getGravity(self):
        return self._gravity
    def getDown(self):
        return self._down
    def getUp(self):
        return self._up
    def getBackgroundTemperature(self):
        return self._backgroundTemperature, self._heatExchangeCoefficient
    def setGravity(self, gravity):
        self._gravity = gravity
        stubbornTry(lambda : pybullet.setGravity(self._gravity[0], self._gravity[1], self._gravity[2], self._pybulletConnection))
        norm = math.sqrt(gravity[0]*gravity[0] + gravity[1]*gravity[1] + gravity[2]*gravity[2])
        if 0.00001 > norm:
            self._down = (0,0,-1)
            self._up = (0,0,1)
        else:
            self._down = tuple([x/norm for x in gravity])    
            self._up = tuple([-x/norm for x in gravity])    
    def getNewObjectCounter(self):
        self._newObjectCounter += 1
        return self._newObjectCounter
    def _getProcessRoleKnowledge(self, processDescription, role):
        if ('process' not in processDescription) or ('patient' not in processDescription) or (role not in self._processKnowledge):
            return None
        relevantKnowledge = self._processKnowledge[role][processDescription['process']][processDescription['patient']]
        specificKey = frozenset([(k, v) for k,v in processDescription.items() if k not in ['process', 'patient']])
        default = relevantKnowledge.get('default', None)
        if specificKey in relevantKnowledge:
            return copy.deepcopy(relevantKnowledge[specificKey])
        retq = None
        minDiff = None
        for k, v in relevantKnowledge.items():
            if 'default' == k:
                continue
            crDiff = len(specificKey.difference(k))
            if (minDiff is None) or (cdDiff < minDiff):
                retq = v
            elif crDiff == minDiff:
                retq = None
        if retq is not None:
            return copy.deepcopy(retq)
        return copy.deepcopy(default)
    def getProcessOutcome(self, processDescription):
        return self._getProcessRoleKnowledge(processDescription, 'outcome')
    def getProcessResource(self, processDescription):
        if ('process' not in processDescription) or ('patient' not in processDescription) or ('resource' not in self._processKnowledge):
            return None
        return self._processKnowledge['resource'][processDescription['process']][processDescription['patient']]
        #return self._getProcessRoleKnowledge(processDescription, 'resource')
    def concludeProcess(self, processDescription, name, link=None, adjustments=None):
        retq = {'added': [], 'deleted': [], 'replaced': []}
        if name not in self._kinematicTrees:
            return retq
        if adjustments is None:
            adjustments = {'toAdd': {}, 'toReplace': {}}
        outcome = self.getProcessOutcome(processDescription)
        identifier = (name,)
        if link is not None:
            identifier = (name, link)
        position = self.getObjectProperty(identifier, 'position')
        orientation = self.getObjectProperty(identifier, 'orientation')
        if ('toReplace' in outcome) and (outcome['toReplace'] in self._objectKnowledge):
            # TODO: ability to swap with a preloaded object
            retq['replaced'].append(name)
            newDescription = copy.deepcopy(self._objectKnowledge[outcome['toReplace']])
            newDescription['name'] = name
            self.reloadObject(name, newDescription['type'], newDescription['filename'])
            self.setObjectProperty((name,), ('customStateVariables',), newDescription['customStateVariables'])
            self.setObjectProperty((name,), ('fn',), newDescription['fn'])
            if newDescription['type'] in adjustments['toReplace']:
                fn = adjustments['toReplace'][newDescription['type']].get('fn', {})
                csv = adjustments['toReplace'][newDescription['type']].get('customStateVariables', {})
                if 'fn' not in self._kinematicTrees[name]:
                   self._kinematicTrees[name]['fn'] = {}
                if 'customStateVariables' not in self._kinematicTrees[name]:
                   self._kinematicTrees[name]['customStateVariables'] = {}
                self._kinematicTrees[name]['fn'].update(fn)
                self._kinematicTrees[name]['customStateVariables'].update(csv)
            self._setCustomUpdaters('ktree', newDescription['name'])
        elif ('toDelete' in outcome) and (outcome['toDelete'] is True):
            retq['deleted'].append(name)
            self.removeObject((name,), sendToLimbo=True)
        if 'toAdd' in outcome:
            for addType, poses in outcome['toAdd']:
                if addType not in self._objectKnowledge:
                    continue
                for addPosition, addOrientation in poses:
                    addPosition, addOrientation = self.objectPoseRelativeToWorld(position, orientation, addPosition, addOrientation)
                    if (addType in self._ylem) and (0 < len(self._ylem[addType])):
                        newDescription = self._ylem[addType].pop()
                        while newDescription['name'] in self._kinematicTrees:
                            newDescription['name'] += '_c'
                        self._kinematicTrees[newDescription['name']] = newDescription
                        self.setObjectProperty((newDescription['name'],), "position", addPosition)
                        self.setObjectProperty((newDescription['name'],), "orientation", addOrientation)
                    else:
                        description = self._objectKnowledge[addType]
                        newDescription = copy.deepcopy(description)
                        newDescription['position'], newDescription['orientation'] = addPosition, addOrientation
                        newDescription['name'] = ('%s_%d' % (newDescription['type'], self.getNewObjectCounter()))
                        self.addObject(newDescription)
                    name = newDescription['name']
                    retq['added'].append(name)
                    if newDescription['type'] in adjustments['toAdd']:
                        fn = adjustments['toAdd'][newDescription['type']].get('fn', {})
                        csv = adjustments['toAdd'][newDescription['type']].get('customStateVariables', {})
                        if 'fn' not in self._kinematicTrees[name]:
                           self._kinematicTrees[name]['fn'] = {}
                        if 'customStateVariables' not in self._kinematicTrees[name]:
                           self._kinematicTrees[name]['customStateVariables'] = {}
                        self._kinematicTrees[name]['fn'].update(fn)
                        self._kinematicTrees[name]['customStateVariables'].update(csv)
                    self._setCustomUpdaters('ktree', newDescription['name'])
        return retq
    def preload(self, otype, oname, position):
        newDescription = copy.deepcopy(self._objectKnowledge[otype])
        newDescription['name'] = oname
        newDescription['position'] = position
        newDescription['orientation'] = [0,0,0,1]
        self.preloadKTree(newDescription)
    def addObjectInstance(self, otype, oname, position, orientation, linearVelocity=None, angularVelocity=None):
        if linearVelocity is None:
            linearVelocity = (0,0,0)
        if angularVelocity is None:
            angularVelocity = (0,0,0)
        newDescription = copy.deepcopy(self._objectKnowledge[otype])
        newDescription['name'] = oname
        newDescription['position'] = position
        newDescription['orientation'] = orientation
        newDescription['linearVelocity'] = linearVelocity
        newDescription['angularVelocity'] = angularVelocity
        self.addObject(newDescription)
        return oname
    def _addKinematicConstraintInternal(self, objData):
        pidx = self._kinematicTrees[objData['parent']]['idx']
        cidx = self._kinematicTrees[objData['child']]['idx']
        plidx = self._kinematicTrees[objData['parent']]['links'][objData['parentLink']]['idx']
        clidx = self._kinematicTrees[objData['child']]['links'][objData['childLink']]['idx']
        jtype = {'fixed': pybullet.JOINT_FIXED, 'revolute': pybullet.JOINT_REVOLUTE, 'prismatic': pybullet.JOINT_PRISMATIC}[objData.get('jointType', 'fixed')]
        jaxis = objData.get('jointAxis', (1,0,0))
        if 0 <= plidx:
            _, _, _, _, parentPosition, parentOrientation, _, _ = stubbornTry(lambda : pybullet.getLinkState(pidx, plidx, 1, False, self._pybulletConnection))
        else:
            parentPosition, parentOrientation = stubbornTry(lambda : pybullet.getBasePositionAndOrientation(pidx, self._pybulletConnection))
        if 0 <= clidx:
            _, _, _, _, childPosition, childOrientation, _, _ = stubbornTry(lambda : pybullet.getLinkState(cidx, clidx, 1, False, self._pybulletConnection))
        else:
            childPosition, childOrientation = stubbornTry(lambda : pybullet.getBasePositionAndOrientation(cidx, self._pybulletConnection))
        invParentPosition, invParentOrientation = stubbornTry(lambda : pybullet.invertTransform(parentPosition, parentOrientation))
        ppos, pq = stubbornTry(lambda : pybullet.multiplyTransforms(invParentPosition, invParentOrientation, childPosition, childOrientation))
        cpos = (0,0,0)
        cq = (0,0,0,1)
        objData['idx'] = stubbornTry(lambda : pybullet.createConstraint(pidx, plidx, cidx, clidx, jtype, jaxis, ppos, cpos, parentFrameOrientation=pq, childFrameOrientation=cq, physicsClientId=self._pybulletConnection))
        stubbornTry(lambda : pybullet.changeConstraint(objData['idx'], maxForce=objData.get('maxForce', 1000.0)))
        self._kinematicTrees[objData['parent']]['parentTo'].append(objData['name'])
        self._kinematicTrees[objData['child']]['childOf'].append(objData['name'])
    def _addKinematicConstraint(self, description):
        if ('parent' not in description) or ('child' not in description) or ('parentLink' not in description) or ('childLink' not in description):
            return None
        if (description['parent'] not in self._kinematicTrees) or (description['child'] not in self._kinematicTrees) or (description['parentLink'] not in self._kinematicTrees[description['parent']]['links']) or (description['childLink'] not in self._kinematicTrees[description['child']]['links']):
            return None
        name = description['name']
        if name in self._kinematicConstraints:
            self._removeKinematicConstraint(name)
        objData = copy.deepcopy(description)
        self._addKinematicConstraintInternal(objData)
        self._kinematicConstraints[name] = objData
        return True
    def _addKinematicTree(self, description):
        if 'name' not in description:
            return None
        name = description['name']
        objData = copy.deepcopy(description)
        toRestore = []
        if name in self._kinematicTrees:
            for e,v in self._kinematicConstraints.items():
                if name in [v['parent'], v['child']]:
                    toRestore.append(self._describeObject(e))
            self._removeKinematicTree(name)
        pos = objData.get('position', (0,0,0))
        orn = objData.get('orientation', (0,0,0,1))
        lin = objData.get('linearVelocity', (0,0,0))
        ang = objData.get('angularVelocity', (0,0,0))
        ufb = {False: 0, True: 1}[objData.get('immobile', False)]
        flags = 0
        fname = objData.get('filename', '')
        objData['idx'] = stubbornTry(lambda : pybullet.loadURDF(fname, basePosition=(0,0,0), baseOrientation=(0,0,0,1), useFixedBase=ufb, flags=flags, physicsClientId=self._pybulletConnection))
        objData['links'] = {}
        objData['joints'] = {}
        objData['idx2Link'] = {}
        objData['idx2Joint'] = {}
        objData['dofList'] = []
        objData['parentTo'] = []
        objData['childOf'] = []
        baseLinkIdx = 0 # or 1 if using maximal coordinates
        baseName = stubbornTry(lambda : pybullet.getBodyInfo(objData['idx'], self._pybulletConnection))[baseLinkIdx].decode('ascii')
        objData['links'][baseName]= {'idx': -1}
        objData['idx2Link'][-1] = baseName
        jointNum = stubbornTry(lambda : pybullet.getNumJoints(objData['idx'], self._pybulletConnection))
        for k in range(jointNum):
            jdata = stubbornTry(lambda : pybullet.getJointInfo(objData['idx'], k, self._pybulletConnection))
            jname = jdata[1].decode('utf-8')
            objData['joints'][jname] = {'idx': k}
            objData['idx2Joint'][k] = jname
            lname = jdata[12].decode('ascii')
            objData['links'][lname] = {'idx': k}
            objData['idx2Link'][k] = lname
            juidx = jdata[4]
            if -1 != juidx:
                jdof = {pybullet.JOINT_PRISMATIC: 1, pybullet.JOINT_REVOLUTE: 1, pybullet.JOINT_GEAR: 1, pybullet.JOINT_PLANAR: 3, pybullet.JOINT_SPHERICAL: 2, pybullet.JOINT_POINT2POINT: 3}[jdata[2]]
                objData['dofList'].append((juidx, jname, jdof))
            maxForce = objData.get('fn', {}).get('maxForce',{}).get(lname, None)
            if maxForce is None:
                maxForce = jdata[10]
                _setDictionaryEntry(objData, ('fn', 'maxForce', lname), maxForce)
            stubbornTry(lambda : pybullet.setJointMotorControl2(objData['idx'], k, pybullet.VELOCITY_CONTROL, force=maxForce, physicsClientId=self._pybulletConnection))
        objData['localAABB'] = {}
        mcs = None
        Mcs = None
        for e in objData['links']:
            objData['localAABB'][e] = stubbornTry(lambda : pybullet.getAABB(objData['idx'], objData['links'][e]['idx'], self._pybulletConnection))
            if mcs is None:
                mcs = list(objData['localAABB'][e][0])
            if Mcs is None:
                Mcs = list(objData['localAABB'][e][1])
            mcs = [min(x,y) for x,y in zip(mcs, objData['localAABB'][e][0])]
            Mcs = [max(x,y) for x,y in zip(Mcs, objData['localAABB'][e][1])]
        objData['localAABB'][0] = tuple([tuple(mcs), tuple(Mcs)])
        stubbornTry(lambda : pybullet.resetBasePositionAndOrientation(objData['idx'], pos, orn, self._pybulletConnection))
        stubbornTry(lambda : pybullet.resetBaseVelocity(objData['idx'], linearVelocity=lin, angularVelocity=ang, physicsClientId=self._pybulletConnection))
        objData['dofList'] = [(x[1], x[2]) for x in sorted(objData['dofList'])]
        for k,v in objData['joints'].items():
            pos = objData.get('jointPositions', {k: 0.0}).get(k, 0.0)
            vel = objData.get('jointVelocities', {k: 0.0}).get(k, 0.0)
            stubbornTry(lambda : pybullet.resetJointState(objData['idx'], v['idx'], pos, targetVelocity=vel, physicsClientId=self._pybulletConnection))
            stubbornTry(lambda : pybullet.enableJointForceTorqueSensor(objData['idx'], v['idx'], enableSensor=True, physicsClientId=self._pybulletConnection))
        if ('fn' in objData):
            for l in objData['links']:
                dynamicInfo = stubbornTry(lambda : pybullet.getDynamicsInfo(objData['idx'], objData['links'][l]['idx'], self._pybulletConnection))
                newDynamics = {'physicsClientId': self._pybulletConnection}
                for prop in ['mass', 'lateralFriction', 'spinningFriction', 'restitution', 'contactStiffness', 'contactDamping', 'localInertiaDiagonal', 'rollingFriction', 'linearDamping', 'angularDamping']:
                    if (prop in objData['fn']) and (l in objData['fn'][prop]):
                        newDynamics[prop] = objData['fn'][prop][l]
                if ('frictionAnchor' in objData['fn']) and (l in objData['fn']['frictionAnchor']):
                    if objData['fn']['frictionAnchor'][l]:
                        newDynamics['frictionAnchor'] = 1
                    else:
                        newDynamics['frictionAnchor'] = 0
                    stubbornTry(lambda : pybullet.changeDynamics(objData['idx'], objData['links'][l]['idx'], **newDynamics))
        for e in objData['links']:
            if (fname, e) not in self._collisionShapes:
                ans = stubbornTry(lambda : pybullet.getCollisionShapeData(objData['idx'], objData['links'][e]['idx'], self._pybulletConnection))
                if 0 < len(ans):
                    _, _, _, _, meshFilename, localPosition, localOrientation = ans[0]
                    meshFilename = meshFilename.decode('utf-8')
                    self._collisionShapes[(fname, e)] = stubbornTry(lambda : pybullet.createCollisionShape(pybullet.GEOM_MESH, fileName=meshFilename, collisionFramePosition=localPosition, collisionFrameOrientation=localOrientation, physicsClientId=self._pybulletConnection))
            mass = stubbornTry(lambda : pybullet.getDynamicsInfo(objData['idx'], objData['links'][e]['idx'], self._pybulletConnection))[0]
            if 0 < mass:
                _setDictionaryEntry(objData, ('fn', 'mass', '_previousNonzeroMass', e), mass)
        if 'jointReactionForces' in objData:
            objData['jointReactionForces'] = {k: [0.0]*len(v) for k,v in objData['jointReactionForces'].items()}
        if 'jointAppliedTorques' in objData:
            objData['jointAppliedTorques'] = {k: 0.0 for k in objData['jointAppliedTorques']}
        self._kinematicTrees[name] = objData
        self._idx2KinematicTree[objData['idx']] = name
        objData['at'] = self.at((name,))
        objData['atComponent'] = self.atComponent((name,))
        for e in toRestore:
            #print("RESTORE", e)
            self._addKinematicConstraint(e)
        return True
    def _addMarker(self, description):
        # TODO
        return
    def _addAggregate(self, description):
        particles = description.get('particles', [])
        for p in particles:
            if 'ktree' == p.get('simtype', None):
                self._addKinematicTree(self, p)
    def addObject(self, description):
        addFn = {'ktree': self._addKinematicTree, 'kcon': self._addKinematicConstraint, 'marker': self._addMarker, 'aggregate': self._addAggregate}
        if ('name' in description) and ('simtype' in description) and (description['simtype'] in addFn):
            retq = addFn[description['simtype']](description)
            self._setCustomUpdaters(description['simtype'], description['name'])
            return retq
        return False
    def preloadKTree(self, description):
        if ('name' in description) and ('ktree' == description.get('simtype', '')) and ('type' in description):
            self._addKinematicTree(description)
            description = self._kinematicTrees.pop(description['name'])
            if description['type'] not in self._ylem:
                self._ylem[description['type']] = []
            self._ylem[description['type']].append(description)
    def _setCustomUpdaters(self, simtype, name):
        def _makeCustomDynamicsAPI(base, name):
            def _gateByName(name, identifier, fn, args, kwargs):
                if (self._isIdentifier(identifier)) and (name == identifier[0]):
                    return fn(*args, **kwargs)
                return None
            retq = base.copy()
            retq['concludeProcess'] = lambda x,link=None: self.concludeProcess(x, name, link=link)
            retq['removeObject'] = lambda : self.removeObject((name,))
            retq['setObjectProperty'] = lambda x, y, z : self.PsetObjectProperty(tuple([name] + list(x)), y, z)
            retq['applyJointControl'] = lambda jnt, mode=None, targetPosition=None, targetVelocity=None, force=None, positionGain=None, velocityGain=None, maxVelocity=None: self.applyJointControl((name, jnt), mode=mode, targetPosition=targetPosition, targetVelocity=targetVelocity, force=force, positionGain=positionGain, velocityGain=velocityGain, maxVelocity=maxVelocity)
            retq['applyExternalForce'] = lambda x, y, z, inWorldFrame=False : _gateByName(name, x, self.applyExternalForce, (x,y,z), {'inWorldFrame': inWorldFrame})
            retq['applyExternalTorque'] = lambda x, y, inWorldFrame=False : _gateByName(name, x, self.applyExternalTorque, (x,y), {'inWorldFrame': inWorldFrame})
            retq['computeJacobian'] = lambda x, y, positions=None, velocities=None, accelerations=None : self.computeJacobian(name, x, y, positions=positions, velocities=velocities, accelerations=accelerations)
            retq['calculateMassMatrix'] = lambda positions=None : self.calculateMassMatrix(name, positions=positions)
            retq['calculateInverseDynamics'] = lambda positions=None, velocities=None, accelerations=None : self.calculateInverseDynamics(name, positions=positions, velocities=velocities, accelerations=accelerations)
            retq['calculateInverseKinematics'] = lambda x, y, targetOrientation=None: self.calculateInverseKinematics(name, x, y, targetOrientation=targetOrientation)
            return retq
        self._customDynamicsAPI[simtype][name] = _makeCustomDynamicsAPI(self._customDynamicsAPIBase, name)
        self._customDynamicsUpdaters[simtype][name] = []
        #for isApplicable, updateFn in self._customDynamics:
        #    if isApplicable(lambda x, y=None: self.getObjectProperty((name,), x, y)):
        for disposition, updateFn in self._customDynamics:
            if (('fn', 'kinematicallyControlable') == disposition):
                self._kincompDyn = updateFn
            if self.getObjectProperty((name,), disposition):
                self._customDynamicsUpdaters[simtype][name].append(updateFn)    
    def addNewObject(self, description):
        if ('name' not in description) or ('simtype' not in description):
            return None
        name = description['name']
        st = description['simtype']
        if name not in {'ktree': self._kinematicTrees, 'kcon': self._kinematicConstraints, 'marker': self._markers}[st]:
            return self.addObject(description)
        return False
    def reloadObject(self, name, objectType, urdf):
        if name not in self._kinematicTrees:
            return None
        description = self._describeObject(name)
        description['filename'] = urdf
        description['type'] = objectType
        return self._addKinematicTree(description)
    def _removeKinematicConstraint(self, name, sendToLimbo=False):
        stubbornTry(lambda : pybullet.removeConstraint(self._kinematicConstraints[name]['idx'], self._pybulletConnection))
        self._kinematicTrees[self._kinematicConstraints[name]['parent']]['parentTo'].remove(name)
        self._kinematicTrees[self._kinematicConstraints[name]['child']]['childOf'].remove(name)
        self._kinematicConstraints.pop(name)
        return True
    def _removeKinematicTree(self, name, sendToLimbo=False):
        self._removeAttachedConstraints(name)
        if sendToLimbo:
            positionDel = list(self.getObjectProperty((name,),'position'))
            positionDel[2] -= 10
            a = self.setObjectProperty((name,),'position',positionDel)
            toDel = self._kinematicTrees.pop(name)
            self._limbo.append(toDel)
        else:
            stubbornTry(lambda : pybullet.removeBody(self._kinematicTrees[name]['idx'], self._pybulletConnection))
            self._kinematicTrees.pop(name)
        return True
    def _removeMarker(self, name, sendToLimbo=False):
        # TODO
        return
    def _removeAttachedConstraints(self, name):
        for e in list(self._kinematicTrees[name]['parentTo'] + self._kinematicTrees[name]['childOf']):
            self._removeKinematicConstraint(e)
        return True
    def _removeCustomDynamics(self, objType, objMap, identifier):
        if identifier[0] in objMap:
            self._customDynamicsAPI[objType].pop(identifier[0])
            self._customDynamicsUpdaters[objType].pop(identifier[0])
            return True
        return False
    def removeObject(self, identifier, sendToLimbo=False):
        if (self._isIdentifier(identifier)) and (1 == len(identifier)):
            for objType, objMap, remFn in [('marker', self._markers, self._removeMarker), ('kcon', self._kinematicConstraints, self._removeKinematicConstraint), ('ktree', self._kinematicTrees, self._removeKinematicTree)]:
                if self._removeCustomDynamics(objType, objMap, identifier):
                    return remFn(identifier[0], sendToLimbo=sendToLimbo)
        return None
    def getAABB(self, identifier):
        if (not self._isIdentifier(identifier)) or (identifier[0] not in self._kinematicTrees):
            return None
        cachedElement = self._getObjectPropertyCache.get((identifier, "aabb"), None)
        if (cachedElement is not None) and (self._cacheTime <= cachedElement[0]):
            return cachedElement[1]
        if (1 < len(identifier)):
            linkId = self._kinematicTrees[identifier[0]]['links'][identifier[1]]['idx']
            retq = stubbornTry(lambda : pybullet.getAABB(self._kinematicTrees[identifier[0]]['idx'], linkId, self._pybulletConnection))
            self._getObjectPropertyCache[(identifier, "aabb")] = [self._cacheTime, retq]
            return retq
        aabbMin = [None, None, None]
        aabbMax = [None, None, None]
        for l, ld in self._kinematicTrees[identifier[0]]['links'].items():
            lidx = ld['idx']
            cachedElement = self._getObjectPropertyCache.get(((identifier[0],l), "aabb"), None)
            if (cachedElement is not None) and (self._cacheTime <= cachedElement[0]):
                cmin,cmax = cachedElement[1]
            else:
                cmin, cmax = stubbornTry(lambda : pybullet.getAABB(self._kinematicTrees[identifier[0]]['idx'], lidx, self._pybulletConnection))
                self._getObjectPropertyCache[((identifier[0],l), "aabb")] = [self._cacheTime, [cmin,cmax]]
            for k, e in enumerate(cmin):
                if (aabbMin[k] is None) or (aabbMin[k] > e):
                    aabbMin[k] = e
            for k, e in enumerate(cmax):
                if (aabbMax[k] is None) or (aabbMax[k] < e):
                    aabbMax[k] = e
        return aabbMin, aabbMax
    def at(self, identifier):
        retq = self.atComponent(identifier)
        if retq is None:
            return None
        return retq[0]
    # Sadly, the closest way to "correctly" identify insideness is the expensive closestPoints query.
    # Because at and atComponent are used a lot we cannot afford it, so must approximate.
    # Suggestion:
    #     inside container if contacting container
    #         if contacting more containers, choose the one with contact normal closest to vertical
    #     else check whether some neighbor is inside a container and choose that one
    #     if all else fails, check overlaps with containers
    def _atComponentInternal(self, identifier, ignore=None):
        cachedElement = self._getObjectPropertyCache.get((identifier, "atComponent"), None)
        if (cachedElement is not None) and (self._cacheTime <= cachedElement[0]):
            return cachedElement[1], []
        contacts = self.checkCollision(identifier)
        #containers = [x for x in contacts if (0.8 < x[4][2]) and self.getObjectProperty((x[1][0],), ("fn", "canContain"), False)]
        containers = sorted([(-x[4][2], x) for x in contacts if self._kinematicTrees[x[1][0]].get("fn", {}).get("canContain", False)])
        picked = None
        aabb = self.getAABB(identifier)
        for e in containers:
            c = e[1][1]
            aabbC = self.getAABB(c)
            if (aabbC[0][0]-0.02 < aabb[0][0]) and (aabbC[0][1]-0.02 < aabb[0][1]) and (aabbC[1][0]+0.02 > aabb[1][0]) and (aabbC[1][1]+0.02 > aabb[1][1]):
                picked = c
                self._getObjectPropertyCache[(identifier, "atComponent")] = [self._cacheTime, picked]
                break
        if picked is not None:
            return picked, []
        if ignore is None:
            ignore = set([])
        candidates = set([(x[1][0],) for x in contacts]).difference(ignore)
        return None, candidates                
    def atComponent(self, identifier):
        cachedElement = self._getObjectPropertyCache.get((identifier, "atComponent"), None)
        if (cachedElement is not None) and (self._cacheTime <= cachedElement[0]):
            return cachedElement[1]
        if identifier[0] not in self._kinematicTrees:
            return None
        ignore = set()
        aabb = self.getAABB(identifier)
        todo = [identifier]
        retq = None
        while todo:
            cr = todo.pop()
            ignore.add(cr)
            retq, candidates = self._atComponentInternal(cr, ignore=ignore)
            if (retq is not None) and (retq[0] != identifier[0]):
                aabbC = self.getAABB(retq)
                if (aabbC[0][0]-0.02 < aabb[0][0]) and (aabbC[0][1]-0.02 < aabb[0][1]) and (aabbC[1][0]+0.02 > aabb[1][0]) and (aabbC[1][1]+0.02 > aabb[1][1]):
                    break
            else:
                retq = None
            todo += candidates
        if retq is None:
            aabbAdj = self.adjustAABBRadius(aabb, 0.02)
            overlaps = [x for x in self.checkOverlap(aabb) if (x[0] != identifier[0]) and self._kinematicTrees[x[0]].get("fn", {}).get("canContain", False) and (x[1] in self._kinematicTrees[x[0]].get("fn", {}).get("links", []))]
            for c in overlaps:
                aabbC = self.getAABB(c)
                if (aabbC[0][0]-0.02 < aabb[0][0]) and (aabbC[0][1]-0.02 < aabb[0][1]) and (aabbC[1][0]+0.02 > aabb[1][0]) and (aabbC[1][1]+0.02 > aabb[1][1]):
                    retq = c
                    break
        self._getObjectPropertyCache[(identifier, "atComponent")] = [self._cacheTime, retq]
        if (retq is not None) and (retq[0] == identifier[0]):
            sys.exit(0)
        return retq
        #aabbMin, aabbMax = self.getAABB(identifier)
        #if aabbMin is None:
        #    return None
        ##center = [(a+b)*0.5 for a,b in zip(aabbMin,aabbMax)]
        #aabbMin = [aabbMin[0], aabbMin[1], aabbMin[2] - 0.03]
        #aabbMax = [aabbMax[0], aabbMax[1], aabbMin[2] + 0.03] # Yep, min -- wouldn't want to trigger overlaps with contained containers.
        #closeObjects = self.checkOverlap((aabbMin, aabbMax))
        #retq = None
        #minD = None
        #for o,c in closeObjects:
        #    if (o == identifier[0]) or (not self.getObjectProperty((o,), ("fn", "canContain"), False)) or (c not in self.getObjectProperty((o,), ("fn", "containment", "links"), [])):
        #        continue
        #    ds = [x[-1] for x in self.checkClosestPoints((identifier), (o,c), maxDistance=0.1)]
        #    if 0 < len(ds):
        #        d = min(ds)
        #        if (None==minD) or (d < minD):
        #            minD = d
        #            retq = [o,c]
        #return retq
    def getJointData(self, identifier):
        objData = self._kinematicTrees[identifier[0]]
        if (1 < len(identifier)) and (identifier[1] in objData['links']) and (-1 != objData['links'][identifier[1]]):
            return stubbornTry(lambda : pybullet.getJointState(objData['idx'], objData['links'][identifier[1]]['idx']))
        return None, None, None, None
    def getKinematicData(self, identifier):
        objData = self._kinematicTrees[identifier[0]]
        linkId = -1
        if 1 < len(identifier):
            linkId = objData['links'].get(identifier[1], {}).get('idx', None)
            if linkId is None:
                return None
        if 0 <= linkId:
            _, _, _, _, position, orientation, linearVelocity, angularVelocity = stubbornTry(lambda : pybullet.getLinkState(objData['idx'], linkId, 1, False, self._pybulletConnection))
        else:
            position, orientation = stubbornTry(lambda : pybullet.getBasePositionAndOrientation(objData['idx'], self._pybulletConnection))
            linearVelocity, angularVelocity = stubbornTry(lambda : pybullet.getBaseVelocity(objData['idx'], self._pybulletConnection))
        return position, orientation, linearVelocity, angularVelocity
    def getJointStates(self, identifier):
        objData = self._kinematicTrees[identifier[0]]
        jointNames = list(objData["joints"].keys())
        aux = stubbornTry(lambda : pybullet.getJointStates(objData['idx'], [objData['joints'][j]['idx'] for j in jointNames]))
        if aux is None:
            aux = []
        return {k: v[0] for k, v in zip(jointNames, aux)},  {k: v[1] for k, v in zip(jointNames, aux)}, {k: v[2] for k, v in zip(jointNames, aux)}, {k: v[3] for k, v in zip(jointNames, aux)}
    def getObjectProperty(self, identifier, propertyIdentifier, defaultValue=None):
        if not self._isIdentifier(identifier):
            return None
        if not isinstance(identifier, tuple):
            identifier = tuple(identifier)
        if isinstance(propertyIdentifier, list):
            propertyIdentifier = tuple(propertyIdentifier)
        s1 = time.perf_counter()
        cachedElement = self._getObjectPropertyCache.get((identifier, propertyIdentifier), None)
        e1 = time.perf_counter()
        if 'atComponent' == propertyIdentifier:
            if 'atComponentCached' not in self._detProfGet:
                self._detProfGet['atComponentCached'] = [0,0]
            self._detProfGet['atComponentCached'][0] += (e1-s1)
        if (cachedElement is not None) and (self._cacheTime <= cachedElement[0]):
            if 'atComponent' == propertyIdentifier:
                self._detProfGet['atComponentCached'][1] += 1
            return cachedElement[1]
        s = time.perf_counter()
        retq = self._getObjectPropertyInternal(identifier, propertyIdentifier, defaultValue=defaultValue)
        self._getObjectPropertyCache[(identifier, propertyIdentifier)] = [self._cacheTime, retq]
        e = time.perf_counter()
        if 'atComponent' == propertyIdentifier:
            if 'atComponentUncached' not in self._detProfGet:
                self._detProfGet['atComponentUncached'] = [0, 0, []]
            self._detProfGet['atComponentUncached'][0] += (e-s)
            self._detProfGet['atComponentUncached'][1] += 1
            self._detProfGet['atComponentUncached'][2].append(identifier)
        return retq
    def _getObjectPropertyInternal(self, identifier, propertyIdentifier, defaultValue=None):
        if (isinstance(propertyIdentifier, tuple) or isinstance(propertyIdentifier, list)) and (1 == len(propertyIdentifier)):
            propertyIdentifier = propertyIdentifier[0]
        objData = None
        if identifier[0] in self._markers:
            objData = self._markers[identifier[0]]
        elif identifier[0] in self._kinematicConstraints:
            objData = self._kinematicConstraints[identifier[0]]
        elif identifier[0] in self._kinematicTrees:
            objData = self._kinematicTrees[identifier[0]]
        if objData is None:
            return None
        if 'kcon' == objData['simtype']:
            if 'force' == propertyIdentifier:
                return stubbornTry(lambda : pybullet.getConstraintState(objData['idx'], self._pybulletConnection))
            elif propertyIdentifier in ['idx']:
                return None
        elif objData['simtype'] in ['ktree', 'marker']:
            if propertyIdentifier in ["position", "orientation", "linearVelocity", "angularVelocity"]:
                position, orientation, linearVelocity, angularVelocity = self.getKinematicData(identifier)
                self._getObjectPropertyCache[(identifier, "position")] = [self._cacheTime, position]
                self._getObjectPropertyCache[(identifier, "orientation")] = [self._cacheTime, orientation]
                self._getObjectPropertyCache[(identifier, "linearVelocity")] = [self._cacheTime, linearVelocity]
                self._getObjectPropertyCache[(identifier, "angularVelocity")] = [self._cacheTime, angularVelocity]
                return {"position": position, "orientation": orientation, "linearVelocity": linearVelocity, "angularVelocity": angularVelocity}[propertyIdentifier]
            elif 'parent' == propertyIdentifier:
                if (1 == len(identifier)) or (identifier[1] not in objData['links']) or (-1 == objData['links'][identifier[1]]['idx']):
                    return None
                return objData['idx2Joint'][objData['links'][identifier[1]]['idx']]
            elif propertyIdentifier in ['lowerLimit', 'upperLimit']:
                if (1 == len(identifier)) or (identifier[1] not in objData['links']) or (-1 == objData['links'][identifier[1]]['idx']):
                    return None
                _, _, _, _, _, _, _, _, lower, upper, _, _, _, _, _, _, _= stubbornTry(lambda : pybullet.getJointInfo(objData['idx'], objData['links'][identifier[1]]['idx'], self._pybulletConnection))
                return {'lowerLimit': lower, 'upperLimit': upper}[propertyIdentifier]
            elif propertyIdentifier in ['frictionAnchor', 'linearDamping', 'angularDamping']:
                if 1==len(identifier):
                    lname = objData['idx2Link'][-1]
                else:
                    lname = identifier[1]
                if lname not in objData['links']:
                    return None
                if ('fn' not in objData) or (propertyIdentifier not in objData['fn']) or (lname not in objData['fn'][propertyIdentifier]):
                    return defaultValue
                return objData['fn'][propertyIdentifier][lname]
            elif propertyIdentifier in ['mass', 'lateralFriction', 'localInertiaDiagonal', 'localInertiaPosition', 'localInertiaOrientation', 'restitution', 'rollingFriction', 'spinningFriction', 'contactDamping', 'contactStiffness']:
                if 1 == len(identifier):
                    if 'mass' == propertyIdentifier:
                        retq = 0.0
                        for l in objData['links']:
                            retq = retq + stubbornTry(lambda : pybullet.getDynamicsInfo(objData['idx'], objData['links'][l]['idx'], self._pybulletConnection))[0]
                        return retq
                    else:
                        lname = objData['idx2Link'][-1]
                else:
                    if identifier[1] not in objData['links']:
                        return None
                    else:
                        lname = identifier[1]
                if ('fn' in objData) and (propertyIdentifier in objData['fn']) and (lname in objData['fn'][propertyIdentifier]):
                    return objData['fn'][propertyIdentifier][lname]
                mass, lateralFriction, localInertiaDiagonal, localInertiaPosition, localInertiaOrientation, restitution, rollingFriction, spinningFriction, contactDamping, contactStiffness, bodyType, _ = stubbornTry(lambda : pybullet.getDynamicsInfo(objData['idx'], objData['links'][lname]['idx'], self._pybulletConnection))
                retq = {'mass': mass, 'lateralFriction': lateralFriction, 'localInertiaDiagonal': localInertiaDiagonal, 'localInertiaPosition': localInertiaPosition, 'localInertiaOrientation': localInertiaOrientation, 'restitution': restitution, 'rollingFriction': rollingFriction, 'spinningFriction': spinningFriction, 'contactDamping': contactDamping, 'contactStiffness': contactStiffness}
                return retq[propertyIdentifier]
            elif 'baseLinkName' == propertyIdentifier:
                return objData['idx2Link'][-1]
            elif 'at' == propertyIdentifier:
                return self.at(identifier)
            elif 'atComponent' == propertyIdentifier:
                return self.atComponent(identifier)
            elif 'parentTo' == propertyIdentifier:
                return objData['parentTo']
            elif 'childOf' == propertyIdentifier:
                return objData['childOf']
            elif 'aabb' == propertyIdentifier:
                return self.getAABB(identifier)
            elif ('localAABB' == propertyIdentifier) or ('localAABB' == propertyIdentifier):
                link = 0
                if 1 < len(identifier):
                    link = identifier[1]
                elif (not isinstance(propertyIdentifier, str)) and (1 < len(propertyIdentifier)):
                    link = propertyIdentifier[1]
                return objData['localAABB'].get(link, None)
            elif propertyIdentifier in ['jointPosition', 'jointVelocity', 'jointReactionForce', 'jointAppliedTorque']:
                if (1 < len(identifier)) and (identifier[1] in objData['links']) and (-1 != objData['links'][identifier[1]]):
                    pos, vel, ref, tor = stubbornTry(lambda : pybullet.getJointState(objData['idx'], objData['links'][identifier[1]]['idx']))
                    return {'jointPosition': pos, 'jointVelocity': vel, 'jointReactionForce': ref, 'jointAppliedTorque': tor}[propertyIdentifier]
                return None
            elif propertyIdentifier in ['jointPositions', 'jointVelocities', 'jointReactionForces', 'jointAppliedTorques']:
                if 1 == len(identifier):
                    jointPs, jointVels, jointForces, jointTorques = self.getJointStates(identifier)
                    self._getObjectPropertyCache[(identifier, 'jointPositions')] = jointPs
                    self._getObjectPropertyCache[(identifier, 'jointVelocities')] = jointVels
                    self._getObjectPropertyCache[(identifier, 'jointReactionForces')] = jointForces
                    self._getObjectPropertyCache[(identifier, 'jointAppliedTorques')] = jointTorques
                    return {'jointPositions': jointPs, 'jointVelocities': jointVels, 'jointReactionForces': jointForces, 'jointAppliedTorques': jointTorques}[propertyIdentifier]

                return None
            elif 'links' == propertyIdentifier:
                return list(self._kinematicTrees[identifier[0]]['links'].keys())
            elif 'joints' == propertyIdentifier:
                return list(self._kinematicTrees[identifier[0]]['joints'].keys())
            elif propertyIdentifier in ['idx', 'idx2Joint', 'idx2Link']:
                return None
        if isinstance(propertyIdentifier, str):
            if 1 < len(identifier):
                return objData.get('links', {}).get(identifier[1], {}).get(propertyIdentifier, defaultValue)
            return objData.get(propertyIdentifier, defaultValue)
        else:
            cr = objData
            if 1 < len(identifier):
                if propertyIdentifier not in ['customStateVariables', 'fn']:
                    return None
                propertyIdentifier = [propertyIdentifier[0], 'links', identifier[1]] + propertyIdentifier[1:]
            for e in propertyIdentifier:
                if not isinstance(cr, dict):
                    return defaultValue
                cr = cr.get(e, None)
                if cr is None:
                    return defaultValue
            return cr
    def setObjectProperty(self, identifier, propertyIdentifier, newValue):
        if not self._isIdentifier(identifier):
            return None
        if (isinstance(propertyIdentifier, tuple) or isinstance(propertyIdentifier, list)) and (1 == len(propertyIdentifier)):
            propertyIdentifier = propertyIdentifier[0]
        if not isinstance(identifier, tuple):
            identifier = tuple(identifier)
        if isinstance(propertyIdentifier, list):
            propertyIdentifier = tuple(propertyIdentifier)
        self._getObjectPropertyCache[(identifier, propertyIdentifier)] = None
        objData = None
        if identifier[0] in self._markers:
            objData = self._markers[identifier[0]]
        elif identifier[0] in self._kinematicConstraints:
            objData = self._kinematicConstraints[identifier[0]]
        elif identifier[0] in self._kinematicTrees:
            objData = self._kinematicTrees[identifier[0]]
        if objData is None:
            return None
        if 'kcon' == objData['simtype']:
            if propertyIdentifier in ['force', 'idx']:
                return None
            elif (propertyIdentifier in ['maxForce', 'childFramePosition', 'childFrameOrientation']):
                kwargs = {'physicsClientId': self._pybulletConnection}
                kwargs[{'maxForce': 'maxForce', 'childFramePosition': 'jointChildPivot', 'childFrameOrientation': 'jointChildFrameOrientation'}[propertyIdentifier]] = newValue
                stubbornTry(lambda : pybullet.changeConstraint(objData['idx'], **kwargs))
                objData[propertyIdentifier] = newValue
                return True
            elif (propertyIdentifier in ['parent', 'child', 'parentLink', 'childLink', 'jointType', 'jointAxis']):
                if (propertyIdentifier in ['child', 'parent']) and (newValue not in self._kinematicTrees):
                    return None
                if ('parentLink' == propertyIdentifier) and (newValue not in self._kinematicTrees[objData['parent']]['links']):
                    return None
                if ('childLink' == propertyIdentifier) and (newValue not in self._kinematicTrees[objData['child']]['links']):
                    return None
                if ('jointType' == propertyIdentifier) and (newValue not in ['fixed', 'prismatic', 'revolute']):
                    return None
                objData[propertyIdentifier] = newValue
                stubbornTry(lambda : pybullet.removeConstraint(objData['idx'], self._pybulletConnection))
                self._addKinematicConstraintInternal(copy.deepcopy(objData))
                self._setCustomUpdaters('kcon', objData['name'])
        elif objData['simtype'] in ['ktree', 'marker']:
            if propertyIdentifier in ['name', 'simtype', 'idx', 'type', 'links', 'joints', 'idx2Joint', 'idx2Link', 'baseLinkName', 'dofList', 'aabb', 'localAABB', 'at', 'atComponent', 'parentTo', 'childOf', 'jointReactionForces', 'jointReactionForce', 'jointAppliedTorques', 'jointAppliedTorque', 'localInertiaPosition', 'localInertiaOrientation', 'upperLimit', 'lowerLimit', 'jointPositions', 'jointVelocities', 'immobile']:
                return None
            elif ('filename' == propertyIdentifier):
                return (newValue == objData['filename']) or self.reloadObject(objData['name'], objData['type'], newValue)
            elif propertyIdentifier in ['position', 'orientation', 'linearVelocity', 'angularVelocity']:
                lidx, lname = _getLIdx(identifier, objData)
                if -1 == lidx:
                    if propertyIdentifier in ['position', 'orientation']:
                        pos, orn = pybullet.getBasePositionAndOrientation(objData['idx'], self._pybulletConnection)
                        if 'position' == propertyIdentifier:
                            pos = newValue
                        elif 'orientation' == propertyIdentifier:
                            orn = newValue
                        self._computedCollisions = False
                        stubbornTry(lambda : pybullet.resetBasePositionAndOrientation(objData['idx'], pos, orn, self._pybulletConnection))
                    if propertyIdentifier in ['linearVelocity', 'angularVelocity']:
                        stubbornTry(lambda : pybullet.resetBaseVelocity(objData['idx'], **{propertyIdentifier: newValue, 'physicsClientId': self._pybulletConnection}))
                else:
                    return None
            elif 'jointPosition' == propertyIdentifier:
                if (1 < len(identifier)) and (identifier[1] in objData['links']):
                    jidx = objData['links'][identifier[1]]['idx']
                    stubbornTry(lambda : pybullet.resetJointState(objData['idx'], jidx, newValue, physicsClientId=self._pybulletConnection))
            elif 'jointVelocity' == propertyIdentifier:
                if (1 < len(identifier)) and (identifier[1] in objData['links']):
                    jidx = objData['links'][identifier[1]]['idx']
                    pos = stubbornTry(lambda : pybullet.getJointState(objData['idx'], jidx, self._pybulletConnection))[0]
                    stubbornTry(lambda : pybullet.resetJointState(objData['idx'], jidx, pos, targetVelocity=newValue, physicsClientId=self._pybulletConnection))
            elif 'mass' == propertyIdentifier:
                lidx, lname = _getLIdx(identifier, objData)
                if lidx is not None:
                    # Adjusting mass automatically adjusts the localInertiaDiagonal on pybullet's side: OK
                    stubbornTry(lambda : pybullet.changeDynamics(objData['idx'], lidx, mass=newValue, physicsClientId=self._pybulletConnection))
                    if 0 < newValue:
                        _setDictionaryEntry(objData, ('fn', '_previousNonzeroMass', lname), newValue)
            elif 'localInertiaDiagonal' == propertyIdentifier:
                lidx, lname = _getLIdx(identifier, objData)
                if lidx is not None:
                    # Adjusting localInertiaDiagonal does NOT change mass on pybullet's side: We fix that here.
                    ans = stubbornTry(lambda : pybullet.getDynamicsInfo(objData['idx'], lidx, self._pybulletConnection))
                    mass = ans[0]
                    localInertiaDiagonal = ans[2]
                    normSqOld = sum([x*x for x in localInertiaDiagonal])
                    normSqNew = sum([x*x for x in newValue])
                    if 0.000001 < normSqOld:
                        massFactor = math.sqrt(normSqNew/normSqOld)
                        mass = mass*massFactor
                        stubbornTry(lambda : pybullet.changeDynamics(objData['idx'], lidx, mass=mass, physicsClientId=self._pybulletConnection))
                        # If mass would be set to 0, then don't also set the diagonal. Pybullet seems to do the right thing here and set the diagonal to 0 when mass is 0, but restore the correct
                        # diagonal if the mass is made non-zero later.
                        if 0.000001 < mass:
                            stubbornTry(lambda : pybullet.changeDynamics(objData['idx'], lidx, localInertiaDiagonal=newValue, physicsClientId=self._pybulletConnection))
                    elif (0.000001 >= normSqNew):
                        # new diagonal is 0, and so is the new one: do nothing because nothing has changed.
                        return
                    else:
                        # previous mass and localInertiaDiagonal were 0, but new localInertiaDiagonal is not. Just give a default value for mass in this case.
                        mass = 1.0
                        if ('fn' in objData) and ('_previousNonzeroMass' in objData['fn']) and (lname in objData['fn']['_previousNonzeroMass']):
                            mass = objData['fn']['_previousNonzeroMass'][lname]
                        stubbornTry(lambda : pybullet.changeDynamics(objData['idx'], lidx, mass=mass, physicsClientId=self._pybulletConnection))
                        stubbornTry(lambda : pybullet.changeDynamics(objData['idx'], lidx, localInertiaDiagonal=newValue, physicsClientId=self._pybulletConnection))
            elif propertyIdentifier in ['lateralFriction', 'restitution', 'rollingFriction', 'spinningFriction', 'contactDamping', 'contactStiffness', 'linearDamping', 'angularDamping']:
                lidx, lname = _getLIdx(identifier, objData)
                if lidx is not None:
                    _setDictionaryEntry(objData, ('fn', propertyIdentifier, lname), newValue)
                    stubbornTry(lambda : pybullet.changeDynamics(objData['idx'], lidx, **{propertyIdentifier: newValue, 'physicsClientId': self._pybulletConnection}))
            elif 'frictionAnchor' == propertyIdentifier:
                lidx, lname = _getLIdx(identifier, objData)
                if lidx is not None:
                    _setDictionaryEntry(objData, ('fn', 'frictionAnchor', lname), newValue)
                    if newValue:
                        newValue = 1
                    else:
                        newValue = 0
                    stubbornTry(lambda : pybullet.changeDynamics(objData['idx'], lidx, frictionAnchor=newValue, physicsClientId=self._pybulletConnection))
            elif isinstance(propertyIdentifier, str):
                if (1 < len(identifier)):
                    return None
                else:
                    objData[propertyIdentifier] = newValue
                if 'fn' == propertyIdentifier:
                    self._setCustomUpdaters(objData['simtype'], objData['name'])
            else:
                cr = objData
                if 1 < len(identifier):
                    if propertyIdentifier[0] not in ['customStateVariables', 'fn']:
                        return None
                    propertyIdentifier = tuple(list(propertyIdentifier) + [identifier[1]])
                if (1 < len(propertyIdentifier)) and ('maxForce' == propertyIdentifier[1]):
                    if (2 > len(identifier)) or (identifier[1] not in objData['links']):
                        return None
                    else:
                        stubbornTry(lambda : pybullet.setJointMotorControl2(objData['idx'], objData['links'][identifier[1]]['idx'], pybullet.VELOCITY_CONTROL, force=newValue, physicsClientId=self._pybulletConnection))
                _setDictionaryEntry(objData, propertyIdentifier, newValue)
                if 'fn' == propertyIdentifier[0]:
                    self._setCustomUpdaters(objData['simtype'], objData['name'])
        return None
    def worldDump(self):
        retq = {}
        aggregates = {}
        for name in list(self._kinematicConstraints.keys()) + list(self._markers.keys()):
            retq[name] = self._describeObject(name)
        for name in self._kinematicTrees.keys():
            objData = self._describeObject(name)
            if objData.get('fn', {}).get('aggregates', False):
                pType = objData.get('type', 'particle')
                at = objData.get('atComponent')
                if at is None:
                    atStr = '_null'
                else:
                    atStr = '%s_%s' % (at[0], at[1])
                aggName = '%s@%s' % (pType, atStr)
                if aggName not in retq:
                    retq[aggName] = {'name': aggName, 'simtype': 'aggregate', 'type': pType, 'at': objData.get('at'), 'atComponent': objData.get('atComponent'), 'particles': []}
                retq[aggName]['particles'].append(objData)
            else:
                retq[name] = objData
        return {name: self._describeObject(name) for name in list(self._kinematicTrees.keys()) + list(self._kinematicConstraints.keys()) + list(self._markers.keys())}
    def greatReset(self, worldState):
        self._computedCollisions = False
        for obj in list(self._markers.keys()) + list(self._kinematicConstraints.keys()) + list(self._kinematicTrees.keys()):
            self.removeObject((obj,))
        newKinematicTrees = []
        newKinematicConstraints = []
        newMarkers = []
        mapFn = {'ktree': (lambda x: newKinematicTrees.append(x)), 'kcon': (lambda x: newKinematicConstraints.append(x)), 'marker': (lambda x: newMarkers.append(x))}
        for description in worldState.values():
            if 'simtype' in description:
                mapFn[description['simtype']](description)
        for description in newKinematicTrees + newKinematicConstraints + newMarkers:
            self.addObject(description)
        self._cacheTime += 1
        stubbornTry(lambda : pybullet.performCollisionDetection(self._pybulletConnection))
        return
    def getCameraImage(self, cameraPosition=None, cameraTarget=None, cameraUpAxis=None, fovInDegrees=60.0, aspectRatio=1.0, near=0.1, far=15, xSize=240, ySize=240):
        if cameraPosition is None:
            cameraPosition = (0, 0, 0)
        if cameraTarget is None:
            cameraTarget = (1, 0, 0)
        if cameraUpAxis is None:
            cameraUpAxis = (0, 0, 1)
        projectionMatrix = p.computeProjectionMatrixFOV(fovInDegrees, aspectRatio, near, far)
        viewMatrix = stubbornTry(lambda : pybullet.computeViewMatrix(cameraPosition, cameraTarget, cameraUpVector))
        ans = stubbornTry(lambda : pybullet.getCameraImage(xSize, ySize, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix))
        rgba = ans[2]
        depth = ans[3]
        # TODO: return segmentation image, but then will need also a map of int id to identifier
        return rgba, depth
    def applyJointControl(self, identifier, mode=None, targetPosition=None, targetVelocity=None, force=None, positionGain=None, velocityGain=None, maxVelocity=None):
        if (not self._isIdentifier(identifier)) or (identifier[0] not in self._kinematicTrees):
            return None
        jidx, lname = _getJIdx(identifier, self._kinematicTrees[identifier[0]])
        if mode is None:
            mode = 'velocity'
        mode = {'velocity': pybullet.VELOCITY_CONTROL, 'position': pybullet.POSITION_CONTROL, 'torque': pybullet.TORQUE_CONTROL}.get(mode, pybullet.VELOCITY_CONTROL)
        if (jidx is not None):
            kwargs = {'physicsClientId': self._pybulletConnection}
            if force is not None:
                kwargs['force'] = force
            if pybullet.VELOCITY_CONTROL == mode:
                if targetVelocity is None:
                    targetVelocity = 0
                kwargs['targetVelocity'] = targetVelocity
            elif pybullet.POSITION_CONTROL == mode:
                if targetVelocity is not None:
                    kwargs['targetVelocity'] = targetVelocity
                if targetPosition is None:
                    targetPosition = 0
                kwargs['targetPosition'] = targetPosition
                if positionGain is not None:
                    kwargs['positionGain'] = positionGain
                if velocityGain is not None:
                    kwargs['velocityGain'] = velocityGain
                if maxVelocity is not None:
                    kwargs['maxVelocity'] = maxVelocity
            elif pybullet.TORQUE_CONTROL == mode:
                if force is None:
                    kwargs['force'] = 0
            else:
                return
            stubbornTry(lambda : pybullet.setJointMotorControl2(self._kinematicTrees[identifier[0]]['idx'], jidx, mode, **kwargs))
    def applyExternalForce(self, identifier, force, position, inWorldFrame=False):
        idxs = self._kinematicTreeOrLinkIdentifier2Idx(identifier)
        if idxs is None:
            return None
        if 1 == len(idxs):
            idxs = list(idxs) + [-1]
        frame = pybullet.LINK_FRAME
        if inWorldFrame:
            frame = pybullet.WORLD_FRAME
        stubbornTry(lambda : pybullet.applyExternalForce(idxs[0], idxs[1], force, position, frame, self._pybulletConnection))
        return True
    def applyExternalTorque(self, identifier, torque, inWorldFrame=False):
        idxs = self._kinematicTreeOrLinkIdentifier2Idx(identifier)
        if idxs is None:
            return None
        if 1 == len(idxs):
            idxs = list(idxs) + [-1]
        frame = pybullet.LINK_FRAME
        if inWorldFrame:
            frame = pybullet.WORLD_FRAME
        stubbornTry(lambda : pybullet.applyExternalTorque(idxs[0], idxs[1], torque, frame, self._pybulletConnection))
        return True
    def PgetObjectProperty(self,x,y,defaultValue=None):
        s = time.perf_counter()
        retq = self.getObjectProperty(x,y,defaultValue=defaultValue)
        e = time.perf_counter()
        self._profileGET += (e-s)
        return retq
    def PsetObjectProperty(self, x, y, z):
        s = time.perf_counter()
        retq = self.setObjectProperty(x, y, z)
        e = time.perf_counter()
        self._profileSET += (e-s)
        return retq
    def update(self, customDynamics=None):
        #print("FRAME")
        self._lastProfile = {"stepSimulation": 0, "customDynamics": {}}
        self._detProfGet = {}
        s = time.perf_counter()
        self._cacheTime += 1
        self._profile = 0
        self._profileCLS = 0
        self._profileCON = 0
        self._profileOVR = 0
        self._profileGET = 0
        self._profileSET = 0
        sR = time.perf_counter()
        if 0 < len(self._limbo):
            c = self._limbo.pop()
            stubbornTry(lambda : pybullet.removeBody(c['idx']))
        eR = time.perf_counter()
        # Only allow custom dynamics to write the properties or potentially remove the object they apply to.
        if customDynamics is None:
            customDynamics = []
        sD = time.perf_counter()
        for objType, objRecs in [('ktree', self._kinematicTrees), ('kcon', self._kinematicConstraints)]:
            for name in list(objRecs.keys()):
                if name in objRecs:
                    for updateFn in list(self._customDynamicsUpdaters[objType][name]):
                        if name not in objRecs:
                            break
                        sF = time.perf_counter()
                        updateFn(name, self._customDynamicsAPI[objType][name])
                        eF = time.perf_counter()
                        inc = 0
                        if updateFn == self._kincompDyn:
                            inc = (eF-sF)
                        self._lastProfile["customDynamics"][name] = self._lastProfile["customDynamics"].get(name,0) + inc
        eD = time.perf_counter()
        sS = time.perf_counter()
        stubbornTry(lambda : pybullet.stepSimulation(self._pybulletConnection))
        eS = time.perf_counter()
        self._lastProfile["stepSimulation"] = (None, (eS-sS) + (eR-sR))
        self._computedCollisions = True
        self._frameStepCount = 1
        e = time.perf_counter()
        #print("    Frame            %f\t(Sim %f\tDyn %f)" % (e-s, eS-sS, eD-sD))
        #print("    Geometry queries %f\t(CLS %f\tCON %f\tOVR %f)" % (self._profile, self._profileCLS, self._profileCON, self._profileOVR))
        #print("    GetOProp queries %f" % self._profileGET)
        #print("    SetOProp queries %f" % self._profileSET)
        #print("    ", [(k,self._detProfGet[k]) for k in self._detProfGet.keys()])
        # TODO: if enabled, update debug objects such as markers. Currently waiting for a newer pybullet where markers are easier to work with.
        return
    def distance(self, vectorEnd, vectorStart):
        d = [x-y for x,y in zip(vectorEnd, vectorStart)]
        return math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
    def orientationDifferenceAA(self, targetOrientation, startOrientation):
        return stubbornTry(lambda : pybullet.getAxisAngleFromQuaternion(pybullet.getDifferenceQuaternion(startOrientation, targetOrientation)))
    def objectPoseRelativeToObject(self, positionObjectRef, orientationObjectRef, positionObjectLoc, orientationObjectLoc):
        positionObjectRefInv, orientationObjectRefInv = stubbornTry(lambda: pybullet.invertTransform(positionObjectRef, orientationObjectRef))
        return stubbornTry(lambda: pybullet.multiplyTransforms(positionObjectRefInv, orientationObjectRefInv, positionObjectLoc, orientationObjectLoc))
    def objectPoseRelativeToWorld(self, positionObjectRef, orientationObjectRef, positionObjectLoc, orientationObjectLoc):
        return stubbornTry(lambda: pybullet.multiplyTransforms(positionObjectRef, orientationObjectRef, positionObjectLoc, orientationObjectLoc))
    def handPoseToBringObjectToPose(self, positionObjectInHand, orientationObjectInHand, targetPositionInWorld, targetOrientationInWorld):
        handPositionInObject, handOrientationInObject = stubbornTry(lambda: pybullet.invertTransform(positionObjectInHand, orientationObjectInHand))
        return self.objectPoseRelativeToWorld(targetPositionInWorld, targetOrientationInWorld, handPositionInObject, handOrientationInObject)
    def _reportFromIdx(self, bidx, lnkIdx):
        ktree = self._idx2KinematicTree[bidx]
        return (ktree, self._kinematicTrees[ktree]['idx2Link'][lnkIdx])
    def checkCollision(self, identifierA, identifierB=None):
        s = time.perf_counter()
        def _reportCollision(contactPoint):
            _, bidA, bidB, lnkA, lnkB, ponA, ponB, normal, distance, force, lateralFriction1, lateralFriction1Dir, lateralFriction2, lateralFrictionDir2 = contactPoint
            identifierA = self._reportFromIdx(bidA, lnkA)
            identifierB = self._reportFromIdx(bidB, lnkB)
            if (identifierA is None) or (identifierB is None):
                return None
            return identifierA, identifierB, ponA, ponB, normal, distance, force, lateralFriction1, lateralFriction1Dir, lateralFriction2, lateralFrictionDir2
        def _getQueries(identifierA, identifierB):
            if (not self._isKinematicTreeOrLinkIdentifier(identifierA)) or ((identifierB is not None) and (not self._isKinematicTreeOrLinkIdentifier(identifierB))):
                return []
            identifiersA = []
            if 1 == len(identifierA):
                for x in self._kinematicTrees[identifierA[0]]['links'].keys():
                    identifiersA.append((identifierA[0], x))
            else:
                identifiersA = [identifierA]
            identifiersB = []
            if identifierB is not None:
                if 1 == len(identifierB):
                    for x in self._kinematicTrees[identifierB[0]]['links'].keys():
                        identifiersB.append((identifierB[0], x))
                else:
                    identifiersB = [identifierB]
            queries = []
            for identifierA in identifiersA:
                bidA, lnkA = self._kinematicTrees[identifierA[0]]['idx'], self._kinematicTrees[identifierA[0]]['links'][identifierA[1]]['idx']
                if [] == identifiersB:
                    kwargs = {'physicsClientId': self._pybulletConnection, 'bodyA': bidA}
                    if lnkA is not None:
                        kwargs['linkIndexA'] = lnkA
                    queries.append(kwargs)
                else:
                    for identifierB in identifiersB:
                        bidB, lnkB = self._kinematicTrees[identifierB[0]]['idx'], self._kinematicTrees[identifierA[B]]['links'][identifierB[1]]['idx']
                        kwargs = {'physicsClientId': self._pybulletConnection, 'bodyA': bidA, 'bodyB': bidB}
                        if lnkA is not None:
                            kwargs['linkIndexA'] = lnkA
                        if lnkB is not None:
                            kwargs['linkIndexB'] = lnkB
                        queries.append(kwargs)
            return queries
        if not self._computedCollisions:
            stubbornTry(lambda : pybullet.performCollisionDetection(self._pybulletConnection))
            self._computedCollisions = True
        queries = _getQueries(identifierA, identifierB)
        answers = []
        for kwargs in queries:
            answers = answers + list(stubbornTry(lambda : pybullet.getContactPoints(**kwargs)))
        retq = [_reportCollision(x) for x in answers]
        retq = [x for x in retq if x is not None]
        e = time.perf_counter()
        self._profile += (e-s)
        self._profileCON += (e-s)
        return retq
    def checkOverlap(self, regionSpec):
        # Check which kinematic trees/links overlap a region. The region can be an AABB described by a tuple, or a dictionary containing mesh, position, and orientation keys where mesh is a string
        # representing a path to a urdf or mesh file, and position and orientation are appropriately-sized tuples
        def _reportOverlap(contactPoint):
            _, bidA, bidB, lnkA, lnkB, ponA, ponB, normal, distance, _ = contactPoint
            identifier = self._reportFromIdx(bidB, lnkB)
            if identifier is None:
                return None
            return identifier, ponB, ponA, normal, distance
        def _checkAABBOverlap(regionSpec):
            aabbMin, aabbMax = regionSpec
            overlaps = stubbornTry(lambda : pybullet.getOverlappingObjects(aabbMin, aabbMax, self._pybulletConnection))
            if overlaps is None:
                return []
            retq = []
            for idx in overlaps:
                name = self._idx2KinematicTree[idx[0]]
                retq.append((name, self._kinematicTrees[name]['idx2Link'][idx[1]]))
            return retq
        def _checkMeshOverlap(regionSpec):
            colShape = self._getCollisionShape(regionSpec['mesh'])
            if colShape is not None:
                mb = stubbornTry(lambda : pybullet.createMultiBody(baseCollisionShapeIndex=colShape, basePosition=regionSpec['position'], baseOrientation=regionSpec['orientation'], physicsClientId=self._pybulletConnection))
                if 0 > mb:
                    return []
                stubbornTry(lambda : pybullet.performCollisionDetection(self._pybulletConnection))
                contactPoints = stubbornTry(lambda : pybullet.getContactPoints(bodyA=mb, physicsClientId=self._pybulletConnection))
                stubbornTry(lambda : pybullet.removeBody(mb, self._pybulletConnection))
                stubbornTry(lambda : pybullet.performCollisionDetection(self._pybulletConnection))
                self._computedCollisions = True
                return [_reportOverlap(x) for x in contactPoints]
            return []
        s = time.perf_counter()
        if self._isRegionSpec(regionSpec):
            retq = _checkMeshOverlap(regionSpec)
        else:
            retq = _checkAABBOverlap(regionSpec)
        retq = [x for x in retq if x is not None]
        e = time.perf_counter()
        self._profile += (e-s)
        self._profileOVR += (e-s)
        return retq
    def getDistance(self, identifierA, identifierB, maxDistance):
        closestPoints = self.checkClosestPoints(identifierA, identifierB, maxDistance=maxDistance)
        if (closestPoints is None) or (0 == len(closestPoints)):
            return 10*maxDistance
        return min([x[-1] for x in closestPoints])
    def probeClosestPoints(self, identifier, position, maxDistance=None):
        s = time.perf_counter()
        def _reportClosestPoint(closestPoint):
            _, _, bidB, _, lnkB, _, posB, normal, distance, _, _, _, _, _ = closestPoint
            obName = self._idx2KinematicTree[bidB]
            linkName = self._kinematicTrees[obName]['idx2Link'][lnkB]
            return (obName, linkName), posB, normal, distance
        if not self._isKinematicTreeOrLinkIdentifier(identifier):
            return []
        if maxDistance is None:
            maxDistance = self._worldSize
        objId = self._kinematicTrees[identifier[0]]['idx']
        kwargs = {'linkIndexA': -1, 'physicsClientId': self._pybulletConnection}
        if 1 < len(identifier):
            if identifier[1] not in self._kinematicTrees[identifier[0]]['links']:
                return []
            linkIndexB=self._kinematicTrees[identifier[0]]['links'][identifier[1]]['idx']
        stubbornTry(lambda : pybullet.resetBasePositionAndOrientation(self._colliderProbe, position, (0,0,0,1), self._pybulletConnection))
        answers = stubbornTry(lambda : pybullet.getClosestPoints(self._colliderProbe, objId, maxDistance, **kwargs))
        stubbornTry(lambda : pybullet.resetBasePositionAndOrientation(self._colliderProbe, (0,0,-110), (0,0,0,1), self._pybulletConnection))
        retq = [_reportClosestPoint(x) for x in answers]
        e = time.perf_counter()
        self._profile += (e-s)
        self._profileCLS += (e-s)
        return retq
    def checkClosestPoints(self, regionSpecOrIdentifierA, regionSpecOrIdentifierB, maxDistance=None):
        s = time.perf_counter()
        def _parseRegionSpecOrIdentifier(regionSpecOrIdentifier):
            if self._isRegionSpec(regionSpecOrIdentifier):
                colShape = self._getCollisionShape(regionSpecOrIdentifier['mesh'])
                if colShape is not None:
                    mb = stubbornTry(lambda : pybullet.createMultiBody(baseCollisionShapeIndex=colShape, basePosition=regionSpecOrIdentifier['position'], baseOrientation=regionSpecOrIdentifier['orientation'], physicsClientId=self._pybulletConnection))
                    return regionSpecOrIdentifier['mesh'], {-1: regionSpecOrIdentifier['mesh']}, mb, None, mb
            elif self._isIdentifier(regionSpecOrIdentifier):
                if regionSpecOrIdentifier[0] in self._kinematicTrees:
                    lnkIdx = None
                    linkMap = self._kinematicTrees[regionSpecOrIdentifier[0]]['idx2Link']
                    if 1 < len(regionSpecOrIdentifier):
                        lnkIdx = self._kinematicTrees[regionSpecOrIdentifier[0]]['links'][regionSpecOrIdentifier[1]]['idx']
                        return (regionSpecOrIdentifier[0],regionSpecOrIdentifier[1]), linkMap, self._kinematicTrees[regionSpecOrIdentifier[0]]['idx'], lnkIdx, None
                    return (regionSpecOrIdentifier[0],), linkMap, self._kinematicTrees[regionSpecOrIdentifier[0]]['idx'], lnkIdx, None
            return None, None, None, None, None
        def _getQueries(regionSpecOrIdentifierA, regionSpecOrIdentifierB, maxDistance):
            identifierA, linkMapA, bidA, lnkA, cleanupIdxA = _parseRegionSpecOrIdentifier(regionSpecOrIdentifierA)
            identifierB, linkMapB, bidB, lnkB, cleanupIdxB = _parseRegionSpecOrIdentifier(regionSpecOrIdentifierB)
            if lnkA is None:
                identifiersA = [(identifierA[0], x) for x in self._kinematicTrees[identifierA[0]]['links'].keys()]
            else:
                identifiersA = [identifierA]
            if lnkB is None:
                identifiersB = [(identifierB[0], x) for x in self._kinematicTrees[identifierB[0]]['links'].keys()]
            else:
                identifiersB = [identifierB]
            queries = []
            for identifierA in identifiersA:
                bidA, lnkA = self._kinematicTrees[identifierA[0]]['idx'], self._kinematicTrees[identifierA[0]]['links'][identifierA[1]]['idx']
                for identifierB in identifiersB:
                    bidB, lnkB = self._kinematicTrees[identifierB[0]]['idx'], self._kinematicTrees[identifierB[0]]['links'][identifierB[1]]['idx']
                    kwargs = {'physicsClientId': self._pybulletConnection}
                    for val, key in [(lnkA, 'linkIndexA'), (lnkB, 'linkIndexB')]:
                        if val is not None:
                            kwargs[key] = val
                    queries.append([[bidA, bidB, maxDistance], kwargs])
            return queries, cleanupIdxA, cleanupIdxB
        def _reportClosestPoint(closestPoint):
            _, bidA, bidB, lnkA, lnkB, posA, posB, normal, distance, _, _, _, _, _ = closestPoint
            identifierA = self._reportFromIdx(bidA, lnkA)
            identifierB = self._reportFromIdx(bidB, lnkB)
            if (identifierA is None) or (identifierB is None):
                return None
            return identifierA, identifierB, posA, posB, normal, distance
        if maxDistance is None:
            maxDistance = self._worldSize
        queries, cleanupIdxA, cleanupIdxB = _getQueries(regionSpecOrIdentifierA, regionSpecOrIdentifierB, maxDistance)
        answers = []
        for args, kwargs in queries:
            answers = answers + list(stubbornTry(lambda : pybullet.getClosestPoints(*args, **kwargs)))
        cleanup = False
        for x in [cleanupIdxA, cleanupIdxB]:
            if x is not None:
                stubbornTry(lambda : pybullet.removeBody(x, self._pybulletConnection))
                cleanup = True
        if cleanup:
            stubbornTry(lambda : pybullet.performCollisionDetection(self._pybulletConnection))
            self._computedCollisions = True
        retq = [_reportClosestPoint(x) for x in answers]
        retq = [x for x in retq if x is not None]
        e = time.perf_counter()
        self._profile += (e-s)
        self._profileCLS += (e-s)
        return retq
    def rayTestBatch(self, fromPositions, toPositions):
        def _reportRayResult(x):
            oid, lnk, hitFraction, hitNormal = x
            if -1 == oid:
                return None
            identifier = self._reportFromIdx(oid, lnk)
            if identifier is None:
                return None
            return (identifier, hitFraction, hitNormal)
        if len(fromPositions) != len(toPositions):
            return None
        retq = [_reportRayResult(x) for x in stubbornTry(lambda : pybullet.rayTestBatch(fromPositions, toPositions, self._pybulletConnection))]
        return [x for x in retq if x is not None]
    # computes two Jacobians such that xdot_t = Jt*q and xdot_r = Jr*q where xdot_t and xdot_r are linear and angular velocity in the link local frame 
    def computeJacobian(self, name, linkName, positionInLink, positions=None, velocities=None, accelerations=None):
        if (name not in self._kinematicTrees) or (linkName not in self._kinematicTrees[name]['links']):
            return None, None
        if positions is None:
            positions = self.getObjectProperty((name,), 'jointPositions')
        if velocities is None:
            velocities = self.getObjectProperty((name,), 'jointVelocities')
        if accelerations is None:
            accelerations = {k: 0.0 for k in velocities.keys()}
        objData = self._kinematicTrees[name]
        bodyId = objData['idx']
        linkId = objData['links'][linkName]['idx']
        objDOFs = objData['dofList']
        objPositions = _dofsForPybullet(objDOFs, positions)
        objVelocities = _dofsForPybullet(objDOFs, velocities)
        objAccelerations = _dofsForPybullet(objDOFs, accelerations)
        return stubbornTry(lambda : pybullet.calculateJacobian(bodyId, linkId, positionInLink, objPositions, objVelocities, objAccelerations, self._pybulletConnection))
    # computes a mass matrix that can be used to calculate kinetic energy from velocity in joint space
    def calculateMassMatrix(self, name, positions=None):
        if name not in self._kinematicTrees:
            return None, None
        if positions is None:
            positions = self.getObjectProperty((name,), 'jointPositions')
        objData = self._kinematicTrees[name]
        bodyId = objData['idx']
        objDOFs = objData['dofList']
        objPositions = _dofsForPybullet(objDOFs, positions)
        return objDOFs, stubbornTry(lambda : pybullet.calculateMassMatrix(bodyId, objPositions, self._pybulletConnection))
    def calculateInverseKinematics(self, name, linkName, targetPosition, targetOrientation=None):
        if (name not in self._kinematicTrees) or (linkName not in self._kinematicTrees[name]['links']):
            return None
        objData = self._kinematicTrees[name]
        bodyId = objData['idx']
        linkId = objData['links'][linkName]['idx']
        if targetOrientation is None:
            return stubbornTry(lambda : pybullet.calculateInverseKinematics(bodyId, linkId, targetPosition, physicsClientId=self._pybulletConnection))
        return stubbornTry(lambda : pybullet.calculateInverseKinematics(bodyId, linkId, targetPosition, targetOrientation=targetOrientation, physicsClientId=self._pybulletConnection))
    def calculateInverseDynamics(self, name, positions=None, velocities=None, accelerations=None):
        if name not in self._kinematicTrees:
            return None
        if positions is None:
            positions = self.getObjectProperty((name,), 'jointPositions')
        if velocities is None:
            velocities = self.getObjectProperty((name,), 'jointVelocities')
        if accelerations is None:
            accelerations = {k: 0.0 for k in velocities.keys()}
        objData = self._kinematicTrees[name]
        bodyId = objData['idx']
        objDOFs = objData['dofList']
        objPositions = _dofsForPybullet(objDOFs, positions)
        objVelocities = _dofsForPybullet(objDOFs, velocities)
        objAccelerations = _dofsForPybullet(objDOFs, accelerations)
        objForces = stubbornTry(lambda : pybullet.calculateInverseDynamics(bodyId, objPositions, objVelocities, objAccelerations, self._pybulletConnection))
        retq = {}
        k = 0
        for jointName, jointDOFs in objDOFs:
            if 1 == jointDOFs:
                retq[jointName] = objForces[k]
            else:
                retq[jointName] = objForces[k:k+jointDOFs]
            k += jointDOFs
        return retq
    def _isIdentifier(self, x):
        return (isinstance(x, list) or isinstance(x, tuple)) and (0 < len(x))
    def _isKinematicTreeOrLinkIdentifier(self, x):
        return self._isIdentifier(x) and (x[0] in self._kinematicTrees) and ((1 == len(x)) or (x[1] in self._kinematicTrees[x[0]]['links']))
    def _isRegionSpec(self, x):
        return isinstance(x, dict) and ('mesh' in x) and ('position' in x) and ('orientation' in x)
    def _kinematicTreeOrLinkIdentifier2Idx(self, identifier):
        if (identifier is None) or (not self._isIdentifier(identifier)) or (identifier[0] not in self._kinematicTrees):
            return None
        retq = [self._kinematicTrees[identifier[0]]['idx']]
        if (1 < len(identifier)) and (identifier[1] in self._kinematicTrees[identifier[0]]['links']):
            retq.append(self._kinematicTrees[identifier[0]]['links'][identifier[1]]['idx'])
        return retq
    def _getCollisionShape(self, path):
        if path not in self._collisionShapes:
            if not os.path.isfile(path):
                return None
        colShape = stubbornTry(lambda : pybullet.createCollisionShape(pybullet.GEOM_MESH, fileName=path, physicsClientId=self._pybulletConnection))
        if 0 > colShape:
            return None
        self._collisionShapes[path] = colShape
        return self._collisionShapes[path]
    def _describeObject(self, name):
        objData = None
        if name in self._markers:
            objData = self._markers[name]
        elif name in self._kinematicConstraints:
            objData = self._kinematicConstraints[name]
        elif name in self._kinematicTrees:
            objData = self._kinematicTrees[name]
        if objData is None:
            return None
        retq = {'name': name, 'simtype': objData['simtype'], 'type': objData.get('type', 'owl:Thing'), 'customStateVariables': copy.deepcopy(objData.get('customStateVariables', {})), 'fn': copy.deepcopy(objData.get('fn', {}))}
        if 'kcon' == retq['simtype']:
            retq['force'] = stubbornTry(lambda : pybullet.getConstraintState(objData['idx'], self._pybulletConnection))
            retq['parent'] = objData['parent']
            retq['child'] = objData['child']
            retq['parentLink'] = objData['parentLink']
            retq['childLink'] = objData['childLink']
            retq['fn'] = objData.get('fn', {})
            retq['customStateVariables'] = objData.get('customStateVariables', {})
        elif retq['simtype'] in ['ktree', 'marker']:
            retq['at'] = self.at((name,))
            retq['atComponent'] = self.atComponent((name,))
            retq['filename'] = objData.get('filename', '')
            pos, orn = stubbornTry(lambda : pybullet.getBasePositionAndOrientation(objData['idx'], self._pybulletConnection))
            lin, ang = stubbornTry(lambda : pybullet.getBaseVelocity(objData['idx'], self._pybulletConnection))
            retq['position'] = pos
            retq['orientation'] = orn
            retq['linearVelocity'] = lin
            retq['angularVelocity'] = ang
            retq['immobile'] = objData.get('immobile', False)
            if 'ktree' == retq['simtype']:
                #retq['links'] = {k: {} for k, v in objData['links'].items()}
                #retq['joints'] = {k: {} for k, v in objData['joints'].items()}
                retq['jointPositions'] = {}
                retq['jointVelocities'] = {}
                retq['jointReactionForces'] = {}
                retq['jointAppliedTorques'] = {}
                for j, jd in objData['joints'].items():
                    jidx = jd['idx']
                    pos, vel, force, torque = stubbornTry(lambda : pybullet.getJointState(objData['idx'], jidx, self._pybulletConnection))
                    retq['jointPositions'][j] = pos
                    retq['jointVelocities'][j] = vel
                    retq['jointReactionForces'][j] = force
                    retq['jointAppliedTorques'][j] = torque
        return retq                

# Convert a map of (joint names, values) to a list indexed implicitly by DOF, as pybullet uses for some functions such as inverse dynamics.
def _dofsForPybullet(objDOFs, dofData):
    retq = []
    for jointName, jointDOFs in objDOFs:
        if (jointName in dofData) and (((1 == jointDOFs) and (type(dofData[jointName]) in [int, float])) or (jointDOFs == len(dofData[jointName]))):
            if 1 == jointDOFs:
                retq.append(dofData[jointName])
            else:
                retq = retq + list(dofData[jointName])
        else:
            retq = retq + ([0.0]*jointDOFs)
    return retq
    
def _getLIdx(identifier, objData):
    if 1 == len(identifier):
        return -1, objData['idx2Link'][-1]
    if identifier[1] in objData['links']:
        return objData['links'][identifier[1]]['idx'], identifier[1]
    return None

def _getJIdx(identifier, objData):
    if 1 == len(identifier):
        return None
    if identifier[1] in objData['joints']:
        return objData['joints'][identifier[1]]['idx'], identifier[1]
    return None
def _setDictionaryEntry(objData, propertyIdentifier, newValue):
    cr = objData
    for p in propertyIdentifier[:-1]:
        if p not in cr:
            cr[p] = {}
        if not isinstance(cr[p], dict):
            return None
        cr = cr[p]
    cr[propertyIdentifier[-1]] = newValue

def getDictionaryEntry(objData, propertyIdentifier, defaultValue):
    cr = objData
    for e in propertyIdentifier:
        if not isinstance(cr, dict):
            return defaultValue
        cr = cr.get(e, None)
        if cr is None:
            return defaultValue
    return cr

# A hack to make this somewhat more stable on M1 MacOSX.
# Turns out, ANY call to the simulation engine (or perhaps just the OpenGL part?) has the potential to fail on the M1.
# However, just repeating the attempt seems to get the system back on track.
if ('Darwin' == platform.system()):
    def stubbornTry(fn):
        doing = True
        retq = None
        while doing:
            try:
                retq = fn()
                doing = False
            except:
                continue
        return retq
else:
    def stubbornTry(fn):
        return fn()

