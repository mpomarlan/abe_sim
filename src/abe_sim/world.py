# Source code for the procedures to maintain a simulated world of pobjects.
# These objects all have rigid body physics simulated by pybullet, and may have other physics attached with custom models (e.g. approximations of heating/cooling).

import pybullet as p
from abe_sim.utils import stubbornTry

def ensureDictionary(data):
    if data is None:
        return {}
    if isinstance(data, list):
        data = {x[0]: x[1:] for x in data}
    return data
    
def ensureHandsOk(data):
    if 'customStateVariables' in data:
        for e in ("hand_right_roll", "hand_left_roll"):
            if e not in data['customStateVariables']:
                continue
            for f in ("pushingclosed", "pullingopen", "uprighting", "pulling", "grasping"):
                if f in data['customStateVariables'][e][f] is None:
                    data['customStateVariables'][e][f] = []

class World():
    def __init__(self, pybulletOptions="", useGUI=True, name="pybulletWorld"):
        self._name = name
        self._pobjects = {}
        self._id2PObjects = {}
        self._ontoTypes = {}
        self._particleTypes = {}
        self._typeMap = {}
        self._debugVisualizationsEnabled = True ### TODO: infer from pybulletOptions
        self._debugObjects = []
        if useGUI:
            self._pybulletConnection = stubbornTry(lambda : p.connect(p.GUI, options=pybulletOptions))
        else:
            self._pybulletConnection = stubbornTry(lambda : p.connect(p.DIRECT, options=pybulletOptions))
    def getPObjectById(self, idx):
        if idx not in self._id2PObjects:
            return None
        return self._id2PObjects[idx]
    def getPObjectSetByIds(self, ids):
        return set([y for y in [self.getPObjectById(x) for x in ids] if None != y])
    def hasPObjectOfId(self, idx):
        return idx in self._id2PObjects
    def hasDbgObjectOfId(self, idx):
         return idx in [x._id for x in self._debugObjects]
    def getSimConnection(self):
        return self._pybulletConnection
    def _testPObjectPresence(self, name, command):
        if name not in self._pobjects:
            print("WARNING: attempting to %s pobject %s from world %s, when such pobject does not exist there." % (command, name, self._name))
            return False
        return True
    def getName(self):
        return str(self._name)
    def worldDump(self):
        retq = {}
        for name, pob in self._pobjects.items():
            retq[name] = {"at": pob.at(), "customStateVariables": pob._customStateVariables, "args": pob._args, "kwargs": pob._kwargs, "type": type(pob).__name__, "position": pob.getBodyProperty((), "position"), "orientation": pob.getBodyProperty((), "orientation"), "joints": pob.getJointStates()}
        return retq
    def greatReset(self, state):
        names = list(self._pobjects.keys())
        for name in names:
            self.removePObject(name)
        for name, data in state.items():
            if data['args'] is None:
                data['args'] = []
            for e in ('customStateVariables', 'joints', 'kwargs'):
                if data[e] is None:
                    data[e] = {}
            pob = self.addPObjectOfType(name, self._typeMap[data["type"]], data["position"], data["orientation"], *data['args'], **data['kwargs'])
            ensureHandsOk(data)
            pob._customStateVariables = data["customStateVariables"]
            data["joints"] = ensureDictionary(data["joints"])
            pob.setJointStates(data["joints"])
        return None
    # A PObject only exists embedded in exactly one world; names of PObjects are unique within a world.
    def addPObjectOfType(self, name, pobType, *args, **kwargs):
        if pobType.__name__ not in self._typeMap:
            self._typeMap[pobType.__name__] = pobType
        if name in self._pobjects:
            self._pobjects[name].remove()
        # TODO: a bit of redundancy here as also the pobject init code will ensure uniqueness of name and registration in the world.
        self._pobjects[name] = pobType(self, name, *args, **kwargs)
        self._id2PObjects[self._pobjects[name].getId()] = self._pobjects[name]
        ontoType = self._pobjects[name].getBodyProperty("", "type")
        if ontoType not in self._ontoTypes:
            self._ontoTypes[ontoType] = set([])
        self._ontoTypes[ontoType].add(name)
        return self._pobjects[name]
    def getPObject(self, name):
        if self._testPObjectPresence(name, "get"):
            return self._pobjects[name]
    def getPObjectNum(self):
        return len(self._pobjects)
    def getPObjectNames(self):
        return sorted([str(x) for x in self._pobjects.keys()])
    def getBodyIdentifiers(self):
        retq = []
        for k, v in self._pobjects.items():
            aux = [k]
            for b in v.getBodyIdentifiers():
                retq.append(tuple(aux + list(b)))
        return tuple(retq)
    def getBodyProperty(identifier, propertyId):
        if self._testPObjectPresence(identifier[0], "getBodyProperty"):
            return self._pobjects[identifier[0]].getBodyProperty(identifier[1:], propertyId)
    def setBodyProperty(identifier, propertyId, value):
        if self._testPObjectPresence(identifier[0], "setBodyProperty"):
            return self._pobjects[identifier[0]].setBodyProperty(identifier[1:], propertyId, value)
    def removePObject(self, name):
        if self._testPObjectPresence(name, "remove"):
            pobject = self._pobjects[name]
            ontoType = pobject.getBodyProperty("", "type")
            if ontoType in self._ontoTypes:
                self._ontoTypes[ontoType].remove(name)
            for linkName, parentName, parentLinkName in pobject._childOfConstraints:
                self._pobjects[parentName]._parentOfConstraints.pop((linkName, name, parentLinkName))
            for linkName, childName, parentLinkName in pobject._parentOfConstraints:
                self._pobjects[childName]._childOfConstraints.pop((linkName, name, parentLinkName))
            stubbornTry(lambda : p.removeBody(pobject._id, self._pybulletConnection))
            self._id2PObjects.pop(pobject.getId())
            self._pobjects.pop(name)
    def update(self):
        # A correct update transforms state variable values for step k, and produces values for step k+1.
        # We have several physics models for each object. Different models have disjoint output variables but may overlap in inputs.
        # Results from physics models must be cached and applied to the pobjects only after all results are known. 
        # But, pybullet is a black box we have to work around -- it immediately applies its updates. Hence, the seq. below.
        # First update all models *except rigid_body* associated to every pobject. These are the custom dynamic models.
        #     a custom dynamic model will not change the state variables associated to a pobject. Instead, it computes new values for the variables and returns a function that will set the variables to these values when called.
        #     it also computes "control" values to feed into the rigid body simulation. These control values are constraints and joint controls.
        updateFunctions = []
        for k, pob in self._pobjects.items():
            rigidBodyControls = []
            for dynamicModel in pob.getCustomDynamicModels():
                updateFn, controls = dynamicModel()
                updateFunctions.append(updateFn)
                rigidBodyControls = rigidBodyControls + controls
            pob.applyRigidBodyControls(rigidBodyControls)
        # Now update "rigid_body" physics using pybullet
        stubbornTry(lambda : p.stepSimulation(self._pybulletConnection))
        # For every pobject in the world, update the other physics using the update functions generated by the respective simulators.
        for fn in updateFunctions:
            fn()
        # If enabled, update debug visualizations
        if self._debugVisualizationsEnabled:
            stubbornTry(lambda : p.removeAllUserDebugItems(self._pybulletConnection))
            for dobj in self._debugObjects:
                dobj.updateLocation()
