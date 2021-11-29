import pybullet as p
import math

class Mixing:
    def __init__(self, world, pobject, link, mixingRadius=0, inputSubstances=set([]), outputSubstance="", nextURDF="", mixDecrement=0.1, mixSpeed=0.1, maxMixing=0, mixKey="", nextMixDynamics=None, nearTypeFilter=None):
        self._world = world
        self._simConnection = self._world.getSimConnection()
        self._pobject = pobject
        self._name = pobject.getName()
        self._pobjectId = self._pobject.getId()
        self._link = link
        self._mixingRadius = mixingRadius
        self._inputSubstances = inputSubstances
        self._outputSubstance = outputSubstance
        self._nextURDF = nextURDF
        self._mixDecrement = mixDecrement
        self._mixSpeed = mixSpeed
        self._maxMixing = maxMixing
        self._mixKey = mixKey
        self._nextMixDynamics = nextMixDynamics
        self._nearTypeFilter = nearTypeFilter
    def _closeMixer(self, position):
        minC = [a - 0.15 for a in position]
        maxC = [a + 0.15 for a in position]
        closePObjects = [x for x in list(set([self._world.getPObjectById(x[0]) for x in p.getOverlappingObjects(minC, maxC, self._simConnection)])) if x.getBodyProperty("fn", "canwhisk")]
        retq = 0
        if 0 < len(closePObjects):
            retq = max([abs(x.getJointStates()["brr"][1]) for x in closePObjects])
        return retq
    def _canMix(self, position):
        retq = False
        closeSubstances = {}
        ownSubstance = self._pobject.getBodyProperty((self._link,), "substance")
        minC = [a - self._mixingRadius for a in position]
        maxC = [a + self._mixingRadius for a in position]
        closePObjects = list(set([self._world.getPObjectById(x[0]) for x in p.getOverlappingObjects(minC, maxC, self._simConnection)]))
        closePObjects = [x for x in closePObjects if isinstance(x, self._nearTypeFilter)]
        for pob in closePObjects:
            bodyIdentifiers = pob.getBodyIdentifiers()
            for b in bodyIdentifiers:
                substance = pob.getBodyProperty(b, "substance")
                if None == substance:
                    continue
                if substance not in closeSubstances:
                    closeSubstances[substance] = 0
        if (self._outputSubstance in closeSubstances):
            retq = True
        else:
            if ownSubstance not in closeSubstances:
                closeSubstances[ownSubstance] = 0
            retq = True
            for s in self._inputSubstances:
                if (s not in closeSubstances):
                    retq = False
                    break
        return retq
    def _swapParticle(self):
        position = self._pobject.getBodyProperty((self._link,), "position")
        orientation = self._pobject.getBodyProperty((self._link,), "orientation")
        self._pobject._urdf = self._nextURDF
        self._pobject.reloadObject(position, orientation)
        self._pobject.setBodyProperty((self._link,), "substance", self._outputSubstance)
        self._pobject.setBodyProperty((self._link,), "mixing", self._maxMixing)
        if None != self._nextMixDynamics:
            self._pobject._dynamics[self._mixKey] = self._nextMixDynamics
        elif self._mixKey in self._pobject._dynamics:
            self._pobject._dynamics.pop(self._mixKey)
    def update(self):
        updateFn = lambda : None
        if True != self._pobject.getBodyProperty((self._link,), "graspable"):
            return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
        position = self._pobject.getBodyProperty((self._link,), "position")
        mixing = self._pobject.getBodyProperty((self._link), "mixing")
        if (0.1 < self._closeMixer(position)) and (self._canMix(position)):
            mixing = mixing - self._mixDecrement
            if 0 >= mixing:
                updateFn = lambda : self._swapParticle()
            else:
                updateFn = lambda : self._pobject.setBodyProperty((self._link,), "mixing", mixing)
        else:
            updateFn = lambda : None
        return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
    def selectInputVariables(self, customStateVariables):
        return ()
    def selectOutputVariables(self, customStateVariables):
        return ((self._link, "grasper"),)

