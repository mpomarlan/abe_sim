import pybullet as p
import math
import abe_sim.Particle.particle as prt

class PourPortioning:
    def __init__(self, world, pobject, link, dripAxis=[0,1,0], dripPoint=[0, 0.206, 0.162], particleType=prt.Particle, maxDelay=80, portionArgs=[], portionKWArgs={}):
        self._world = world
        self._simConnection = self._world.getSimConnection()
        self._pobject = pobject
        self._name = pobject.getName()
        self._link = link
        self._dripAxis = dripAxis
        self._dripPoint = dripPoint
        self._particleType = particleType
        self._maxDelay = maxDelay
        self._portionArgs = portionArgs
        self._portionKWArgs = portionKWArgs
    def _makePortion(self, position, orientation):
        dripPoint = [a+b for a,b in zip(position, p.rotateVector(orientation, self._dripPoint))]
        self._world.addPObjectOfType(self._name+("_particle_%d" % self._world.getPObjectNum()), self._particleType, dripPoint, orientation, *self._portionArgs, **self._portionKWArgs)
        self._pobject.setBodyProperty((self._link,), "pourPortioningDelay", self._maxDelay)
    def update(self):
        updateFn = lambda : None
        controls = [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
        position = self._pobject.getBodyProperty((self._link,), "position")
        orientation = self._pobject.getBodyProperty((self._link,), "orientation")
        dripAxis = p.rotateVector(orientation, self._dripAxis)
        drip = dripAxis[2]
        if -0.9 > drip:
            # drip axis points down
            delay = self._pobject.getBodyProperty((self._link,), "pourPortioningDelay")
            if 0 == delay:
                updateFn = lambda : self._makePortion(position, orientation)
            else:
                updateFn = lambda : self._pobject.setBodyProperty((self._link,), "pourPortioningDelay", delay - 1)
        else:
            updateFn = lambda : None
        return updateFn, controls
    def selectInputVariables(self, customStateVariables):
        return ((self._link, "pourPortioningDelay"),)
    def selectOutputVariables(self, customStateVariables):
        return ((self._link, "pourPortioningDelay"),)

