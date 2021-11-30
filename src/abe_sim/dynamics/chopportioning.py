import pybullet as p
import math
import abe_sim.Particle.particle as prt
from abe_sim.utils import stubbornTry

class ChopPortioning:
    def __init__(self, world, pobject, link, chopRadiusGrob=0.5, chopRadius=0.05, chopAxis=[0,0,1], chopPoint=[0,0,0], portionPoint=[0,0,0], initialPortioningIntegrity=0.06, particleType=prt.Particle, portionArgs=[], portionKWArgs={}):
        self._world = world
        self._simConnection = self._world.getSimConnection()
        self._pobject = pobject
        self._name = pobject.getName()
        self._link = link
        self._chopRadiusGrob = chopRadiusGrob
        self._chopRadius = chopRadius
        self._chopAxis = chopAxis
        self._chopPoint = chopPoint
        self._portionPoint = portionPoint
        self._particleType = particleType
        self._portionArgs = portionArgs
        self._portionKWArgs = portionKWArgs
        self._initialPortioningIntegrity = initialPortioningIntegrity
    def _makePortion(self, position, orientation):
        portionPoint = [a+b for a,b in zip(position, p.rotateVector(orientation, self._portionPoint))]
        self._pobject.setBodyProperty((self._link,), "portioningIntegrity", self._initialPortioningIntegrity)
        self._world.addPObjectOfType(self._name+("_particle_%d" % self._world.getPObjectNum()), self._particleType, portionPoint, orientation, *self._portionArgs, **self._portionKWArgs)
    def _closeChopper(self, position, orientation):
        chopPoint = [a+b for a,b in zip(position, p.rotateVector(orientation, self._chopPoint))]
        minC = [a - self._chopRadiusGrob for a in chopPoint]
        maxC = [a + self._chopRadiusGrob for a in chopPoint]
        closePObjects = list(set([self._world.getPObjectById(x[0]) for x in stubbornTry(lambda : p.getOverlappingObjects(minC, maxC, self._simConnection))]))
        chopAxis = p.rotateVector(orientation, self._chopAxis)
        retq = None
        dt = stubbornTry(lambda : p.getPhysicsEngineParameters())['fixedTimeStep']
        for pob in closePObjects:
            bodyIdentifiers = pob.getBodyIdentifiers()
            for b in bodyIdentifiers:
                if pob.getBodyProperty(b, "canCut"):
                    positionC = pob.getBodyProperty(b, "position")
                    orientationC = pob.getBodyProperty(b, "orientation")
                    ## the velocity of the refC point should be calculated based on angular velocity too; we just assume
                    ## for now that this will be 0
                    velocityC = pob.getBodyProperty(b, "linearVelocity")
                    refC = pob.getBodyProperty(b, "cuttingPoint")
                    refC = [a+b for a,b in zip(positionC, p.rotateVector(orientationC, refC))]
                    cutLen = ((velocityC[0]*chopAxis[0] + velocityC[1]*chopAxis[1] + velocityC[2]*chopAxis[2])*dt)
                    d = [a - b for a, b in zip(refC, chopPoint)]
                    print(cutLen, self._chopRadius, refC, chopPoint)
                    if (0 > cutLen) and (self._chopRadius > math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])):
                        if None == retq:
                            retq = 0
                        retq = retq - cutLen
        return retq
    def update(self):
        updateFn = lambda : None
        controls = [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
        position = self._pobject.getBodyProperty((self._link,), "position")
        orientation = self._pobject.getBodyProperty((self._link,), "orientation")
        chopping = self._closeChopper(position, orientation)
        print("_", chopping)
        if None != chopping:
            integrity = self._pobject.getBodyProperty((self._link,), "portioningIntegrity")
            integrity = integrity - chopping
            if 0 < integrity:
                updateFn = lambda : self._pobject.setBodyProperty((self._link,), "portioningIntegrity", integrity)
            else:
                updateFn = lambda : self._makePortion(position, orientation)
        return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
        return updateFn, controls
    def selectInputVariables(self, customStateVariables):
        return ((self._link, "portioningIntegrity"),)
    def selectOutputVariables(self, customStateVariables):
        return ((self._link, "portioningIntegrity"),)

