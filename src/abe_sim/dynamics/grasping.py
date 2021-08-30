import pybullet as p
import math

class Grasping:
    def __init__(self, world, pobject, link, graspingRadius=0):
        self._world = world
        self._simConnection = self._world.getSimConnection()
        self._pobject = pobject
        self._name = pobject.getName()
        self._pobjectId = self._pobject.getId()
        self._link = link
        self._graspingRadius = graspingRadius
    def _closeGripper(self, position):
        minC = [a - self._graspingRadius for a in position]
        maxC = [a + self._graspingRadius for a in position]
        closePObjects = list(set([self._world.getPObjectById(x[0]) for x in p.getOverlappingObjects(minC, maxC, self._simConnection)]))
        retq = None
        minD = self._graspingRadius*10.0
        for pob in closePObjects:
            bodyIdentifiers = pob.getBodyIdentifiers()
            for b in bodyIdentifiers:
                grasping = pob.getBodyProperty(b, "grasping")
                if (None != grasping) and (self._name in grasping):
                    positionG = pob.getBodyProperty(b, "position")
                    d = [a - b for a, b in zip(position, positionG)]
                    d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
                    if (d < minD) and (d < self._graspingRadius):
                        minD = d
                        retq = (pob.getName(), b[0])
        return retq
    def update(self):
        updateFn = lambda : None
        if True != self._pobject.getBodyProperty((self._link,), "graspable"):
            return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
        conPlus = []
        conMinus = []
        position = self._pobject.getBodyProperty((self._link,), "position")
        grasper = self._pobject.getBodyProperty((self._link,), "grasper")
        closeGripper = self._closeGripper(position)
        if (None != closeGripper) and (grasper != closeGripper):
            if None != grasper:
                conMinus = [(self._link, grasper[0], grasper[1])]
            conPlus = [(self._link, closeGripper[0], closeGripper[1])]
            updateFn = lambda : self._pobject.setBodyProperty((self._link,), "grasper", closeGripper)
        elif (None == closeGripper) and (None != grasper):
            conMinus = [(self._link, grasper[0], grasper[1])]
            updateFn = lambda : self._pobject.setBodyProperty((self._link,), "grasper", None)
        controls = [{"+constraints": conPlus, "-constraints": conMinus, "jointTargets": {}}]
        return updateFn, controls
    def selectInputVariables(self, customStateVariables):
        return ()
    def selectOutputVariables(self, customStateVariables):
        return ((self._link, "grasper"),)

