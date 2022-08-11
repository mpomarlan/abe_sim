import pybullet as p
import math
from abe_sim.utils import stubbornTry

class DoorHinge:
    def __init__(self, world, pobject, doorJoint="", handlePoint=[0,0,0], handleRadius=0, openedAngle=0, closedAngle=0, openingAxis=[0,1,0]):
        self._world = world
        self._simConnection = self._world.getSimConnection()
        self._pobject = pobject
        self._pobjectId = self._pobject.getId()
        self._doorJoint = doorJoint
        self._doorJointId = self._pobject.getJointId(self._doorJoint)
        self._doorLink = stubbornTry(lambda : p.getJointInfo(self._pobjectId, self._doorJointId, self._simConnection))[12].decode('ascii')
        self._doorLinkId = self._pobject.getLinkId(self._doorLink)
        self._handlePoint = handlePoint
        self._handleRadius = handleRadius
        self._openedAngle = openedAngle
        self._closedAngle = closedAngle
        self._openingAxis = openingAxis
    def _closeHandlerVelocity(self):
        _, _, _, _, position, orientation = stubbornTry(lambda : p.getLinkState(self._pobjectId, self._doorLinkId, 0, 0, self._simConnection))
        refAxis = p.rotateVector(orientation, self._openingAxis)
        refPt = [a+b for a,b in zip(position, p.rotateVector(orientation, self._handlePoint))]
        minC = [a - self._handleRadius for a in refPt]
        maxC = [a + self._handleRadius for a in refPt]
        closePObjects = list(self._world.getPObjectSetByIds([x[0] for x in stubbornTry(lambda : p.getOverlappingObjects(minC, maxC, self._simConnection))]))
        retq = None, None, None
        minD = self._handleRadius*10.0
        wrappedName = self._pobject.getName() + ":" + self._doorLink
        for pob in closePObjects:
            bodyIdentifiers = pob.getBodyIdentifiers()
            for b in bodyIdentifiers:
                pulling = pob.getBodyProperty(b, "pulling")
                if pulling and (wrappedName in pulling):
                    position = pob.getBodyProperty(b, "position")
                    d = [a-b for a,b in zip(position, refPt)]
                    d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
                    if (d < minD) and (d < self._handleRadius):
                        minD = d
                        velocity = pob.getBodyProperty(b, "linearVelocity")
                        retq = pob, b, velocity[0]*refAxis[0] + velocity[1]*refAxis[1] + velocity[2]*refAxis[2]
        return retq
    def update(self):
        updateFn = lambda : None
        angle = stubbornTry(lambda : p.getJointState(self._pobjectId, self._doorJointId, self._simConnection))[0]
        if True != self._pobject.getBodyProperty(self._doorLink, "pullable"):
            return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {self._doorJoint: (angle,0,1)}}]
        pob, b, closeHandlerVelocity = self._closeHandlerVelocity()
        if None != closeHandlerVelocity:
            if pob.getBodyProperty(b, "pullingopen"):
                angle = self._openedAngle
            elif pob.getBodyProperty(b, "pushingclosed"):
                angle = self._closedAngle
        controls = [{"+constraints": [], "-constraints": [], "jointTargets": {self._doorJoint: (angle,0,1)}}]
        return updateFn, controls
    def selectInputVariables(self, customStateVariables):
        return ()
    def selectOutputVariables(self, customStateVariables):
        return ()

