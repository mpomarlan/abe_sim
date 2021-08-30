import pybullet as p
import math

class DoorHinge:
    def __init__(self, world, pobject, doorJoint="", handlePoint=[0,0,0], handleRadius=0, openedAngle=0, closedAngle=0, openingAxis=[0,1,0]):
        self._world = world
        self._simConnection = self._world.getSimConnection()
        self._pobject = pobject
        self._pobjectId = self._pobject.getId()
        self._doorJoint = doorJoint
        self._doorJointId = self._pobject.getJointId(self._doorJoint)
        self._doorLink = p.getJointInfo(self._pobjectId, self._doorJointId, self._simConnection)[12].decode('ascii')
        self._doorLinkId = self._pobject.getLinkId(self._doorLink)
        self._handlePoint = handlePoint
        self._handleRadius = handleRadius
        self._openedAngle = openedAngle
        self._closedAngle = closedAngle
        self._openingAxis = openingAxis
    def _closeHandlerVelocity(self):
        _, _, _, _, position, orientation = p.getLinkState(self._pobjectId, self._doorLinkId, 0, 0, self._simConnection)
        refPt, refAxis = [a+b for a,b in zip(position, p.rotateVector(orientation, self._handlePoint))], p.rotateVector(orientation, self._openingAxis)
        minC = [a - self._handleRadius for a in refPt]
        maxC = [a + self._handleRadius for a in refPt]
        closePObjects = list(set([self._world.getPObjectById(x[0]) for x in p.getOverlappingObjects(minC, maxC, self._simConnection)]))
        retq = None
        minD = self._handleRadius*10.0
        for pob in closePObjects:
            bodyIdentifiers = pob.getBodyIdentifiers()
            for b in bodyIdentifiers:
                pulling = pob.getBodyProperty(b, "pulling")
                if True == pulling:
                    position = pob.getBodyProperty(b, "position")
                    d = [a-b for a,b in zip(position, refPt)]
                    d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
                    if (d < minD) and (d < self._handleRadius):
                        minD = d
                        velocity = pob.getBodyProperty(b, "linearVelocity")
                        retq = velocity[0]*refAxis[0] + velocity[1]*refAxis[1] + velocity[2]*refAxis[2]
        return retq
    def update(self):
        updateFn = lambda : None
        angle = p.getJointState(self._pobjectId, self._doorJointId, self._simConnection)[0]
        if True != self._pobject.getBodyProperty((self._doorLink,), "pullable"):
            return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {self._doorJoint: (angle,0,1)}}]
        closeHandlerVelocity = self._closeHandlerVelocity()
        if None != closeHandlerVelocity:
            if 0 < closeHandlerVelocity:
                angle = self._openedAngle
            else:
                angle = self._closedAngle
        controls = [{"+constraints": [], "-constraints": [], "jointTargets": {self._doorJoint: (angle,0,1)}}]
        return updateFn, controls
    def selectInputVariables(self, customStateVariables):
        return ()
    def selectOutputVariables(self, customStateVariables):
        return ()

