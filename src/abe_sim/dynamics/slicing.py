import pybullet as p
import math

class Slicing:
    def __init__(self, world, pobject, link, sliceProcRadiusGrob=0.5, sliceProcRadiusAlong=0.15, sliceProcRadiusFine=0.05, sliceStackAxis=[1,0,0], sliceAxis=[0,1,0], initialSliceIntegrity=1, sliceType=None, sliceArgs=[], sliceKWArgs={}):
        self._world = world
        self._simConnection = self._world.getSimConnection()
        self._pobject = pobject
        self._name = pobject.getName()
        self._pobjectId = self._pobject.getId()
        self._link = link
        self._sliceProcRadiusGrob = sliceProcRadiusGrob
        self._sliceProcRadiusAlong = sliceProcRadiusAlong
        self._sliceProcRadiusFine = sliceProcRadiusFine
        self._sliceStackAxis = sliceStackAxis
        self._sliceAxis = sliceAxis
        self._initialSliceIntegrity = initialSliceIntegrity
        self._sliceType = sliceType
        self._sliceArgs = sliceArgs
        self._sliceKWArgs = sliceKWArgs
    def _closeSlicer(self, position, orientation):
        slicePoints = self._pobject.getBodyProperty((self._link,), "slicePoints")
        refPt = [a+b for a,b in zip(position, p.rotateVector(orientation, slicePoints[0]))]
        minC = [a - self._sliceProcRadiusGrob for a in refPt]
        maxC = [a + self._sliceProcRadiusGrob for a in refPt]
        closePObjects = list(set([self._world.getPObjectById(x[0]) for x in p.getOverlappingObjects(minC, maxC, self._simConnection)]))
        slicingAxis = p.rotateVector(orientation, self._sliceAxis)
        retq = None
        dt = p.getPhysicsEngineParameters()['fixedTimeStep']
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
                    cutLen = abs((velocityC[0]*slicingAxis[0] + velocityC[1]*slicingAxis[1] + velocityC[2]*slicingAxis[2])*dt)
                    d = [a - b for a, b in zip(refC, refPt)]
                    along = d[0]*slicingAxis[0] + d[1]*slicingAxis[1] + d[2]*slicingAxis[2]
                    if abs(along) < self._sliceProcRadiusAlong:
                        d = [d[0] - along*slicingAxis[0], d[1] - along*slicingAxis[1], d[2] - along*slicingAxis[2]]
                        d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
                        if d < self._sliceProcRadiusFine:
                            if None == retq:
                                retq = 0
                            retq = retq + cutLen
        return retq
    def _makeSlice(self, position, orientation):
        slicePoints = self._pobject.getBodyProperty((self._link,), "slicePoints")
        selfBases = self._pobject.getBodyProperty((self._link,), "selfBases")
        sliceBases = self._pobject.getBodyProperty((self._link,), "sliceBases")
        urdfs = self._pobject.getBodyProperty((self._link,), "urdfs")
        positionAdj = [a+b for a,b in zip(position, p.rotateVector(orientation, selfBases[0]))]
        positionSlice = [a+b for a,b in zip(position, p.rotateVector(orientation, sliceBases[0]))]
        print("Bla")
        self._pobject._urdf = urdfs[0]
        self._pobject.reloadObject(positionAdj, orientation)
        print("Blaa")
        self._world.addPObjectOfType(self._name+("_slice_%d" % len(sliceBases)), self._sliceType, positionSlice, orientation, *self._sliceArgs, **self._sliceKWArgs)
        print("Blaaa")
        self._pobject.setBodyProperty((self._link,), "urdfs", urdfs[1:])
        self._pobject.setBodyProperty((self._link,), "slicePoints", slicePoints[1:])
        self._pobject.setBodyProperty((self._link,), "selfBases", selfBases[1:])
        self._pobject.setBodyProperty((self._link,), "sliceBases", sliceBases[1:])
        self._pobject.setBodyProperty((self._link,), "sliceIntegrity", self._initialSliceIntegrity)
        print("Blaaaa")
    def update(self):
        updateFn = lambda : None
        if (True != self._pobject.getBodyProperty((self._link,), "sliceable")) or (0 == len(self._pobject.getBodyProperty((self._link,), "slicePoints"))):
            return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
        position = self._pobject.getBodyProperty((self._link,), "position")
        orientation = self._pobject.getBodyProperty((self._link,), "orientation")
        integrity = self._pobject.getBodyProperty((self._link,), "sliceIntegrity")
        closeSlicer = self._closeSlicer(position, orientation)
        if None != closeSlicer:
            integrity = integrity - closeSlicer
            if 0 < integrity:
                updateFn = lambda : self._pobject.setBodyProperty((self._link,), "sliceIntegrity", integrity)
            else:
                updateFn = lambda : self._makeSlice(position, orientation)
        return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
    def selectInputVariables(self, customStateVariables):
        return ((self._link, "sliceIntegrity"),)
    def selectOutputVariables(self, customStateVariables):
        return ((self._link, "sliceIntegrity"),)

