import pybullet as p
import math
from abe_sim.geom import translateVector, angleDifference
from abe_sim.utils import stubbornTry

class Baking:
    def __init__(self, world, pobject):
        self._world = world
        self._pobject = pobject
    def _isBaking(self):
        found = False
        cob = self._pobject
        while cob:
            ato = cob.at()
            if ato in self._world._pobjects:
                ato = self._world._pobjects[ato]
            else:
                ato = None
            if not ato:
                break
            if ato.getBodyProperty("fn", "canbake"):
                door = ato.getBodyProperty("fn", "door")
                if door in self._world._pobjects:
                    door = self._world._pobjects[door]
                else:
                    door = None
                if door:
                    angle = door.getJointStates()[door.getBodyProperty("fn", "doorhandlejoint")][0]
                    if 0.1 > angleDifference(angle, door.getBodyProperty("fn", "closedangle")):
                        found = True
                        break
            cob = ato
        return found
    def _swapShape(self):
        position = self._pobject.getBodyProperty((), "position")
        orientation = self._pobject.getBodyProperty((), "orientation")
        nextURDF = self._pobject.getBodyProperty("fn", "bakedurdf")
        if nextURDF:
            self._pobject._urdf = nextURDF
            self._pobject.reloadObject(position, orientation)
        self._pobject.setBodyProperty("fn", "bakeable", False)
    def update(self):
        updateFn = lambda : None
        if not self._pobject.getBodyProperty("fn", "bakeable"):
            return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
        if self._isBaking():
            hp = self._pobject.getBodyProperty("fn", "bakinghp") - 1
            if 0 >= hp:
                updateFn = lambda : self._swapShape()
            else:
                updateFn = lambda : self._pobject.setBodyProperty("fn", "bakinghp", hp)
        return updateFn, [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
    def selectInputVariables(self, customStateVariables):
        return ()
    def selectOutputVariables(self, customStateVariables):
        return ()

