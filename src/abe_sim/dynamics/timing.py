import pybullet as p
import math

class Timing:
    def __init__(self, world, pobject):
        self._pobject = pobject
    def update(self):
        time = self._pobject.getBodyProperty("fn", "time")
        if time is None:
            time = 0.0
        time = time + (1.0/240.0)
        return (lambda : self._pobject.setBodyProperty("fn", "time", time)), [{"+constraints": [], "-constraints": [], "jointTargets": {}}]
    def selectInputVariables(self, customStateVariables):
        return (("fn", "time"),)
    def selectOutputVariables(self, customStateVariables):
        return (("fn", "time"),)

