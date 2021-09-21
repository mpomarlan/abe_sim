import os
import abe_sim.pobject as pob

class Pantry(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./pantry.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"": {"type": "pantry"}, "fn": {"cancontain": True, "exitdirection": (0,1,0), "supportbbs": [[(-0.82, -0.183, 0.45), (0.82, 0.296, 0.458)], [(-0.82, -0.183, -0.206), (0.82, 0.296, -0.198)], [(-0.82, -0.183, -0.863), (0.82, 0.296, -0.855)]]}}

