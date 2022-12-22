import os
import abe_sim.pobject as pob

class SaucePan(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./saucepan.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"type": "saucepan", "fn": {"dfltype": "dfl:saucepot.n.wn.artifact"}}

