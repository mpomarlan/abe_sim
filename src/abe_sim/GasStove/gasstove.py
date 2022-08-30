import os
import abe_sim.pobject as pob

class GasStove(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./gasstove.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"":  {"type": "gasstove"}, "fn": {"dfltype": "dfl:gas_oven.n.wn.artifact..domestic"}}

class GasStoveDoor(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./gasstovedoor.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"":  {"type": "gasstovedoor"}, "fn": {}}

