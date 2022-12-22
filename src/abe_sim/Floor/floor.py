import os
import abe_sim.pobject as pob

class Floor(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./floor.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"type": "floor"}

