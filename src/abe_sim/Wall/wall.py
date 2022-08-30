import os
import abe_sim.pobject as pob

class Wall(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./wall.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"":  {"type": "wall"}, "fn": {"dfltype": "dfl:wall.n.wn.artifact..architecture"}}

