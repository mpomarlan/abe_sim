import os
import abe_sim.pobject as pob

class PlasticCup(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./plasticcup.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"":  {"type": "plasticcup"}, "fn": {"dfltype": "dfl:plastic_cup.n.wn.artifact"}}

