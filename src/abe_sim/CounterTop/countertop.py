import os
import abe_sim.pobject as pob

class CounterTop(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./countertop.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"":  {"type": "countertop"}, "fn": {"boards": [[[-1.884,-0.568,0.747],[1.884,0.568,0.75]],[[-1.884,-0.464,0],[1.884,0.464,0.7]]], "supportbbs": [[[-0.5,-0.5,0.747],[0.5,0.5,0.75]], [[-1.7,-0.5,0.747],[1.7,0.5,0.75]]], "cancontain": True, "facingdistance": 1, "facingyaw": 3.14159/2.0}}

