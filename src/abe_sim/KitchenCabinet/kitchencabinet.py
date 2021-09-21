import os
import abe_sim.pobject as pob

class KitchenCabinet(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./kitchencabinet.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"": {"type": "kitchencabinet"}, "fn": {"cancontain": True, "exitdirection": (0,1,0), "supportbbs": [[(-2.794,-0.126,0.165), (3.224,0.594,0.175)], [(-2.794,-0.126,-0.558), (3.224,0.594,-0.548)]]}}

