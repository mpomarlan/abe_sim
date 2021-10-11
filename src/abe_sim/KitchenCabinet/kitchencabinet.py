import os
import abe_sim.pobject as pob

class KitchenCabinet(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./kitchencabinet.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"": {"type": "kitchencabinet"}, "fn": {"boards": [[[-2.927,-0.141,-0.953], [-2.798,0.674,1.036]], [[-2.794,-0.127,-0.953], [3.224,0.634,-0.558]], [[-2.924,-0.225,-0.952], [3.34,-0.146,1.036]], [[3.223,-0.141,-0.953], [3.352,0.674,1.036]], [[-2.927,-0.225,0.768], [3.352,0.674,1.036]], [[-2.794,-0.126,0.04], [3.224,0.594,0.17]]], "cancontain": True, "exitdirection": (0,1,0), "supportbbs": [[(-2.794,-0.126,0.165), (3.224,0.594,0.175)], [(-2.794,-0.126,-0.558), (3.224,0.594,-0.548)]]}}

