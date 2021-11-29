import os
import abe_sim.pobject as pob

class KitchenCabinet(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./kitchencabinet.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"": {"type": "kitchencabinet"}, "fn": {"boards": [[[-2.927,-0.641,-0.953], [-2.798,0.674,1.236]], [[-2.794,-0.627,-0.953], [3.224,0.634,-0.558]], [[-2.924,-0.725,-0.952], [3.34,-0.646,1.236]], [[3.223,-0.641,-0.953], [3.352,0.674,1.236]], [[-2.927,-0.725,0.968], [3.352,0.674,1.236]], [[-2.794,-0.626,0.14], [3.224,0.594,0.27]]], "cancontain": True, "exitdirection": (0,1,0), "supportbbs": [[(-2.794,-0.626,-0.558), (3.224,0.594,-0.55)]]}}
#"supportbbs": [[(-2.794,-0.126,0.265), (3.224,0.594,0.27)], [(-2.794,-0.126,-0.558), (3.224,0.594,-0.55)]]

class KitchenCabinetLow(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./kitchencabinet.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"": {"type": "kitchencabinet"}, "fn": {"boards": [[[-2.927,-0.141,-0.953], [-2.798,0.674,1.236]], [[-2.794,-0.127,-0.953], [3.224,0.634,-0.558]], [[-2.924,-0.225,-0.952], [3.34,-0.146,1.236]], [[3.223,-0.141,-0.953], [3.352,0.674,1.236]], [[-2.927,-0.225,0.968], [3.352,0.674,1.236]], [[-2.794,-0.126,0.14], [3.224,0.594,0.27]]], "cancontain": True, "exitdirection": (0,1,0), "supportbbs": [[(-2.794,-0.126,-0.558), (3.224,0.594,-0.55)]]}}


