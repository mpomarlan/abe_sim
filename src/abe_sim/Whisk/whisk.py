import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr

class Whisk(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./whisk.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"": {"type": "whisk"}, "whisk": {"graspable": True}, "fn": {"canwhisk": True, "graspingradius": 0.2, "handle": [-0.226,0,0], "refpt": [0.341,0,0], "refaxis": [1,0,0]}}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "whisk", graspingRadius=self._customStateVariables["fn"]["graspingradius"])
        self._dynamics = {"graspable": graspable}

