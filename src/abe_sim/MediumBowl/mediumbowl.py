import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr

class MediumBowl(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./mediumbowl.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"mediumbowl": {"graspable": True}, "": {"type": "mediumbowl"}, "fn": {"graspingradius": 0.45, "refpt": (0,0.156,0.061)}}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "mediumbowl", graspingRadius=self._customStateVariables["fn"]["graspingradius"])
        self._dynamics = {"graspable": graspable}

