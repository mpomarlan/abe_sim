import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr

class BakingTray(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./bakingtray.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"bakingtray": {"graspable": True}, "": {"type": "bakingtray"}, "fn": {"cancontain": True, "graspingradius": 0.2, "supportbbs": [[(-0.204,-0.103,-0.003), (0.204,0.103,0.0)]]}}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "bakingtray", graspingRadius=self._customStateVariables["fn"]["graspingradius"])
        self._dynamics = {"graspable": graspable}

