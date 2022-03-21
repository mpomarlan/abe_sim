import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr

class BakingSheet(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./bakingsheet.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"bakingsheet": {"graspable": True}, "": {"type": "bakingsheet"}, "fn": {"cancontain": False, "graspingradius": 0.2, "supportbbs": [[(-0.188,-0.106,0.001), (0.188,0.106,0.003)]]}}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "bakingsheet", graspingRadius=self._customStateVariables["fn"]["graspingradius"])
        self._dynamics = {"graspable": graspable}

