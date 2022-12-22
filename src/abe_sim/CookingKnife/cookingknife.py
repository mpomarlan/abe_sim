import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr

class CookingKnife(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./cookingknife.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"type": "knife", "knife": {"graspable": True, "canCut": True, "cuttingPoint": [-0.103, 0.0, -0.064]}}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "knife", graspingRadius=0.3)
        self._dynamics = {"graspable": graspable}

