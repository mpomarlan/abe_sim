import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr
#import abe_sim.dynamics.mixing as mx

class Shaker(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, self._urdfName())
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"shaker": {"graspable": True}, "": {"type": self._ontoTypeName()}, "fn": {"graspingradius": 0.2, "contents": "sugar"}}
    def _ontoTypeName(self):
        return "shaker"
    def _urdfName(self):
        return "./shaker.urdf"
    def _customInitDynamicModels(self):
        self._dynamics = {"graspable": gr.Grasping(self._world, self, "shaker", graspingRadius=self._customStateVariables["fn"]["graspingradius"])}


