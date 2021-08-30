import os
import abe_sim.pobject as pob

class Abe(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./abe.urdf")
        self._useMaximalCoordinates = False
        self._customStateVariables = {"hand_right_roll": {"canPull": True, "pulling": False, "grasping": []}, "hand_left_roll": {"canPull": True, "pulling": False, "grasping": []}}
    def _customInitDynamicModels(self):
        self._dynamics = {}

