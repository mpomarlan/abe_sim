import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr
import abe_sim.dynamics.slicing as sl

class BreadSlice(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./breadslice.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"type": "breadslice", "slice": {"graspable": True}}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "bread", graspingRadius=0.35)
        self._dynamics = {"graspable": graspable}

class Bread(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./bread.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"type": "bread", "bread": {"graspable": True, "sliceable": True, "slicePoints": [(0.175, 0, 0.167), (0.125, 0, 0.167), (0.075, 0, 0.167), (0.025, 0, 0.167), (-0.025, 0, 0.167), (-0.075, 0, 0.167), (-0.125, 0, 0.167)], "sliceBases": [(0.183, 0, 0), (0.15, 0, 0), (0.075, 0, 0), (0.025, 0, 0), (-0.025, 0, 0), (-0.075, 0, 0), (-0.125, 0, 0)], "selfBases": [(-0.03, 0, 0), (-0.03, 0, 0), (0.075, 0, 0), (0.025, 0, 0), (-0.025, 0, 0), (-0.075, 0, 0), (-0.125, 0, 0)], "sliceIntegrity": 0.18}}
        self._customStateVariables["bread"]["urdfs"] = [os.path.join(cDir, ("./breadS%d.urdf"%k)) for k in range(1,7)]
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "bread", graspingRadius=0.35)
        sliceable = sl.Slicing(self._world, self, "bread", sliceProcRadiusGrob=0.5, sliceProcRadiusAlong=0.15, sliceProcRadiusFine=0.05, sliceStackAxis=[1,0,0], sliceAxis=[0,1,0], initialSliceIntegrity=0.06, sliceType=BreadSlice)
        self._dynamics = {"graspable": graspable, "sliceable": sliceable}

