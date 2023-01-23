import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr

import math

class Spoon(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./spoon.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"type": "spoon", "spoon": {"graspable": True}, "fn": {"referenceOrthogonal": True, "uprightLocalReference": [1,0,0], "uprightGlobalReference": [1,0,0],
 "boards": [[[0.008,-0.04,-0.04],[0.25,0.04,0.04]],[[-0.11,-0.02,-0.01],[0.008,0.02,0.01]]], 
 "canwhisk": True, "graspingradius": 0.2, "handle": [-0.07,0,0], "refpt": [0.26,0,0], "refaxis": [1,0,0]}}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "spoon", graspingRadius=self._customStateVariables["fn"]["graspingradius"])
        self._dynamics = {"graspable": graspable}
    def spin(self):
        x = 45*(math.pi/180.0) + self.getJointStates()["brr"][0]
        self.applyRigidBodyControls([{"+constraints": [], "-constraints": [], "jointTargets": {"brr": (x,0,1)}}])

