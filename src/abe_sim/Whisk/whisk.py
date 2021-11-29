import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr

import math

class Whisk(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./whisk.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"": {"type": "whisk"}, "whisk": {"graspable": True}, "fn": {"referenceOrthogonal": True, "uprightLocalReference": [1,0,0], "uprightGlobalReference": [1,0,0], "boards": [[[0.005,-0.082,-0.082],[0.363,0.082,0.082]],[[-0.362,-0.032,-0.01],[0.005,0.032,0.01]]], "canwhisk": True, "graspingradius": 0.2, "handle": [-0.226,0,0], "refpt": [0.341,0,0], "refaxis": [1,0,0]}}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "whisk", graspingRadius=self._customStateVariables["fn"]["graspingradius"])
        self._dynamics = {"graspable": graspable}
    def spin(self):
        x = 45*(math.pi/180.0) + self.getJointStates()["brr"][0]
        self.applyRigidBodyControls([{"+constraints": [], "-constraints": [], "jointTargets": {"brr": (x,0,1)}}])

