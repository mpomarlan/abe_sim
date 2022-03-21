import os
import abe_sim.pobject as pob
import abe_sim.dynamics.doorhinge as dh

class KitchenStoveDoor(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./kitchenstovedoor.urdf")
        self._useMaximalCoordinates = False
        self._customStateVariables = {"kitchenstove_door": {"pullable": True}, "": {"type": "kitchenStoveDoor"}, "fn": {"pullabledoor": True, "doorhandlelink": "kitchenstove_door", "doorhandlejoint": "kitchenstove_hinge", "openmin": -1.6, "openmax": -1.0, "closedangle": 0, "graspingradius": 0.3, "openingaxis": [0,1,0], "handlepoint": [0.0, 0.07, 0.9]}}
    def _customInitDynamicModels(self):
        kitchenStoveDoorHinge = dh.DoorHinge(self._world, self, doorJoint=self._customStateVariables["fn"]["doorhandlejoint"], handlePoint=self._customStateVariables["fn"]["handlepoint"], handleRadius=self._customStateVariables["fn"]["graspingradius"], openedAngle=0.5*(self._customStateVariables["fn"]["openmin"]+self._customStateVariables["fn"]["openmax"]), closedAngle=self._customStateVariables["fn"]["closedangle"], openingAxis=self._customStateVariables["fn"]["openingaxis"])
        self._dynamics = {"kitchenStoveDoorHinge": kitchenStoveDoorHinge}

class KitchenStove(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./kitchenstove.urdf")
        self._useMaximalCoordinates = False
        self._customStateVariables = {"": {"type": "kitchenstove"}, "fn": {"boards": [[[-0.884,-0.537,0.042], [0.884,0.656,0.024]], [[0.886, -0.582,0.01], [1.322,0.523,0.076]], [[-1.322,-0.582,0.01], [-0.886,0.523,0.076]], [[-1.374,-0.725,-0.416], [1.374,-0.541,0.461]], [[1.195,-0.567,-0.417], [1.365,0.656,0.467]], [[-1.365,-0.567,-0.417], [-1.195,0.656,0.467]], [[-1.374, -0.729,-0.502], [1.374,0.649,-0.418]], [[-1.374,-0.723,-0.691], [1.374,0.587,-0.506]], [[-1.374,-0.729,0.47], [1.374,0.649,0.554]], [[-1.374,-0.715,0.57], [1.374,0.715,0.751]]], "supportbbs": [[[-0.883,-0.537,0.024], [0.884,0.656,0.027]]], "cancontain": True, "canbake": True, "exitdirection": (0,1,0), "door": None, "facingdistance": 1.2, "facingyaw": -3.14159/2.0}}
    def _customInitDynamicModels(self):
        return

