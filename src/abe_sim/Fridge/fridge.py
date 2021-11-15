import os
import abe_sim.pobject as pob
import abe_sim.dynamics.doorhinge as dh

class FridgeDoor(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./fridgeDoor.urdf")
        self._useMaximalCoordinates = False
        self._customStateVariables = {"fridge_door": {"pullable": True}, "": {"type": "fridgeDoor"}, "fn": {"pullabledoor": True, "doorhandlelink": "fridge_door", "doorhandlejoint": "fridge_hinge", "openmin": -1.6, "openmax": -1.5, "closedangle": 0, "graspingradius": 0.4, "openingaxis": [0,1,0], "handlepoint": [-0.43268-0.315, 0.0548 + 0.063, 0.26239 + 0.011]}}
    def _customInitDynamicModels(self):
        fridgeDoorHinge = dh.DoorHinge(self._world, self, doorJoint=self._customStateVariables["fn"]["doorhandlejoint"], handlePoint=self._customStateVariables["fn"]["handlepoint"], handleRadius=self._customStateVariables["fn"]["graspingradius"], openedAngle=0.5*(self._customStateVariables["fn"]["openmin"]+self._customStateVariables["fn"]["openmax"]), closedAngle=self._customStateVariables["fn"]["closedangle"], openingAxis=self._customStateVariables["fn"]["openingaxis"])
        self._dynamics = {"fridgeDoorHinge": fridgeDoorHinge}

class FreezerDoor(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./freezerDoor.urdf")
        self._useMaximalCoordinates = False
        self._customStateVariables = {"freezer_door": {"pullable": True}, "": {"type": "freezerDoor"}, "fn": {"pullabledoor": True, "doorhandlelink": "freezer_door", "doorhandlejoint": "freezer_hinge", "openmin": 1.5, "openmax": 1.6, "closedangle": 0, "graspingradius": 0.4, "openingaxis": [0,1,0], "handlepoint": [0.2347 + 0.268, 0.00977 + 0.115, 0.26239 + 0.018]}}
    def _customInitDynamicModels(self):
        freezerDoorHinge = dh.DoorHinge(self._world, self, doorJoint=self._customStateVariables["fn"]["doorhandlejoint"], handlePoint=self._customStateVariables["fn"]["handlepoint"], handleRadius=self._customStateVariables["fn"]["graspingradius"], openedAngle=0.5*(self._customStateVariables["fn"]["openmin"]+self._customStateVariables["fn"]["openmax"]), closedAngle=self._customStateVariables["fn"]["closedangle"], openingAxis=self._customStateVariables["fn"]["openingaxis"])
        self._dynamics = {"freezerDoorHinge": freezerDoorHinge}

class Fridge(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./fridge.urdf")
        self._useMaximalCoordinates = False
        self._customStateVariables = {"": {"type": "fridge"}, "fn": {"boards": [[[0.959,-0.226,-0.484], [1.047,0.813,0.973]], [[0.077,-0.267,-0.493], [0.155,0.763,0.974]], [[-0.53,-0.226,-0.485], [-0.442,0.813,0.973]], [[-0.462,-0.269,0.977], [0.982,0.808,1.005]], [[-0.462,-0.266,-0.619], [0.982,-0.182,0.982]], [[-0.515,-0.269,-0.631], [1.035,0.808,-0.462]], [[-0.439,-0.180,0.46], [0.959,0.720,0.508]], [[-0.439,-0.180,-0.064], [0.959,0.720,-0.016]]], "supportbbs": [[(0.155,-0.180,0.5), (0.959,0.720,0.508)], [(0.155,-0.180,-0.02), (0.959,0.720,-0.016)]], "cancontain": True, "exitdirection": (0,1,0), "door": None}}
    def _customInitDynamicModels(self):
        return

class Freezer(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./freezer.urdf")
        self._useMaximalCoordinates = False
        self._customStateVariables = {"": {"type": "freezer"}, "fn": {"boards": [[[0.959,-0.226,-0.484], [1.047,0.813,0.973]], [[0.077,-0.267,-0.493], [0.155,0.763,0.974]], [[-0.53,-0.226,-0.485], [-0.442,0.813,0.973]], [[-0.462,-0.269,0.977], [0.982,0.808,1.005]], [[-0.462,-0.266,-0.619], [0.982,-0.182,0.982]], [[-0.515,-0.269,-0.631], [1.035,0.808,-0.462]], [[-0.439,-0.180,0.610], [0.959,0.720,0.658]], [[-0.439,-0.180,0.213], [0.959,0.720,0.261]], [[-0.439,-0.180,-0.164], [0.959,0.720,-0.116]]], "supportbbs": [[(-0.439,-0.180,0.657), (0.076,0.720,0.658)], [(-0.439,-0.180,0.260), (0.076,0.720,0.261)], [(-0.439,-0.180,-0.117), (0.076,0.720,-0.116)]], "cancontain": True, "exitdirection": (0,1,0), "door": None}}
    def _customInitDynamicModels(self):
        return

