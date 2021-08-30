import os
import abe_sim.pobject as pob
import abe_sim.dynamics.doorhinge as dh

class Fridge(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./fridge.urdf")
        self._useMaximalCoordinates = False
        self._customStateVariables = {"freezer_door": {"pullable": True}, "fridge_door": {"pullable": True}}
    def _customInitDynamicModels(self):
        fridgeDoorHinge = dh.DoorHinge(self._world, self, doorJoint="fridge_hinge", handlePoint=[-0.43268-0.315, 0.0548 + 0.063, 0.26239 + 0.011], handleRadius=0.5, openedAngle=-1.57, closedAngle=0, openingAxis=[0,1,0])
        freezerDoorHinge = dh.DoorHinge(self._world, self, doorJoint="freezer_hinge", handlePoint=[0.2347 + 0.268, 0.00977 + 0.115, 0.26239 + 0.018], handleRadius=0.5, openedAngle=1.57, closedAngle=0, openingAxis=[0,1,0])
        self._dynamics = {"fridgeDoorHinge": fridgeDoorHinge, "freezerDoorHinge": freezerDoorHinge}

