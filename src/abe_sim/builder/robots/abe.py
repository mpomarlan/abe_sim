from morse.builder import *

from abe_sim.builder.actuators import Hands, Base, Head, GreatReset
from abe_sim.builder.sensors import WorldDump

import logging; logger = logging.getLogger("morse." + __name__)

class Abe(GroundRobot):
    """
    A template robot model for abe, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # abe.blend is located in the data/robots directory
        Robot.__init__(self, 'abe_sim/robots/abe.blend', name)
        self.properties(classpath = "abe_sim.robots.abe.Abe")

        ###################################
        # Actuators
        ###################################


        self.hands = Hands()
        self.append(self.hands)
        self.base = Base()
        self.append(self.base)
        self.head = Head()
        self.append(self.head)

        # Optionally allow to move the robot with the keyboard
        if debug:
            keyboard = Keyboard()
            keyboard.properties(ControlType = 'Position')
            self.append(keyboard)

        ###################################
        # Sensors
        ###################################

        semcam = SemanticCamera()
        semcam.translate(0.0, 0.0, 1.309)
        semcam.rotate(0.0, 0.0, 0.0)
        semcam.add_interface('socket')
        self.head.append(semcam)

        self.pose = Pose()
        self.append(self.pose)
        self.worlddump = WorldDump()
        self.append(self.worlddump)
        self.greatreset = GreatReset()
        self.append(self.greatreset)

