import logging; logger = logging.getLogger("morse." + __name__)

from morse.core.actuator import Actuator
from morse.helpers.components import add_data
from morse.core import blenderapi
from morse.core import mathutils

from morse.sensors.collision import Collision

xh = 0.0
yhL = 0.4
yhR = -0.4
zh = 0.88215

class Hands(Actuator):
    _name = "Hands"
    _short_desc = "Controls the hands"

    add_data('lx', 0.0, 'float', 'Left hand x-position, in meters')
    add_data('rx', 0.0, 'float', 'Right hand x-position, in meters')
    add_data('ly', 0.4, 'float', 'Left hand y-position, in meters')
    add_data('ry', -0.4, 'float', 'Right hand y-position, in meters')
    add_data('lz', 0.9589, 'float', 'Left hand z-position, in meters')
    add_data('rz', 0.9589, 'float', 'Right hand z-position, in meters')

    add_data('lrx', 0.0, 'float', 'Left hand roll, in radians')
    add_data('rrx', 0.0, 'float', 'Right hand roll, in radians')
    add_data('lry', 0.0, 'float', 'Left hand pitch, in radians')
    add_data('rry', 0.0, 'float', 'Right hand pitch, in radians')
    add_data('lrz', 0.0, 'float', 'Left hand yaw, in radians')
    add_data('rrz', 0.0, 'float', 'Right hand yaw, in radians')

    add_data('lgrab', '', 'string', 'Name of object to grab with left gripper')
    add_data('rgrab', '', 'string', 'Name of object to grab with right gripper')
    add_data('lrelease', False, 'bool', 'Release whatever is in the left gripper')
    add_data('rrelease', False, 'bool', 'Release whatever is in the right gripper')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        Actuator.__init__(self, obj, parent)

        self.left_hand = parent.bge_object.children["LeftHand"]
        self.right_hand = parent.bge_object.children["RightHand"]
        self.left_holding = None
        self.right_holding = None

        self.left_handToucher = None
        self.right_handToucher = None
        self.collision_data = {"left": {}, "right": {}}

        logger.info('Component initialized')

    def default_action(self):
        l_orientation = mathutils.Euler([self.local_data['lrx'], self.local_data['lry'], self.local_data['lrz']])
        self.left_hand.localOrientation = l_orientation.to_matrix()
        r_orientation = mathutils.Euler([self.local_data['rrx'], self.local_data['rry'], self.local_data['rrz']])
        self.right_hand.localOrientation = r_orientation.to_matrix()
        self.left_hand.localPosition = mathutils.Vector([self.local_data['lx'], self.local_data['ly'], self.local_data['lz']])
        self.right_hand.localPosition = mathutils.Vector([self.local_data['rx'], self.local_data['ry'], self.local_data['rz']])
        if self.local_data['lrelease']:
            if self.left_holding:
                if self.left_holding.parent == self.left_hand:
                    self.left_holding.removeParent()
                    self.left_holding.restoreDynamics()
                self.left_holding = None
        elif ('' != self.local_data['lgrab']) and (None == self.left_holding) and (self.local_data['lgrab'] in self.left_hand.scene.objects):
            ## TODO: add some distance based sanity check here
            self.left_holding = self.left_hand.scene.objects[self.local_data['lgrab']]
            self.left_holding.setParent(self.left_hand)
            self.left_holding.suspendDynamics()
        if self.local_data['rrelease']:
            if self.right_holding:
                if self.right_holding.parent == self.right_hand:
                    self.right_holding.removeParent()
                    self.right_holding.restoreDynamics()
                self.right_holding = None
        elif ('' != self.local_data['rgrab']) and (None == self.right_holding) and (self.local_data['rgrab'] in self.right_hand.scene.objects):
            ## TODO: add some distance based sanity check here
            self.right_holding = self.right_hand.scene.objects[self.local_data['rgrab']]
            self.right_holding.setParent(self.right_hand)
            self.right_holding.suspendDynamics()

