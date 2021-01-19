import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.core.actuator import Actuator
from morse.helpers.components import add_data, add_property

class Base(Actuator):
    _name = "Base actuator"
    _short_desc="An actuator for the base."

    add_data('v', 0.0, 'float', 'Forward displacement, in meters')
    add_data('w', 0.0, 'float', 'Angular displacement, in meters')

    add_property('_type', 'Position', 'ControlType', 'string',
                 "Kind of control to move the parent robot, in ['Position', "
                 "'Velocity', 'Differential']")
    add_property('_speed', 1.0, 'Speed', 'float',
                 "Movement speed of the parent robot, in m/s")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Actuator.__init__(self, obj, parent)

        # Correct the speed considering the Blender clock
        if self._type == 'Position':
            self._speed = self._speed / self.frequency

        self.zero_motion = True

        logger.info('Component initialized')

    def default_action(self):
        """ Interpret keyboard presses and assign them to movement
            for the robot."""

        # Reset movement variables
        vx, vy, vz = 0.0, 0.0, 0.0
        rx, ry, rz = 0.0, 0.0, 0.0

        vx = self._speed*self.local_data['v']
        rz = self._speed*self.local_data['w']

        # Send a 'zero motion' only once in a row.
        if self.zero_motion and (vx,vy,vz,rx,ry,rz) == (0,0,0,0,0,0):
            return

        if self._type == 'Position' or self._type == 'Velocity':
            self.robot_parent.apply_speed(self._type, [vx, vy, vz], [rx, ry, rz / 2.0])
        elif self._type == 'Differential':
            self.robot_parent.apply_vw_wheels(vx, rz)


        if (vx,vy,vz,rx,ry,rz) == (0,0,0,0,0,0):
            self.zero_motion = True
        else:
            self.zero_motion = False

