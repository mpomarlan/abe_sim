import logging; logger = logging.getLogger("morse." + __name__)

from morse.core.actuator import Actuator
from morse.helpers.components import add_data
from morse.core import mathutils

class Head(Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Head"
    _short_desc = "Pans/tilts head."

    # define here the data fields required by your actuator
    # format is: field name, initial value, type, description
    add_data('pan', 0.0, 'float', 'Pan, in radians')
    add_data('tilt', 0.0, 'float', 'Tilt, in radians')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        Actuator.__init__(self, obj, parent)

        self.head = parent.bge_object.children["Head"]

        for child in self.bge_object.children:
            child.setParent(self.head)

        logger.info('Component initialized')

    def default_action(self):
        h_orientation = mathutils.Euler([0.0, self.local_data['tilt'], self.local_data['pan']])
        self.head.localOrientation = h_orientation.to_matrix()

