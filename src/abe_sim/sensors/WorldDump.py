import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

from morse.core import blenderapi
from morse.helpers.transformation import Transformation3d

import json

class WorldDump(morse.core.sensor.Sensor):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "WorldDump"
    _short_desc = "Produce a list of all Objects with their types, graspable statuses, descriptions, and positions"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('jsondump', '{}', 'str', 'A json string containing available data on objects in the scene.')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)
        logger.info('Component initialized')
        self.local_data = {'jsondump': '{}'}

    @service
    def world_dump(self):
        """ This is a sample (blocking) service (use 'async_service' decorator
        for non-blocking ones).

        Simply returns the value of the internal counter.

        You can access it as a RPC service from clients.
        """
        retq = {}
        for obj in blenderapi.scene().objects:
            if (obj.get('Object', False)) and (not obj.get('Particle', False)):
                name = obj.name
                description = obj.get('Description', '')
                objType = obj.get('Type', '')
                furniture = obj.get('Furniture', False)
                graspable = obj.get('Graspable', False)
                particle = obj.get('Particle', False)
                mesh = obj.get('MeshFile', '')
                transformation = Transformation3d(obj)
                pos = {"x": transformation.translation[0], "y": transformation.translation[1], "z": transformation.translation[2]}
                rot = {"x": transformation.rotation[1], "y": transformation.rotation[2], "z": transformation.rotation[3], "w": transformation.rotation[0]}
                parent = obj.parent
                if parent:
                    parent = parent.name
                else:
                    parent = ""
                retq[name] = {"description": description, "furniture": furniture, "graspable": graspable, "type": objType, "position": pos, "orientation": rot, "mesh": mesh, "parent": parent, "particle": particle}
        self.local_data['jsondump'] = json.dumps(retq)
        return self.local_data['jsondump']

    def default_action(self):
        return
