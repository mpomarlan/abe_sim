import logging; logger = logging.getLogger("morse." + __name__)

from morse.core.actuator import Actuator
from morse.helpers.components import add_data, add_property
from morse.core import mathutils

from morse.core.services import service, async_service
from morse.core import status

from morse.core import blenderapi
from morse.helpers.transformation import Transformation3d

import ast
import json

import GameLogic as logic
import math

class GreatReset(Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "GreatReset"
    _short_desc = "(Re)Sets object positions, velocities, properties etc."

    # define here the data fields required by your actuator
    # format is: field name, initial value, type, description
    add_data('state', '', 'string', 'Data about world state')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        self._raw = ""
        self._pkg = 0
        self._whitelist = {}
        self._blacklist = {"aggregate"}
        # Call the constructor of the parent class
        Actuator.__init__(self, obj, parent)
        logger.info('Component initialized')
    def _getObjectVector(self, obj, entry, key, defVec):
        retq = mathutils.Vector(defVec)
        if key in entry:
            for i,k in [(0, 'x'), (1, 'y'), (2, 'z')]:
                if k in entry[key]:
                    retq[i] = entry[key][k]
        return retq
    def setObject(self, obj, entry, oMap):
        furniture = False
        active = False
        for propName in entry["props"].keys():
            if propName in self._blacklist:
                continue
            opropName = propName.capitalize()
            if "meshfile" == propName:
                opropName = "MeshFile"
            obj[propName.capitalize()] = entry["props"][propName]
            if "furniture" == propName:
                furniture = entry["props"][propName]
            if "active" == propName:
                active = entry["props"][propName]
        parent = ""
        if 'parent' in entry:
            parent = entry['parent']
            if parent in oMap:
                parent = oMap[parent]
        if parent:
            obj.setParent(parent)
            obj.suspendDynamics()
        else:
            obj.removeParent()
            if (not furniture) and active:
                obj.restoreDynamics()
        #obj.setLinearVelocity(mathutils.Vector([entry["velocity"]["x"], entry["velocity"]["y"], entry["velocity"]["z"]]))
        #obj.setAngularVelocity(mathutils.Vector([entry["angular_velocity"]["x"], entry["angular_velocity"]["y"], entry["angular_velocity"]["z"]]))
        #obj.worldPosition = mathutils.Vector([entry["position"]["x"], entry["position"]["y"], entry["position"]["z"]])
        if obj.getPhysicsId():
            obj.setLinearVelocity(self._getObjectVector(obj, entry, "velocity", obj.worldLinearVelocity))
            obj.setAngularVelocity(self._getObjectVector(obj, entry, "angular_velocity", obj.worldAngularVelocity))
        obj.worldPosition = self._getObjectVector(obj, entry, "position", obj.worldPosition)
        obj.worldOrientation = mathutils.Quaternion([entry["orientation"]["w"], entry["orientation"]["x"], entry["orientation"]["y"], entry["orientation"]["z"]])

    def default_action(self):
        return

    @service
    def great_reset(self, filepath):
        """ This is a sample (blocking) service (use 'async_service' decorator
        for non-blocking ones).

        """
        state = open(filepath).read()
        #return json.dumps({'status': 'ok'})
        self.local_data['state'] = state
        data = json.loads(self.local_data['state'])
        oMap = {}
        for obj in blenderapi.scene().objects:
            name = obj.name
            oMap[name] = obj
        for obj in blenderapi.scene().objects:
            name = obj.name
            if (name in data) and ("particles" not in data[name]):
                self.setObject(obj, data[name], oMap)
        for ik, entry in data.items():
            if "particles" in entry:
                for name, ent in entry["particles"].items():
                    self.setObject(oMap[name], ent, oMap)
        return json.dumps({'status': 'ok'})


