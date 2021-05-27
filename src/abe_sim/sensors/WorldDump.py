import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

from morse.core import blenderapi
from morse.helpers.transformation import Transformation3d

import json

import GameLogic as logic
import math

class CCComputer:
    def __init__(self, ccMap):
        self.ccMap = ccMap
    def find(self, item):
        if self.ccMap[item] == item:
            return item
        else:
            res = self.find(self.ccMap[item])
            self.ccMap[item] = res
            return res
    def link(self, item1, item2):
        root1 = self.find(item1)
        root2 = self.find(item2)
        self.ccMap[root1] = root2

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
    def world_dump(self, fullDump):
        """ This is a sample (blocking) service (use 'async_service' decorator
        for non-blocking ones).

        Simply returns the value of the internal counter.

        You can access it as a RPC service from clients.
        """
        retq = {}
        ccMap = {}
        oMap = {}
        dMap = {}
        for obj in blenderapi.scene().objects:
            if (obj.get('Object', False)) or (fullDump and obj.get('Robot', True)):
                name = obj.name
                props = {}
                for propName in obj.getPropertyNames():
                    props[propName.lower()] = obj.get(propName)
                transformation = Transformation3d(obj)
                pos = {"x": transformation.translation[0], "y": transformation.translation[1], "z": transformation.translation[2]}
                rot = {"x": transformation.rotation[1], "y": transformation.rotation[2], "z": transformation.rotation[3], "w": transformation.rotation[0]}
                vel = obj.getLinearVelocity()
                vel = {"x": vel[0], "y": vel[1], "z": vel[2]}
                aVel = obj.getAngularVelocity()
                aVel = {"x": aVel[0], "y": aVel[1], "z": aVel[2]}
                parent = obj.parent
                if parent:
                    parent = parent.name
                else:
                    parent = ""
                if (not obj.get('Particle', False)):
                    if fullDump and obj.get('Robot', False):
                        retq[name] = {"parent": parent, "position": pos, "orientation": rot, "velocity": vel, "angular_velocity": aVel, "props": {}}
                    else:
                        retq[name] = {"parent": parent, "position": pos, "orientation": rot, "velocity": vel, "angular_velocity": aVel, "props": props}
                elif (obj.get('Particle', False)):
                    ccMap[name] = name
                    oMap[name] = obj
                    dMap[name] = {"parent": parent, "position": pos, "orientation": rot, "velocity": vel, "angular_velocity": aVel, "props": props}
        persistentData = logic.globalDict
        cccomputer = CCComputer(ccMap)
        pMap = {}
        for k in ccMap.keys():
            for n in persistentData['neighborhoods'][k]:
                if n in ccMap.keys():
                    cccomputer.link(n, k)
        clumps = {}
        for k in ccMap.keys():
            pk = cccomputer.find(k)
            if pk not in pMap:
                pMap[pk] = []
            pMap[pk].append(k)
            if pk not in clumps:
                transformation = Transformation3d(oMap[pk])
                clumps[pk] = {"name": pk, "pos": [transformation.translation[0], transformation.translation[1], transformation.translation[2]], "num": 1, "temperature": oMap[pk].get('Temperature', 0.0), "type": set([oMap[pk].get('Substance', '')]), "parts": [pk]}
            else:
                transformation = Transformation3d(oMap[pk])
                pos = clumps[pk]["pos"]
                clumps[pk]["pos"][0] = clumps[pk]["pos"][0] + transformation.translation[0]
                clumps[pk]["pos"][1] = clumps[pk]["pos"][1] + transformation.translation[1]
                clumps[pk]["pos"][2] = clumps[pk]["pos"][2] + transformation.translation[2]
                clumps[pk]["num"] = clumps[pk]["num"] + 1
                clumps[pk]["temperature"] = clumps[pk]["temperature"] + oMap[pk].get('Temperature', 0.0)
                clumps[pk]["type"].add(oMap[pk].get('Substance', ''))
                clumps[pk]["parts"].append(pk)
        for k in clumps.keys():
            clumps[k]["pos"] = [clumps[k]["pos"][0]/clumps[k]["num"], clumps[k]["pos"][1]/clumps[k]["num"], clumps[k]["pos"][2]/clumps[k]["num"]]
            clumps[k]["temperature"] = clumps[k]["temperature"]/clumps[k]["num"]
            clumps[k]["type"] = str(sorted(list(clumps[k]["type"])))
            minD = None
            minP = None
            for p in clumps[k]["parts"]:
                transformation = Transformation3d(oMap[pk])
                dx = clumps[k]["pos"][0] - transformation.translation[0]
                dy = clumps[k]["pos"][1] - transformation.translation[1]
                dz = clumps[k]["pos"][2] - transformation.translation[2]
                d = dx*dx+dy*dy+dz*dz
                if (None == minD) or (minD > d):
                    minD = d
                    minP = p
            transformation = Transformation3d(oMap[minP])
            clumps[k]["pos"] = [transformation.translation[0], transformation.translation[1], transformation.translation[2]]
            clumps[k]["name"] = minP
        for k, c in clumps.items():
            pname = c["name"]
            particles = {name: dMap[name] for name in pMap[k]}
            parent = oMap[pname].parent
            if parent:
                parent = parent.name
            else:
                parent = ""
            transformation = Transformation3d(oMap[pname])
            pos = {"x": transformation.translation[0], "y": transformation.translation[1], "z": transformation.translation[2]}
            props = {}
            for propName in oMap[pname].getPropertyNames():
                props[propName.lower()] = oMap[pname].get(propName)
            props["particle"] = False
            props["meshfile"] = "PAggregate.stl"
            props["type"] = c["type"]
            props["temperature"] = c["temperature"]
            if fullDump:
                retq[pname] = {"position": pos, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "props": props, "particles": particles}
            else:
                retq[pname] = {"particle": False, "parent": parent, "position": pos, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "props": props}
        self.local_data['jsondump'] = json.dumps(retq)
        return self.local_data['jsondump']

    def default_action(self):
        return
