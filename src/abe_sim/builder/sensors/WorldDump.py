from morse.builder.creator import SensorCreator
from morse.version import VERSION

class WorldDump(SensorCreator):
    _classpath = "abe_sim.sensors.WorldDump.WorldDump"
    _blendname = "WorldDump"
    def __init__(self, name=None):
        if VERSION < "1.3":
            SensorCreator.__init__(self, name, \
                                   "abe_sim.sensors.WorldDump.WorldDump",\
                                   "WorldDump")
        else:
            SensorCreator.__init__(self, name)

