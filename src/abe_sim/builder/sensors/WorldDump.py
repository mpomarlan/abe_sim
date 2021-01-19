from morse.builder.creator import SensorCreator

class WorldDump(SensorCreator):
    def __init__(self, name=None):
        SensorCreator.__init__(self, name, \
                               "abe_sim.sensors.WorldDump.WorldDump",\
                               "WorldDump")

