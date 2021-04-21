from morse.builder.creator import ActuatorCreator
from morse.version import VERSION

class GreatReset(ActuatorCreator):
    _classpath = "abe_sim.actuators.GreatReset.GreatReset"
    def __init__(self, name=None):
        if VERSION < "1.3":
            ActuatorCreator.__init__(self, name, \
                                     "abe_sim.actuators.GreatReset.GreatReset",\
                                     "")
        else:
            ActuatorCreator.__init__(self, name)

