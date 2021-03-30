from morse.builder.creator import ActuatorCreator
from morse.version import VERSION

class Hands(ActuatorCreator):
    _classpath = "abe_sim.actuators.hands.Hands"
    _blendname = "hands"
    def __init__(self, name=None):
        if VERSION < "1.3":
            ActuatorCreator.__init__(self, name, \
                                     "abe_sim.actuators.hands.Hands",\
                                     "hands")
        else:
            ActuatorCreator.__init__(self, name)

