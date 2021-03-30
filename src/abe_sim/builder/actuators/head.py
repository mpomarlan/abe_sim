from morse.builder.creator import ActuatorCreator
from morse.version import VERSION

class Head(ActuatorCreator):
    _classpath = "abe_sim.actuators.head.Head"
    _blendname = "head"
    def __init__(self, name=None):
        if VERSION < "1.3":
            ActuatorCreator.__init__(self, name, \
                                     "abe_sim.actuators.head.Head",\
                                     "head")
        else:
            ActuatorCreator.__init__(self, name)

