from morse.builder.creator import ActuatorCreator
from morse.version import VERSION

class Base(ActuatorCreator):
    _classpath = "abe_sim.actuators.base.Base"
    _blendname = "base"
    def __init__(self, name=None):
        if VERSION < "1.3":
            ActuatorCreator.__init__(self, name, \
                                     "abe_sim.actuators.base.Base",\
                                     "base")
        else:
            ActuatorCreator.__init__(self, name)

