from morse.builder.creator import ActuatorCreator

class Base(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "abe_sim.actuators.base.Base",\
                                 "base")

