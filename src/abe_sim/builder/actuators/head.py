from morse.builder.creator import ActuatorCreator

class Head(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "abe_sim.actuators.head.Head",\
                                 "head")

