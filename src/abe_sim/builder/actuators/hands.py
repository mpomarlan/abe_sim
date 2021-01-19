from morse.builder.creator import ActuatorCreator

class Hands(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "abe_sim.actuators.hands.Hands",\
                                 "hands")

