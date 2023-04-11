import math
import numpy
import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

numpy.set_printoptions(precision=3)

def updateStickiness(name, customDynamicsAPI):
    at = customDynamicsAPI['getObjectProperty']((name,), 'at')
    if at is None:
        return None
    mass = customDynamicsAPI['getObjectProperty']((name,), 'mass')
    stickiness = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'stickiness', 'stickiness'), 0.93)
    position = customDynamicsAPI['getObjectProperty']((name,), 'position')
    velocity = customDynamicsAPI['getObjectProperty']((name,), 'linearVelocity')
    force = [-mass*240*stickiness*velocity[0], -mass*240*stickiness*velocity[1], 0]
    vp = numpy.array(velocity)
    fp = numpy.array(force)/mass
    customDynamicsAPI['applyExternalForce']((name,), force, position, inWorldFrame=True)

