import math
import numpy

from abe_sim.world import getDictionaryEntry

def updateTiming(name, customDynamicsAPI):
    timing = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'timing', 'timer'), 0.0) + 1.0/240.0
    customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'timing', 'timer'), timing)

