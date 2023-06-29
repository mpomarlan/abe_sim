import math
import numpy

from abe_sim.world import getDictionaryEntry

def updateTiming(name, customDynamicsAPI):
    timing = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'timing', 'timer'), 0.0) + customDynamicsAPI['getFrameStepCount']()/(1.0*customDynamicsAPI['getSFR']())
    customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'timing', 'timer'), timing)

