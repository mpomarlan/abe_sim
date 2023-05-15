import math
import numpy

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateTemperatureGetter(name, customDynamicsAPI):
    backgroundTemperature, heatExchangeCoefficient = customDynamicsAPI['getBackgroundTemperature']()
    at = customDynamicsAPI['getObjectProperty']((name,), 'atComponent')
    for l in customDynamicsAPI['getObjectProperty']((name,), 'links'):
        ownTemperature = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'temperature', l), backgroundTemperature)
        if at is not None:
            atTemperature = customDynamicsAPI['getObjectProperty']((at[0],), ('customStateVariables', 'temperature', at[1]), backgroundTemperature)
            targetTemperature = atTemperature
        else:
            targetTemperature = backgroundTemperature
        newTemperature = ownTemperature + heatExchangeCoefficient*(targetTemperature - ownTemperature)
        customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'temperature', l), newTemperature)

