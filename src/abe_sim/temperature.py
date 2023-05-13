import math
import numpy

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateTemperatureGetter(name, customDynamicsAPI):
    targetTemperature, heatExchangeCoefficient = customDynamicsAPI['getBackgroundTemperature']()
    ownTemperature = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'temperature'), targetTemperature)
    at = customDynamicsAPI['getObjectProperty']((name,), 'at')
    if at is not None:
        atTemperature = customDynamicsAPI['getObjectProperty']((at,), ('customStateVariables', 'temperature'))
        if atTemperature is not None:
            targetTemperature = atTemperature
    newTemperature = ownTemperature + heatExchangeCoefficient*(targetTemperature - ownTemperature)
    customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'temperature'), newTemperature)

