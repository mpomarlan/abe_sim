import math
import numpy

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateTemperatureGetter(name, customDynamicsAPI):
    thermalTimeConstant = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'thermal', 'timeConstant'), 1.0)
    backgroundTemperature, defaultHeatExchangeCoefficient = customDynamicsAPI['getBackgroundTemperature']()
    at = customDynamicsAPI['getObjectProperty']((name,), 'atComponent')
    for l in customDynamicsAPI['getObjectProperty']((name,), 'links'):
        ownTemperature = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'temperature', l), backgroundTemperature)
        if at is not None:
            atTemperature = customDynamicsAPI['getObjectProperty']((at[0],), ('customStateVariables', 'temperature', at[1]), backgroundTemperature)
            targetTemperature = atTemperature
        else:
            targetTemperature = backgroundTemperature
        dt = 1.0/customDynamicsAPI['getSFR']()
        nfr = 1.0/customDynamicsAPI['getFrameStepCount']()
        newTemperature = ownTemperature + (targetTemperature - ownTemperature)*(1-math.exp(-nfr*dt/thermalTimeConstant))
        customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'temperature', l), newTemperature)
