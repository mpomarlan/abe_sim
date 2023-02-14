import math
import numpy

from world import getDictionaryEntry

def isABakeableThing(getFn):
    return getFn(('fn', 'bakeable'), None) is True

def updateBaking(name, customDynamicsAPI):
    csvBaking = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'baking'), {})
    hp = getDictionaryEntry(csvBaking, ('hp',), 0)
    bakingIntensity = 0
    baker = None
    cr = name
    while cr is not None:
        if customDynamicsAPI['getObjectProperty']((cr,), ('customStateVariables', 'baking', 'isBaking'), False):
            bakingIntensity = customDynamicsAPI['getObjectProperty']((cr,), ('customStateVariables', 'baking', 'bakingIntensity'), 0)
        if 0 < bakingIntensity:
            baker = cr
            break
        cr = customDynamicsAPI['getObjectProperty']((cr,), 'at', None)
    if 0 < bakingIntensity:
        hp -= bakingIntensity
    if 0 >= hp:
        nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
        bakerType = customDynamicsAPI['getObjectProperty']((baker,), 'type')
        customDynamicsAPI['concludeProcess']({'process': 'baking', 'patient': nameType, 'instrument': bakerType})
    else:
        customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'baking', 'hp'), hp)
