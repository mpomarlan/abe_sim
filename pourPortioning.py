import math
import numpy

import pybullet

from world import getDictionaryEntry

def isAPourableThing(getFn):
    return getFn(('fn', 'pourable'), None) is True

def updatePouring(name, customDynamicsAPI):
    csvPouring = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'pourPortioning'), {})
    fnPouring = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'pourPortioning'), {})
    identifiers = [(name,)]
    pouringLinks = getDictionaryEntry(fnPouring, ('pouringLinks',), None)
    if pouringLinks is not None:
        if isintance(pouringLinks, str):
            identifiers = [(name, pouringLinks)]
        else:
            identifiers = [(name, x) for x in pouringLinks]
    for identifier in identifiers:
        orientation = customDynamicsAPI['getObjectProperty'](identifier, 'orientation')
        dripAxis = getDictionaryEntry(fnPouring, ('dripAxis',), (1,0,0))
        dripPoint = getDictionaryEntry(fnPouring, ('dripPoint',), (0,0,0))
        pouringThreshold = getDictionaryEntry(fnPouring, ('pouringT',), (1,0,0))
        down = customDynamicsAPI['getDown']()
        dripAxisWorld = pybullet.rotateVector(orientation, dripAxis)
        dotProduct = dripAxisWorld[0]*down[0] + dripAxisWorld[1]*down[1] + dripAxisWorld[2]*down[2]
        if pouringThreshold < dotProduct:
            if 1 == len(identifier):
                identifier = tuple([identifier[0], customDynamicsAPI['getObjectProperty']((name,), 'baseLinkName')])
            pourPortioningDelay = getDictionaryEntry(csvPouring, ('pourPortioningDelay', identifier[1]), 0) - 1
            if 0 >= pourPortioningDelay:
                nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
                customDynamicsAPI['concludeProcess']({'process': 'pourPortioning', 'patient': nameType}, link=identifier[1])
                pourPortioningDelay = getDictionaryEntry(fnPouring, ('maxDelay',), 0)
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'pourPortioning', 'pourPortioningDelay', identifier[1]), pourPortioningDelay)


