import math
import numpy

import pybullet

import copy

from world import getDictionaryEntry

def isAMashableThing(getFn):
    return getFn(('fn', 'mashable'), None) is True

def updateMashing(name, customDynamicsAPI):
    csvMashing = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'mashing'), {})
    fnMashing = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'mashing'), {})
    nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
    linearVelocity = customDynamicsAPI['getObjectProperty']((name,), 'linearVelocity')
    mashingVelocity = getDictionaryEntry(fnMashing, ('masherVelocity',), 0)
    aabb = customDynamicsAPI['getObjectProperty']((name,), 'aabb')
    # TODO: fix this need for a hack
    #aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMashing, ('mashableRadius',), 1.0))
    aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, 0.2)
    closeObjects = customDynamicsAPI['checkOverlap'](aabbAdj)
    isMashing = False
    for closeObject in closeObjects:
        if not customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'canMash'), False):
            continue
        for link in customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'mashing', 'links')):
            masherVelocity = customDynamicsAPI['getObjectProperty']((closeObject, link), 'linearVelocity')
            masherOrientation = customDynamicsAPI['getObjectProperty']((closeObject, link), 'orientation')
            masherAxis = customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'mashing', 'axis', link), None)
            if masherAxis is None:
                masherAxis = customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'longitudinalAxis'), (1,0,0))
            masherAxis = pybullet.rotateVector(masherOrientation, masherAxis)
            masherToMashed = [x-y for x,y in zip(masherVelocity,linearVelocity)]
            if mashingVelocity < numpy.dot(masherToMashed, masherAxis):
                isMashing = True
                break
        if isMashing:
            break
    if isMashing:
        hp = getDictionaryEntry(csvMashing, ('hp',), 0) - 1
        if 0 < hp:
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'mashing', 'hp'), hp)
        else:
            customDynamicsAPI['concludeProcess']({'process': 'mashing', 'patient': nameType})

