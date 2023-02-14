import math
import numpy

import copy

import pybullet

from world import getDictionaryEntry
from geom_utils import vector2Axis

def isASeedableThing(getFn):
    return getFn(('fn', 'seedable'), False) is True

def updateSeeding(name, customDynamicsAPI):
    csvSeeding = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'seeding'), {})
    fnSeeding = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'seeding'), {})
    seedableLinks = getDictionaryEntry(fnSeeding, ('links',), [])
    isSeeding = False
    for link in seedableLinks:
        position = customDynamicsAPI['getObjectProperty']((name, link), 'position')
        linearVelocity = customDynamicsAPI['getObjectProperty']((name, link), 'linearVelocity')
        aabb = customDynamicsAPI['getObjectProperty']((name, link), 'aabb')
        aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnSeeding, ('seedableRadius',), 1.0))
        closeObjects = customDynamicsAPI['checkOverlap'](aabbAdj)
        for closeObject in closeObjects:
            if not (customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'canSeed'), False) or customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'canCut'), False)):
                continue
            seederLinks = customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'seeding', 'links'), [])
            if [] == seederLinks:
                seederLinks = customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'cutting', 'links'), [])
            for seederLink in seederLinks:
                seederAxis = customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'seeding', 'axis', seederLink), None)
                if seederAxis is None:
                    seederAxis = customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'cutting', 'axis', seederLink), (0,0,1))
                seederVelocity = customDynamicsAPI['getObjectProperty']((closeObject,seederLink), 'linearVelocity')
                seederPosition = customDynamicsAPI['getObjectProperty']((closeObject,seederLink), 'position')
                seederOrientation = customDynamicsAPI['getObjectProperty']((closeObject,seederLink), 'orientation')
                seederAxisInWorld = pybullet.rotateVector(seederOrientation, seederAxis)
                seederToSeededVelocityDirection = vector2Axis([x-y for x,y in zip(seederVelocity,linearVelocity)])
                seederToSeededDisplacementDirection = vector2Axis([x-y for x,y in zip(position, seederPosition)])
                if (0.2 > abs(numpy.dot(seederToSeededVelocityDirection, seederAxisInWorld))) and (0.2 > abs(numpy.dot(seederToSeededVelocityDirection, seederToSeededDisplacementDirection))):
                    isSeeding = True
                    break
            if isSeeding:
                break
        if isSeeding:
            break
    if isSeeding:
        hp = getDictionaryEntry(csvSeeding, ('hp',), 0) - 1
        if 0 < hp:
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'seeding', 'hp'), hp)
        else:
            nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
            customDynamicsAPI['concludeProcess']({'process': 'seeding', 'patient': nameType})

