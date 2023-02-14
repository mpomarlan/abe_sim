import math
import numpy

import copy

from geom_utils import vector2Axis

import pybullet

from world import getDictionaryEntry

def isAPeelableThing(getFn):
    return getFn(('fn', 'peelable'), None) is True

def updatePeeling(name, customDynamicsAPI):
    csvPeeling = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'peeling'), {})
    fnPeeling = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'peeling'), {})
    position = customDynamicsAPI['getObjectProperty']((name,), 'position')
    linearVelocity = customDynamicsAPI['getObjectProperty']((name,), 'linearVelocity')
    aabb = customDynamicsAPI['getObjectProperty']((name,), 'aabb')
    aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnPeeling, ('peelableRadius',), 1.0))
    closeObjects = customDynamicsAPI['checkOverlap'](aabbAdj)
    isPeeling = False
    for closeObject in closeObjects:
        if not (customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'canPeel'), False) or customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'canCut'), False)):
            continue
        fnPeeler = customDynamicsAPI['getObjectProperty']((closeObject,), ('fn',))
        if 'peeling' in fnPeeler:
            spec = fnPeeler['peeling']
        elif 'cutting' in fnPeeler:
            spec= fnPeeler['cutting']
        for link in spec.get('links', []):
            peelerVelocity = customDynamicsAPI['getObjectProperty']((closeObject, link), 'linearVelocity')
            _, _, peelerPosition, _, _, _ = customDynamicsAPI['checkClosestPoints']((closeObject, link), (name,))[0]
            peelerOrientation = customDynamicsAPI['getObjectProperty']((closeObject, link), 'orientation')
            peelerAxis = getDictionaryEntry(spec, ('axis', link), (0,1,0))
            peelerAxis = pybullet.rotateVector(peelerOrientation, peelerAxis)
            peelerToPeeledVelocityDirection = vector2Axis([x-y for x,y in zip(peelerVelocity,linearVelocity)])
            peelerToPeeledDisplacementDirection = vector2Axis([x-y for x,y in zip(position, peelerPosition)])
            if (0.95 < numpy.dot(peelerToPeeledVelocityDirection, peelerAxis)) and (0.2 > abs(numpy.dot(peelerToPeeledVelocityDirection, peelerToPeeledDisplacementDirection))):
                isPeeling = True
                break
        if isPeeling:
            break
    if isPeeling:
        hp = getDictionaryEntry(csvPeeling, ('hp',), 0) - 1
        if 0 < hp:
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'peeling', 'hp'), hp)
        else:
            nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
            customDynamicsAPI['concludeProcess']({'process': 'peeling', 'patient': nameType})

