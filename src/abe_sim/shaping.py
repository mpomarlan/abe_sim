import copy
import math
import numpy
import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateShaped(name, customDynamicsAPI):
    aabb = customDynamicsAPI['getObjectProperty']((name,), 'aabb')
    aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, 1.0)
    closeObjects = set([x[0] for x in customDynamicsAPI['checkOverlap'](aabbAdj)])
    for e in closeObjects:
        if name in customDynamicsAPI['getObjectProperty']((e,), ('customStateVariables', 'provenance'), []):
            customDynamicsAPI['removeObject']()
            return

def updateShaping(name, customDynamicsAPI):
    fnShaping = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'shaping'))
    csvShaping = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'shaping'))
    for a in getDictionaryEntry(fnShaping, ('actuators',), []):
        shapedType = getDictionaryEntry(csvShaping, ('outcome', a), None)
        ingredients = set(getDictionaryEntry(csvShaping, ('ingredients', a), []))
        if (shapedType is None) or (0 == len(ingredients)):
            continue
        radius = getDictionaryEntry(fnShaping, ('radius', a), 0)
        handLink = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl', 'efLink', a))
        aabb = customDynamicsAPI['getObjectProperty']((name, handLink), 'aabb')
        aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, radius)
        closeObjects = set([x[0] for x in customDynamicsAPI['checkOverlap'](aabbAdj)])
        if 0 == len(ingredients.difference(closeObjects)):
            handP = customDynamicsAPI['getObjectProperty']((name, handLink), 'position')
            handQ = customDynamicsAPI['getObjectProperty']((name, handLink), 'orientation')
            axis = stubbornTry(lambda : pybullet.rotateVector(handQ, [0,0,-0.07]))
            objDesc = customDynamicsAPI['getObjectTypeKnowledge'](shapedType)
            if 'customStateVariables' not in objDesc:
                objDesc['customStateVariables'] = {}
            objDesc['customStateVariables']['provenance'] = list(ingredients)
            objDesc['orientation'] = [0,0,0,1]
            objDesc['position'] = [handP[0] + axis[0], handP[1] + axis[1], handP[2] + axis[2]]
            objDesc['name'] = "%s_%d" % (shapedType, customDynamicsAPI['getNewObjectCounter']())
            customDynamicsAPI['addObject'](objDesc)
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'shaping', 'ingredients', a), [])
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'shaping', 'outcome', a), None)
