import math
import numpy

import pybullet

from abe_sim.world import getDictionaryEntry

def updateClopening(name, customDynamicsAPI):
    fnClopening = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'clopening'), {})
    clopenableLinks = getDictionaryEntry(fnClopening, ('clopenableLinks',), [])
    for link in clopenableLinks:
        clopeningRadius = getDictionaryEntry(fnClopening, ('radius', link), 0.2)
        handleLink = getDictionaryEntry(fnClopening, ('handle', link), None)
        joint = customDynamicsAPI['getObjectProperty']((name, link), 'parent')
        if joint is None:
            continue
        maxVelocity = getDictionaryEntry(fnClopening, ('maxVelocity', link), None)
        positionGain = getDictionaryEntry(fnClopening, ('positionGain', link), None)
        velocityGain = getDictionaryEntry(fnClopening, ('velocityGain', link), None)
        aabb = customDynamicsAPI['getObjectProperty']((name, handleLink), 'aabb')
        aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, clopeningRadius)
        closeObjects = customDynamicsAPI['checkOverlap'](aabbAdj)
        action = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'clopening', 'action', link), None)
        for candidate in closeObjects:
            if customDynamicsAPI['getObjectProperty']((candidate,), ('fn', 'canClopen'), False):
                for clopenerEF in customDynamicsAPI['getObjectProperty']((candidate,), ('fn', 'clopening', 'clopeningEFs'), []):
                    actionCandidate = customDynamicsAPI['getObjectProperty']((candidate,), ('customStateVariables', 'clopening', 'action', clopenerEF), None)
                    if (action is None) or (actionCandidate is not None):
                        action = actionCandidate
        customDynamicsAPI['setObjectProperty']((name,), ('customStateVariables', 'clopening', 'action', link), action)
        angle = None
        if 'open' == action:
            angle = getDictionaryEntry(fnClopening, ('openingAngle', link), 0)
        elif 'close' == action:
            angle = getDictionaryEntry(fnClopening, ('closingAngle', link), 0)
        if angle is not None:
            customDynamicsAPI['applyJointControl'](joint, mode='position', targetPosition=angle, maxVelocity=maxVelocity, positionGain=positionGain, velocityGain=velocityGain)

