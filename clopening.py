import math
import numpy

import pybullet

from world import getDictionaryEntry

def isAClopenableThing(getFn):
    return getFn(('fn', 'clopenable'), None) is True

def updateClopening(name, customDynamicsAPI):
    csvClopening = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'clopening'), {})
    fnClopening = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'clopening'), {})
    clopenableLinks = getDictionaryEntry(fnClopening, ('clopenableLinks',), [])
    closeObjects = customDynamicsAPI['checkOverlap'](customDynamicsAPI['getObjectProperty']((name,), 'aabb'))
    for link in clopenableLinks:
        clopeningRadius = getDictionaryEntry(fnClopening, ('clopeningRadius', link), 0)
        clopeningAxis = getDictionaryEntry(fnClopening, ('clopeningAxis', link), 0)
        handleLink = getDictionaryEntry(fnClopening, ('handleLink', link), (0,0,0))
        position = customDynamicsAPI['getObjectProperty']((name, handleLink), 'position')
        orientation = customDynamicsAPI['getObjectProperty']((name, handleLink), 'orientation')
        clopeningAxisWorld = pybullet.rotateVector(orientation, clopeningAxis)
        joint = customDynamicsAPI['getObjectProperty']((name, link), 'parent')
        maxVelocity = getDictionaryEntry(fnClopening, ('maxVelocity', link), None)
        positionGain = getDictionaryEntry(fnClopening, ('positionGain', link), None)
        velocityGain = getDictionaryEntry(fnClopening, ('velocityGain', link), None)
        if joint is None:
            continue
        for candidate in closeObjects:
            csvClopener = customDynamicsAPI['getObjectProperty']((candidate,), ('customStateVariables', 'clopening'), {})
            fnClopener = customDynamicsAPI['getObjectProperty']((candidate,), ('fn', 'clopening'), {})
            if getDictionaryEntry(fnClopener, ('canClopen',), False):
                for clopenerLink in getDictionaryEntry(fnClopener, ('clopeningLinks',), []):
                    angle = None
                    if getDictionaryEntry(csvClopener, ('opening', clopenerLink,), False):
                        if clopeningRadius > customDynamicsAPI['getDistance']((name, link), (candidate, clopenerLink), clopeningRadius):
                            angle = getDictionaryEntry(fnClopening, ('openingAngle', link), 0)
                    elif getDictionaryEntry(csvClopener, ('closing', clopenerLink,), False):
                        if clopeningRadius > customDynamicsAPI['getDistance']((name, link), (candidate, clopenerLink), clopeningRadius):
                            angle = getDictionaryEntry(fnClopening, ('closingAngle', link), 0)
                    if angle is not None:
                        customDynamicsAPI['applyJointControl'](joint, mode='position', targetPosition=angle, maxVelocity=maxVelocity, positionGain=positionGain, velocityGain=velocityGain)

