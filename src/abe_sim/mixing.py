import math
import numpy

from abe_sim.world import getDictionaryEntry

def isAMixableThing(getFn):
    return getFn(('fn', 'mixable'), None) is True

def updateMixing(name, customDynamicsAPI):
    def setStr(l):
        l = sorted(list(l))
        return ';'.join(l)
    csvMixing = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'mixing'), {})
    fnMixing = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'mixing'), {})
    nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
    aabb = customDynamicsAPI['getObjectProperty']((name,), 'aabb')
    aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMixing, ('mixableRadius',), 1.0))
    closeObjects = customDynamicsAPI['checkOverlap'](aabbAdj)
    isMixing = False
    for closeObject in closeObjects:
        if not customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'canMix'), False):
            continue
        fnMixer = customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'mixing'), {})
        mixerRadius = getDictionaryEntry(fnMixer, ('mixerRadius',), 0.0)
        mixerLinks = getDictionaryEntry(fnMixer, ('mixerLinks',), '')
        for mixerLink in mixerLinks:
            if mixerRadius > customDynamicsAPI['getDistance']((name,), (closeObject, mixerLink), mixerRadius):
                v = customDynamicsAPI['getObjectProperty']((closeObject, mixerLink), 'linearVelocity')
                w = customDynamicsAPI['getObjectProperty']((closeObject, mixerLink), 'angularVelocity')
                if getDictionaryEntry(fnMixing, ('minMixerVelocity',), 1.0) < math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]) + math.sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]):
                    isMixing = True
                    break
        if isMixing:
            break
    if isMixing:
        hp = getDictionaryEntry(csvMixing, ('hp',), 0) - 1
        if 0 < hp:
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'mixing', 'hp'), hp)
        else:
            aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMixing, ('substanceRadius',), 0))
            closeParticles = customDynamicsAPI['checkOverlap'](aabbAdj)
            neighboringTypes = set([])
            neighboringMixinTypes = set([])
            for particle in closeParticles:
                pt = customDynamicsAPI['getObjectProperty']((particle,), 'type')
                if customDynamicsAPI['getObjectProperty']((particle,), ('fn', 'mixable'), False):
                    if getDictionaryEntry(customDynamicsAPI['getObjectProperty']((particle,), ('fn', 'mixing', 'canMixIn'), False)):
                        neighboringMixinTypes.add(pt)
                    else:
                        neighboringTypes.add(pt)
            for pt in neighboringMixinTypes:
                outcome = customDynamicsAPI['getProcessOutcome']({'process': 'mixing', 'patient': nameType, 'mixInto': pt})
                if isinstance(outcome, dict) and 'toReplace' in outcome:
                    customDynamicsAPI['concludeProcess']({'process': 'mixing', 'patient': nameType, 'mixInto': pt})
            neighboringTypes = ';'.join(sorted(list(neighboringTypes)))
            customDynamicsAPI['concludeProcess']({'process': 'mixing', 'patient': setStr()})

