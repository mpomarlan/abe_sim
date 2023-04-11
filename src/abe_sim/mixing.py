import math
import numpy

from abe_sim.world import getDictionaryEntry

def isAMixableThing(getFn):
    return getFn(('fn', 'mixable'), None) is True

def updateMixing(name, customDynamicsAPI):
    def setStr(l):
        l = sorted(list(l))
        return ';'.join(l)
    at = customDynamicsAPI['getObjectProperty']((name,), 'at')
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
        mixerLinks = getDictionaryEntry(fnMixer, ('links',), [])
        for mixerLink in mixerLinks:
            mixerRadius = getDictionaryEntry(fnMixer, ('radius', mixerLink), 0.0)
            if mixerRadius > customDynamicsAPI['getDistance']((name,), (closeObject, mixerLink), mixerRadius):
                isMixing = True
                break
                #v = customDynamicsAPI['getObjectProperty']((closeObject, mixerLink), 'linearVelocity')
                #w = customDynamicsAPI['getObjectProperty']((closeObject, mixerLink), 'angularVelocity')
                #if getDictionaryEntry(fnMixing, ('minMixerVelocity',), 1.0) < math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]) + math.sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]):
                #    isMixing = True
                #    break
        if isMixing:
            break
    if isMixing:
        aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMixing, ('substanceRadius',), 0))
        closeParticles = [x for x in customDynamicsAPI['checkOverlap'](aabbAdj) if at == customDynamicsAPI['getObjectProperty']((x,), 'at')]
        neighboringTypes = set([nameType])
        for particle in closeParticles:
            pt = customDynamicsAPI['getObjectProperty']((particle,), 'type')
            if customDynamicsAPI['getObjectProperty']((particle,), ('fn', 'mixable'), False) or customDynamicsAPI['getObjectProperty']((particle,), ('fn', 'mixMakeable'), False):
                neighboringTypes.add(pt)
        bestMatch = None
        for ingredientList in customDynamicsAPI['getProcessResource']({'process': 'mixing', 'patient': nameType}):
            if 0 == len(set(ingredientList).difference(neighboringTypes)):
                if (bestMatch is None) or (len(bestMatch) < len(ingredientList)):
                    bestMatch = ingredientList
        if bestMatch is not None:
            hp = getDictionaryEntry(csvMixing, ('hp',), 0) - 1
            if 0 < hp:
                customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'mixing', 'hp'), hp)
            else:
                #aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMixing, ('substanceRadius',), 0))
                #closeParticles = [x for x in customDynamicsAPI['checkOverlap'](aabbAdj) if at == customStateVariables['getObjectProperty']((x,), 'at')]
                #neighboringTypes = set([nameType])
                #neighboringMixinTypes = set([])
                #for particle in closeParticles:
                #    pt = customDynamicsAPI['getObjectProperty']((particle,), 'type')
                #    if customDynamicsAPI['getObjectProperty']((particle,), ('fn', 'mixable'), False) or customDynamicsAPI['getObjectProperty']((particle,), ('fn', 'mixMakeable'), False):
                #        neighboringTypes.add(pt)
                        #if getDictionaryEntry(customDynamicsAPI['getObjectProperty']((particle,), ('fn', 'mixing', 'canMixIn'), False)):
                        #    neighboringMixinTypes.add(pt)
                        #else:
                        #    neighboringTypes.add(pt)
                #for pt in neighboringMixinTypes:
                #    outcome = customDynamicsAPI['getProcessOutcome']({'process': 'mixing', 'patient': nameType, 'mixInto': pt})
                #    if isinstance(outcome, dict) and 'toReplace' in outcome:
                #        customDynamicsAPI['concludeProcess']({'process': 'mixing', 'patient': nameType, 'mixInto': pt})
                #neighboringTypes = ';'.join(sorted(list(neighboringTypes)))
                customDynamicsAPI['concludeProcess']({'process': 'mixing', 'patient': setStr(bestMatch)})

