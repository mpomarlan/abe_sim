import math
import numpy

from abe_sim.world import getDictionaryEntry

def updateMingling(name, customDynamicsAPI):
    at = customDynamicsAPI['getObjectProperty']((name,), 'atComponent')
    fnMingling = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'mingling'), {})
    csvMingling = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'mingling'), {})
    nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
    aabb = customDynamicsAPI['getObjectProperty']((name,), 'aabb')
    aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMingling, ('minglableRadius',), 1.0))
    closeObjects = set([x[0] for x in customDynamicsAPI['checkOverlap'](aabbAdj)])
    isMingling = False
    for closeObject in closeObjects:
        if not customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'canMingle'), False):
            continue
        fnMingler = customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'mingling'), {})
        minglerLinks = getDictionaryEntry(fnMingler, ('links',), [])
        for minglerLink in minglerLinks:
            minglerRadius = getDictionaryEntry(fnMingler, ('radius', minglerLink), 0.0)
            if minglerRadius > customDynamicsAPI['getDistance']((name,), (closeObject, minglerLink), minglerRadius):
                isMingling = True
                break
        if isMingling:
            break
    if isMingling:
        hp = getDictionaryEntry(csvMingling, ('hp',), 0) - 1
        if 0 < hp:
            nameP = customDynamicsAPI['getObjectProperty']((name,), 'position')
            minglerP = customDynamicsAPI['getObjectProperty']((closeObject, minglerLink), 'position')
            dx = nameP[0] - minglerP[0]
            dy = nameP[1] - minglerP[1]
            norm = math.sqrt(dx*dx+dy*dy)
            if 0.1 < norm:
                dx = 0.1*dx/norm
                dy = 0.07*dy/norm
            vel = [-dy, dx, 0]
            customDynamicsAPI['setObjectProperty']((), 'linearVelocity', vel)
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'mingling', 'mingling'), True)
            customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'mingling', 'hp'), hp)
    else:
        customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'mingling', 'mingling'), False)

def updateMixing(name, customDynamicsAPI):
    def setStr(l):
        l = sorted(list(l))
        return ';'.join(l)
    at = customDynamicsAPI['getObjectProperty']((name,), 'atComponent')
    csvMixing = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'mixing'), {})
    fnMixing = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'mixing'), {})
    nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
    aabb = customDynamicsAPI['getObjectProperty']((name,), 'aabb')
    aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMixing, ('mixableRadius',), 1.0))
    closeObjects = set([x[0] for x in customDynamicsAPI['checkOverlap'](aabbAdj)])
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
        if isMixing:
            break
    if isMixing:
        aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMixing, ('substanceRadius',), 0))
        closeParticles = set([x[0] for x in customDynamicsAPI['checkOverlap'](aabbAdj)])
        closeParticles = [x for x in closeParticles if at == customDynamicsAPI['getObjectProperty']((x,), 'atComponent')]
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
                customDynamicsAPI['concludeProcess']({'process': 'mixing', 'patient': setStr(bestMatch)})

