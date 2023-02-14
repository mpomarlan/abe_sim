import math
import numpy

from world import getDictionaryEntry

from world import getDictionaryEntry

def isAConsumableThing(getFn):
    return getFn(('fn', 'consumable'), None) is True

def updateConsumable(name, customDynamicsAPI):
    aabb = customDynamicsAPI['getObjectProperty']((name,), 'aabb')
    aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMixing, ('shapeableRadius',), 1.0))
    closeObjects = customDynamicsAPI['checkOverlap'](aabbAdj)
    for closeObject in closeObjects:
        if not customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'produceable'), False):
            continue
        if name in customDynamicsAPI['getObjectProperty']((closeObject,), ('fn', 'producing', 'provenance'), []):
            nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
            customDynamicsAPI['concludeProcess']({'process': 'consuming', 'patient': nameType})

def isAShaperThing(getFn):
    return getFn(('fn', 'canShape'), None) is True

def updateShaper(name, customDynamicsAPI):
    csvShaping = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'shaping'), {})
    fnShaping = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'shaping'), {})
    shapingLinks = getDictionaryEntry(csvShaping, ('shapingLinks',), [])
    for shapingLink in shapingLinks:
        outcomeType = getDictionaryEntry(csvShaping, ('desiredOutcome', shapingLink), '')
        shapingRadius = getDictionaryEntry(fnShaping, ('shapingRadius', shapingLink), 1.0)
        resources = customDynamicsAPI['getProcessResource']({'process': 'shaping', 'patient': outcomeType})
        aabb = customDynamicsAPI['getObjectProperty']((name, shapingLink), 'aabb')
        aabbAdj = customDynamicsAPI['adjustAABBRadius'](aabb, getDictionaryEntry(fnMixing, ('shapingRadius',), 1.0))
        closeObjects = customDynamicsAPI['checkOverlap'](aabbAdj)
        available = {}
        for closeObject in closeObjects:
            cot = customDynamicsAPI['getObjectProperty']((closeObject,), 'type')
            if cot in resources:
                if cot not in available:
                    available[cot] = []
                available[cot].append(closeObject)
        if 0 == len(set(resources).difference(set(available.keys()))):
            requiredAmounts = getDictionaryEntry(csvShaping, ('requiredAmounts', shapingLink), {})
            provenance = []
            for resType, resCount in requiredAmounts.items():
                if resType in available:
                    provenance = provenance + available[resType][:min(resCount, len(available[resType]))]
            nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
            result = customDynamicsAPI['concludeProcess']({'process': 'shaping', 'patient': outcomeType}, link=shapingLink, adjustments={'toAdd': {outcomeType: {'fn': {'producing': {'provenance': provenance}}}}})
            csvShaping['shapingLinks'].remove(shapingLink)
            csvShaping['desiredOutcome'].pop(shapingLink)
            csvShaping['requiredAmounts'].pop(shapingLink)
            customDynamicsAPI['setObjectProperty'](('customStateVariables', 'shaping'), csvShaping)
            intendToGrasp = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'grasping', 'intendToGrasp', shapingLink), [])
            intendToGrasp.append(result['added'][0])
            customDynamicsAPI['setObjectProperty'](('customStateVariables', 'grasping', 'intendToGrasp', shapingLink), intendToGrasp)

