import math
import numpy

from world import getDictionaryEntry

def updateGraspingConstraint(name, customDynamicsAPI):
    parent = customDynamicsAPI['getObjectProperty']((name,), 'parent', '')
    parentLink = customDynamicsAPI['getObjectProperty']((name,), 'parentLink', '')
    child = customDynamicsAPI['getObjectProperty']((name,), 'child', '')
    childLink = customDynamicsAPI['getObjectProperty']((name,), 'childLink', '')
    parentActuallyGrasps = customDynamicsAPI['getObjectProperty']((parent,), ('customStateVariables', 'grasping', 'actuallyGrasping', parentLink), [])
    #if [[child, childLink], name] not in parentActuallyGrasps:
    if [[child], name] not in parentActuallyGrasps:
        customDynamicsAPI['removeObject']()

def updateGrasping(name, customDynamicsAPI):
    def _toIdentifier(x):
        if isinstance(x, str):
            return (x,)
        return tuple(x)
    csvGrasping = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'grasping'), {})
    fnGrasping = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'grasping'), {})
    intendedGrasped = csvGrasping['intendToGrasp']
    actuallyGrasped = csvGrasping['actuallyGrasping']
    efs = getDictionaryEntry(fnGrasping, ('effectors',), [])
    newActuallyGrasped = {}
    for ef in efs:
        graspingActivationRadius = getDictionaryEntry(fnGrasping, ('graspingActivationRadius', ef), 0.1)
        graspingDeactivationRadius = getDictionaryEntry(fnGrasping, ('graspingDeactivationRadius', ef), 0.2)
        maxForce = getDictionaryEntry(fnGrasping, ('maxForce', ef), 1000)
        intendedSet = set([_toIdentifier(x) for x in intendedGrasped.get(ef, [])])
        actuallyGraspedEF = {_toIdentifier(x[0]): x[1] for x in actuallyGrasped.get(ef, [])}
        dists = {}
        toRemove = []
        for e in actuallyGraspedEF.keys():
        # check whether actually grasped items are still intended; if not, remove from grasping and destroy the constraint
            if (e not in intendedSet) or (not customDynamicsAPI['getObjectProperty']((e[0],), ('fn', 'graspable'), False)):
                toRemove.append(e)
        for e in toRemove:
            actuallyGraspedEF.pop(e)
        toRemove = []
        for e in actuallyGraspedEF.keys():
        # check whether actually grasped items have drifted out of grasping range; if so, remove from grasping and destroy the constraint
            if graspingDeactivationRadius < customDynamicsAPI['getDistance']((name, ef), e, graspingDeactivationRadius):
                toRemove.append(e)
        for e in toRemove:
            actuallyGraspedEF.pop(e)
        for e in intendedSet:
            if not customDynamicsAPI['getObjectProperty']((e[0],), ('fn', 'graspable'), False):
                continue
        # check whether intended grasped items are not actually grasped, but close enough; if so, add a constraint to make them grasped
            if ((e not in actuallyGraspedEF) or (customDynamicsAPI['getObjectProperty']((actuallyGraspedEF[e],), 'name') is None)) and (graspingActivationRadius > customDynamicsAPI['getDistance']((name, ef), e, graspingDeactivationRadius)):
                child = e[0]
                if 1 == len(e):
                    childLink = customDynamicsAPI['getObjectProperty'](e, 'baseLinkName')
                else:
                    childLink = e[1]
                constraintName = ('GRASPING_%s_%s_%s_%s' % (name, ef, child, childLink))
                customDynamicsAPI['addObject']({'fn': {'graspingConstraint': True}, 'name': constraintName, 'simtype': 'kcon', 'parent': name, 'child': child, 'parentLink': ef, 'childLink': childLink, 'jointType': 'fixed', 'maxForce': maxForce})
                actuallyGraspedEF[e] = constraintName
        newActuallyGrasped[ef] = sorted([[list(k), v] for k, v in actuallyGraspedEF.items()])
    customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'grasping', 'actuallyGrasping'), newActuallyGrasped)

