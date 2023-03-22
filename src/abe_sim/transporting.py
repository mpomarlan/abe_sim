import math
import numpy
import pybullet

from abe_sim.world import getDictionaryEntry

def updateTransportingConstraint(name, customDynamicsAPI):
    child = customDynamicsAPI['getObjectProperty']((name,), 'child', '')
    transportedBy = customDynamicsAPI['getObjectProperty']((child,), ('fn', 'transporting', 'constraint'), None)
    if transportedBy != name:
        customDynamicsAPI['removeObject']()

def updateTransporting(name, customDynamicsAPI):
    def ornOrientation(a, b, th):
        ax = stubbornTry(lambda : pybullet.rotateVector(a, [1,0,0]))
        ay = stubbornTry(lambda : pybullet.rotateVector(a, [0,1,0]))
        bx = stubbornTry(lambda : pybullet.rotateVector(b, [1,0,0]))
        by = stubbornTry(lambda : pybullet.rotateVector(b, [0,1,0]))
        return (th < numpy.dot(ax, bx)) and (th < numpy.dot(ay, by))
    transportedBy = None
    at = customDynamicsAPI['getObjectProperty']((name,), 'at')
    atGraspable = customDynamicsAPI['getObjectProperty']((at,), ('fn', 'graspable'), False)
    relativeStillness = False
    if atGraspable:
        atVelocity = customDynamicsAPI['getObjectProperty']((at,), 'linearVelocity')
        nameVelocity = customDynamicsAPI['getObjectProperty']((name,), 'linearVelocity')
        dv = [x-y for x,y in zip(atVelocity, nameVelocity)]
        dv = numpy.dot(dv, dv)
        if 0.00001 > dv:
            relativeStillness = True
    if relativeStillness:
        atOrientation = customDynamicsAPI['getObjectProperty']((at,), 'orientation')
        tippedOrientation = customDynamicsAPI['getObjectProperty']((at,), ('fn', 'containment', 'pouring', 'outof', 'tipped'))
        tipped = ornDistance(atOrientation, tippedOrientation, 0.95)
        # is this itself grasped? : if so, remove
        if not tipped:
            aabb = customDynamicsAPI['getObjectProperty']((name,), 'aabb')
            graspingOverlaps = [x for x in set(customDynamicsAPI['checkOverlap'](aabb)) if customDynamicsAPI['getObjectProperty']((x,), ('fn', 'canGrasp'), False)]
            grasped = False
            for x in graspingOverlaps:
                for _, grasps in customDynamicsAPI['getObjectProperty']((x,), ('customStateVariables', 'grasping', 'actuallyGrasping'), {}).items():
                    for grasp in grasps:
                        if name == grasp[0][0]:
                            grasped = True
                            break
                    if grasped:
                        break
                if grasped:
                    break
            if not grasped:
                transportedBy = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'transporting', 'constraint'), None)
                if transportedBy is None:
                    atBaseLink = customDynamicsAPI['getObjectProperty']((at,), 'baseLinkName')
                    nameBaseLink = customDynamicsAPI['getObjectProperty']((name,), 'baseLinkName')
                    transportedBy = ('GRASPING_%s_%s_%s_%s' % (at, atBaseLink, name, nameBaseLink))
                    maxForce = customDynamicsAPI['getObjectProperty']((at,), ('fn', 'transporting', 'maxForce'), 1000)
                    customDynamicsAPI['addObject']({'fn': {'transportingConstraint': True}, 'name': transportedBy, 'simtype': 'kcon', 'parent': at, 'child': name, 'parentLink': atBaseLink, 'childLink': nameBaseLink, 'jointType': 'fixed', 'maxForce': maxForce})
    customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'transporting', 'constraint'), transportedBy)
