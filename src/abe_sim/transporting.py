import math
import numpy
import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateTransportingConstraint(name, customDynamicsAPI):
    child = customDynamicsAPI['getObjectProperty']((name,), 'child', '')
    transportedBy = customDynamicsAPI['getObjectProperty']((child,), ('customStateVariables', 'transporting', 'constraint'), None)
    if transportedBy != name:
        customDynamicsAPI['removeObject']()

def updateTransporting(name, customDynamicsAPI):
    def ornCloseness(a, b, th):
        ax = stubbornTry(lambda : pybullet.rotateVector(a, [1,0,0]))
        ay = stubbornTry(lambda : pybullet.rotateVector(a, [0,1,0]))
        bx = stubbornTry(lambda : pybullet.rotateVector(b, [1,0,0]))
        by = stubbornTry(lambda : pybullet.rotateVector(b, [0,1,0]))
        return (th < numpy.dot(ax, bx)) and (th < numpy.dot(ay, by))
    transportedBy = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'transporting', 'constraint'), None)
    newTransportedBy = None
    at = customDynamicsAPI['getObjectProperty']((name,), 'atComponent')
    if (at is None) and (transportedBy is not None):
        at = (customDynamicsAPI['getObjectProperty']((transportedBy,), 'parent'), customDynamicsAPI['getObjectProperty']((transportedBy,), 'parentLink'))
    if at is None:
        return
    atGraspable = customDynamicsAPI['getObjectProperty']((at[0],), ('fn', 'graspable'), False)
    relativeStillness = atGraspable
    if atGraspable and (transportedBy is None):
        atVelocity = customDynamicsAPI['getObjectProperty'](at, 'linearVelocity')
        nameVelocity = customDynamicsAPI['getObjectProperty']((name,), 'linearVelocity')
        dv = [x-y for x,y in zip(atVelocity, nameVelocity)]
        dv = numpy.dot(dv, dv)
        if 0.0001 < dv:
            relativeStillness = False
    mingling = customDynamicsAPI['getObjectProperty']((name,), ('customDynamicsAPI', 'mingling', 'mingling'), False)
    if relativeStillness and (not mingling):
        atOrientation = customDynamicsAPI['getObjectProperty']((at[0],), 'orientation')
        pourAxisInAt = customDynamicsAPI['getObjectProperty']((at[0],), ('fn', 'containment', 'pouring', 'outof', 'axis'), (0,1,0))
        pourAxis = stubbornTry(lambda : pybullet.rotateVector(atOrientation, pourAxisInAt))
        down = customDynamicsAPI['getDown']()
        tipped = bool(0.9 < numpy.dot(down, pourAxis))
        # is this itself grasped? : if so, remove
        if not tipped:
            childOf = customDynamicsAPI['getObjectProperty']((name,), 'childOf')
            grasped = False
            for x in childOf:
                parent = customDynamicsAPI['getObjectProperty']((x,), 'parent')
                for _, grasps in customDynamicsAPI['getObjectProperty']((parent,), ('customStateVariables', 'grasping', 'actuallyGrasping'), {}).items():
                    for grasp in grasps:
                        if name == grasp[0][0]:
                            grasped = True
                            break
                    if grasped:
                        break
                if grasped:
                    break
            if not grasped:
                newTransportedBy = transportedBy
                if newTransportedBy is None:
                    nameBaseLink = customDynamicsAPI['getObjectProperty']((name,), 'baseLinkName')
                    newTransportedBy = ('GRASPING_%s_%s_%s_%s' % (at[0], at[1], name, nameBaseLink))
                    maxForce = customDynamicsAPI['getObjectProperty']((at[0],), ('fn', 'transporting', 'maxForce'), 1000)
                    customDynamicsAPI['addObject']({'fn': {'transportingConstraint': True}, 'name': newTransportedBy, 'simtype': 'kcon', 'parent': at[0], 'child': name, 'parentLink': at[1], 'childLink': nameBaseLink, 'jointType': 'fixed', 'maxForce': maxForce})
    customDynamicsAPI['setObjectProperty']((), ('customStateVariables', 'transporting', 'constraint'), newTransportedBy)

