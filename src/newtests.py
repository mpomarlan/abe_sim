import copy
import json
import time
import math
import numpy
import sys

import pybullet
import abe_sim.world as world

from abe_sim.motionPlanning import updateMotionPlanning
from abe_sim.kinematicControl import updateKinematicControl
from abe_sim.grasping import updateGrasping, updateGraspingConstraint
from abe_sim.timing import updateTiming

import abe_sim.customDynamics as cd

customDynamics = cd.buildSpecs('./abe_sim/procdesc.yml') + [[('fn', 'canTime'), updateTiming], [('fn', 'kinematicallyControlable'), updateKinematicControl], [('fn', 'canGrasp'), updateGrasping], [('fn', 'graspingConstraint'), updateGraspingConstraint]]

#customDynamics = cd.buildSpecs('procdesc.log') + [[('fn', 'canMotionPlan'), updateMotionPlanning], [('fn', 'kinematicallyControlable'), updateKinematicControl], [('fn', 'canGrasp'), updateGrasping], [('fn', 'graspingConstraint'), updateGraspingConstraint]]

def addObjectInstance(w, otype, objKnowledge, position, orientation, linearVelocity=None, angularVelocity=None):
    if linearVelocity is None:
        linearVelocity = (0,0,0)
    if angularVelocity is None:
        angularVelocity = (0,0,0)
    newDescription = copy.deepcopy(objKnowledge[otype])
    newDescription['name'] = "%s_%s" % (otype, w.getNewObjectCounter())
    newDescription['position'] = position
    newDescription['orientation'] = orientation
    w.addObject(newDescription)
    return newDescription['name']

def timestepWorld(w, timestep):
    frames = int(timestep*240)
    for k in range(frames):
        s = time.time()
        w.update()
        e = time.time()
        time.sleep(max(0, 1.0/240.0 - e + s))

def preload(w, description, name, position):
    newDescription = copy.deepcopy(description)
    newDescription['name'] = name
    newDescription['position'] = position
    newDescription['orientation'] = [0,0,0,1]
    w.preloadKTree(newDescription)

def makePreloadList(num, otype, pos0, inc):
    return [[otype, "%s_%d" % (otype, k), [pos0[0], pos0[1]+k*inc, pos0[2]]] for k in range(num)]

def getOfType(w, otype):
    for k, v in w._kinematicTrees.items():
        if otype == v['type']:
            return k

def closeQuats(a,b):
    xa = pybullet.rotateVector(a,[1,0,0])
    ya = pybullet.rotateVector(a,[0,1,0])
    xb = pybullet.rotateVector(b,[1,0,0])
    yb = pybullet.rotateVector(b,[0,1,0])
    #print("CQ", numpy.dot(xa,xb), numpy.dot(ya,yb))
    return (0.999 < numpy.dot(xa,xb)) and (0.999 < numpy.dot(ya,yb)), (0.99 < numpy.dot(xa,xb)) and (0.99 < numpy.dot(ya,yb))

def plannedMoveTo(w, name, ef, pos, orn, posInLink=None, ornInLink=None):
    efLink = w.getObjectProperty((name,), ('fn', 'kinematicControl', 'efLink', ef))
    w.setObjectProperty((abe,), ('customStateVariables', 'motionPlanning', 'goal', efLink), [pos, orn])
    w.setObjectProperty((abe,), ('customStateVariables', 'motionPlanning', 'visitedVertices', efLink), {})
    w.setObjectProperty((abe,), ('customStateVariables', 'motionPlanning', 'openVertices', efLink), [])
    w.setObjectProperty((abe,), ('customStateVariables', 'motionPlanning', 'obstacles', efLink), {})
    w.setObjectProperty((abe,), ('customStateVariables', 'motionPlanning', 'startDistance', efLink), {})
    w.setObjectProperty((abe,), ('customStateVariables', 'motionPlanning', 'partialGoal', efLink), None)
    if posInLink is None:
        posInLink = [0,0,0]
    if ornInLink is None:
        ornInLink = [0,0,0,1]
    w.setObjectProperty((name,), ('customStateVariables', 'kinematicControl', 'positionInLink', ef), posInLink)
    w.setObjectProperty((name,), ('customStateVariables', 'kinematicControl', 'orientationInLink', ef), ornInLink)
    while True:
        timestepWorld(w,0.01)
        posL = w.getObjectProperty((name, efLink), 'position')
        ornL = w.getObjectProperty((name, efLink), 'orientation')
        posW, ornW = w.objectPoseRelativeToWorld(posL, ornL, posInLink, ornInLink)
        d = [a-b for a,b in zip(posW, pos)]
        v = w.getObjectProperty((name, efLink), 'linearVelocity')
        #print("DV", numpy.dot(d,d), numpy.dot(v,v))
        cq, acq = closeQuats(ornW,orn)
        if ((0.0002 > numpy.dot(d,d)) and cq) or ((0.001 > numpy.dot(d,d)) and acq and (0.0001 > numpy.dot(v,v))):
            break

def moveTo(w, name, ef, pos, orn, stopOnContact=False, contacter=None, posInLink=None, ornInLink=None):
    w.setObjectProperty((name,), ('customStateVariables', 'kinematicControl', 'target', ef), [pos, orn])
    efLink = w.getObjectProperty((name,), ('fn', 'kinematicControl', 'efLink', ef))
    print("MOVETO", efLink, pos, orn, w.getObjectProperty((name,), ('fn', 'kinematicallyControlable')))
    if contacter is None:
        contacter = name
    if posInLink is None:
        posInLink = [0,0,0]
    if ornInLink is None:
        ornInLink = [0,0,0,1]
    w.setObjectProperty((name,), ('customStateVariables', 'kinematicControl', 'positionInLink', ef), posInLink)
    w.setObjectProperty((name,), ('customStateVariables', 'kinematicControl', 'orientationInLink', ef), ornInLink)
    while True:
        timestepWorld(w,0.01)
        if stopOnContact:
            contacts = w.checkCollision((contacter,))
            if 0 < len(contacts):
                break
        posL = w.getObjectProperty((name, efLink), 'position')
        ornL = w.getObjectProperty((name, efLink), 'orientation')
        posW, ornW = w.objectPoseRelativeToWorld(posL, ornL, posInLink, ornInLink)
        d = [a-b for a,b in zip(posW, pos)]
        v = w.getObjectProperty((name, efLink), 'linearVelocity')
        omega = w.getObjectProperty((name, efLink), 'angularVelocity')
        cq, acq = closeQuats(ornW,orn)
        if ((0.0002 > numpy.dot(d,d)) and cq) or ((0.001 > numpy.dot(d,d)) and acq and (0.0001 > numpy.dot(v,v)) and (0.0001 > numpy.dot(omega,omega))):
            break

#objectTypeKnowledge = [json.loads(x) for x in open('objectknowledge.log').read().splitlines() if x.strip()]
objectTypeKnowledge = json.loads(open('./abe_sim/objectknowledge.json').read())
objectTypeKnowledge = {x['type']: x for x in objectTypeKnowledge}

processKnowledge = json.loads(open('./abe_sim/processknowledge.json').read())
w = world.World(pybulletOptions="--opengl3", useGUI=True, customDynamics=customDynamics, objectKnowledge=objectTypeKnowledge, processKnowledge=processKnowledge)

toPreload = [
        ["CrackedEggshellPointyHalf", "CrackedEggshellPointyHalf_0", [0,-1,-10]],
        ["PeeledPotato", "PeeledPotato_0", [0, -1, -15]]
    ]
toPreload += makePreloadList(5, "EggYolkParticle", [-1,-1,-10], 0.05)
toPreload += makePreloadList(5, "EggWhiteParticle", [-1.1,-1,-10], 0.05)
toPreload += makePreloadList(20, "PotatoParticle", [-1.2,-1,-10], 0.05)
toPreload += makePreloadList(6, "PotatoPeel", [-0.7,-1.2,-10], 0.24)
toPreload += makePreloadList(2, "HalfApple", [-0.7,-1.2,-12], 0.4)
toPreload += makePreloadList(20, "PepperParticle", [-0.7,-1.2,-11], 0.05)

for oType, name, position in toPreload:
    preload(w, objectTypeKnowledge[oType], name, position)

w.setGravity((0,0,-10))

floor = addObjectInstance(w, 'Floor', objectTypeKnowledge, (0,0,0), (0,0,0,1))
kitchenCabinet = addObjectInstance(w, 'KitchenCabinet', objectTypeKnowledge, (0,3,0.15), (0,0,1,0))
kitchenCounter = addObjectInstance(w, 'KitchenCounter', objectTypeKnowledge, (0,-3,0), (0,0,0,1))
fridge = addObjectInstance(w, 'Fridge', objectTypeKnowledge, (-3,0,0.1), (0,0,-0.707,0.707))
aabbKitchenCounter = w.getObjectProperty((kitchenCounter,), 'aabb')

tomato=addObjectInstance(w, 'Tomato', objectTypeKnowledge, (0,0,3), (0,0,0,1))
tomato2=addObjectInstance(w, 'Tomato', objectTypeKnowledge, (0,1,0.5), (0,0,0,1))
tomato3=addObjectInstance(w, 'Tomato', objectTypeKnowledge, (0,1,15.5), (0,0,0,1))
timestepWorld(w,7)

abe = addObjectInstance(w, 'Abe', objectTypeKnowledge, (0,0,0), (0,0,0,1))

timingS = w.getObjectProperty((abe,), ('customStateVariables', 'timing', 'timer'))
timestepWorld(w,1.2)
timingE = w.getObjectProperty((abe,), ('customStateVariables', 'timing', 'timer'))
print(timingS, timingE)

'''
bakingSheet = addObjectInstance(w, 'BakingSheet', objectTypeKnowledge, (0,0,1.5), (0,0,0,1))
bakingTray = addObjectInstance(w, 'BakingTray', objectTypeKnowledge, (0,0,2), (0,0,0,1))
bakingTray = addObjectInstance(w, 'SugarBag', objectTypeKnowledge, (0,0,2.1), (0,0,0,1))
bakingTray = addObjectInstance(w, 'DoughParticle', objectTypeKnowledge, (0,0,2.2), (0,0,0,1))
bakingTray = addObjectInstance(w, 'DoughClump', objectTypeKnowledge, (0,0,2.3), (0,0,0,1))
bakingTray = addObjectInstance(w, 'Cookie', objectTypeKnowledge, (0,0,2.4), (0,0,0,1))
bakingTray = addObjectInstance(w, 'SugarSprinkledCookie', objectTypeKnowledge, (0,0,2.5), (0,0,0,1))
bakingTray = addObjectInstance(w, 'Oven', objectTypeKnowledge, (-5,0,0.5), (0,0,0,1))
'''

input('Press Enter to continue ...')

moveTo(w, abe, 'base', [0,-3,0], [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=[1,0,0], ornInLink=[0,0,0,1])

#moveTo(w, abe, 'base', [0,2.5,0], [0,0,0.707,0.707], stopOnContact=False, contacter=None, posInLink=[1,0,0], ornInLink=[0,0,0,1])
#w.setObjectProperty((abe,), ('customStateVariables', 'motionPlanning', 'goal', 'hand_right_roll'), [[0, 3, 0.5], [0,0,0,1]])
#timestepWorld(w,0.010)
#plannedMoveTo(w, abe, 'hand_right', [0,3,0.5], [0,0,0,1], posInLink=None, ornInLink=None)
#plannedMoveTo(w, abe, 'hand_right', [0,3,1.5], [0,0,0,1], posInLink=None, ornInLink=None)

#w._kinematicTrees[abe]['customStateVariables']['kinematicControl']['target']['hand_right']
#w.getObjectProperty((abe,'hand_right_roll'), 'position')

input('Press Enter to continue ...')


egg = addObjectInstance(w, 'EggInShell', objectTypeKnowledge, (0,-2.7,1), (0.707,0,0,0.707))
aabbEgg = w.getObjectProperty((egg,), 'aabb')
gap = aabbEgg[0][2] - aabbKitchenCounter[1][2]
w.setObjectProperty((egg,), 'position', (0,-2.7,1-gap))
moveTo(w, abe, 'hand_right', [0,-2.7,1-gap+0.13], [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
w.setObjectProperty((abe,), ('customStateVariables', 'grasping', 'intendToGrasp', 'hand_right'), [egg])
moveTo(w, abe, 'hand_right', [0,-2.7,1.6], [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
moveTo(w, abe, 'hand_right', [0,-2.7,1-gap+0.04], [0,0,-0.707,0.707], stopOnContact=True, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
moveTo(w, abe, 'hand_right', [0,-2.7,1-gap+0.13], [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
moveTo(w, abe, 'hand_right', [0,-2.7,1.2], [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
#moveTo(w, abe, 'hand_right', [0,-2.7,1.2], [0.707,0,0,0.707], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
#moveTo(w, abe, 'hand_right', [0,-2.7,1.4], [0.707,0,0,0.707], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])

input('Press Enter to continue ...')

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['CrackedEggshellFlatHalf', 'CrackedEggshellPointyHalf', 'EggWhiteParticle', 'EggYolkParticle']):
        w.removeObject((n,), sendToLimbo=True)

w.setObjectProperty((abe,), ('customStateVariables', 'grasping', 'intendToGrasp', 'hand_right'), [])

cookingKnife = addObjectInstance(w, 'CookingKnife', objectTypeKnowledge, (-0.5,-3,1), (0.707,0,0,0.707))
aabbCookingKnife = w.getObjectProperty((cookingKnife,), 'aabb')
gap = aabbCookingKnife[0][2] - aabbKitchenCounter[1][2]
w.setObjectProperty((cookingKnife,), 'position', (-0.5,-3,1-gap))


positionHandle = list(w.getObjectProperty((cookingKnife, 'handle'), 'position'))
positionHandle[2] += 0.07
moveTo(w, abe, 'hand_right', positionHandle, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
w.setObjectProperty((abe,), ('customStateVariables', 'grasping', 'intendToGrasp', 'hand_right'), [cookingKnife])
positionHandle[2] += 0.9
moveTo(w, abe, 'hand_right', positionHandle, [0.707,0,0,0.707], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
positionBlade, orientationBlade = w.objectPoseRelativeToObject(w.getObjectProperty((abe, 'hand_right_roll'), 'position'), w.getObjectProperty((abe, 'hand_right_roll'), 'orientation'), w.getObjectProperty((cookingKnife, 'blade'), 'position'), w.getObjectProperty((cookingKnife, 'blade'), 'orientation'))

potato = addObjectInstance(w, 'Potato', objectTypeKnowledge, (0,-3,1), (0,0,0,1))
aabbPotato = w.getObjectProperty((potato,), 'aabb')
gap = aabbPotato[0][2] - aabbKitchenCounter[1][2]
w.setObjectProperty((potato,), 'position', (0,-2.7,1-gap))

positionPotato = list(w.getObjectProperty((potato,), 'position'))
positionPotato[2] += 0.25
moveTo(w, abe, 'hand_left', positionPotato, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
positionPotato[2] -= 0.1
moveTo(w, abe, 'hand_left', positionPotato, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
w.setObjectProperty((abe,), ('customStateVariables', 'grasping', 'intendToGrasp', 'hand_left'), [potato])
positionPotato[2] += 0.3
positionPotato[0] += 0.5
moveTo(w, abe, 'hand_left', positionPotato, [-0.707,0,0,0.707], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])

aabbPotato = w.getObjectProperty((potato,), 'aabb')
aabbBlade = w.getObjectProperty((cookingKnife, 'blade'), 'aabb')
bladeWidth = (aabbBlade[1][1] - aabbBlade[0][1])*0.5

positionTarget = [aabbPotato[0][0], aabbPotato[0][1]-bladeWidth+0.02, aabbPotato[1][2]]
positionBlade, orientationBlade = w.objectPoseRelativeToObject(w.getObjectProperty((abe, 'hand_right_roll'), 'position'), w.getObjectProperty((abe, 'hand_right_roll'), 'orientation'), w.getObjectProperty((cookingKnife, 'blade'), 'position'), w.getObjectProperty((cookingKnife, 'blade'), 'orientation'))
moveTo(w, abe, 'hand_right', positionTarget, [0.707,0,0,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget = [aabbPotato[0][0], aabbPotato[0][1]-bladeWidth+0.02, aabbPotato[0][2]]
moveTo(w, abe, 'hand_right', positionTarget, [0.707,0,0,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget = [aabbPotato[0][0], aabbPotato[0][1]-bladeWidth+0.02, aabbPotato[1][2]]
moveTo(w, abe, 'hand_right', positionTarget, [0.707,0,0,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget[0] -= 0.7
positionTarget[1] -= 0.15
moveTo(w, abe, 'hand_right', positionTarget, [0.707,0,0,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])

input('Press Enter to continue ...')

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['PotatoPeel']):
        w.removeObject((n,), sendToLimbo=True)

moveTo(w, abe, 'hand_left', positionPotato, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
positionPotato[2] -= 0.2 
moveTo(w, abe, 'hand_left', positionPotato, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
w.setObjectProperty((abe,), ('customStateVariables', 'grasping', 'intendToGrasp', 'hand_left'), [])
positionPotato[2] += 0.4 
moveTo(w, abe, 'hand_left', positionPotato, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])

masher = addObjectInstance(w, 'Masher', objectTypeKnowledge, (0.1,-3,1), (0,0,-0.707,0.707))
aabbMasher = w.getObjectProperty((masher,), 'aabb')
gap = aabbMasher[0][2]-aabbKitchenCounter[1][2]
w.setObjectProperty((masher,), 'position', (0.1,-3,1-gap))
positionHandle = list(w.getObjectProperty((masher, 'handle'), 'position'))
positionMasher = list(w.getObjectProperty((masher, 'masher'), 'position'))
orientationMasher = list(w.getObjectProperty((masher, 'masher'), 'orientation'))
positionTarget = [positionHandle[0], positionHandle[1], positionHandle[2]+0.05]
moveTo(w, abe, 'hand_left', positionTarget, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
posMH, ornMH = w.objectPoseRelativeToObject(w.getObjectProperty((abe, 'hand_left_roll'), 'position'), w.getObjectProperty((abe, 'hand_left_roll'), 'orientation'), w.getObjectProperty((masher, 'masher'), 'position'), w.getObjectProperty((masher, 'masher'), 'orientation'))
w.setObjectProperty((abe,), ('customStateVariables', 'grasping', 'intendToGrasp', 'hand_left'), [masher])
positionTarget[1] += 0.2
positionTarget[2] += 0.3
moveTo(w, abe, 'hand_left', positionTarget, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
positionTarget[2] += 0.3
moveTo(w, abe, 'hand_left', positionTarget, [0,0.707,0,0.707], stopOnContact=False, contacter=None, posInLink=posMH, ornInLink=ornMH)
positionTarget = list(w.getObjectProperty((potato,), 'position'))
positionTarget[2] += 0.9
moveTo(w, abe, 'hand_left', positionTarget, [0,0.707,0,0.707], stopOnContact=False, contacter=None, posInLink=posMH, ornInLink=ornMH)
positionTarget[2] -= 0.75
positionTarget[1] -= 0.02
moveTo(w, abe, 'hand_left', positionTarget, [0,0.707,0,0.707], stopOnContact=False, contacter=None, posInLink=posMH, ornInLink=ornMH)

timestepWorld(w,1)

input('Press Enter to continue ...')

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['PotatoParticle', 'PotatoPeel', 'Masher']):
        w.removeObject((n,), sendToLimbo=True)


avocado = addObjectInstance(w, 'Avocado', objectTypeKnowledge, (0, -3, 1), (0,0,0,1))
aabbAvocado = w.getObjectProperty((avocado,), 'aabb')
gap = aabbAvocado[0][2] - aabbKitchenCounter[1][2]
w.setObjectProperty((avocado,), 'position', (0, -3, 1-gap))
positionTarget = list(w.getObjectProperty((avocado,), 'position'))
aabbAvocado = w.getObjectProperty((avocado,), 'aabb')
positionTarget[2] = aabbAvocado[1][2] + 0.06
auxP = list(w.getObjectProperty((abe, 'hand_left_roll'), 'position'))
auxP[2] += 0.02
moveTo(w, abe, 'hand_left', auxP, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
moveTo(w, abe, 'hand_left', auxP, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
moveTo(w, abe, 'hand_left', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
w.setObjectProperty((abe,), ('customStateVariables', 'grasping', 'intendToGrasp', 'hand_left'), [avocado])
positionTarget[2] += 0.2
moveTo(w, abe, 'hand_left', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)

positionTarget = list(w.getObjectProperty((avocado,), 'position'))
positionTarget[0] -= 0.5
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget[0] += 0.4
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget[0] -= 0.25
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget[0] += 0.25
positionTarget[2] -= 0.03
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget[1] -= 0.2
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget[1] += 0.2
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget[1] -= 0.2
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget[1] += 0.2
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
positionTarget[1] -= 0.2
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=[0,0,0,1])
timestepWorld(w, 2)

input('Press Enter to continue ...')

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['Avocado', 'AvocadoHalf', 'AvocadoHalfSeeded', 'AvocadoSeed']):
        w.removeObject((n,), sendToLimbo=True)

bread = addObjectInstance(w, 'Bread', objectTypeKnowledge, (0,-3,1), (0,0,1,0))
aabbBread = w.getObjectProperty((bread,), 'aabb')
gap = aabbBread[0][2] - aabbKitchenCounter[1][2]
w.setObjectProperty((bread,), 'position', (0, -3, 1-gap))

positionL = list(w.getObjectProperty((abe, 'hand_left_roll'), 'position'))
positionL[0] += 0.4
orientationL = list(w.getObjectProperty((abe, 'hand_left_roll'), 'orientation'))
moveTo(w, abe, 'hand_left', positionL, orientationL, stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)

positionR = list(w.getObjectProperty((abe, 'hand_right_roll'), 'position'))
positionR[0] -= 0.4
orientationR = list(w.getObjectProperty((abe, 'hand_right_roll'), 'orientation'))
moveTo(w, abe, 'hand_right', positionR, orientationR, stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
positionR[2] += 0.2
moveTo(w, abe, 'hand_right', positionR, orientationR, stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
orientationR = [-0.5,0.5,-0.5,0.5]
moveTo(w, abe, 'hand_right', positionR, orientationR, stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
breadPosition = w.getObjectProperty((bread,), 'position')
positionR = [breadPosition[0], positionR[1]+0.2, positionR[2]]
moveTo(w, abe, 'hand_right', positionR, orientationR, stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
aabbBread = w.getObjectProperty((bread,), 'aabb')
aabbKnife = w.getObjectProperty((cookingKnife,), 'aabb')
gap = aabbKnife[0][2] - aabbBread[1][2]
positionR[2] -= gap - 0.01
moveTo(w, abe, 'hand_right', positionR, orientationR, stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
while True:
    positionR[1] -= 0.12
    moveTo(w, abe, 'hand_right', positionR, orientationR, stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
    positionR[1] += 0.12
    moveTo(w, abe, 'hand_right', positionR, orientationR, stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
    found = False
    for treeK, treeD in w._kinematicTrees.items():
        if 'BreadSlice' == treeD['type']:
            found = True
    if found:
        break

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['Bread']):
        w.removeObject((n,), sendToLimbo=True)

positionTarget[2] += 0.5
moveTo(w, abe, 'hand_left', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
butter = addObjectInstance(w, 'Butter', objectTypeKnowledge, (-0.6,-3,1), (0,0,0,1))
aabbButter = w.getObjectProperty((butter,), 'aabb')
gap = aabbButter[0][2] - aabbKitchenCounter[1][2]
w.setObjectProperty((butter,), 'position', (-0.6,-3,1-gap))
aabbButter = w.getObjectProperty((butter,), 'aabb')
positionTarget = list(w.getObjectProperty((butter,), 'position'))

aabbBlade = w.getObjectProperty((cookingKnife, 'blade'), 'aabb')
bladeWidth = (aabbBlade[1][2] - aabbBlade[0][2])*0.5
positionTarget[2] = aabbButter[1][2] + bladeWidth
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=orientationBlade)
positionTarget[0] += 0.1
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=orientationBlade)

breadSlice = None
for n in w._kinematicTrees:
    if 'BreadSlice' == w._kinematicTrees[n]['type']:
        breadSlice = n
        break

positionTarget = list(w.getObjectProperty((breadSlice,), 'position'))
aabbBreadSlice = w.getObjectProperty((breadSlice,), 'aabb')
positionTarget[0] = aabbBreadSlice[0][0]
positionTarget[2] = aabbBreadSlice[1][2] + bladeWidth
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=orientationBlade)
positionTarget[0] = aabbBreadSlice[1][0]
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=orientationBlade)
positionTarget[0] = aabbBreadSlice[0][0]
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=orientationBlade)
positionTarget[0] = aabbBreadSlice[0][0]-0.2
moveTo(w, abe, 'hand_right', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=positionBlade, ornInLink=orientationBlade)

positionTarget = list(w.getObjectProperty((breadSlice,), 'position'))
positionTarget[0] += 0.7
positionTarget[2] = 1
pepperShaker = addObjectInstance(w, 'PepperShaker', objectTypeKnowledge, positionTarget, (0,0,0,1))
aabbPepperShaker = w.getObjectProperty((pepperShaker,), 'aabb')
gap = aabbPepperShaker[0][2] - aabbKitchenCounter[1][2]
positionTarget[2] -= gap
w.setObjectProperty((pepperShaker,), 'position', positionTarget)
positionTarget = list(w.getObjectProperty((pepperShaker,), 'position'))
positionTarget[1] += 0.15
positionTarget[2] += 0.04
moveTo(w, abe, 'hand_left', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
w.setObjectProperty((abe,), ('customStateVariables', 'grasping', 'intendToGrasp', 'hand_left'), [pepperShaker])
positionShaker, orientationShaker = w.objectPoseRelativeToObject(w.getObjectProperty((abe, 'hand_left_roll'), 'position'), w.getObjectProperty((abe, 'hand_left_roll'), 'orientation'), w.getObjectProperty((pepperShaker, 'cap'), 'position'), w.getObjectProperty((pepperShaker, 'cap'), 'orientation'))
positionTarget[2] += 0.3
moveTo(w, abe, 'hand_left', positionTarget, [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)
positionTarget = list(w.getObjectProperty((breadSlice,), 'position'))
positionTarget[2] += 0.1
moveTo(w, abe, 'hand_left', positionTarget, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=positionShaker, ornInLink=orientationShaker)
positionTarget[2] += 0.1
moveTo(w, abe, 'hand_left', positionTarget, [0,1,0,0], stopOnContact=False, contacter=None, posInLink=positionShaker, ornInLink=orientationShaker)
timestepWorld(w,2)
positionTarget[2] += 0.3
moveTo(w, abe, 'hand_left', positionTarget, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=positionShaker, ornInLink=orientationShaker)
moveTo(w, abe, 'hand_left', positionTarget, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=positionShaker, ornInLink=orientationShaker)
moveTo(w, abe, 'hand_left', positionTarget, [0,0,0,1], stopOnContact=False, contacter=None, posInLink=positionShaker, ornInLink=orientationShaker)

input('Press Enter to continue ...')
sys.exit()

