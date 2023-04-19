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

def placeOnTable(w, table, otype, objKnowledge, position, orientation, letStabilize=False):
    retq = addObjectInstance(w, otype, objectTypeKnowledge, position, orientation)
    aabb = w.getObjectProperty((retq,), 'aabb')
    aabbT = w.getObjectProperty((table,), 'aabb')
    gap = aabb[0][2] - aabbT[1][2]
    w.setObjectProperty((retq,), 'position', (position[0], position[1], position[2] - gap))
    if letStabilize:
        timeStepWorld(2)
    return retq
    
def grabItem(w, agent, item, hand, handTQ, handle=None):
    handLink = {'hand_right': 'hand_right_roll', 'hand_left': 'hand_left_roll'}[hand]
    handP = w.getObjectProperty((agent, handLink), 'position')
    handQ = w.getObjectProperty((agent, handLink), 'orientation')
    moveTo(w, agent, hand, [handP[0], handP[1], 1.9], handQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    moveTo(w, agent, hand, [handP[0], handP[1], 1.9], handTQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    if handle is None:
        handle = w.getObjectProperty((item,), 'baseLinkName')
    aabb = w.getObjectProperty((item, handle), 'aabb')
    handleP = w.getObjectProperty((item, handle), 'position')
    moveTo(w, agent, hand, [handleP[0], handleP[1], 1.9], handTQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    moveTo(w, agent, hand, [handleP[0], handleP[1], aabb[1][2]+0.1], handTQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    w.setObjectProperty((agent,), ('customStateVariables', 'grasping', 'intendToGrasp', hand), [item])

def releaseItem(w, agent, table, item, hand, handTQ, itemTP):
    handLink = {'hand_right': 'hand_right_roll', 'hand_left': 'hand_left_roll'}[hand]
    handP = w.getObjectProperty((agent, handLink), 'position')
    handQ = w.getObjectProperty((agent, handLink), 'orientation')
    itemP = w.getObjectProperty((item,), 'position')
    itemQ = w.getObjectProperty((item,), 'orientation')
    itemHP, itemHQ = w.objectPoseRelativeToObject(handP, handQ, itemP, itemQ)
    moveTo(w, agent, 'hand_left', [handP[0], handP[1], 1.9], handQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    moveTo(w, agent, 'hand_left', [handP[0], handP[1], 1.9], handTQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    moveTo(w, agent, 'hand_left', [itemTP[0], itemTP[1], 1.9], handTQ, stopOnContact=False, contacter=None, posInLink=itemHP, ornInLink=[0,0,0,1])
    aabb = w.getObjectProperty((item,), 'aabb')
    aabbT = w.getObjectProperty((table,), 'aabb')
    TZ = itemTP[2] - (aabb[0][2] - aabbT[1][2]) + 0.04
    moveTo(w, agent, 'hand_left', [itemTP[0], itemTP[1], TZ], handTQ, stopOnContact=True, contacter=None, posInLink=itemHP, ornInLink=[0,0,0,1])
    w.setObjectProperty((agent,), ('customStateVariables', 'grasping', 'intendToGrasp', hand), [])
    moveTo(w, agent, 'hand_left', [itemTP[0], itemTP[1], 1.9], handTQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    
def liftHand(w, agent, hand, height, handTQ):
    handLink = {'hand_right': 'hand_right_roll', 'hand_left': 'hand_left_roll'}[hand]
    handP = w.getObjectProperty((agent, handLink), 'position')
    handQ = w.getObjectProperty((agent, handLink), 'orientation')
    moveTo(w, agent, hand, [handP[0], handP[1], height], handQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    moveTo(w, agent, hand, [handP[0], handP[1], height], handTQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])

def hammerMotion(w, agent, hand, positionLow):
    handLink = {'hand_right': 'hand_right_roll', 'hand_left': 'hand_left_roll'}[hand]
    handP = w.getObjectProperty((agent, handLink), 'position')
    handQ = w.getObjectProperty((agent, handLink), 'orientation')
    moveTo(w, agent, hand, [handP[0], handP[1], 1.9], handQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    moveTo(w, agent, hand, [positionLow[0], positionLow[1], 1.9], handQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    moveTo(w, agent, hand, positionLow, handQ, stopOnContact=True, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    moveTo(w, agent, hand, [positionLow[0], positionLow[1], 1.9], handQ, stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    
def peelManeuver(w, agent, hand, item, posInLink):
    itemP = w.getObjectProperty((item,), 'position')
    aabbItem = w.getObjectProperty((item,), 'aabb')
    moveTo(w, abe, hand, [itemP[0], aabbItem[0][1]-0.05, aabbItem[0][2]], [0.707,0,0,0.707], posInLink=posInLink, ornInLink=[0,0,0,1])
    moveTo(w, abe, hand, [itemP[0], aabbItem[0][1]-0.05, aabbItem[1][2]], [0.707,0,0,0.707], posInLink=posInLink, ornInLink=[0,0,0,1])
    moveTo(w, abe, hand, [itemP[0]-1, aabbItem[0][1]-0.05, aabbItem[1][2]], [0.707,0,0,0.707], posInLink=posInLink, ornInLink=[0,0,0,1])
    
def mashManeuver(w, agent, hand, item, tool, toolLink, toolTQ):
    itemP = w.getObjectProperty((item,), 'position')
    handLink = {'hand_right': 'hand_right_roll', 'hand_left': 'hand_left_roll'}[hand]
    handP = w.getObjectProperty((agent, handLink), 'position')
    handQ = w.getObjectProperty((agent, handLink), 'orientation')
    toolP = w.getObjectProperty((tool, toolLink), 'position')
    toolQ = w.getObjectProperty((tool, toolLink), 'orientation')
    posInLink, ornInLink = w.objectPoseRelativeToObject(handP, handQ, toolP, toolQ)
    moveTo(w, abe, hand, [handP[0], handP[1], 1.9], handQ, posInLink=[0,0,0], ornInLink=[0,0,0,1])
    toolP = w.getObjectProperty((tool, toolLink), 'position')
    moveTo(w, abe, hand, [toolP[0], toolP[1], 1.9], toolTQ, posInLink=posInLink, ornInLink=ornInLink)
    moveTo(w, abe, hand, [itemP[0], itemP[1], 1.9], toolTQ, posInLink=posInLink, ornInLink=ornInLink)
    aabbItem = w.getObjectProperty((item,), 'aabb')
    aabbTool = w.getObjectProperty((tool, toolLink), 'aabb')
    gap = aabbTool[0][2] - aabbItem[1][2]
    toolP = w.getObjectProperty((tool, toolLink), 'position')
    moveTo(w, abe, hand, [itemP[0], itemP[1], toolP[2]-gap+0.03], toolTQ, posInLink=posInLink, ornInLink=ornInLink)
    moveTo(w, abe, hand, [itemP[0], itemP[1], 1.9], toolTQ, posInLink=posInLink, ornInLink=ornInLink)
    moveTo(w, abe, hand, [itemP[0], itemP[1], 1.9], handQ, posInLink=[0,0,0], ornInLink=[0,0,0,1])

def sideChop(w, agent, hand, item, tool, toolLink, toolTQ):
    itemP = w.getObjectProperty((item,), 'position')
    handLink = {'hand_right': 'hand_right_roll', 'hand_left': 'hand_left_roll'}[hand]
    handP = w.getObjectProperty((agent, handLink), 'position')
    handQ = w.getObjectProperty((agent, handLink), 'orientation')
    toolP = w.getObjectProperty((tool, toolLink), 'position')
    toolQ = w.getObjectProperty((tool, toolLink), 'orientation')
    posInLink, ornInLink = w.objectPoseRelativeToObject(handP, handQ, toolP, toolQ)
    moveTo(w, abe, hand, [-1.4, handP[1], 1.9], handQ, posInLink=posInLink, ornInLink=[0,0,0,1])
    moveTo(w, abe, hand, [-1.4, handP[1], 1.9], toolTQ, posInLink=posInLink, ornInLink=[0,0,0,1])
    moveTo(w, abe, hand, [-1.4, handP[1], itemP[2]], toolTQ, posInLink=posInLink, ornInLink=[0,0,0,1])
    aabbItem = w.getObjectProperty((item,), 'aabb')
    aabbTool = w.getObjectProperty((tool, toolLink), 'aabb')
    gap = aabbTool[1][0] - aabbItem[0][0]
    toolP = w.getObjectProperty((tool, toolLink), 'position')
    moveTo(w, abe, hand, [toolP[0]-gap+0.02, itemP[1], itemP[2]], toolTQ, posInLink=posInLink, ornInLink=[0,0,0,1], stopOnContact=True)
    moveTo(w, abe, hand, [-1.4, itemP[1], itemP[2]], toolTQ, posInLink=posInLink, ornInLink=[0,0,0,1])
    
def topChop(w, agent, hand, item, tool, toolLink, toolTQ):
    itemP = w.getObjectProperty((item,), 'position')
    handLink = {'hand_right': 'hand_right_roll', 'hand_left': 'hand_left_roll'}[hand]
    handP = w.getObjectProperty((agent, handLink), 'position')
    handQ = w.getObjectProperty((agent, handLink), 'orientation')
    toolP = w.getObjectProperty((tool, toolLink), 'position')
    toolQ = w.getObjectProperty((tool, toolLink), 'orientation')
    posInLink, ornInLink = w.objectPoseRelativeToObject(handP, handQ, toolP, toolQ)
    moveTo(w, abe, hand, [handP[0], handP[1], 1.7], handQ, posInLink=posInLink, ornInLink=[0,0,0,1])
    moveTo(w, abe, hand, [handP[0], handP[1], 1.7], toolTQ, posInLink=posInLink, ornInLink=[0,0,0,1])
    moveTo(w, abe, hand, [itemP[0], itemP[1], 1.7], toolTQ, posInLink=posInLink, ornInLink=[0,0,0,1])
    aabbItem = w.getObjectProperty((item,), 'aabb')
    aabbTool = w.getObjectProperty((tool, toolLink), 'aabb')
    gap = aabbTool[0][2] - aabbItem[1][2]
    moveTo(w, abe, hand, [itemP[0], itemP[1], toolP[2]-gap-0.01], toolTQ, posInLink=posInLink, ornInLink=[0,0,0,1], stopOnContact=True)
    moveTo(w, abe, hand, [itemP[0], itemP[1], 1.7], toolTQ, posInLink=posInLink, ornInLink=[0,0,0,1], stopOnContact=True)
    moveTo(w, abe, hand, [-1.4, handP[1], 1.7], toolTQ, posInLink=posInLink, ornInLink=[0,0,0,1], stopOnContact=True)

#objectTypeKnowledge = [json.loads(x) for x in open('objectknowledge.log').read().splitlines() if x.strip()]
objectTypeKnowledge = json.loads(open('./abe_sim/objectknowledge.json').read())
objectTypeKnowledge = {x['type']: x for x in objectTypeKnowledge}

processKnowledge = json.loads(open('./abe_sim/processknowledge.json').read())
w = world.World(pybulletOptions="--opengl2", useGUI=True, customDynamics=customDynamics, objectKnowledge=objectTypeKnowledge, processKnowledge=processKnowledge)

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

egg = placeOnTable(w, kitchenCounter, 'EggInShell', objectTypeKnowledge, (0,-2.7,1), (0.707,0,0,0.707), letStabilize=False)
eggP = w.getObjectProperty((egg,), 'position')

grabItem(w, abe, egg, 'hand_right', [0,0,-0.707,0.707], handle=None)
hammerMotion(w, abe, 'hand_right', [eggP[0], eggP[1], eggP[2] + 0.04])

input('Press Enter to continue ...')

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['CrackedEggshellFlatHalf', 'CrackedEggshellPointyHalf', 'EggWhiteParticle', 'EggYolkParticle']):
        w.removeObject((n,), sendToLimbo=True)

w.setObjectProperty((abe,), ('customStateVariables', 'grasping', 'intendToGrasp', 'hand_right'), [])

cookingKnife = placeOnTable(w, kitchenCounter, 'CookingKnife', objectTypeKnowledge, (-0.5,-3,1), (0.707,0,0,0.707), letStabilize=False)
grabItem(w, abe, cookingKnife, 'hand_right', [0,0,0,1], handle='handle')
liftHand(w, abe, 'hand_right', 1.9, [0.707,0,0,0.707])

positionBlade, orientationBlade = w.objectPoseRelativeToObject(w.getObjectProperty((abe, 'hand_right_roll'), 'position'), w.getObjectProperty((abe, 'hand_right_roll'), 'orientation'), w.getObjectProperty((cookingKnife, 'blade'), 'position'), w.getObjectProperty((cookingKnife, 'blade'), 'orientation'))

potato = placeOnTable(w, kitchenCounter, 'Potato', objectTypeKnowledge, (0,-2.5,1), (0,0,0,1), letStabilize=False)

grabItem(w, abe, potato, 'hand_left', [0,0,0,1], handle=None)
moveTo(w, abe, 'hand_left', [0.3, -2.5, 1.5], [-0.707,0,0,0.707], stopOnContact=False, contacter=None, posInLink=[0,0,0], ornInLink=[0,0,0,1])

peelManeuver(w, abe, 'hand_right', potato, positionBlade)

input('Press Enter to continue ...')

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['PotatoPeel']):
        w.removeObject((n,), sendToLimbo=True)

releaseItem(w, abe, kitchenCounter, potato, 'hand_left', [0,0,0,1], [0,-2.7,1])

masher = placeOnTable(w, kitchenCounter, 'Masher', objectTypeKnowledge, (0.1,-3,1), (0,0,-0.707,0.707), letStabilize=False)

grabItem(w, abe, masher, 'hand_left', [0,0,0,1], handle='handle')
mashManeuver(w, abe, 'hand_left', potato, masher, 'masher', [0,0.707,0,0.707])

input('Press Enter to continue ...')


for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['PotatoParticle', 'PotatoPeel', 'Masher']):
        w.removeObject((n,), sendToLimbo=True)

avocado = placeOnTable(w, kitchenCounter, 'Avocado', objectTypeKnowledge, (0,-3,1), (0,0,0,1), letStabilize=False)

topChop(w, abe, 'hand_right', avocado, cookingKnife, 'blade', [-0.707,0,0,0.707])

input('Press Enter to continue ...')

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['Avocado', 'AvocadoHalf', 'AvocadoHalfSeeded', 'AvocadoSeed']):
        w.removeObject((n,), sendToLimbo=True)

avocado = placeOnTable(w, kitchenCounter, 'Avocado', objectTypeKnowledge, (0,-3,1), (0,0,0,1), letStabilize=False)
avocadoP = list(w.getObjectProperty((avocado,), 'position'))

grabItem(w, abe, avocado, 'hand_left', [0,0,-0.707,0.707], handle=None)
moveTo(w, abe, 'hand_left', [avocadoP[0], avocadoP[1], 1.5], [0,0,-0.707,0.707], stopOnContact=False, contacter=None, posInLink=None, ornInLink=None)

sideChop(w, abe, 'hand_right', avocado, cookingKnife, 'blade', [0,0,-0.707,0.707])

input('Press Enter to continue ...')

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['Avocado', 'AvocadoHalf', 'AvocadoHalfSeeded', 'AvocadoSeed']):
        w.removeObject((n,), sendToLimbo=True)

bread = placeOnTable(w, kitchenCounter, 'Bread', objectTypeKnowledge, (0,-3,1), (0,0,1,0), letStabilize=False)

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

butter = placeOnTable(w, kitchenCounter, 'Butter', objectTypeKnowledge, (-0.6,-3,1), (0,0,0,1), letStabilize=False)

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

pepperShaker = placeOnTable(w, kitchenCounter, 'PepperShaker', objectTypeKnowledge, (0.9,-3,1), (0,0,0,1), letStabilize=False)

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

