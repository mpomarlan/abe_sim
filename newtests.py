import copy
import json
import time

import world

from cracking import isACrackableThing, updateCracking
from mashing import isAMashableThing, updateMashing
from peeling import isAPeelableThing, updatePeeling
from cutting import isACuttableThing, updateCutting
from seeding import isASeedableThing, updateSeeding

customDynamics = [[isACrackableThing, updateCracking], [isAMashableThing, updateMashing], [isAPeelableThing, updatePeeling], [isACuttableThing, updateCutting], [isASeedableThing, updateSeeding]]

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
        w.update()
        time.sleep(1.0/240.0)

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

objectTypeKnowledge = [json.loads(x) for x in open('objectknowledge.log').read().splitlines() if x.strip()]
objectTypeKnowledge = {x['type']: x for x in objectTypeKnowledge}

processKnowledge = json.loads(open('processknowledge.log').read())
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

for oType, name, position in toPreload:
    preload(w, objectTypeKnowledge[oType], name, position)

floor = addObjectInstance(w, 'Floor', objectTypeKnowledge, (0,0,0), (0,0,0,1))
kitchenCounter = addObjectInstance(w, 'KitchenCounter', objectTypeKnowledge, (0,-3,0), (0,0,0,1))

egg = addObjectInstance(w, 'EggInShell', objectTypeKnowledge, (0,-3,2), (0.707,0,0,0.707))

w.setGravity((0,0,-10))
timestepWorld(w, 3)

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['CrackedEggshellFlatHalf', 'CrackedEggshellPointyHalf', 'EggWhiteParticle', 'EggYolkParticle']):
        w.removeObject((n,), sendToLimbo=True)

w.setGravity((0,0,0))
cookingKnife = addObjectInstance(w, 'CookingKnife', objectTypeKnowledge, (-0.2,-2.75,2.1), (-0.5,0.5,0.5,-0.5))
potato = addObjectInstance(w, 'Potato', objectTypeKnowledge, (0,-3,2), (0,0,0,1))

w.setObjectProperty((cookingKnife,), 'linearVelocity', (0.1, 0, 0))
timestepWorld(w, 3)

w.removeObject((cookingKnife,))

w.setGravity((0,0,-10))
timestepWorld(w, 3)

masher = addObjectInstance(w, 'Masher', objectTypeKnowledge, (0,-3,2), (0,0.707,0,0.707))
timestepWorld(w, 3)

for n in list(w._kinematicTrees.keys()):
    if w._kinematicTrees[n]['type'] in set(['PotatoParticle', 'PotatoPeel', 'Masher']):
        w.removeObject((n,), sendToLimbo=True)


apple = addObjectInstance(w, 'Apple', objectTypeKnowledge, (0,-3,2), (0,0,0,1))
w.setGravity((0,0,-10))
timestepWorld(w, 3)

pos = list(w.getObjectProperty((apple,), 'position'))
pos[0] -= 0.15
pos[2] += 2
cookingKnife = addObjectInstance(w, 'CookingKnife', objectTypeKnowledge, pos, (0,0,0,1))

timestepWorld(w, 3)

halfApple = getOfType(w, 'HalfApple')
w.setGravity((0,0,0))

w.setObjectProperty((halfApple,), 'position', (0, -3, 2))
w.setObjectProperty((halfApple,), 'orientation', (0, 0.707, 0, 0.707))
w.setObjectProperty((halfApple,), 'linearVelocity', (0, 0, 0))
w.setObjectProperty((cookingKnife,), 'position', (-0.2, -3, 2.15))
w.setObjectProperty((cookingKnife,), 'orientation', (0, 0, 0, 1))
w.setObjectProperty((cookingKnife,), 'linearVelocity', (0, 0.1, 0))
timestepWorld(w, 3)

