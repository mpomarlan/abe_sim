import math
import numpy

import copy

from world import getDictionaryEntry

def isACrackableThing(getFn):
    return getFn(('fn', 'crackable'), None) is True

def updateCracking(name, customDynamicsAPI):
    crackForce = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'cracking', 'crackForce'), 0)
    collisions = customDynamicsAPI['checkCollision']((name,))
    for collision in collisions:
        if crackForce < collision[6]:
            nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
            customDynamicsAPI['concludeProcess']({'process': 'cracking', 'patient': nameType})
            return
