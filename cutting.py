import math
import numpy

import copy

from world import getDictionaryEntry

def isACuttableThing(getFn):
    return getFn(('fn', 'cuttable'), None) is True

def updateCutting(name, customDynamicsAPI):
    def isCuttingContact(collision):
       _, identifierB, ponA, ponB, normal, distance, force, _, _, _, _ = collision
       return customDynamicsAPI['getObjectProperty']((identifierB[0],), ('fn', 'canCut'), False) and (1 < len(identifierB)) and (identifierB[1] in customDynamicsAPI['getObjectProperty']((identifierB[0],), ('fn', 'cutting', 'links'), []))
    cuttingForce = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'cutting', 'cuttingForce'), 0)
    collisions = customDynamicsAPI['checkCollision']((name,))
    for collision in collisions:
        if (cuttingForce < collision[6]) and (isCuttingContact(collision)):
            nameType = customDynamicsAPI['getObjectProperty']((name,), 'type')
            customDynamicsAPI['concludeProcess']({'process': 'cutting', 'patient': nameType})
            return
