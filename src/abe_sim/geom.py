import pybullet as p
import math

from abe_sim.utils import stubbornTry

def angleDifference(x,y):
    return math.atan2(math.sin(x-y), math.cos(x-y))

def vectorNorm(v):
    return math.sqrt(sum([x*x for x in v]))
def vectorNormalize(v):
    norm = vectorNorm(v)
    if 0.00001 > norm:
        return v
    return [x/norm for x in v]

def quaternionProduct(qa, qb):
    b1,c1,d1,a1 = qa
    b2,c2,d2,a2 = qb
    return (a1*b2+b1*a2+c1*d2-d1*c2, a1*c2-b1*d2+c1*a2+d1*b2, a1*d2+b1*c2-c1*b2+d1*a2,a1*a2-b1*b2-c1*c2-d1*d2)

def overlappingObjects(aabbMin, aabbMax, world):
    needGoodIds = True
    retq = []
    while needGoodIds:
        retq = stubbornTry(lambda : p.getOverlappingObjects(aabbMin, aabbMax, world.getSimConnection()))
        if None == retq:
            retq = []
        allGood = True
        for x in retq:
            if (not world.hasPObjectOfId(x[0])) and (not world.hasDbgObjectOfId(x[0])):
                allGood = False
                break
        needGoodIds = not allGood
    return list(world.getPObjectSetByIds([x[0] for x in retq]))

def overlappingObjectNames(aabbMin, aabbMax, world):
    return [x.getName() for x in overlappingObjects(aabbMin, aabbMax, world)]

def translateVector(va, vb):
    return [a+b for a,b in zip(va, vb)]

def scaleVector(v, s):
    return [x*s for x in v]

def vectorDifference(va, vb):
    return [a-b for a,b in zip(va, vb)]

def distance(va, vb):
    return vectorNorm(vectorDifference(va, vb))

def interpolatePoint(vS, vE, f):
    vD = [x*f for x in vectorDifference(vE, vS)]
    return translateVector(vS, vD)

def extrudeBox(box, a, b=None):
    aabbMin, aabbMax = box
    if None == b:
        b = a
    extAMin = translateVector(aabbMin, a)
    extAMax = translateVector(aabbMax, a)
    extBMin = translateVector(aabbMin, b)
    extBMax = translateVector(aabbMax, b)
    retqMin = [min(a,b) for a,b in zip(extAMin, extBMin)]
    retqMax = [max(a,b) for a,b in zip(extAMax, extBMax)]
    return retqMin, retqMax

def splitBox(box):
    aabbMin, aabbMax = box
    mid = [(a+b)/2.0 for a,b in zip(aabbMin, aabbMax)]
    retq = []
    for k in range(8):
        aabbMinAdj = list(aabbMin)
        aabbMaxAdj = list(mid)
        if 0 != k&1:
            aabbMinAdj[0] = mid[0]
            aabbMaxAdj[0] = aabbMax[0]
        if 0 != k&2:
            aabbMinAdj[1] = mid[1]
            aabbMaxAdj[1] = aabbMax[1]
        if 0 != k&4:
            aabbMinAdj[2] = mid[2]
            aabbMaxAdj[2] = aabbMax[2]
        retq.append((aabbMinAdj,aabbMaxAdj))
    return retq

def boxHasPoint(box, point):
    return all([((a <= b) and (b <= c)) for a,b,c in zip(box[0], point, box[1])])

