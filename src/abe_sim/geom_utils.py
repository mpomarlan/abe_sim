import math
import numpy

def vector2Axis(v):
    norm = numpy.linalg.norm(v)
    if 0.000001 > norm:
        return v
    return v/norm
    
def orthogonalComponent(vector, axis):
    return numpy.linalg.norm(vector-axis*numpy.dot(vector, axis))
