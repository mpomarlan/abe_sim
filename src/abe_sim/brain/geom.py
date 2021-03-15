import os
import sys

import math
import numpy as np

import trimesh

def angle_diff(target, current):
    retq = target - current
    if math.pi < retq:
        retq = retq - 2*math.pi
    elif -math.pi > retq:
        retq = retq + 2*math.pi
    return retq

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def invert_quaternion(q):
    return [q[0], q[1], q[2], -q[3]]

def quaternion_product(qA, qB):
    b1, c1, d1, a1 = qA
    b2, c2, d2, a2 = qB
    return [a1*b2+b1*a2+c1*d2-d1*c2, a1*c2-b1*d2+c1*a2+d1*b2, a1*d2+b1*c2-c1*b2+d1*a2, a1*a2-b1*b2-c1*c2-d1*d2]

def euler_diff_to_angvel(fromRPY, toRPY, dt):
    qF = euler_to_quaternion(fromRPY)
    qT = euler_to_quaternion(toRPY)
    qD = quaternion_product(qT, invert_quaternion(qF))
    halfAlpha = math.acos(qD[3])
    if 0.005 > halfAlpha:
        return [0,0,0]
    s = math.sin(halfAlpha)
    u = [qD[0]/s, qD[1]/s, qD[2]/s]
    return [u[0]*halfAlpha/dt, u[1]*halfAlpha/dt, u[2]*halfAlpha/dt]

def vdot(va, vb):
    return va[0]*vb[0]+va[1]*vb[1]+va[2]*vb[2]

def poseFromTQ(t, q):
    pose = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    t2 = w*x
    t3 = w*y
    t4 = w*z
    t5 = -x*x
    t6 = x*y
    t7 = x*z
    t8 = -y*y
    t9 = y*z
    t10 = -z*z
    pose[0][0] = 2*(t8 + t10 + 0.5)
    pose[0][1] = 2*(t6 - t4)
    pose[0][2] = 2*(t3 + t7)
    pose[1][0] = 2*(t4 + t6)
    pose[1][1] = 2*(t5 + t10 + 0.5)
    pose[1][2] = 2*(t9 - t2)
    pose[2][0] = 2*(t7 - t3)
    pose[2][1] = 2*(t2 + t9)
    pose[2][2] = 2*(t5 + t8 + 0.5)
    pose[0][3] = t[0]
    pose[1][3] = t[1]
    pose[2][3] = t[2]
    pose = np.array(pose)
    return pose

def poseMultiply(a, b):
    return list(np.dot(np.array(a), np.array(b)))

def transformVector(v, t, q):
    pose = poseFromTQ(t, q)
    rv = [0,0,0,0]
    nv = v + [1.0]
    for k in list(range(4)):
        for j in list(range(4)):
            rv[k] = rv[k] + pose[k][j]*nv[j]
    return rv[:3]

class Box:
    def __init__(self):
        self.vertices = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0]]
    def makeFromBox(self, box, pos, rot):
        for k in range(8):
            self.vertices[k] = transformVector(box.vertices[k], pos, rot)

def boxFromPath(meshPath):
    path = os.path.join("../meshes", meshPath)
    path = os.path.join(os.path.dirname(__file__), path)
    mesh = trimesh.load(path)
    bbox = list(mesh.bounds)
    retq = Box()
    xMin = bbox[0][0]
    xMax = bbox[1][0]
    yMin = bbox[0][1]
    yMax = bbox[1][1]
    zMin = bbox[0][2]
    zMax = bbox[1][2]
    retq.vertices = [[xMin, yMin, zMin], [xMax, yMin, zMin], [xMin, yMax, zMin], [xMax, yMax, zMin], [xMin, yMin, zMax], [xMax, yMin, zMax], [xMin, yMax, zMax], [xMax, yMax, zMax]]
    return retq

def axisSeparation(boxA, boxB, axis):
    aMin = None
    aMax = None
    bMin = None
    bMax = None
    for k in range(8):
        ac = vdot(boxA.vertices[k], axis)
        bc = vdot(boxB.vertices[k], axis)
        if (None == aMin) or (aMin > ac):
            aMin = ac
        if (None == aMax) or (aMax < ac):
            aMax = ac
        if (None == bMin) or (bMin > bc):
            bMin = bc
        if (None == bMax) or (bMax < bc):
            bMax = bc
        if not ((bMax < aMin) or (aMax < bMin)):
            return False
    return True

def doBoxesCollide(boxA, boxB, posA, posB, rotA, rotB):
    boxAT = Box()
    boxAT.makeFromBox(boxA, posA, rotA)
    boxBT = Box()
    boxBT.makeFromBox(boxB, posB, rotB)
    axes = []
    for axis in [[1,0,0], [0,1,0], [0,0,1]]:
        axes.append(transformVector(axis, [0,0,0], rotA))
        axes.append(transformVector(axis, [0,0,0], rotB))
    for axis in axes:
        if axisSeparation(boxAT, boxBT, axis):
            return False
    return True

class BoxCollisionManager:
    def __init__(self):
        self.objects = {}
    def clear_objects(self):
        self.objects = {}
    def remove_object(self, name):
        if name in self.objects:
            self.objects.pop(name)
    def add_object(self, name, box, transform):
        pos, rot = transform
        self.objects[name] = {"box": box, "pos": pos, "rot": rot}
    def in_collision_single(self, box, transform):
        pos, rot = transform
        for k in self.objects.keys():
            if doBoxesCollide(box, self.objects[k]["box"], pos, self.objects[k]["pos"], rot, self.objects[k]["rot"]):
                return True
        return False

