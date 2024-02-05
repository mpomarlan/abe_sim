import math
import numpy
import time

from abe_sim.world import getDictionaryEntry
from abe_sim.geom_utils import vector2Axis

###############################
### WARNING WARNING WARNING ###
###############################
#
# Just because numpy is usually fast does not mean it is always fast.
# In particular, for 3D vectors numpy.dot and numpy.cross are SLOWER 
# than manually implementing the dot and cross product in python code!
# numpy.cross is MUCH slower. AVOID it.
# Convert a list of numbers to numpy array only later, when doing matmul,
# mat inversion, or the like.

def computeError(w, fnKinematicControl, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, efBasePosition, efBaseOrientation, efBaseLinearVelocity, efBaseAngularVelocity, linearD=0.1, angularD=0.1):
    # e = clampmag(target-actual, d)
    positionEFLink, orientationEFLink, linearVelocityEFLink, angularVelocityEFLink = w.getKinematicData((robotName, efLink))
    efPosition, efOrientation = w.objectPoseRelativeToWorld(positionEFLink, orientationEFLink, efPositionInLink, efOrientationInLink)
    positionE = [a-b for a,b in zip(targetPosition, efPosition)]
    lenPE = positionE[0]*positionE[0]+positionE[1]*positionE[1]+positionE[2]*positionE[2]
    linearDSq = linearD*linearD
    adj = 1.0
    if linearDSq < lenPE:
        adj = math.sqrt(linearDSq/lenPE)
    positionE = numpy.array(positionE)
    positionE = 10*adj*positionE
    axis, angle = w.orientationDifferenceAA(targetOrientation, efOrientation)
    angle = min(angularD, angle)
    orientationE = 10*numpy.array([angle*x for x in axis])
    efPositionInBase, efOrientationInBase = w.objectPoseRelativeToObject(efBasePosition, efBaseOrientation, efPosition, efOrientation)
    tangentialVelocity = [efBaseAngularVelocity[1]*efPositionInBase[2]-efBaseAngularVelocity[2]*efPositionInBase[1],
                          efBaseAngularVelocity[2]*efPositionInBase[0]-efBaseAngularVelocity[0]*efPositionInBase[2],
                          efBaseAngularVelocity[0]*efPositionInBase[1]-efBaseAngularVelocity[1]*efPositionInBase[0]]
    eLV = positionE - efBaseLinearVelocity - tangentialVelocity
    eAV = orientationE - efBaseAngularVelocity
    vpCross = [angularVelocityEFLink[1]*efPositionInLink[2] - angularVelocityEFLink[2]*efPositionInLink[1],
               angularVelocityEFLink[2]*efPositionInLink[0] - angularVelocityEFLink[0]*efPositionInLink[2],
               angularVelocityEFLink[0]*efPositionInLink[1] - angularVelocityEFLink[1]*efPositionInLink[0]]
    currentVelocity = numpy.array(linearVelocityEFLink) - efBaseLinearVelocity - tangentialVelocity + vpCross
    accelerationLimitSquared = fnKinematicControl.get("accelerationLimitSquared", {}).get(efLink) or 1.0
    #acceleration = eLV - currentVelocity
    #accelerationDirection = vector2Axis(acceleration)
    #accelerationMagnitude = numpy.linalg.norm(acceleration)
    return numpy.concatenate((eLV, eAV)), currentVelocity, accelerationLimitSquared
    #print(eLV, currentVelocity, currentVelocity + accelerationLimit*accelerationDirection)
    #if accelerationMagnitude < accelerationLimit:
    #    return numpy.concatenate((eLV, eAV))
    #return numpy.concatenate((currentVelocity + accelerationLimit*accelerationDirection, eAV))

def computeAvailableJacobian(w, robotName, efLink, efPositionInLink, efOrientationInLink, usableDoFs, mobileBase=False):
    jointPositions, jointVelocities, _, _ = w.getJointStates((robotName,))
    linearJacobian, angularJacobian = w.computeJacobian(robotName, efLink, efPositionInLink, positions=jointPositions, velocities=jointVelocities)
    linearJacobian = numpy.array(linearJacobian)
    angularJacobian = numpy.array(angularJacobian)
    dofList = w._kinematicTrees[robotName].get('dofList', [])
    selectedDoF = []
    baseK = 0
    if mobileBase:
        baseK = 6
    for k,e in enumerate(dofList):
        if e[0] in usableDoFs:
            selectedDoF.append(baseK + k)
    J = numpy.concatenate((linearJacobian[:, selectedDoF], angularJacobian[:, selectedDoF]))
    Jt = numpy.transpose(J)
    return J, Jt

def getFactorForTranspose(newVelocity, oldVelocity, accelerationLimitSquared):
    nv = numpy.dot(newVelocity, newVelocity)
    nov = numpy.dot(newVelocity, oldVelocity)
    ov = numpy.dot(oldVelocity, oldVelocity)
    if 0.01 < nv:
        #aa*|nv| - a*2*(nv*ov) + |ov| - "accsqlim"
        delta = 4*nov*nov - 4*nv*(ov - accelerationLimitSquared)
        if 0 >= delta:
            return 1.0
        delta = math.sqrt(delta)
        x1 = (2.0*nov - delta)/(2*nv)
        x2 = (2.0*nov + delta)/(2*nv)
        return min(max(1.0, x1, x2), 5)
    else:
        return 5
    
def jacobianTranspose(w, fnKinematicControl, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, usableDoFs, efBasePosition, efBaseOrientation, efBaseLinearVelocity, efBaseAngularVelocity, dampingSq=1.0, linearD=0.1, angularD=0.1, mobileBase=False):
    e, currentVelocity, accelerationLimitSquared = computeError(w, fnKinematicControl, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, efBasePosition, efBaseOrientation, efBaseLinearVelocity, efBaseAngularVelocity, linearD, angularD)
    J, Jt = computeAvailableJacobian(w, robotName, efLink, efPositionInLink, efOrientationInLink, usableDoFs, mobileBase=mobileBase)
    Jte = numpy.matmul(Jt,e)
    JJte = numpy.matmul(J, Jte)
    dotJJte = numpy.dot(JJte, JJte)
    doteJJte = numpy.dot(e, JJte)
    # alpha = <e, J*Jt*e>/<J*Jt*e, J*Jt*e>
    alpha = 0.001
    if 0.000001 < dotJJte:
        alpha = doteJJte/dotJJte
    # dq = alpha*Jt*e
    factor = getFactorForTranspose(alpha*e[:3], currentVelocity, accelerationLimitSquared)
    return factor*alpha*Jte

def jacobianPseudoinverseDLS(w, fnKinematicControl, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, usableDoFs, efBasePosition, efBaseOrientation, efBaseLinearVelocity, efBaseAngularVelocity, dampingSq=0.1, linearD=0.1, angularD=0.1, mobileBase=False):
    e, _, _ = computeError(w, fnKinematicControl, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, efBasePosition, efBaseOrientation, efBaseLinearVelocity, efBaseAngularVelocity, linearD, angularD)
    # dq = Jt*inv(J*Jt + dampingSq*I)*e
    J, Jt = computeAvailableJacobian(w, robotName, efLink, efPositionInLink, efOrientationInLink, usableDoFs, mobileBase=mobileBase)
    JJt = numpy.matmul(J, Jt)
    invMat = numpy.linalg.inv(JJt + dampingSq*numpy.eye(len(JJt)))
    return numpy.matmul(Jt, numpy.matmul(invMat, e))

def updateKinematicControl(name, customDynamicsAPI):
    startD = time.perf_counter()
    w = customDynamicsAPI["leetHAXXOR"]()
    objData = w._kinematicTrees[name]
    fnKinematicControl = objData.get("fn", {}).get("kinematicControl", {})
    csvKinematicControl = objData.get("customStateVariables", {}).get("kinematicControl", {})
    controlType = fnKinematicControl.get("method", "transpose")
    controlFn = {'transpose': jacobianTranspose, 'pseudoinverse': jacobianPseudoinverseDLS}.get(controlType, jacobianTranspose)
    dofs = [x[0] for x in objData.get('dofList', [])]
    dofSet = set(dofs)
    mobileBase = not objData.get("immobile", False)
    for ef in fnKinematicControl.get("endEffectors", []):
        efTarget = csvKinematicControl.get("target", {}).get(ef, None)
        usableDoFs = set(fnKinematicControl.get("availableDoFs", {}).get(ef, dofSet))
        if efTarget is not None:
            efPositionInLink = csvKinematicControl.get("positionInLink", {}).get(ef) or (0,0,0)
            efOrientationInLink = csvKinematicControl.get("orientationInLink", {}).get(ef) or (0,0,0,1)
            efLink = fnKinematicControl.get("efLink", {}).get(ef, '')
            efBasePosition, efBaseOrientation, efBaseLinearVelocity, efBaseAngularVelocity = w.getKinematicData((name,))
            dampingSq = fnKinematicControl.get("dampingSq", {}).get(ef) or 1.0
            linearD = fnKinematicControl.get("linearD", {}).get(ef) or 0.1
            angularD = fnKinematicControl.get("angularD", {}).get(ef) or 0.1
            jointVels = controlFn(w, fnKinematicControl, name, efLink, efPositionInLink, efOrientationInLink, efTarget[0], efTarget[1], usableDoFs, efBasePosition, efBaseOrientation, efBaseLinearVelocity, efBaseAngularVelocity, dampingSq=dampingSq, linearD=linearD, angularD=angularD, mobileBase=mobileBase)
        else:
            jointVels = [0]*len(usableDoFs)
        k = 0
        for d in dofs:
            if d in usableDoFs:
                force = fnKinematicControl.get("maxForce", {}).get(d) or 1000
                w.applyJointControl((name, d), targetVelocity=jointVels[k], force=force)
                k = k + 1
    endD = time.perf_counter()
    #print("    kinematicC %f" % (endD-startD))
    return
