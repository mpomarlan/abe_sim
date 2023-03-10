import math
import numpy

from world import getDictionaryEntry
from geom_utils import vector2Axis

def computeError(customDynamicsAPI, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, efBaseLinearVelocity, efBaseAngularVelocity, linearD=0.1, angularD=0.1):
    # e = clampmag(target-actual, d)
    positionEFLink = customDynamicsAPI['getObjectProperty']((robotName, efLink), 'position')
    orientationEFLink = customDynamicsAPI['getObjectProperty']((robotName, efLink), 'orientation')
    linearVelocityEFLink = customDynamicsAPI['getObjectProperty']((robotName, efLink), 'linearVelocity')
    angularVelocityEFLink = customDynamicsAPI['getObjectProperty']((robotName, efLink), 'angularVelocity')
    positionBase = customDynamicsAPI['getObjectProperty']((robotName,), 'position')
    orientationBase = customDynamicsAPI['getObjectProperty']((robotName,), 'orientation')
    efPosition, efOrientation = customDynamicsAPI['objectPoseRelativeToWorld'](positionEFLink, orientationEFLink, efPositionInLink, efOrientationInLink)
    positionE = numpy.array([a-b for a,b in zip(targetPosition, efPosition)])
    lenPE = numpy.dot(positionE, positionE)
    linearDSq = linearD*linearD
    adj = 1.0
    if linearDSq < lenPE:
        adj = math.sqrt(linearDSq/lenPE)
    positionE = 10*adj*positionE
    axis, angle = customDynamicsAPI['orientationDifferenceAA'](targetOrientation, efOrientation)
    angle = min(angularD, angle)
    orientationE = numpy.array([angle*x for x in axis])
    efPositionInBase, efOrientationInBase = customDynamicsAPI['objectPoseRelativeToObject'](positionBase, orientationBase, efPosition, efOrientation)
    tangentialVelocity = numpy.cross(efBaseAngularVelocity, efPositionInBase)
    eLV = positionE - efBaseLinearVelocity - tangentialVelocity
    eAV = orientationE - efBaseAngularVelocity
    currentVelocity = numpy.array(linearVelocityEFLink) - efBaseLinearVelocity - tangentialVelocity + numpy.cross(angularVelocityEFLink, efPositionInLink)
    accelerationLimitSquared = customDynamicsAPI['getObjectProperty']((robotName,), ('fn', 'kinematicControl', 'accelerationLimitSquared', efLink), 1.0)
    #acceleration = eLV - currentVelocity
    #accelerationDirection = vector2Axis(acceleration)
    #accelerationMagnitude = numpy.linalg.norm(acceleration)
    return numpy.concatenate((eLV, eAV)), currentVelocity, accelerationLimitSquared
    #print(eLV, currentVelocity, currentVelocity + accelerationLimit*accelerationDirection)
    #if accelerationMagnitude < accelerationLimit:
    #    return numpy.concatenate((eLV, eAV))
    #return numpy.concatenate((currentVelocity + accelerationLimit*accelerationDirection, eAV))

def computeAvailableJacobian(customDynamicsAPI, robotName, efLink, efPositionInLink, efOrientationInLink, usableDoFs, mobileBase=False):
    linearJacobian, angularJacobian = customDynamicsAPI['computeJacobian'](efLink, efPositionInLink)
    linearJacobian = numpy.array(linearJacobian)
    angularJacobian = numpy.array(angularJacobian)
    dofList = customDynamicsAPI['getObjectProperty']((robotName,), 'dofList')
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
        return min(max(1.0, x1, x2), 10)
    else:
        return 10
    
def jacobianTranspose(customDynamicsAPI, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, usableDoFs, efBaseLinearVelocity, efBaseAngularVelocity, dampingSq=1.0, linearD=0.1, angularD=0.1, mobileBase=False):
    e, currentVelocity, accelerationLimitSquared = computeError(customDynamicsAPI, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, efBaseLinearVelocity, efBaseAngularVelocity, linearD, angularD)
    J, Jt = computeAvailableJacobian(customDynamicsAPI, robotName, efLink, efPositionInLink, efOrientationInLink, usableDoFs, mobileBase=mobileBase)
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

def jacobianPseudoinverseDLS(customDynamicsAPI, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, usableDoFs, efBaseLinearVelocity, efBaseAngularVelocity, dampingSq=0.1, linearD=0.1, angularD=0.1, mobileBase=False):
    e, _, _ = computeError(customDynamicsAPI, robotName, efLink, efPositionInLink, efOrientationInLink, targetPosition, targetOrientation, efBaseLinearVelocity, efBaseAngularVelocity, linearD, angularD)
    # dq = Jt*inv(J*Jt + dampingSq*I)*e
    J, Jt = computeAvailableJacobian(customDynamicsAPI, robotName, efLink, efPositionInLink, efOrientationInLink, usableDoFs, mobileBase=mobileBase)
    JJt = numpy.matmul(J, Jt)
    invMat = numpy.linalg.inv(JJt + dampingSq*numpy.eye(len(JJt)))
    return numpy.matmul(Jt, numpy.matmul(invMat, e))

def updateKinematicControl(name, customDynamicsAPI):
    fnKinematicControl = customDynamicsAPI['getObjectProperty']((name,), ('fn', 'kinematicControl'), {})
    csvKinematicControl = customDynamicsAPI['getObjectProperty']((name,), ('customStateVariables', 'kinematicControl'), {})
    controlType = fnKinematicControl.get('method', 'transpose')
    controlFn = {'transpose': jacobianTranspose, 'pseudoinverse': jacobianPseudoinverseDLS}.get(controlType, jacobianTranspose)
    dofs = [x[0] for x in customDynamicsAPI['getObjectProperty']((name,), 'dofList', [])]
    dofSet = set(dofs)
    mobileBase = not customDynamicsAPI['getObjectProperty']((name,), 'immobile', False)
    for ef in fnKinematicControl.get('endEffectors', []):
        efTarget = getDictionaryEntry(csvKinematicControl, ('target', ef), None)
        if efTarget is not None:
            efPositionInLink = getDictionaryEntry(csvKinematicControl, ('positionInLink', ef), (0,0,0))
            efOrientationInLink = getDictionaryEntry(csvKinematicControl, ('orientationInLink', ef), (0,0,0,1)) 
            efLink = getDictionaryEntry(fnKinematicControl, ('efLink', ef), '')
            usableDoFs = set(getDictionaryEntry(fnKinematicControl, ('availableDoFs', ef), dofSet))
            #efBaseLink = ###
            efBaseLinearVelocity = customDynamicsAPI['getObjectProperty']((name,), 'linearVelocity')
            efBaseAngularVelocity = customDynamicsAPI['getObjectProperty']((name,), 'angularVelocity')
            dampingSq = getDictionaryEntry(fnKinematicControl, ('dampingSq', ef), 1.0)
            linearD = getDictionaryEntry(fnKinematicControl, ('linearD', ef), 0.1)
            angularD = getDictionaryEntry(fnKinematicControl, ('angularD', ef), 0.1) 
            jointVels = controlFn(customDynamicsAPI, name, efLink, efPositionInLink, efOrientationInLink, efTarget[0], efTarget[1], usableDoFs, efBaseLinearVelocity, efBaseAngularVelocity, dampingSq=dampingSq, linearD=linearD, angularD=angularD, mobileBase=mobileBase)
            k = 0
            for d in dofs:
                if d in usableDoFs:
                    force = getDictionaryEntry(fnKinematicControl, ('maxForce', d), 1000)
                    customDynamicsAPI['applyJointControl'](d, targetVelocity=jointVels[k], force=force)
                    k = k + 1
    return
