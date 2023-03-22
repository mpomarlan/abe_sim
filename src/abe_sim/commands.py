import math
import numpy
import pybullet
import requests

from abe_sim.world import stubbornTry

def _cancelGardenAction(w, agentName, todos):
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), {})
    if todos["currentAction"] is not None:
        todos["cancelled"] = True
        
def _cancelClopens(w, agentName):
    for ef in w.getObjectProperty((agentName,), ('fn', 'clopening', 'clopeningEFs'), []):
        w.setObjectProperty((), ('customStateVariables', 'clopening', 'action', ef), None)
        
def _checkArgs(args):
    retq = []
    for a in args:
        var, msg = a
        if var is None:
            retq.append(msg)
    return retq

def toGetTime(requestData, w, agentName, todos):
    return requests.status_codes.codes.ALL_OK, {"response": {"time": w.getObjectProperty((agentName,), ('customStateVariables', 'timing', 'timer'), None)}}
    
def toCancel(requestData, w, agentName, todos):
    for ef in w.getObjectProperty((agentName,), ('fn', 'grasping', 'effectors'), []):
        w.setObjectProperty((), ('customStateVariables', 'grasping', 'intendToGrasp', ef), [])
    _cancelClopens(w, agentName)
    _cancelGardenAction(w, agentName, todos)
    return requests.status_codes.codes.ALL_OK, {"response": "Ok."}

def toGetKitchen(requestData, w, agentName, todos):
    kitchenStateIn = requestData.get('kitchenStateIn', None)
    lacks = _checkArgs([[kitchenStateIn, "Request lacks kitchen-state-in parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    return requests.status_codes.codes.ALL_OK, {"response": {kitchenStateIn: w.worldDump()}}
    
def toSetKitchen(requestData, w, agentName, todos):
    kitchenStateIn = requestData.get('kitchenStateIn', None)
    lacks = _checkArgs([[kitchenStateIn, "Request lacks kitchen-state-in parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _cancelGardenAction(w, agentName, todos)
    w.greatReset(kitchenStateIn)
    return requests.status_codes.codes.ALL_OK, {"response": "Ok."}
    
def toGoToPose(requestData, w, agentName, todos):
    def angleAdjust(dest, cr):
        base = int(cr/(2*math.pi))*2*math.pi
        dest = (dest%(2*math.pi)) + base
        dif = dest - cr
        if -math.pi > dif:
            dif = dest+2*math.pi - cr
        if math.pi < dif:
            dif = dest-2*math.pi - cr
        return cr + dif
    position = requestData.get('position', None)
    yaw = requestData.get('yaw', None)
    lacks = _checkArgs([[position, "Request lacks position parameter."],
                        [yaw, "Request lacks yaw parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    tp = [x for x in position if type(x) in [int, float]]
    if (2 != len(tp)) or (type(yaw) not in [int, float]):
        return requests.status_codes.codes.BAD_REQUEST, {'response': 'Pose data is malformed: either the position is not a vector of 2 numbers or the yaw is not a number.'}
    position = [position[0], position[1], 0]
    mobileBaseLink = w.getObjectProperty((agentName,), ('fn', 'kinematicControl', 'mobileBaseLink'))
    baseOrientation = w.getObjectProperty((agentName, mobileBaseLink), 'orientation')
    _, _, crYaw = stubbornTry(lambda : pybullet.getEulerFromQuaternion(baseOrientation))
    yaw = angleAdjust(yaw, crYaw)
    _cancelClopens(w, agentName)
    _cancelGardenAction(w, agentName, todos)
    orientation = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,yaw)))
    w.setObjectProperty((agentName,), ('customStateVariables', 'kinematicControl', 'target', 'base'), [position, orientation])
    return requests.status_codes.codes.ALL_OK, {"response": "Ok."}
    
def toSetJoint(requestData, w, agentName, todos):
    oname = requestData.get('object', None)
    joint = requestData.get('joint', None)
    position = requestData.get('position', None)
    lacks = _checkArgs([[oname, "Request lacks object parameter."],
                        [joint, "Request lacks joint parameter."],
                        [position, "Request lacks position parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    if (not isinstance(position, float)) and (not isinstance(position, int)):
        return requests.status_codes.codes.BAD_REQUEST, {'response': 'Position is not a number.'}
    if w.getObjectProperty((oname, joint), 'jointPosition', None) is None:
        return requests.status_codes.codes.BAD_REQUEST, {'response': 'Requested object or joint do not exist in the world.'}
    w.setObjectProperty((oname, joint), 'jointPosition', position)
    return requests.status_codes.codes.ALL_OK, {"response": "Ok."}
    
def toSetCustomVariable(requestData, w, agentName, todos):
    oname = requestData.get('object', None)
    varname = requestData.get('varname', None)
    varvalue = requestData.get('varvalue', None)
    lacks = _checkArgs([[oname, "Request lacks object parameter."],
                        [varname, "Request lacks varname parameter."],
                        [varvalue, "Request lacks varvalue parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    if oname not in w._kinematicTrees:
        return requests.status_codes.codes.BAD_REQUEST, {'response': 'Requested object does not exist in world.'}
    if isinstance(varname, str):
        varname = ('customStateVariables', varname)
    else:
        varname = ['customStateVariables'] + varname
    w.setObjectProperty((oname,), varname, varvalue)
    return requests.status_codes.codes.ALL_OK, {"response": "Ok."}
   
def toSetObjectPose(requestData, w, agentName, todos):
    oname = requestData.get('object', None)
    position = requestData.get('position', None)
    orientation = requestData.get('orientation', None)
    lacks = _checkArgs([[oname, "Request lacks object parameter."],
                        [orientation, "Request lacks orientation parameter."],
                        [position, "Request lacks position parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    if oname not in w._kinematicTrees:
        return requests.status_codes.codes.BAD_REQUEST, {'response': 'Requested object does not exist in world.'}
    tp = [x for x in position if type(x) in [int, float]]
    to = [x for x in orientation if type(x) in [int, float]]
    if (3 != len(tp)) or (4 != len(to)):
        return requests.status_codes.codes.BAD_REQUEST, {'response': 'Pose data is malformed: either the position is not a vector of 3 numbers or the orientation is not a vector of 4 numbers.'}
    w.setObjectProperty((oname,), 'position', position)
    w.setObjectProperty((oname,), 'orientation', orientation)
    return requests.status_codes.codes.ALL_OK, {"response": "Ok."}
    
def toGetStateUpdates(requestData, w, agentName, todos):
    garden = w.getObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), {})
    bps = [x for x in garden.value() if 'target' in x]
    updates = {}
    # filename, position, orientation, linkjointpos
    return requests.status_codes.codes.ALL_OK, {"updates": updates, "currentCommand": todos["command"], "abeActions": bps}
    
def processActionRequest(fn, requestData, w, agentName, todos):
    if todos['currentAction'] is not None:
        return False, requests.status_codes.codes.SERVICE_UNAVAILABLE, 'Robot already performs an action.'
    return True, status, response
    
def processInstantRequest(fn, requestData, w, agentName, todos):
    status, response = fn(requestData, w, agentName, todos)
    return False, status, response

commandFns = {
    "to-get-time": [processInstantRequest, toGetTime, None], 
    "to-cancel": [processInstantRequest, toCancel, None],
    "to-get-kitchen": [processInstantRequest, toGetKitchen, None],
    "to-set-kitchen": [processInstantRequest, toSetKitchen, None],
    "to-go-to-pose": [processInstantRequest, toGoToPose, None],
    "to-set-joint": [processInstantRequest, toSetJoint, None],
    "to-set-custom-variable": [processInstantRequest, toSetCustomVariable, None],
    "to-set-object-pose": [processInstantRequest, toSetObjectPose, None],
    "to-get-state-updates": [processInstantRequest, toGetStateUpdates, None]}
