import copy
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
    
def _getStorage(w):
    # TODO: choose storage more intelligently so as to avoid overcrowding one piece of furniture.
    for k, v in w._kinematicTrees.items():
        if 'KitchenCabinet' == v['type']:
            return k
    for k, v in w._kinematicTrees.items():
        if 'Pantry' == v['type']:
            return k
    return None
    
def _getPouredType(w, oname):
    otype = w.getObjectProperty((oname,), 'type')
    outcomes = w._getProcessRoleKnowledge({'patient': otype, 'process': 'pouring'}, 'outcome')
    pouringType = None
    if outcomes is not None:
        pouringType = outcomes['toAdd'][0][0]
    return pouringType

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
    bps = [copy.deepcopy(x) for x in garden.values() if 'target' in x]
    updates = {}
    for name, data in w._kinematicTrees.items():
        mesh = stubbornTry(lambda : pybullet.getVisualShapeData(data['idx'], -1, w._pybulletConnection))[0][4].decode("utf-8")
        updates[name] = {'filename': str(data['filename']), 'position': list(data['position']), 'orientation': list(data['orientation']), 'at': str(data['at']), 'mesh': mesh, 'customStateVariables': copy.deepcopy(data['customStateVariables']), 'joints': copy.deepcopy(data.get('joints', None))}
    return requests.status_codes.codes.ALL_OK, {"response": {"updates": updates, "currentCommand": str(todos["command"]), "abeActions": bps}}

def toGetLocation(requestData, w, agentName, todos):
    def _freeOfContent(o):
        if not w.getObjectProperty((o,), ('fn', 'canContain'), False):
            return True
        aabb = w.getObjectProperty((o,), 'aabb')
        overlaps = w.checkOverlap(aabb)
        return 0 == len([x for x in overlaps if o == w.getObjectProperty((x,), 'at')])
    varname = requestData.get('availableLocation', None)
    otype = requestData.get('type', None)
    lacks = _checkArgs([[varname, "Request lacks availableLocation parameter."],
                        [otype, "Request lacks type parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    objs = [k for k,v in w._kinematicTrees.items() if otype == v['type']]
    if 0 == len(objs):
        return requests.status_codes.codes.NOT_FOUND, {'response': ('World does not have an object of type %s' % otype)}
    if 1 == len(objs):
        return requests.status_codes.codes.ALL_OK, {'response': {varname: objs[0]}}
    for o in objs:
        if _freeOfContent(o):
            requests.status_codes.codes.ALL_OK, {'response': {varname: o}}
    return requests.status_codes.codes.ALL_OK, {'response': {varname: objs[0]}}

def _checkGreatReset(requestData, w):
    kitchenStateIn = requestData.get('kitchenStateIn', None)
    sws = requestData.get('setWorldState', False)
    if sws and (kitchenStateIn is not None):    
        w.greatReset(kitchenStateIn)
    
def _checkTopGoal(w, agentName):
    garden = w.getObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), {})
    topGoal = garden.get(0, None)
    if (topGoal is None) or ('G' != topGoal.get('type', None)):
        return topGoal, requests.status_codes.codes.INTERNAL_SERVER_ERROR, {'response': 'Malformed garden: missing top goal.'}
    error = topGoal.get('error', None)
    if (error is not None):
        return requests.status_codes.codes.PRECONDITION_FAILED, {'response': ('Top goal error: %s' % error)}
    return topGoal, requests.status_codes.codes.ALL_OK, {}
    
def toFetchStart(requestData, w, agentName, todos):
    oname = requestData.get('object', None)
    lacks = _checkArgs([[oname, "Request lacks object parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if oname not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested object does not exist in world.'}
    kitchenCounter = None
    for k, v in w._kinematicTrees.items():
        if 'KitchenCounter' == v['type']:
            kitchenCounter = k
            break
    if kitchenCounter is None:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'World seems not to have a kitchen counter.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'placedItem', 'item': oname, 'hand': 'hand_right', 'container': kitchenCounter}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}
    
def toFetchEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'fetchedObject': requestData.get('object', None), 'kitchenStateOut': w.worldDump()}}

def toPortionStart(requestData, w, agentName, todos):
    oname = requestData.get("containerWithIngredient", None)
    storeName = requestData.get("targetContainer", None)
    amount = requestData.get("quantity", None)
    ### TODO: pick container based on contents concept, as expressed by a key 'ingredientConcept' in requests_data
    ### TODO: also read a 'unit' key from requests_data
    ### TODO: associate a mass and/or volume with portioned particles
    amount = int(amount/30.0)
    lacks = _checkArgs([[oname, "Request lacks containetWithIngredient parameter."],
                        [amount, "Request lacks quantity parameter."],
                        [storeName, "Request lacks targetContainer parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if oname not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithIngredient does not exist in world.'}
    if storeName not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested targetContainer does not exist in world.'}
    storage = _getStorage(w)
    pouredType = _getPouredType(w, oname)
    if storage is None:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Nowhere to return containerWithIngredient to.'}
    if pouredType is None:
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'containerWithIngredient is not portionable.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'transferredAndStored', 'item': oname, 'hand': 'hand_right', 'container': storeName, 'storage': storage, 'amount': amount, 'pouredType': pouredType}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}
    
def toPortionEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'outputContainer': requestData.get('targetContainer', None), 'kitchenStateOut': w.worldDump()}}

def processActionRequest(fn, requestData, w, agentName, todos):
    if todos['currentAction'] is not None:
        return False, requests.status_codes.codes.SERVICE_UNAVAILABLE, 'Robot already performs an action.'
    status, response = fn(requestData, w, agentName, todos)
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
    "to-get-state-updates": [processInstantRequest, toGetStateUpdates, None],
    "to-get-location": [processInstantRequest, toGetLocation, None],
    "to-fetch": [processActionRequest, toFetchStart, toFetchEnd],
    "to-portion": [processActionRequest, toPortionStart, toPortionEnd]}
