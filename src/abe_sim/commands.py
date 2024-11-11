import copy
import math
import numpy
import pybullet
import requests

from abe_sim.world import stubbornTry
from abe_sim.handle_abort_commands import handle_abort_commands

def _cancelGardenAction(w, agentName, todos):
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), {})
    if todos["currentAction"] is not None:
        todos["cancelled"] = True
        
def _cancelClopens(w, agentName):
    for ef in w.getObjectProperty((agentName,), ('fn', 'clopening', 'clopeningEFs'), []):
        w.setObjectProperty((agentName,), ('customStateVariables', 'clopening', 'action', ef), None)
        
def _checkArgs(args):
    retq = []
    for a in args:
        var, msg = a
        if var is None:
            retq.append(msg)
    return retq
    
def _getStorage(w, working=False, temperature=None):
    # TODO: choose storage more intelligently so as to avoid overcrowding one piece of furniture.
    if 'hot' == temperature:
        types = ['Oven']
    elif 'cold' == temperature:
        types = ['Fridge']
    else:
        types = ['KitchenCabinet', 'Pantry']
        if working:
            types = ['KitchenCounter'] + types
    for e in types:
        for k, v in w._kinematicTrees.items():
            if e == v['type']:
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
    def _getHand(w, agentName, item):
        free = None
        for ef in w.getObjectProperty((agentName,), ('fn', 'grasping', 'effectors'), []):
            iGs = w.getObjectProperty((agentName,), ('customStateVariables', 'grasping', 'intendToGrasp', ef))
            if item in w.getObjectProperty((agentName,), ('customStateVariables', 'grasping', 'intendToGrasp', ef)):
                return ef
            if 0 == len(iGs):
                free = ef
        if free is None:
            free = "hand_right"
        return free
    def _pddl2PG(step):
        action, parameters = step
        if "put" == action:
            #:action put :parameters (?gr - locatable ?dest - container)
            item, dest = parameters
            hand = _getHand(w, agentName, item) # put is used for objects already in hand
            return {'type': 'G', 'description': {'goal': 'placedItem', 'item': item, 'container': dest, 'hand': hand}}
        if "move" == action:
            #:action move :parameters (?gr - locatable ?src - container ?dest - container)
            item, _, dest = parameters
            return {'type': 'G', 'description': {'goal': 'placedItem', 'item': item, 'container': dest, 'hand': 'hand_right'}}
        if "close" == action:
            #:action close :parameters (?clo - clopenable)
            item = parameters[0]
            return {'type': 'G', 'description': {'goal': 'closed', 'container': item, 'hand': 'hand_right'}}
        if "turnoff" == action:
            #:action turnoff :parameters (?dev - device)
            item = parameters[0]
            return {'type': 'G', 'description': {'goal': 'turnedOff', 'device': dest, 'hand': 'hand_right'}}
    plan = None
    garden = w.getObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), {})
    if 0 not in garden:
        return requests.status_codes.codes.ALL_OK, {"response": "Ok."}
    if requestData.get("smart", False):
        topGoal = garden[0]["description"]
        blacklist = set(["goal", "process", "amount", "unit", "duration", "hand", "temperature"])
        relevantEntities = set([v for k,v in topGoal.items() if k not in blacklist])
        for ef in w.getObjectProperty((agentName,), ('fn', 'grasping', 'effectors'), []):
            intended = w.getObjectProperty((agentName,), ('customStateVariables', 'grasping', 'intendToGrasp', ef), [])
            relevantEntities = relevantEntities.union(intended)
        plan = handle_abort_commands(w, relevantEntities)
    if (not requestData.get("smart", False)) or (plan is None) or (0 == len(plan)):
        for ef in w.getObjectProperty((agentName,), ('fn', 'grasping', 'effectors'), []):
            w.setObjectProperty((agentName,), ('customStateVariables', 'grasping', 'intendToGrasp', ef), [])
        _cancelClopens(w, agentName)
        _cancelGardenAction(w, agentName, todos)
        #return requests.status_codes.codes.ALL_OK, {"response": "Ok."}
    else:
        garden = {0: _pddl2PG(plan[0])}
        w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
        todos['goals'] = [{0: _pddl2PG(x)} for x in plan[1:]]
    return requests.status_codes.codes.ALL_OK, {"response": "Ok."}


def toUpdateAvatar(requestData, w, agentName, todos):
    def _adjustTarget(p,q,ef,csv):
        if isinstance(p, str):
            p = json.loads(p)
        if isinstance(q, str):
            q = json.loads(q)
        #previousTarget = csv.get("kinematicControl", {}).get("target", {}).get(ef)
        #if p is None:
        #    p = previousTarget[0]
        #if q is None:
        #    q = previousTarget[1]
        return p, q
    bea = requestData.get("avatarName", "bea")
    if (bea not in w._kinematicTrees):
        return requests.status_codes.codes.NOT_FOUND, {"response": "Did not find object %s." % bea}
    if ("Bea" != w._kinematicTrees[bea]["type"]):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {"response": "Object %s is not a human avatar." % bea}
    posHead = requestData.get("positionHead")
    ornHead = requestData.get("orientationHead")
    posLeft = requestData.get("positionLeft")
    ornLeft = requestData.get("orientationLeft")
    posRight = requestData.get("positionRight")
    ornRight = requestData.get("orientationRight")
    graspLeft = requestData.get("graspLeft", [])
    graspRight = requestData.get("graspRight", [])
    clopenLeft = requestData.get("clopenLeft")
    clopenRight = requestData.get("clopenRight")
    turnLeft = requestData.get("turnLeft", [])
    turnRight = requestData.get("turnRight", [])
    csv = w._kinematicTrees[bea]["customStateVariables"]
    posHead, ornHead = _adjustTarget(posHead, ornHead, "base", csv)
    posHead, ornHead = _adjustTarget(posHead, ornHead, "base", csv)
    posHead, ornHead = _adjustTarget(posHead, ornHead, "base", csv)
    targetBase = None
    targetLeft = None
    targetRight = None
    if (posHead is not None) and (ornHead is not None):
        targetBase = [[posHead[0], posHead[1], 0], stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,pybullet.getEulerFromQuaternion(ornHead)[2])))]
    if (posLeft is not None) and (ornLeft is not None):
        targetLeft = [posLeft, ornLeft]
    if (posRight is not None) and (ornRight is not None):
        targetRight = [posRight, ornRight]
    csv["kinematicControl"]["target"] = {"base": targetBase,
                                         "hand_left": targetLeft,
                                         "hand_right": targetRight}
    csv["grasping"]["intendToGrasp"] = {"hand_left": graspLeft, "hand_right": graspRight}
    csv["clopening"]["action"] = {"hand_left": clopenLeft, "hand_right": clopenRight}
    csv["turning"]["action"] = {"hand_left_roll": turnLeft, "hand_right_roll": turnRight}
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
    agent = requestData.get("agent")
    if agent is None:
        agent = agentName
    else:
        if agent not in w._kinematicTrees:
            return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested agent does not exist in world.'}
    position = [position[0], position[1], 0]
    mobileBaseLink = w.getObjectProperty((agent,), ('fn', 'kinematicControl', 'mobileBaseLink'))
    baseOrientation = w.getObjectProperty((agent, mobileBaseLink), 'orientation')
    _, _, crYaw = stubbornTry(lambda : pybullet.getEulerFromQuaternion(baseOrientation))
    yaw = angleAdjust(yaw, crYaw)
    _cancelClopens(w, agent)
    if agentName == agent:
        _cancelGardenAction(w, agent, todos)
    orientation = stubbornTry(lambda : pybullet.getQuaternionFromEuler((0,0,yaw)))
    w.setObjectProperty((agent,), ('customStateVariables', 'kinematicControl', 'target', 'base'), [position, orientation])
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

def toGetObjectConstants(requestData, w, agentName, todos):
    def _link2MinMax(w, oname, linkName):
        linkIdx = w._kinematicTrees[oname]["links"][linkName]["idx"]
        ans = stubbornTry(lambda : pybullet.getJointInfo(w._kinematicTrees[oname]["idx"], linkIdx, w._pybulletConnection))
        return (linkName, ans[8], ans[9])
    oname = requestData.get("object")
    if oname not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested object does not exist in world.'}
    mass = w.getObjectProperty((oname,), "mass")
    dispositions = {k:v for k,v in w._kinematicTrees[oname]["fn"].items() if isinstance(v, bool)}
    retq = {"mass": mass, "dispositions": dispositions, "clopenableLinks": [], "turnableLinks": []}
    if dispositions.get("clopenable", False):
        clopenableLinks = w.getObjectProperty((oname,), ("fn", "clopening", "clopenableLinks"))
        retq["clopenableLinks"] = [_link2MinMax(w, oname, l) for l in clopenableLinks]
    if dispositions.get("turnable", False):
        turnableLinks = w.getObjectProperty((oname,), ("fn", "turning", "links"))
        retq["turnableLinks"] = [_link2MinMax(w, oname, l) for l in turnableLinks]
    return requests.status_codes.codes.ALL_OK, {'response': retq}

def toGetStateUpdates(requestData, w, agentName, todos):
    def _joints2Links(objData, joints2Values):
        return {objData["idx2Link"][objData["joints"][k]["idx"]]: v for k, v in joints2Values.items()}
    def _active(p, garden):
        for e in p['children']:
            if (not garden[e]['previousStatus']) and ('constraintFollowed' != garden[e]['description']['goal']):
                return False
        return True
    def _strP(p):
        description = p['description']
        if 'nearing' == description['process']:
            return 'NavigateTo(%s)' % (description['relatum'])
        elif 'parkingArm' == description['process']:
            return 'ParkingArms(%s)' % (description['hand'])
        elif 'grasping' == description['process']:
            return 'SwitchingOnGrasping(%s, %s)' % (description['item'], description['hand'])
        elif 'ungrasping' == description['process']:
            return 'SwitchingOffGrasping(%s, %s)' % (description['item'], description['hand'])
        elif 'armNearingItemHandle' == description['process']:
            return 'PickingItem(%s, %s)' % (description['item'], description['hand'])
        elif 'loweringItem' == description['process']:
            return 'PlacingItem(%s, %s, %s)' % (description['item'], description['container'], description['hand'])
        elif 'opening' == description['process']:
            return 'PullingOpen(%s)' % (description['hand'])
        elif 'closing' == description['process']:
            return 'PushingClosed(%s)' % (description['hand'])
        elif 'transferringContents' == description['process']:
            return 'Pouring(%s, %s, %s)' % (description['item'], description['container'], description['hand'])
        elif 'mixing' == description['process']:
            return 'MixingStuff(%s, %s, %s)' % (description['container'], description['tool'], description['hand'])
        elif 'lining' == description['process']:
            return 'LiningContainer(%s, %s, %s)' % (description['item'], description['lining'], description['hand'])
        elif 'shaping' == description['process']:
            return 'ShapingStuffInto(%s, %s, %s, %s)' % (description['item'], description['shapedType'], description['destination'], description['hand'])
        elif 'sprinkling' == description['process']:
            return 'SprinklingContents(%s, %s, %s)' % (description['item'], description['shaker'], description['hand'])
        elif 'cuttingItem' == description['process']:
            return 'Chopping(%s, %s, %s)' % (description['item'], description['tool'], description['hand'])
        return ''
    trackedProcs = {'nearing', 'parkingArm', 'grasping', 'ungrasping', 'armNearingItemHandle', 'loweringItem', 'opening', 'closing', 'transferringContents', 'mixing', 'lining', 'shaping', 'sprinkling', 'cuttingItem'}
    garden = w.getObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), {})
    acts = [_strP(x) for x in garden.values() if ('process' in x['description']) and (x['description']['process'] in trackedProcs) and _active(x, garden)]
    agentLoad = w.getObjectProperty((agentName,), ("customStateVariables", "agentLoad"), 0)
    updates = {}
    for name, data in w._kinematicTrees.items():
        mesh = stubbornTry(lambda : pybullet.getVisualShapeData(data['idx'], -1, w._pybulletConnection))[0][4].decode("utf-8")
        updates[name] = {'filename': str(data['filename']), 'position': list(w.getObjectProperty((name,), 'position')), 'orientation': list(w.getObjectProperty((name,), 'orientation')), 'at': str(w.getObjectProperty((name,), 'at')), 'mesh': mesh, 'customStateVariables': copy.deepcopy(data.get('customStateVariables', {})), 'joints': _joints2Links(w._kinematicTrees[name], w.getObjectProperty((name,), 'jointPositions'))}
        if name == agentName:
            updates[name]['position'] = {'base': [updates[name]['joints']['base_x'], updates[name]['joints']['base_y'], 0], 'hand_right_roll': list(w.getObjectProperty((name, 'hand_right_roll'), 'position')), 'hand_left_roll': list(w.getObjectProperty((name, 'hand_left_roll'), 'position'))}
            halfYaw = 0.5*updates[name]['joints']['base_yaw']
            updates[name]['orientation'] = {'base': [0,0, math.sin(halfYaw), math.cos(halfYaw)], 'hand_right_roll': list(w.getObjectProperty((name, 'hand_right_roll'), 'orientation')), 'hand_left_roll': list(w.getObjectProperty((name, 'hand_left_roll'), 'orientation'))}
    return requests.status_codes.codes.ALL_OK, {"response": {"updates": updates, "agentLoad": agentLoad, "currentCommand": str(todos["command"]), "abeActions": acts}}

def toPreloadObject(requestData, w, agentName, todos):
    otype = requestData.get('type', None)
    if otype not in w._objectKnowledge:
        return requests.status_codes.codes.NOT_FOUND, {"response": ('World does not have object type %s in its knowledge base' % otype)}
    ## TODO: add a return value to check that this is ok ...
    w.preload(otype, None, [0,0,-10])
    return requests.status_codes.codes.ALL_OK, {'response': 'ok'}

def toGetLocation(requestData, w, agentName, todos):
    def _freeOfContent(o):
        if not w.getObjectProperty((o,), ('fn', 'canContain'), False):
            return True
        aabb = w.getObjectProperty((o,), 'aabb')
        overlaps = set([x[0] for x in w.checkOverlap(aabb)])
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
        return topGoal, requests.status_codes.codes.PRECONDITION_FAILED, {'response': ('Top goal error: %s' % error)}
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

def toBringToTemperatureStart(requestData, w, agentName, todos):
    container = requestData.get('containerWithIngredients', None)
    temperature = requestData.get('temperatureQuantity', None)
    unit = requestData.get('temperatureUnit', None)
    lacks = _checkArgs([[container, "Request lacks containerWithIngredients parameter."],
                        [temperature, "Request lacks temperatureQuantity parameter."],
                        [unit, "Request lacks temperatureUnit parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithIngredients does not exist in world.'}
    # TODO: temperature conversion. For now assume Celsius.
    timeStart = w.getObjectProperty((agentName,), ('customStateVariables', 'timing', 'timer'), None)
    if timeStart is None:
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Agent is not able to keep time.'}
    allowedComponents = None
    if 30 < temperature:
        destination = _getStorage(w, temperature='hot')
    elif 10 > temperature:
        destination = _getStorage(w, temperature='cold')
        if (4 > temperature) and (destination is not None):
            fn = w._kinematicTrees[destination].get('fn', {})
            allowedComponents = list(set(fn.get('refrigeration', {}).get('links', [])).intersection(fn.get('containment', {}).get('links', [])))
            if 0 == len(allowedComponents):
                return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Found no place to refrigerate in.'}
            component = 'freezer'
    else:
        destination = _getStorage(w, working=True)
    if destination is None:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Cannot find a destination object of appropriate temperature.'}    
    garden = {0: {'type': 'G', 'description': {'goal': 'placedItem', 'container': destination, 'allowedComponents': allowedComponents, 'hand': 'hand_right', 'item': container}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    # TODO: adaptive wait, e.g. perhaps some new goal of the robot
    time = 600 # wait 10min for something to get to the temperature
    todos['goals'] = [{0: {'type': 'G', 'description': {'goal': 'waited', 'timeEnd': time + timeStart}}}]
    return requests.status_codes.codes.ALL_OK, {}
        
def toBringToTemperatureEnd():
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithIngredientsAtTemperature': requestData.get('containerWithIngredients', None), 'kitchenStateOut': w.worldDump()}}

def toWaitStart(requestData, w, agentName, todos):
    time = requestData.get('timeToWait', None)
    unit = requestData.get('timeUnit', None)
    lacks = _checkArgs([[time, "Request lacks timeToWait parameter."],
                        [unit, "Request lacks timeUnit parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    conversion = {'s': 1, 'sec': 1, 'second': 1, 'seconds': 1,
                  'm': 60, 'min': 60, 'minute': 60, 'minutes': 60,
                  'h': 3600, 'hr': 3600, 'hrs': 3600, 'hour': 3600, 'hours': 3600,
                  'd': 86400, 'ds': 86400, 'day': 86400, 'days': 86400}.get(unit, None)
    if conversion is None:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested unit not found among time units.'}
    time = 1.0*time
    timeStart = w.getObjectProperty((agentName,), ('customStateVariables', 'timing', 'timer'), None)
    if timeStart is None:
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Agent is not able to keep time.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'waited', 'timeEnd': time + timeStart}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}
    
def toWaitEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'kitchenStateOut': w.worldDump()}}

def toLeaveForTimeStart(requestData, w, agentName, todos):
    container = requestData.get('containerWithIngredients', None)
    time = requestData.get('coolingQuantity', None)
    unit = requestData.get('timeUnit', None)
    lacks = _checkArgs([[container, "Request lacks containerWithIngredients parameter."],
                        [time, "Request lacks coolingQuantity parameter."],
                        [unit, "Request lacks timeUnit parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithIngredients does not exist in world.'}
    conversion = {'s': 1, 'sec': 1, 'second': 1, 'seconds': 1,
                  'm': 60, 'min': 60, 'minute': 60, 'minutes': 60,
                  'h': 3600, 'hr': 3600, 'hrs': 3600, 'hour': 3600, 'hours': 3600,
                  'd': 86400, 'ds': 86400, 'day': 86400, 'days': 86400}.get(unit, None)
    if conversion is None:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested unit not found among time units.'}
    time = 1.0*time
    timeStart = w.getObjectProperty((agentName,), ('customStateVariables', 'timing', 'timer'), None)
    if timeStart is None:
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Agent is not able to keep time.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'waited', 'timeEnd': time + timeStart}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}
    
def toLeaveForTimeEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithIngredientsAtTemperature': requestData.get('containerWithIngredients', None), 'kitchenStateOut': w.worldDump()}}

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

def toTransferStart(requestData, w, agentName, todos):
    oname = requestData.get("containerWithInputIngredients", None)
    storeName = requestData.get("targetContainer", None)
    lacks = _checkArgs([[oname, "Request lacks containetWithInputIngredients parameter."],
                        [storeName, "Request lacks targetContainer parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if oname not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithInputIngredients does not exist in world.'}
    if storeName not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested targetContainer does not exist in world.'}
    storage = _getStorage(w)
    if storage is None:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Nowhere to return containerWithInputIngredients to.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'transferredAndStored', 'item': oname, 'hand': 'hand_right', 'container': storeName, 'storage': storage, 'amount': None, 'pouredType': None}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}
    
def toTransferEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithAllIngredients': requestData.get('targetContainer', None), 'kitchenStateOut': w.worldDump()}}
    
def toBeatStart(requestData, w, agentName, todos):
    container = requestData.get("containerWithIngredients", None)
    tool = requestData.get("tool", None)
    lacks = _checkArgs([[container, "Request lacks containerWithIngredients parameter."],
                        [tool, "Request lacks tool parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithIngredients does not exist in world.'}
    if tool not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested tool does not exist in world.'}
    if not w._kinematicTrees[tool].get('fn', {}).get('canMix', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested tool cannot mix.'}
    mixedType = 'default' # TODO: get mixed type
    toolLink = w._kinematicTrees[tool]['fn']['mixing']['links'][0]
    storage = _getStorage(w)
    garden = {0: {'type': 'G', 'description': {'goal': 'mixedAndStored', 'tool': tool, 'toolLink': toolLink, 'hand': 'hand_right', 'container': container, 'storage': storage, 'mixedType': mixedType}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toBeatEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithIngredientsBeaten': requestData.get('containerWithIngredients', None), 'kitchenStateOut': w.worldDump()}}

def toMixStart(requestData, w, agentName, todos):
    container = requestData.get("containerWithInputIngredients", None)
    tool = requestData.get("mixingTool", None)
    lacks = _checkArgs([[container, "Request lacks containerWithInputIngredients parameter."],
                        [tool, "Request lacks mixingTool parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithInputIngredients does not exist in world.'}
    if tool not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested mixingTool does not exist in world.'}
    if not w._kinematicTrees[tool].get('fn', {}).get('canMix', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested mixingTool cannot mix.'}
    mixedType = 'default' # TODO: get mixed type
    toolLink = w._kinematicTrees[tool]['fn']['mixing']['links'][0]
    storage = _getStorage(w)
    garden = {0: {'type': 'G', 'description': {'goal': 'mixedAndStored', 'tool': tool, 'toolLink': toolLink, 'hand': 'hand_right', 'container': container, 'storage': storage, 'mixedType': mixedType}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toMixEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithMixture': requestData.get('containerWithInputIngredients', None), 'kitchenStateOut': w.worldDump()}}

def toMingleStart(requestData, w, agentName, todos):
    container = requestData.get("containerWithInputIngredients", None)
    tool = requestData.get("minglingTool", None)
    lacks = _checkArgs([[container, "Request lacks containerWithInputIngredients parameter."],
                        [tool, "Request lacks minglingTool parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithInputIngredients does not exist in world.'}
    if tool not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested minglingTool does not exist in world.'}
    if not w._kinematicTrees[tool].get('fn', {}).get('canMix', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested minglingTool cannot mix.'}
    mixedType = None
    toolLink = w._kinematicTrees[tool]['fn']['mixing']['links'][0]
    storage = _getStorage(w)
    garden = {0: {'type': 'G', 'description': {'goal': 'mixedAndStored', 'tool': tool, 'toolLink': toolLink, 'hand': 'hand_right', 'container': container, 'storage': storage, 'mixedType': mixedType}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toMingleEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithMixture': requestData.get('containerWithInputIngredients', None), 'kitchenStateOut': w.worldDump()}}

def toMashStart(requestData, w, agentName, todos):
    inputIngredient = requestData.get("inputIngredient", None)
    tool = requestData.get("mashingTool", None)
    lacks = _checkArgs([[inputIngredient, "Request lacks inputIngredient parameter."],
                        [tool, "Request lacks mashingTool parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if inputIngredient not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested inputIngredient does not exist in world.'}
    if tool not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested mashingTool does not exist in world.'}
    if not w._kinematicTrees[tool].get('fn', {}).get('canMash', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested mashingTool cannot mash.'}
    toolLink = w._kinematicTrees[tool]['fn']['mashing']['links'][0]
    storage = _getStorage(w)
    garden = {0: {'type': 'G', 'description': {'goal': 'mashedAndStored', "disposition": "mashable", 'tool': tool, 'toolLink': toolLink, 'hand': 'hand_right', 'container': inputIngredient, 'storage': storage}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}
    
def toMashEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'mashedIngredient': requestData.get('inputIngredient', None), 'kitchenStateOut': w.worldDump()}}

def toGrindStart(requestData, w, agentName, todos):
    inputIngredient = requestData.get("containerWithIngredientsToBeGround", None)
    tool = requestData.get("grindingTool", None)
    lacks = _checkArgs([[inputIngredient, "Request lacks containerWithIngredientsToBeGround parameter."],
                        [tool, "Request lacks grindingTool parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if inputIngredient not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithIngredientsToBeGround does not exist in world.'}
    if tool not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested grindingTool does not exist in world.'}
    if not w._kinematicTrees[tool].get('fn', {}).get('canGrind', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested grindingTool cannot grind.'}
    toolLink = w._kinematicTrees[tool]['fn']['grinding']['links'][0]
    storage = _getStorage(w)
    garden = {0: {'type': 'G', 'description': {'goal': 'mashedAndStored', "disposition": "grindable", 'tool': tool, 'toolLink': toolLink, 'hand': 'hand_right', 'container': inputIngredient, 'storage': storage}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}
    
def toGrindEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithGroundIngredients': requestData.get('containerWithIngredientsToBeGround', None), 'kitchenStateOut': w.worldDump()}}

def toFlattenStart(requestData, w, agentName, todos):
    portions = requestData.get("portion", None)
    tool = requestData.get("flatteningTool", None)
    lacks = _checkArgs([[portions, "Request lacks portions parameter."],
                        [tool, "Request lacks flatteningTool parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    if 0 == len(portions):
        return requests.status_codes.codes.BAD_REQUEST, {"response": "Must request flattening something."}
    _checkGreatReset(requestData, w)
    if any([x not in w._kinematicTrees for x in portions]):
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested portion does not exist in world.'}
    if tool not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested flatteningTool does not exist in world.'}
    if not w._kinematicTrees[tool].get('fn', {}).get('canFlatten', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested flatteningTool cannot flatten.'}
    toolLink = w._kinematicTrees[tool]['fn']['flattening']['links'][0]
    storage = _getStorage(w)
    requestData["at"] = w.at((portions[0],))
    garden = {0: {'type': 'G', 'description': {'goal': 'mashedAndStored', "disposition": "flattenable", 'tool': tool, 'toolLink': toolLink, 'hand': 'hand_right', 'container': portions[0], 'storage': storage}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = [{0: {'type': 'G', 'description': {'goal': 'mashedAndStored', "disposition": "flattenable", 'tool': tool, 'toolLink': toolLink, 'hand': 'hand_right', 'container': portion, 'storage': storage}}} for portion in portions[1:]]
    return requests.status_codes.codes.ALL_OK, {}
    
def toFlattenEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithFlattenedItems': requestData.get('at', None), 'kitchenStateOut': w.worldDump()}}

def toLineStart(requestData, w, agentName, todos):
    item = requestData.get("bakingTray", None)
    lining = requestData.get("bakingPaper", None)
    lacks = _checkArgs([[item, "Request lacks bakingTray parameter."],
                        [lining, "Request lacks bakingPaper parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested bakingTray does not exist in world.'}
    if lining not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested bakingPaper does not exist in world.'}
    if not w._kinematicTrees[lining].get('fn', {}).get('canLine', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested bakingPaper cannot line.'}
    if not w._kinematicTrees[item].get('fn', {}).get('lineable', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested bakingTray cannot be lined.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'linedAndParked', 'lining': lining, 'hand': 'hand_right', 'item': item}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toLineEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'linedBakingTray': requestData.get('bakingTray', None), 'kitchenStateOut': w.worldDump()}}

def toCoverStart(requestData, w, agentName, todos):
    item = requestData.get("object", None)
    cover = requestData.get("cover", None)
    lacks = _checkArgs([[item, "Request lacks object parameter."],
                        [cover, "Request lacks cover parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested object does not exist in world.'}
    if cover not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested cover does not exist in world.'}
    if not w._kinematicTrees[cover].get('fn', {}).get('canCover', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested cover cannot cover.'}
    if not w._kinematicTrees[item].get('fn', {}).get('coverable', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested object cannot be covered.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'coveredAndParked', 'cover': cover, 'hand': 'hand_right', 'item': item}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toCoverEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'coveredObject': requestData.get('object', None), 'kitchenStateOut': w.worldDump()}}

def toUncoverStart(requestData, w, agentName, todos):
    item = requestData.get("object", None)
    lacks = _checkArgs([[item, "Request lacks object parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    storage = _getStorage(w)
    contacts = [x[1][0] for x in w.checkCollision((item,)) if w._kinematicTrees[x[1][0]].get("fn",{}).get("canCover", False)]
    if 0 == len(contacts):
        garden = {0: {'type': "G", "description": {"goal": "done"}}}
        cover = None
    else:
        cover = contacts[0]
        if item not in w._kinematicTrees:
            return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested object does not exist in world.'}
        if not w._kinematicTrees[item].get('fn', {}).get('coverable', False):
            return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested object cannot be covered.'}
        garden = {0: {'type': 'G', 'description': {'goal': 'placedItem', "container": storage, "hand": "hand_right", "item": cover}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    requestData["cover"] = cover
    return requests.status_codes.codes.ALL_OK, {}

def toUncoverEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'uncoveredObject': requestData.get('object', None), "cover": requestData.get("cover", None), 'kitchenStateOut': w.worldDump()}}

def toPortionAndArrangeStart(requestData, w, agentName, todos):
    container = requestData.get("containerWithDough", None)
    destination = requestData.get("destination", None)
    shapedType = 'DoughClump' ### TODO
    ingredientTypes = {'DoughParticle': 3} ### TODO
    lacks = _checkArgs([[container, "Request lacks containerWithDough parameter."],
                        [destination, "Request lacks destination parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithDough does not exist in world.'}
    if destination not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested destination does not exist in world.'}
    if not w._kinematicTrees[container].get('fn', {}).get('canContain', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested containerWithDough cannot contain.'}
    if not w._kinematicTrees[destination].get('fn', {}).get('canContain', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested destination cannot contain.'}
    _checkGreatReset(requestData, w)
    garden = {0: {'type': 'G', 'description': {'goal': 'shapedAndParked', 'item': container, 'hand': 'hand_right', 'destination': destination, 'shapedType': shapedType, 'ingredientTypes': ingredientTypes, "itemIsShaped": False}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toPortionAndArrangeEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    shapes = w.getObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'log', 'shaping', 'products'), [])
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'log', 'shaping', 'products'), [])
    return requests.status_codes.codes.ALL_OK, {'response': {'portions': shapes, 'kitchenStateOut': w.worldDump()}}

def toShapeStart(requestData, w, agentName, todos):
    portions = requestData.get("portions", None)
    shapedType = 'DoughClump' ### TODO
    lacks = _checkArgs([[portions, "Request lacks portions parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    if any([portion not in w._kinematicTrees for portion in portions]):
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested portions do not exist in world.'}
    if any([not w._kinematicTrees[portion].get('fn', {}).get('shapeable', False) for portion in portions]):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested portions are not shapeable.'}
    _checkGreatReset(requestData, w)
    if 0 == len(portions):
        return requests.status_codes.codes.ALL_OK, {}
    garden = {0: {'type': 'G', 'description': {'goal': 'shapedAndParked', 'itemIsShaped': True, 'item': portions[0], 'hand': 'hand_right', 'destination': w.getObjectProperty((portions[0],), 'at'), 'shapedType': shapedType, 'ingredientTypes': {w.getObjectProperty((portions[0],), 'type'): 1}}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = [{0: {'type': 'G', 'description': {'goal': 'shapedAndParked', 'itemIsShaped': True, 'item': portion, 'hand': 'hand_right', 'destination': w.getObjectProperty((portion,), 'at'), 'shapedType': shapedType, 'ingredientTypes': {w.getObjectProperty((portion,), 'type'): 1}}}} for portion in portions[1:]]
    return requests.status_codes.codes.ALL_OK, {}

def toShapeEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    #shapes = w.getObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'log', 'shaping', 'products'), [])
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'log', 'shaping', 'products'), [])
    return requests.status_codes.codes.ALL_OK, {'response': {'shapedPortions': requestData.get('portions', None), 'kitchenStateOut': w.worldDump()}}

def toBakeStart(requestData, w, agentName, todos):
    item = requestData.get("thingToBake", None)
    oven = requestData.get("oven", None)
    destination = requestData.get("inputDestinationContainer", None)
    bakedType = 'Cookie' ### TODO
    lacks = _checkArgs([[item, "Request lacks thingToBake parameter."],
                        [oven, "Request lacks oven parameter."],
                        [destination, "Request lacks inputDestinationContainer parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested thingToBake does not exist in world.'}
    if oven not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested oven does not exist in world.'}
    if destination not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested inputDestinationContainer does not exist in world.'}
    if not w._kinematicTrees[oven].get('fn', {}).get('canBake', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested oven cannot bake.'}
    if not w._kinematicTrees[destination].get('fn', {}).get('canContain', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested inputDestinationContainer cannot contain.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'bakedItem', 'oven': oven, 'hand': 'hand_right', 'item': item, 'destination': destination, 'processDisposition': 'bakeable', 'bakedType': bakedType}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toBakeEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'thingBaked': requestData['thingToBake'], 'outputDestinationContainer': requestData['inputDestinationContainer'], 'kitchenStateOut': w.worldDump()}}

def toBoilStart(requestData, w, agentName, todos):
    item = requestData.get("thingToBoil", None)
    oven = requestData.get("stoveToBoilOn", None)
    heatingMode = requestData.get("heatingMode", None)
    timeToBoilAmount = requestData.get("timeToBoilQuantity", None)
    timeToBoilUnit = requestData.get("timeToBoilUnit", None)
    destination = _getStorage(w, working=True)
    boiledType = 'BoiledPotatoes' ### TODO
    lacks = _checkArgs([[item, "Request lacks thingToBoil parameter."],
                        [oven, "Request lacks stoveToBoilOn parameter."],
                        [heatingMode, "Request lacks heatingMode parameter."],
                        [timeToBoilAmount, "Request lacks timeToBoilQuantity parameter."],
                        [timeToBoilUnit, "Request lacks timeToBoilUnit parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested thingToBoil does not exist in world.'}
    if oven not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested stoveToBoilOn does not exist in world.'}
    if not w._kinematicTrees[oven].get('fn', {}).get('canBoil', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested stoveToBoilOn cannot boil.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'bakedItem', 'oven': oven, 'hand': 'hand_right', 'item': item, 'destination': destination, 'bakedType': boiledType, "processDisposition": "boilable", "timeAmount": timeToBoilAmount, "timeUnit": timeToBoilUnit}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toBoilEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'thingBoiled': requestData['thingToBoil'], 'kitchenStateOut': w.worldDump()}}

def toFryStart(requestData, w, agentName, todos):
    item = requestData.get("thingToFry", None)
    oven = requestData.get("stoveToFryOn", None)
    heatingMode = requestData.get("heatingMode", None)
    timeToFryAmount = requestData.get("timeToFryQuantity", None)
    timeToFryUnit = requestData.get("timeToFryUnit", None)
    destination = _getStorage(w, working=True)
    friedType = 'CookedBacon' ### TODO
    lacks = _checkArgs([[item, "Request lacks thingToFry parameter."],
                        [oven, "Request lacks stoveToFryOn parameter."],
                        [heatingMode, "Request lacks heatingMode parameter."],
                        [timeToFryAmount, "Request lacks timeToFryQuantity parameter."],
                        [timeToFryUnit, "Request lacks timeToFryUnit parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested thingToFry does not exist in world.'}
    if oven not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested stoveToFryOn does not exist in world.'}
    if not w._kinematicTrees[oven].get('fn', {}).get('canFry', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested stoveToFryOn cannot fry.'}
    unit = 1.0
    if timeToFryUnit in ["min", "m", "minute", "minutes"]:
        unit = 60.0
    elif timeToFryUnit in ["hr", "hrs", "h", "hour", "hours"]:
        unit = 3600
    garden = {0: {'type': 'G', 'description': {'goal': 'bakedItem', 'oven': oven, 'hand': 'hand_right', 'item': item, 'destination': destination, 'bakedType': friedType, "processDisposition": "fryable", "timeAmount": timeToFryAmount, "timeUnit": unit}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toFryEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'thingFried': requestData['thingToFry'], 'kitchenStateOut': w.worldDump()}}

def toMeltStart(requestData, w, agentName, todos):
    item = requestData.get("containerWithInputIngredients", None)
    oven = requestData.get("meltingTool", None)
    destination = _getStorage(w, working=True)
    meltedType = 'MeltedButter' ### TODO
    lacks = _checkArgs([[item, "Request lacks containerWithInputIngredients parameter."],
                        [oven, "Request lacks meltingTool parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithInputIngredients does not exist in world.'}
    if oven not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested meltingTool does not exist in world.'}
    if not w._kinematicTrees[oven].get('fn', {}).get('canMelt', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested meltingTool cannot melt.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'bakedItem', 'oven': oven, 'hand': 'hand_right', 'item': item, 'destination': destination, 'bakedType': meltedType, "processDisposition": "meltable"}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toMeltEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithMeltedIngredients': requestData['containerWithInputIngredients'], 'kitchenStateOut': w.worldDump()}}

def toWashStart(requestData, w, agentName, todos):
    item = requestData.get("thingToWash", None)
    sink = requestData.get("sink", None)
    destination = requestData.get("inputDestinationContainer", None)
    washedType = 'Broccoli' ### TODO
    lacks = _checkArgs([[item, "Request lacks thingToWash parameter."],
                        [sink, "Request lacks sink parameter."],
                        [destination, "Request lacks inputDestinationContainer parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested thingToWash does not exist in world.'}
    if sink not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested sink does not exist in world.'}
    if destination not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested inputDestinationContainer does not exist in world.'}
    if not w._kinematicTrees[sink].get('fn', {}).get('canWash', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested sink cannot bake.'}
    if not w._kinematicTrees[destination].get('fn', {}).get('canContain', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested inputDestinationContainer cannot contain.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'bakedItem', 'oven': sink, 'hand': 'hand_right', 'item': item, 'destination': destination, 'processDisposition': 'washable', 'bakedType': washedType}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toWashEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'thingWashed': requestData['thingToWash'], 'outputDestinationContainer': requestData['inputDestinationContainer'], 'kitchenStateOut': w.worldDump()}}

def toSprinkleStart(requestData, w, agentName, todos):
    item = requestData.get("object", None)
    shaker = requestData.get("toppingContainer", None)
    storage = _getStorage(w)
    sprinkledType = 'Cookie' ### TODO
    lacks = _checkArgs([[item, "Request lacks object parameter."],
                        [shaker, "Request lacks toppingContainer parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested object does not exist in world.'}
    if shaker not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested shaker does not exist in world.'}
    _checkGreatReset(requestData, w)
    garden = {0: {'type': 'G', 'description': {'goal': 'sprinkled', 'shaker': shaker, 'hand': 'hand_right', 'item': item, 'storage': storage, 'sprinkledType': sprinkledType}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toSprinkleEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'sprinkledObject': requestData['object'], 'kitchenStateOut': w.worldDump()}}

def toFlourStart(requestData, w, agentName, todos):
    item = requestData.get("containerToFlour", None)
    shaker = requestData.get("ingredientToFlourWith", None)
    storage = _getStorage(w)
    sprinkledType = 'FlouredTray' ### TODO
    lacks = _checkArgs([[item, "Request lacks containerToFlour parameter."],
                        [shaker, "Request lacks ingredientToFlourWith parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerToFlour does not exist in world.'}
    if shaker not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested ingredientToFlourWith does not exist in world.'}
    _checkGreatReset(requestData, w)
    garden = {0: {'type': 'G', 'description': {'goal': 'sprinkled', 'shaker': shaker, 'hand': 'hand_right', 'item': item, 'storage': storage, 'sprinkledType': sprinkledType}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toFlourEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'flouredContainer': requestData['containerToFlour'], 'kitchenStateOut': w.worldDump()}}

def toGreaseStart(requestData, w, agentName, todos):
    item = requestData.get("containerToGrease", None)
    shaker = requestData.get("ingredientToGreaseWith", None)
    storage = _getStorage(w)
    sprinkledType = 'GreasedTray' ### TODO
    lacks = _checkArgs([[item, "Request lacks containerToGrease parameter."],
                        [shaker, "Request lacks ingredientToGreaseWith parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerToGrease does not exist in world.'}
    if shaker not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested ingredientToGreaseWith does not exist in world.'}
    _checkGreatReset(requestData, w)
    garden = {0: {'type': 'G', 'description': {'goal': 'sprinkled', 'shaker': shaker, 'hand': 'hand_right', 'item': item, 'storage': storage, 'sprinkledType': sprinkledType}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toGreaseEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'greasedContainer': requestData['containerToGrease'], 'kitchenStateOut': w.worldDump()}}


def toCutStart(requestData, w, agentName, todos):
    item = requestData.get("object", None)
    tool = requestData.get("cuttingTool", None)
    # TODO: implement several patterns
    cutPattern = requestData.get("cutPattern", None)
    storage = _getStorage(w, working=True)
    lacks = _checkArgs([[item, "Request lacks object parameter."],
                        [tool, "Request lacks cuttingTool parameter."],
                        [cutPattern, "Request lacks cutPattern parameter"]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested object does not exist in world.'}
    if tool not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested cuttingTool does not exist in world.'}
    if not w._kinematicTrees[item].get('fn', {}).get('cuttable', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested object is not cuttable.'}
    if not w._kinematicTrees[tool].get('fn', {}).get('canCut', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested cuttingTool cannot cut.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'cutItem', 'tool': tool, 'hand': 'hand_right', 'item': item, 'storage': storage}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toCutEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'cutObject': requestData['object'], 'kitchenStateOut': w.worldDump()}}

def toPeelStart(requestData, w, agentName, todos):
    item = requestData.get("inputIngredient", None)
    tool = requestData.get("peelingTool", None)
    container = requestData.get("containerForPeeledIngredient", None)
    containerForPeels = requestData.get("containerForPeels", None)
    storage = _getStorage(w, working=True)
    lacks = _checkArgs([[item, "Request lacks inputIngredient parameter."],
                        [tool, "Request lacks peelingTool parameter."],
                        [container, "Request lacks containerForPeeledIngredient."],
                        [containerForPeels, "Request lacks containerForPeels."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested inputIngredient does not exist in world.'}
    if tool not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested peelingTool does not exist in world.'}
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerForPeeledIngredient does not exist in world.'}
    if containerForPeels not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerForPeels does not exist in world.'}
    if not w._kinematicTrees[tool].get('fn', {}).get('canPeel', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested peelingTool cannot peel.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'peeledAndStored', 'tool': tool, 'hand': 'hand_right', "otherHand": "hand_left", 'item': item, 'storage': storage, 'container': container, "containerForPeels": containerForPeels, "disposition": "peelable"}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toPeelEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {"peeledIngredient": requestData["containerForPeeledIngredient"], 'peelOfIngredient': requestData['inputIngredient'], 'kitchenStateOut': w.worldDump()}}

def toSeedStart(requestData, w, agentName, todos):
    item = requestData.get("inputIngredient", None)
    tool = requestData.get("seedingTool", None)
    container = requestData.get("containerForSeededIngredient", None)
    containerForSeeds = requestData.get("containerForSeeds", None)
    storage = _getStorage(w, working=True)
    lacks = _checkArgs([[item, "Request lacks inputIngredient parameter."],
                        [tool, "Request lacks seedingTool parameter."],
                        [container, "Request lacks containerForSeededIngredient."],
                        [containerForPeels, "Request lacks containerForSeeds."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested inputIngredient does not exist in world.'}
    if tool not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested seedingTool does not exist in world.'}
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerForSeededIngredient does not exist in world.'}
    if containerForSeeds not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerForSeeds does not exist in world.'}
    if not w._kinematicTrees[tool].get('fn', {}).get('canSeed', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested seedingTool cannot seed.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'peeledAndStored', 'tool': tool, 'hand': 'hand_right', "otherHand": "hand_left", 'item': item, 'storage': storage, 'container': container, "containerForPeels": containerForSeeds, "disposition": "seedable"}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toSeedEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {"seededIngredient": requestData["containerForSeededIngredient"], 'seedOfIngredient': requestData['inputIngredient'], 'kitchenStateOut': w.worldDump()}}

def toPlaceStart(requestData, w, agentName, todos):
    item = requestData.get("object", None)
    container = requestData.get("container", None)
    lacks = _checkArgs([[item, "Request lacks object parameter."],
                        [container, "Request lacks container parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested object does not exist in world.'}
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested container does not exist in world.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'placedItem', 'container': container, 'hand': 'hand_right', 'item': item}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toPlaceEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'placedObject': requestData['object'], 'kitchenStateOut': w.worldDump()}}

def toTransferItemsStart(requestData, w, agentName, todos):
    items = requestData.get("itemsToTransfer", None)
    container = requestData.get("destination", None)
    lacks = _checkArgs([[items, "Request lacks itemsToTransfer parameter."],
                        [container, "Request lacks destination parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if any([item not in w._kinematicTrees for item in items]):
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested item does not exist in world.'}
    if container not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested destination does not exist in world.'}
    if 0 == len(items):
        return requests.status_codes.codes.ALL_OK, {}
    garden = {0: {'type': 'G', 'description': {'goal': 'placedItem', 'container': container, 'hand': 'hand_right', 'item': items[0]}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = [{0: {'type': 'G', 'description': {'goal': 'placedItem', 'container': container, 'hand': 'hand_right', 'item': item}}} for item in items[1:]]
    return requests.status_codes.codes.ALL_OK, {}

def toTransferItemsEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'transferredContainer': requestData['destination'], 'kitchenStateOut': w.worldDump()}}

def toRefrigerateStart(requestData, w, agentName, todos):
    item = requestData.get("containerWithIngredients", None)
    refrigerator = requestData.get("refrigerator", None)
    lacks = _checkArgs([[item, "Request lacks containerWithIngredients parameter."],
                        [refrigerator, "Request lacks refrigerator parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if item not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested containerWithIngredients does not exist in world.'}
    if refrigerator not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested refrigerator does not exist in world.'}
    fn = w._kinematicTrees[refrigerator].get('fn', {})
    if not fn.get('canRefrigerate', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested refrigerator cannot refrigerate.'}
    allowedComponents = tuple(set(fn.get('refrigeration', {}).get('links', [])).intersection(fn.get('containment', {}).get('links', [])))
    if 0 == len(allowedComponents):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested refrigerator has no room to refrigerate in.'}
    garden = {0: {'type': 'G', 'description': {'goal': 'placedItem', 'container': refrigerator, 'hand': 'hand_right', 'item': item, 'allowedComponents': allowedComponents}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = [{0: {'type': 'G', 'description': {'goal': 'closed', 'container': refrigerator, "component": l, 'hand': 'hand_left'}}} for l in fn.get('containment', {}).get('links', [])]
    return requests.status_codes.codes.ALL_OK, {}

def toRefrigerateEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'containerWithIngredientsAtTemperature': requestData['containerWithIngredients'], 'kitchenStateOut': w.worldDump()}}

def toPreheatOvenStart(requestData, w, agentName, todos):
    oven = requestData.get("oven", None)
    quantity = requestData.get("quantity", None)
    unit = requestData.get("unit", None)
    lacks = _checkArgs([[oven, "Request lacks oven parameter."],
                        [quantity, "Request lacks quantity parameter."],
                        [unit, "Request lacks unit parameter."]])
    if 0 < len(lacks):
        return requests.status_codes.codes.BAD_REQUEST, {'response': ' '.join(lacks)}
    _checkGreatReset(requestData, w)
    if oven not in w._kinematicTrees:
        return requests.status_codes.codes.NOT_FOUND, {'response': 'Requested oven does not exist in world.'}
    fn = w._kinematicTrees[oven].get('fn', {})
    if not fn.get('canUpdateTemperature', False):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested oven cannot heat.'}
    ## TODO: make sure to allow picking out oven link; for now, assume this is shelf
    fnThermal = fn.get("thermal",{})
    link = fnThermal.get("control",{}).get("shelf",None)
    if link is None:
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested oven has no control to set its temperature with.'}
    if unit.lower() in ["f", "of", "deg fahrenheit", "degrees fahrenheit", "fahrenheit"]:
        quantity = (quantity-32)/1.8
    elif unit.lower() in ["k", "ok", "deg kelvin", "degrees kelvin", "kelvin"]:
        quantity = quantity - 273.15
    temperatureMin = fnThermal.get("temperatureMin",{}).get("shelf", None)
    temperatureMax = fnThermal.get("temperatureMax",{}).get("shelf", None)
    controlMin = fnThermal.get("controlMin",{}).get("shelf", None)
    controlMax = fnThermal.get("controlMax",{}).get("shelf", None)
    if (temperatureMin is None) or (temperatureMax is None) or (controlMin is None) or (controlMax is None):
        return requests.status_codes.codes.I_AM_A_TEAPOT, {'response': 'Requested oven has ill-defined temperature or control range.'}
    setting = controlMin + (controlMax-controlMin)*(quantity-temperatureMin)/(temperatureMax-temperatureMin)
    garden = {0: {'type': 'G', 'description': {'goal': 'turnedControlAndParked', 'item': oven, 'hand': 'hand_left', 'link': link, 'setting': setting}}}
    w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), garden)
    todos['goals'] = []
    return requests.status_codes.codes.ALL_OK, {}

def toPreheatOvenEnd(requestData, w, agentName):
    topGoal, status, response = _checkTopGoal(w, agentName)
    if requests.status_codes.codes.ALL_OK != status:
        return status, response
    return requests.status_codes.codes.ALL_OK, {'response': {'preheatedOven': requestData['oven'], 'kitchenStateOut': w.worldDump()}}

def processActionRequest(fn, requestData, w, agentName, todos):
    if todos['currentAction'] is not None:
        return False, requests.status_codes.codes.SERVICE_UNAVAILABLE, 'Robot already performs an action.'
    status, response = fn(requestData, w, agentName, todos)
    return (requests.status_codes.codes.ALL_OK == status), status, response
    
def processInstantRequest(fn, requestData, w, agentName, todos):
    status, response = fn(requestData, w, agentName, todos)
    return False, status, response

commandFns = {
    "to-get-time": [processInstantRequest, toGetTime, None], 
    "to-cancel": [processInstantRequest, toCancel, None],
    "to-update-avatar": [processInstantRequest, toUpdateAvatar, None],
    "to-get-kitchen": [processInstantRequest, toGetKitchen, None],
    "to-set-kitchen": [processInstantRequest, toSetKitchen, None],
    "to-go-to-pose": [processInstantRequest, toGoToPose, None],
    "to-set-joint": [processInstantRequest, toSetJoint, None],
    "to-set-custom-variable": [processInstantRequest, toSetCustomVariable, None],
    "to-set-object-pose": [processInstantRequest, toSetObjectPose, None],
    "to-get-state-updates": [processInstantRequest, toGetStateUpdates, None],
    "to-get-object-constants": [processInstantRequest, toGetObjectConstants, None],
    "to-preload-object": [processInstantRequest, toPreloadObject, None],
    "to-get-location": [processInstantRequest, toGetLocation, None],
    "to-fetch": [processActionRequest, toFetchStart, toFetchEnd],
    "to-bring-to-temperature": [processActionRequest, toBringToTemperatureStart, toBringToTemperatureEnd],
    "to-wait": [processActionRequest, toWaitStart, toWaitEnd],
    "to-leave-for-time": [processActionRequest, toLeaveForTimeStart, toLeaveForTimeEnd],
    "to-portion": [processActionRequest, toPortionStart, toPortionEnd],
    "to-transfer": [processActionRequest, toTransferStart, toTransferEnd],
    "to-beat": [processActionRequest, toBeatStart, toBeatEnd],
    "to-mix": [processActionRequest, toMixStart, toMixEnd],
    "to-mingle": [processActionRequest, toMingleStart, toMingleEnd],
    "to-mash": [processActionRequest, toMashStart, toMashEnd],
    "to-grind": [processActionRequest, toGrindStart, toGrindEnd],
    "to-flatten": [processActionRequest, toFlattenStart, toFlattenEnd],
    "to-line": [processActionRequest, toLineStart, toLineEnd],
    "to-cover": [processActionRequest, toCoverStart, toCoverEnd],
    "to-uncover": [processActionRequest, toUncoverStart, toUncoverEnd],
    "to-shape": [processActionRequest, toShapeStart, toShapeEnd],
    "to-portion-and-arrange": [processActionRequest, toPortionAndArrangeStart, toPortionAndArrangeEnd],
    "to-bake": [processActionRequest, toBakeStart, toBakeEnd],
    "to-boil": [processActionRequest, toBoilStart, toBoilEnd],
    "to-fry": [processActionRequest, toFryStart, toFryEnd],
    "to-melt": [processActionRequest, toMeltStart, toMeltEnd],
    "to-sprinkle": [processActionRequest, toSprinkleStart, toSprinkleEnd],
    "to-flour": [processActionRequest, toFlourStart, toFlourEnd],
    "to-grease": [processActionRequest, toGreaseStart, toGreaseEnd],
    "to-cut": [processActionRequest, toCutStart, toCutEnd],
    "to-peel": [processActionRequest, toPeelStart, toPeelEnd],
    "to-seed": [processActionRequest, toSeedStart, toSeedEnd],
    "to-place": [processActionRequest, toPlaceStart, toPlaceEnd],
    "to-transfer-items": [processActionRequest, toTransferItemsStart, toTransferItemsEnd],
    "to-refrigerate": [processActionRequest, toRefrigerateStart, toRefrigerateEnd],
    "to-preheat-oven": [processActionRequest, toPreheatOvenStart, toPreheatOvenEnd]}

