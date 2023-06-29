import argparse
import copy
import json
import math
import numpy
import os
import platform
import pybullet
import requests
import signal
import sys
import threading
import time

from flask import Flask, request

import abe_sim.world as world

from abe_sim.motionPlanning import updateMotionPlanning
from abe_sim.kinematicControl import updateKinematicControl
from abe_sim.grasping import updateGrasping, updateGraspingConstraint
from abe_sim.timing import updateTiming
from abe_sim.processGardening import updateGarden
from abe_sim.transporting import updateTransportingConstraint, updateTransporting
from abe_sim.shaping import updateShaping, updateShaped
from abe_sim.stickiness import updateStickiness
from abe_sim.mixing import updateMingling, updateMixing
from abe_sim.clopening import updateClopening
from abe_sim.temperature import updateTemperatureGetter

from abe_sim.customDynamics import buildSpecs
from abe_sim.commands import commandFns

def startProcessCommand(command, requestData, w, agentName, todos):
    return commandFns[command][0](commandFns[command][1], requestData, w, agentName, todos)
    #return doingAction, status, response
    
def stopProcessCommand(command, requestData, w, agentName):
    status, response = commandFns[command][2](requestData, w, agentName)
    return False, status, response

def placeCamera(w, item, relatum):
    iP = w.getObjectProperty((item,), "position")
    cP = w.getObjectProperty((relatum,), "position")
    yaw = 180*math.atan2(iP[1]-cP[1],iP[0]-cP[0])/math.pi
    world.stubbornTry(lambda : pybullet.resetDebugVisualizerCamera(4,yaw-90,-35, cP))
    
def serviceRequest(command, request, requestDictionary, responseDictionary, updating, executingAction):
    def print_circular_refs(ob, _path=None, _seen=None):
        if _path is None:
            _path = []
        if _seen is None:
            _seen = set()
        if id(ob) in _seen:
            print("CIRCULAR", _path, ob)
            return None
        _seen.add(id(ob))
        res = ob
        if isinstance(ob, dict):
            res = {
                print_circular_refs(k, _seen=_seen, _path=_path): print_circular_refs(v, _seen=_seen, _path=_path + [k])
                for k, v in ob.items()}
        elif isinstance(ob, (list, tuple, set, frozenset)):
            res = type(ob)(print_circular_refs(v, _seen=_seen, _path=_path) for v in ob)
        # remove id again; only *nested* references count
        _seen.remove(id(ob))
        return res
    if command not in commandFns:
        return {'response': 'Unrecognized command.'}, requests.status_codes.codes.NOT_IMPLEMENTED
    with updating:
        commandId = str(time.time())
        answeringRequest = threading.Condition()
        try:
            requestDictionary[commandId] = [answeringRequest, command, request.get_json(force=True)]
        except SyntaxError:
            return {'response': 'Malformed json.'}, requests.status_codes.codes.BAD_REQUEST
    with answeringRequest:
        answeringRequest.wait()
    doingAction = False
    with updating:
        doingAction, status, response = responseDictionary.pop(commandId)
    if doingAction:
        with executingAction:
            executingAction.wait()
        with updating:
            _, status, response = responseDictionary.pop(commandId)
        #print_circular_refs(response, _path=None, _seen=None)
    return json.dumps(response), status

def thread_function_flask(requestDictionary, responseDictionary, updating, executingAction):
    flask = Flask(__name__)
    #@flask.route("/abe-sim-command/to-get-time", methods = ['POST'])
    #def to_get_time():
    #    return serviceRequest("to-get-time", request, requestDictionary, responseDictionary, updating, executingAction)
    @flask.route("/abe-sim-command/<string:command>", methods=['POST'])
    def flaskRequest(command):
        return serviceRequest(command, request, requestDictionary, responseDictionary, updating, executingAction)
    flask.run(port=54321, debug=True, use_reloader=False)

def handleINT(signum, frame):
    sys.exit(0)

def runBrain():
    parser = argparse.ArgumentParser(prog='runBrain', description='Run the Abe Sim', epilog='Text at the bottom of help')
    parser.add_argument('-fdf', '--frameDurationFactor', default="1.0", help='Attempts to adjust the ratio of real time of frame to simulated time of frame. A frame will always last, in real time, at least as long as it is computed. WARNING: runBrain will become unresponsive to HTTP API calls if this is set too low. Recommended values are above 0.2.')
    parser.add_argument('-sfr', '--simFrameRate', default="240", help='Number of frames in one second of simulated time. Should be above 60.')
    parser.add_argument('-a', '--agent', help='Name of the agent to control in the loaded scene')
    parser.add_argument('-g', '--useGUI', action='store_true', help='Flag to enable the GUI')
    parser.add_argument('-o', '--useOpenGL', action='store_true', help='Flag to enable hardware acceleration. Warning: often unstable on Linux; ignored on MacOS')
    parser.add_argument('-p', '--preloads', default=None, help='Path to a file containing a json list of objects to preload. Each element of this list must be of form [type, name, position]')
    parser.add_argument('-w', '--loadWorldDump', default=None, help='Path to a file containing a json world dump from a previous run of Abe Sim')
    parser.add_argument('-l', '--loadObjectList', default='./abe_sim/defaultScene.json', help='Path containing a json list of objects to load in the scene. Each element in the list must be of form [type, name, position, orientation]')
    arguments = parser.parse_args()
    customDynamics = buildSpecs('./abe_sim/procdesc.yml') + [[('fn', 'canTime'), updateTiming], [('fn', 'kinematicallyControlable'), updateKinematicControl], [('fn', 'canGrasp'), updateGrasping], [('fn', 'graspingConstraint'), updateGraspingConstraint], [('fn', 'processGardener'), updateGarden], [('fn', 'transportingConstraint'), updateTransportingConstraint], [('fn', 'transportable'), updateTransporting], [('fn', 'sticky'), updateStickiness], [('fn', 'temperatureUpdateable'), updateTemperatureGetter], [('fn', 'mixable'), updateMixing], [('fn', 'shapeable'), updateShaped], [('fn', 'canShape'), updateShaping], [('fn', 'clopenable'), updateClopening], [('fn', 'mingleable'), updateMingling]]

    objectTypeKnowledge = json.loads(open('./abe_sim/objectknowledge.json').read())
    objectTypeKnowledge = {x['type']: x for x in objectTypeKnowledge}

    processKnowledge = json.loads(open('./abe_sim/processknowledge.json').read())

    useGUI = arguments.useGUI
    useOpenGL = arguments.useOpenGL
    preloads = arguments.preloads
    gravity = (0,0,-10) # TODO argparse
    loadWorldDump = arguments.loadWorldDump
    loadObjectList = arguments.loadObjectList
    frameDurationFactor = float(arguments.frameDurationFactor)
    sfr = int(arguments.simFrameRate)
    agentName = arguments.agent

    if useOpenGL:
        pybulletOptions = "--opengl3"
    else:
        pybulletOptions = "--opengl2" # Software-only "tiny" renderer. Should work on Linux and when support for graphical hardware acceleration is inconsistent.

    isAMac = ('Darwin' == platform.system())
    ## WORLD CREATION line: adjust this as needed on your system.
    # TODO if you want to run headless: useGUI=False in World()
    if not isAMac:
        w = world.World(pybulletOptions = pybulletOptions, useGUI=useGUI, customDynamics=customDynamics, objectKnowledge=objectTypeKnowledge, processKnowledge=processKnowledge, simFrameRate=sfr)
    else:
        w = world.World(pybulletOptions = "", useGUI=useGUI, customDynamics=customDynamics, objectKnowledge=objectTypeKnowledge, processKnowledge=processKnowledge, simFrameRate=sfr) # Hardware-accelerated rendering. Seems necessary on newer Macs.
        
    w.setGravity(gravity)

    if preloads is not None:
        preloads = json.loads(open(preloads).read())
        for oType, oname, position in toPreload:
            w.preload(oType, oname, position)
    if loadWorldDump is not None:
        w.greatReset(json.loads(open(loadWorldDump).read()))
    elif loadObjectList is not None:
        _ = [w.addObjectInstance(*x) for x in json.loads(open(loadObjectList).read())]
            
    waitingFor = 0
    
    if (agentName is None) or (agentName not in w._kinematicTrees.keys()):
        agentName = [x['name'] for x in w._kinematicTrees.values() if 'Abe' == x['type']][0]
    
    executingAction = threading.Condition()
    updating = threading.Lock()
    requestDictionary = {}
    responseDictionary = {}
    currentAction = None

    flaskThread = threading.Thread(target=thread_function_flask, args=(requestDictionary, responseDictionary, updating, executingAction))
    flaskThread.start()
    signal.signal(signal.SIGINT, handleINT)
    signal.signal(signal.SIGTERM, handleINT)
    
    todos = {"currentAction": None, "goals": [], "requestData": {}, "command": None, "cancelled": False}
    
    while True:
        stepStart = time.time()
        with updating:
            for commandId, commandSpec in list(requestDictionary.items()):
                if commandId == todos["currentAction"]:
                    continue
                answeringRequest, command, requestData = commandSpec
                doingAction, status, response = startProcessCommand(command, requestData, w, agentName, todos)
                if doingAction:
                    todos["currentAction"] = commandId
                    todos["requestData"] = requestData
                    todos["command"] = command
                    todos["cancelled"] = False
                else:
                    if todos["currentAction"] is None:
                        todos["goals"] = []
                        todos["cancelled"] = False
                    requestDictionary.pop(commandId)
                responseDictionary[commandId] = doingAction, status, response
                with answeringRequest:
                    answeringRequest.notify_all()
            w.update()
            #print(todos)
            #print(w.getObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), {}))
            if todos["currentAction"] is not None:
                garden = w.getObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), {})
                if (0 not in garden) or (garden[0].get('previousStatus', False)):
                    if 0 == len(todos["goals"]):
                        requestDictionary.pop(todos["currentAction"])
                        if todos["cancelled"]:
                            responseDictionary[todos["currentAction"]] = [False, requests.status_codes.codes.GONE, 'Action cancelled by user.']
                        else:
                            responseDictionary[todos["currentAction"]] = stopProcessCommand(todos["command"], todos["requestData"], w, agentName)
                        todos["currentAction"] = None
                        todos["cancelled"] = False
                        with executingAction:
                            executingAction.notify_all()
                    else:
                        w.setObjectProperty((agentName,), ('customStateVariables', 'processGardening', 'garden'), todos["goals"][0])
                        todos["goals"] = todos["goals"][1:]
        stepEnd = time.time()
        #print(w.getObjectProperty(('abe', 'hand_right_roll'), 'position'), w.getObjectProperty(('abe', 'hand_right_roll'), 'linearVelocity'), pybullet.getContactPoints(bodyA=w._kinematicTrees['abe']['idx']))
        if not isAMac:
            time.sleep(max((frameDurationFactor/(sfr*1.0))-(stepEnd-stepStart), 0.001))

if "__main__" == __name__:
    runBrain()
