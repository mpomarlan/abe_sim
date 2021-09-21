## TODO: set up scene with 3 bowls, 2 bags and portion bags into 2 bowls
## TODO: mixing
## TODO: placing into pantry/cabinet (low priority)

##### Init sim, place objects
import os
import signal
import sys

import threading
from flask import Flask
from flask import request
import json

import platform

import math
import pybullet as p
import time
from abe_sim.world import World
import abe_sim.garden as garden
import abe_sim.procs as procs

isAMac = ('Darwin' == platform.system())
## WORLD CREATION line: adjust this as needed on your system.
if not isAMac:
    w = World(pybulletOptions = "--opengl2") # Software-only "tiny" renderer. Should work on Linux and when support for graphical hardware acceleration is inconsistent.
else:
    w = World(pybulletOptions = "") # Hardware-accelerated rendering. Seems necessary on newer Macs.
p.setGravity(0,0,-5, w.getSimConnection())
p.resetDebugVisualizerCamera(10.8,-90.0,-37.566, [0,0,0])

import abe_sim.Particle.particle as prt
w._particleTypes = {"particle": prt.Particle, "sugarparticle": prt.SugarParticle, "butterparticle": prt.ButterParticle}

from abe_sim.Abe.abe import Abe
from abe_sim.Floor.floor import Floor
from abe_sim.CounterTop.countertop import CounterTop
from abe_sim.KitchenCabinet.kitchencabinet import KitchenCabinet
from abe_sim.MediumBowl.mediumbowl import MediumBowl
from abe_sim.Pantry.pantry import Pantry
from abe_sim.Bag.bag import SugarBag, ButterBag

a = w.addPObjectOfType("abe", Abe, [0,0,0], [0,0,0,1])
f = w.addPObjectOfType("floor", Floor, [0,0,0], [0,0,0,1])
c = w.addPObjectOfType("counterTop", CounterTop, [-0.051,4.813,0], [0,0,0,1])
k = w.addPObjectOfType("kitchenCabinet", KitchenCabinet, [-1.667,-4.677,0.963], [0,0,0,1])
mb1 = w.addPObjectOfType("mediumBowl1", MediumBowl, [1.03,-4.17,1.205], [0,0,0,1])
mb2 = w.addPObjectOfType("mediumBowl2", MediumBowl, [0.48,-4.17,1.205], [0,0,0,1])
mb3 = w.addPObjectOfType("mediumBowl3", MediumBowl, [-0.07,-4.17,1.205], [0,0,0,1])
p1 = w.addPObjectOfType("pantry1", Pantry, [4.31,-1.793,1.054], [0,0,0.707,0.707])
p2 = w.addPObjectOfType("pantry2", Pantry, [4.31,-3.683,1.054], [0,0,0.707,0.707])
sg = w.addPObjectOfType("sugarBag", SugarBag, [-1.555,-4.174,1.35], [0,0,1,0])
bg = w.addPObjectOfType("butterBag", ButterBag, [-1.7,-4.174,0.68], [0,0,1,0])


g = garden.Garden()
executingAction = threading.Condition()
updating = threading.Lock()
flask = Flask(__name__)

cgr = None
cwd = {}
ccd = None

def placeCamera(item):
    iP = item.getBodyProperty((), "position")
    cP = w._pobjects["counterTop"].getBodyProperty((), "position")
    yaw = 180*math.atan2(iP[1]-cP[1],iP[0]-cP[0])/math.pi
    p.resetDebugVisualizerCamera(4,yaw-90,-35, cP)

def thread_function_flask():
    @flask.route("/abe-sim-command/to-get-kitchen", methods = ['POST'])
    def to_get_kitchen():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            with updating:
                request_data = request.get_json(force=True)
                varName = request_data['kitchen']
                retq['response'] = {varName: cwd}
        except KeyError:
            retq['status'] = 'missing entries from state data'
        except SyntaxError:
            retq['status'] = 'ill-formed json for command'
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-set-kitchen", methods = ['POST'])
    def to_set_kitchen():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            with updating:
                request_data = request.get_json(force=True)
                if None != request_data["kitchenInputState"]:
                    cgr = request_data["kitchenInputState"]
        except KeyError:
            retq['status'] = 'missing entries from state data'
        except SyntaxError:
            retq['status'] = 'ill-formed json for command'
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-get-location", methods = ['POST'])
    def to_get_location():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            request_data = request.get_json(force=True)
            varName = request_data['availableLocation']
            locType = request_data['type'].lower()
            with updating:
                inputState = None
                if "kitchen" in request_data:
                    inputState = request_data["kitchen"]
                sws = False
                if "setWorldState" in request_data:
                    sws = request_data["setWorldState"]
                if sws and (None != inputState):
                    cgr = inputState
                objName = None
                if (locType in w._ontoTypes) and (0 < len(w._ontoTypes[locType])):
                    objName = list(w._ontoTypes[locType])[0]
                retq["response"] = {varName: objName}
        except KeyError:
            retq['status'] = 'missing entries from state data'
        except SyntaxError:
            retq['status'] = 'ill-formed json for command'
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-fetch", methods = ['POST'])
    def to_fetch():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = None
                if "kitchenInputState" in request_data:
                    inputState = request_data["kitchenInputState"]
                sws = False
                if "setWorldState" in request_data:
                    sws = request_data["setWorldState"]
                if sws and (None != inputState):
                    cgr = inputState
                name = request_data["object"]
                if name in w._pobjects:
                    doAction = True
                    ccd = {'op': 'fetch', 'item': name}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"fetchedObject": name, "kitchenOutputState": cwd}
            else:
                with updating:
                    retq["response"] = {"fetchedObject": None, "kitchenOutputState": cwd}
        except KeyError:
            retq['status'] = 'missing entries from state data'
        except SyntaxError:
            retq['status'] = 'ill-formed json for command'
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-portion", methods = ['POST'])
    def to_portion():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = None
                if "kitchenInputState" in request_data:
                    inputState = request_data["kitchenInputState"]
                sws = False
                if "setWorldState" in request_data:
                    sws = request_data["setWorldState"]
                if sws and (None != inputState):
                    cgr = inputState
                name = request_data["inputContainer"]
                storeName = request_data["newContainer"]
                amount = request_data["amount"]
                if (name in w._pobjects) and (storeName in w._pobjects):
                    doAction = True
                    ccd = {'op': 'portion', 'item': name, 'destination': storeName, 'amount': amount}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"outputContainer": storeName, "kitchenOutputState": w.worldDump()}
            else:
                with updating:
                    retq["response"] = {"outputContainer": None, "kitchenOutputState": w.worldDump()}
        except KeyError:
            retq['status'] = 'missing entries from state data'
        except SyntaxError:
            retq['status'] = 'ill-formed json for command'
        return json.dumps(retq)
    flask.run(port=54321, debug=True, use_reloader=False)

flaskThread = threading.Thread(target=thread_function_flask, args=())
flaskThread.start()

def handleINT(signum, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, handleINT)
signal.signal(signal.SIGTERM, handleINT)

while True:
    with updating:
        if None != cgr:
            w.greatReset(cgr)
            cgr = None
        cwd = w.worldDump()
        if None != ccd:
            if 'fetch' == ccd['op']:
                item = w._pobjects[ccd['item']]
                agent = w._pobjects[list(w._ontoTypes["agent"])[0]]
                g._processes = {}
                g._commandProcess = garden.Process(coherence=[procs.ItemOnCounter(item),procs.ParkedArms(agent)])
                placeCamera(item)
            elif 'portion' == ccd['op']:
                item = w._pobjects[ccd['item']]
                store = w._pobjects[ccd['destination']]
                amount = ccd['amount']
                agent = w._pobjects[list(w._ontoTypes["agent"])[0]]
                counter = w._pobjects["counterTop"]
                g._processes = {}
                g._commandProcess = garden.Process(coherence=[procs.ProportionedItem(item, amount, store),procs.ItemOnLocation(item,counter),procs.ParkedArms(agent)])
                placeCamera(item)
            ccd = None
        w.update()
        if (None != g._commandProcess) and (0 < len(g._commandProcess._coherence)):
            bodyProcs = g.updateGarden()
            if all([x.isFulfilled() for x in g._commandProcess._coherence]):
                if None != g._commandProcess:
                    g._commandProcess = None
                    with executingAction:
                        executingAction.notify_all()
            else:
                [x.bodyAction() for x in bodyProcs]
    if not isAMac:
        time.sleep(1.0/240.0)

