##### Init sim, place objects
import os
import signal
import sys

import threading
from flask import Flask
from flask import request
import json


import math
import pybullet as p
import time
from abe_sim.world import World
import abe_sim.garden as garden
import abe_sim.procs as procs

w = World(pybulletOptions = "--opengl2")
p.setGravity(0,0,-5, w.getSimConnection())

from abe_sim.Abe.abe import Abe
from abe_sim.Floor.floor import Floor
from abe_sim.CounterTop.countertop import CounterTop
from abe_sim.KitchenCabinet.kitchencabinet import KitchenCabinet
from abe_sim.MediumBowl.mediumbowl import MediumBowl
from abe_sim.Pantry.pantry import Pantry

a = w.addPObjectOfType("abe", Abe, [0,0,0], [0,0,0,1])
f = w.addPObjectOfType("floor", Floor, [0,0,0], [0,0,0,1])
c = w.addPObjectOfType("counterTop", CounterTop, [-0.051,4.813,0], [0,0,0,1])
k = w.addPObjectOfType("kitchenCabinet", KitchenCabinet, [-1.667,-4.677,0.963], [0,0,0,1])
mb1 = w.addPObjectOfType("mediumBowl1", MediumBowl, [1.03,-4.17,1.205], [0,0,0,1])
p1 = w.addPObjectOfType("pantry1", Pantry, [4.31,-1.793,1.054], [0,0,0.707,0.707])
p2 = w.addPObjectOfType("pantry2", Pantry, [4.31,-3.683,1.054], [0,0,0.707,0.707])

g = garden.Garden()
executingAction = threading.Condition()
updating = threading.Lock()
flask = Flask(__name__)

def thread_function_flask():
    @flask.route("/abe-sim-command/to-get-kitchen", methods = ['POST'])
    def to_get_kitchen():
        retq = {'status': 'ok', 'response': ''}
        try:
            with updating:
                request_data = request.get_json(force=True)
                varName = request_data['kitchen']
                retq['response'] = {varName: w.worldDump()}
        except KeyError:
            retq['status'] = 'missing entries from state data'
        except SyntaxError:
            retq['status'] = 'ill-formed json for command'
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-set-kitchen", methods = ['POST'])
    def to_set_kitchen():
        retq = {'status': 'ok', 'response': ''}
        try:
            with updating:
                request_data = request.get_json(force=True)
                if None != request_data["kitchen-input-state"]:
                    w.greatReset(request_data["kitchen-input-state"])
        except KeyError:
            retq['status'] = 'missing entries from state data'
        except SyntaxError:
            retq['status'] = 'ill-formed json for command'
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-fetch", methods = ['POST'])
    def to_fetch():
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
                    w.greatReset(inputState)
                name = request_data["object"]
                if name in w._pobjects:
                    doAction = True
                    item = w._pobjects[name]
                    agent = w._pobjects[list(w._ontoTypes["agent"])[0]]
                    g._processes = {}
                    g._commandProcess = garden.Process(coherence=[procs.ItemOnCounter(item),procs.ParkedArms(agent)])
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"fetchedObject": name, "kitchenOutputState": w.worldDump()}
            else:
                with updating:
                    retq["response"] = {"fetchedObject": None, "kitchenOutputState": w.worldDump()}
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
        if (None != g._commandProcess) and (0 < len(g._commandProcess._coherence)):
            w.update()
            bodyProcs = g.updateGarden()
            if [] == bodyProcs:
                if None != g._commandProcess:
                    g._commandProcess = None
                    with executingAction:
                        executingAction.notify_all()
            else:
                [x.bodyAction() for x in bodyProcs]
    time.sleep(1.0/240.0)

