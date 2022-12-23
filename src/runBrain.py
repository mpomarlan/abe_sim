## TODO: mixing

##### Init sim, place objects
import os
import signal
import sys

from pprint import pprint
import threading
from flask import Flask, abort
from flask import request
import json

import platform

import math
import pybullet as p
import time
from abe_sim.world import World
from abe_sim.pobject import DebugText, DebugCapsule
import abe_sim.garden as garden
import abe_sim.procs as procs

from abe_sim.geom import vectorNorm
from abe_sim.utils import stubbornTry

from abe_sim.procs import getContents

isAMac = ('Darwin' == platform.system())
## WORLD CREATION line: adjust this as needed on your system.
# TODO if you want to run headless: useGUI=False in World()
if not isAMac:
    w = World(pybulletOptions = "--opengl3", useGUI=True) # Software-only "tiny" renderer. Should work on Linux and when support for graphical hardware acceleration is inconsistent.
else:
    w = World(pybulletOptions = "", useGUI=True) # Hardware-accelerated rendering. Seems necessary on newer Macs.
    
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

stubbornTry(lambda : p.setGravity(0,0,-5, w.getSimConnection()))
stubbornTry(lambda : p.resetDebugVisualizerCamera(10.8,-90.0,-37.566, [0,0,0]))

import abe_sim.Particle.particle as prt

from abe_sim.Abe.abe import Abe
from abe_sim.Floor.floor import Floor
from abe_sim.CounterTop.countertop import CounterTop
from abe_sim.KitchenCabinet.kitchencabinet import KitchenCabinet,KitchenCabinetLow
from abe_sim.MediumBowl.mediumbowl import MediumBowl
from abe_sim.Pantry.pantry import Pantry
from abe_sim.Bag.bag import SugarBag, ButterBag
from abe_sim.Particle.particle import Particle, SugarParticle, ButterParticle, SweetButterParticle
from abe_sim.Fridge.fridge import Fridge, Freezer, FridgeDoor, FreezerDoor
from abe_sim.Whisk.whisk import Whisk
from abe_sim.Cookie.cookie import DoughClump
from abe_sim.BakingTray.bakingtray import BakingTray
from abe_sim.BakingSheet.bakingsheet import BakingSheet
from abe_sim.KitchenStove.kitchenstove import KitchenStove, KitchenStoveDoor
from abe_sim.Shaker.shaker import Shaker

from abe_sim.Wall.wall import Wall
from abe_sim.GasStove.gasstove import GasStove, GasStoveDoor
from abe_sim.KitchenSink.kitchensink import KitchenSink
from abe_sim.Boiler.boiler import Boiler
from abe_sim.SaucePan.saucepan import SaucePan
from abe_sim.PlasticCup.plasticcup import PlasticCup
from abe_sim.WoodenBeerMug.woodenbeermug import WoodenBeerMug

w._typeMap = {'GasStove': GasStove, 'Floor': Floor, 'CounterTop': CounterTop, 'WoodenBeerMug': WoodenBeerMug, 'PlasticCup': PlasticCup, 'Abe': Abe, 'Boiler': Boiler, 'KitchenSink': KitchenSink, 'GasStoveDoor': GasStoveDoor, 'Wall': Wall, 'SaucePan': SaucePan}

w._particleTypes = {"particle": prt.Particle, "sugarparticle": prt.SugarParticle, "butterparticle": prt.ButterParticle, "sweetbutterparticle": prt.SweetButterParticle, "bakingsheet": BakingSheet, "doughclump": DoughClump}

a = w.addPObjectOfType("abe", Abe, [0,0,0], [0,0,0,1])
f = w.addPObjectOfType("floor", Floor, [0,0,0], [0,0,0,1])
c = w.addPObjectOfType("counterTop", CounterTop, [-0.051,4.813,0], [0,0,0,1])
k = w.addPObjectOfType("kitchenCabinet", KitchenCabinet, [-1.667,-4.677,0.963], [0,0,0,1])
mb1 = w.addPObjectOfType("mediumBowl1", MediumBowl, [4.278,0.687,0.999], [0,0,0,1])
mb2 = w.addPObjectOfType("mediumBowl2", MediumBowl, [0.48,-4.17,1.305], [0,0,0,1])
mb3 = w.addPObjectOfType("mediumBowl3", MediumBowl, [-0.07,-4.17,1.305], [0,0,0,1])
mb4 = w.addPObjectOfType("mediumBowl4", MediumBowl, [-0.7,-4.17,1.305], [0,0,0,1])
p1 = w.addPObjectOfType("pantry1", Pantry, [4.31,-1.793,1.054], [0,0,0.707,0.707])
p2 = w.addPObjectOfType("pantry2", Pantry, [4.31,-3.683,1.054], [0,0,0.707,0.707])
sg = w.addPObjectOfType("sugarBag", SugarBag, [-1.555,-4.174,1.45], [0,0,1,0])
bg = w.addPObjectOfType("butterBag", ButterBag, [-1.7,-4.174,0.68], [0,0,1,0])
fg = w.addPObjectOfType("fridge", Fridge, [4.781,0.05,0.691], [0,0,0.707,0.707])
fz = w.addPObjectOfType("freezer", Freezer, [4.781,0.05,0.691], [0,0,0.707,0.707])
fgd = w.addPObjectOfType("fridgeDoor", FridgeDoor, [4.781,0.05,0.691], [0,0,0.707,0.707])
fzd = w.addPObjectOfType("freezerDoor", FreezerDoor, [4.781,0.05,0.691], [0,0,0.707,0.707])
wh = w.addPObjectOfType("whisk", Whisk, [-3.956, -4.54, 1.247], [0.5, -0.5, -0.5, 0.5])
btr = w.addPObjectOfType("bakingTray1", BakingTray, [-3.258, -4.325, 1.305], [0,0,0.707,0.707])
bsh = w.addPObjectOfType("bakingSheet1", BakingSheet, [-2.806, -4.296, 1.3], [0,0,0.707,0.707])
ks = w.addPObjectOfType("kitchenStove", KitchenStove, [4.56, 3.3, 0.7], [0,0,0.707,0.707])
ksd = w.addPObjectOfType("kitchenStoveDoor", KitchenStoveDoor, [4.56, 3.3, 0.7], [0,0,0.707,0.707])
shk = w.addPObjectOfType("sugarShaker", Shaker, [1.2,-4.16,1.288], [0,0,0,1])
fg.setBodyProperty("fn", "door", "fridgeDoor")
fz.setBodyProperty("fn", "door", "freezerDoor")
ks.setBodyProperty("fn", "door", "kitchenStoveDoor")
aux = w.addPObjectOfType("aux", ButterParticle,[0,0,0],[0,0,0,1])
w.removePObject("aux")
aux = w.addPObjectOfType("aux", SugarParticle,[0,0,0],[0,0,0,1])
w.removePObject("aux")
aux = w.addPObjectOfType("aux", SweetButterParticle,[0,0,0],[0,0,0,1])
w.removePObject("aux")
aux = w.addPObjectOfType("aux", DoughClump,[0,0,0],[0,0,0,1])
w.removePObject("aux")


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
    stubbornTry(lambda : p.resetDebugVisualizerCamera(4,yaw-90,-35, cP))

def thread_function_flask():
    @flask.route("/abe-sim-command/to-add-highlight", methods = ['POST'])
    def to_add_hightlight():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            with updating:
                request_data = request.get_json(force=True)
                if request_data['object'] in w._pobjects:
                    doAction = True
                    text = request_data.get('text')
                    label = request_data.get('label')
                    ccd = {'op': 'highlight', 'item': request_data['object'], 'text': text, 'label': label}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = "done"
            else:
                with updating:
                    retq["response"] = "wrong name for object?"
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-remove-highlight", methods = ['POST'])
    def to_remove_hightlight():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            with updating:
                request_data = request.get_json(force=True)
                if request_data['object'] in w._pobjects:
                    doAction = True
                    ccd = {'op': 'remove_highlight', 'item': request_data['object']}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = "done"
            else:
                with updating:
                    retq["response"] = "wrong name for object?"
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-get-kitchen", methods = ['POST'])
    def to_get_kitchen():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            with updating:
                request_data = request.get_json(force=True)
                varName = request_data['kitchenStateIn']
                retq['response'] = {varName: cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-set-kitchen", methods = ['POST'])
    def to_set_kitchen():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            with updating:
                request_data = request.get_json(force=True)
                if request_data["kitchenStateIn"] is not None:
                    cgr = request_data["kitchenStateIn"]
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
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
                inputState = request_data.get("kitchenStateIn")
                sws = request_data.get("setWorldState")
                if sws and (inputState is not None):
                    cgr = inputState
                objName = None
                if (locType in w._ontoTypes) and (0 < len(w._ontoTypes[locType])):
                    # TODO a way find first free item?
                    # In get location or a new extra route?
                    # for k, v in w._pobjects.items():
                    #     if k in list(w._ontoTypes[locType]):
                    #         #print(k, v.__dict__)
                    #         print(v._customStateVariables)
                    objName = None
                    for e in list(w._ontoTypes[locType]):
                        eAt = w._pobjects[e].at()
                        eAtType = None
                        if eAt in w._pobjects:
                            eAtType = w._pobjects[eAt].getBodyProperty("", "type")
                        if eAtType not in ["countertop", "counterTop"]:
                            objName = e
                            break
                    if objName is None:
                        objName = list(w._ontoTypes[locType])[0]                
                retq["response"] = {varName: objName}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-fetch", methods = ['POST'])
    def to_fetch():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = request_data.get("kitchenStateIn")
                sws = request_data.get("setWorldState")
                if sws and (inputState is not None):
                    cgr = inputState
                name = request_data["object"]
                if name in w._pobjects:
                    doAction = True
                    ccd = {'op': 'fetch', 'item': name}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"fetchedObject": name, "kitchenStateOut": cwd}
            else:
                with updating:
                    retq["response"] = {"fetchedObject": None, "kitchenStateOut": cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-transfer", methods = ['POST'])
    def to_transfer():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        print(request.get_json(force=True))
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = request_data.get("kitchenStateIn")
                sws = request_data.get("setWorldState")
                if sws and (inputState is not None):
                    cgr = inputState
                name = request_data["containerWithInputIngredients"]
                storeName = request_data["targetContainer"]
                ### TODO: partial transfers if keys "quantity" and "unit" are present in request_data
                if (name in w._pobjects) and (storeName in w._pobjects):
                    doAction = True
                    ccd = {'op': 'transfer', 'item': name, 'destination': storeName}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"containerWithRest": name, "containerWithAllIngredients": storeName, "kitchenStateOut": cwd}
            else:
                with updating:
                    retq["response"] = {"containerWithRest": None, "containerWithAllIngredients": None, "kitchenStateOut": cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-mix", methods = ['POST'])
    def to_mix():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = request_data.get("kitchenStateIn")
                sws = request_data.get("setWorldState")
                if sws and (inputState is not None):
                    cgr = inputState
                name = request_data["containerWithInputIngredients"]
                tool = request_data["mixingTool"]
                if (name in w._pobjects) and (tool in w._pobjects):
                    doAction = True
                    ccd = {'op': 'mix', 'item': name, 'tool': tool}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"containerWithMixture": name, "kitchenStateOut": cwd}
            else:
                with updating:
                    retq["response"] = {"containerWithMixture": None, "kitchenStateOut": cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-portion", methods = ['POST'])
    def to_portion():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        print(request.get_json(force=True))
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = request_data.get("kitchenStateIn")
                sws = request_data.get("setWorldState")
                if sws and (inputState is not None):
                    cgr = inputState
                ### TODO: pick container based on contents concept, as expressed by a key 'ingredientConcept' in requests_data
                name = request_data["containerWithIngredient"]
                storeName = request_data["targetContainer"]
                ### TODO: also read a 'unit' key from requests_data
                amount = request_data["quantity"]
                if (name in w._pobjects) and (storeName in w._pobjects):
                    doAction = True
                    ccd = {'op': 'portion', 'item': name, 'destination': storeName, 'amount': amount}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"outputContainer": storeName, "kitchenStateOut": cwd}
            else:
                with updating:
                    retq["response"] = {"outputContainer": None, "kitchenStateOut": cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-shape",methods=['POST'])
    def to_shape():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = request_data.get("kitchenStateIn")
                sws = request_data.get("setWorldState")
                if sws and (inputState is not None):
                    cgr = inputState
                sourceContainer = request_data["containerWithDough"]
                ### TODO: read shape data from 'shape', 'quantity', 'unit' keys in request_data
                shapeType = DoughClump
                destinationContainer = request_data["destination"]
                if (sourceContainer in w._pobjects) and (destinationContainer in w._pobjects):
                    doAction = True
                    ccd = {'op': "shape", 'sourceContainer': sourceContainer, 'destinationContainer': destinationContainer}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"shapedPortions": [x.getName() for x in getContents(w._pobjects[destinationContainer]) if isinstance(x, shapeType)], "kitchenStateOut": cwd}
            else:
                with updating:
                    retq["response"] = {"shapedPortions": None, "kitchenStateOut": cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-line",methods=['POST'])
    def to_line():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = request_data.get("kitchenStateIn")
                sws = request_data.get("setWorldState")
                if sws and (inputState is not None):
                    cgr = inputState
                inputContainer = request_data["bakingTray"]
                lining = request_data["bakingPaper"]
                if (inputContainer in w._pobjects) and (lining in w._pobjects):
                    doAction = True
                    ccd = {'op': "line", 'inputContainer': inputContainer, 'lining': lining}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"linedBakingTray": inputContainer, "kitchenStateOut": cwd}
            else:
                with updating:
                    retq["response"] = {"linedBakingTray": None, "kitchenStateOut": cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-bake",methods=['POST'])
    def to_bake():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = request_data.get("kitchenStateIn")
                sws = request_data.get("setWorldState")
                if sws and (inputState is not None):
                    cgr = inputState
                inputContainer = request_data["thingToBake"]
                ### TODO: select oven and destination automatically
                oven = request_data["oven"]
                destinationContainer = request_data["inputDestinationContainer"]
                ### TODO: make use of 'timeToBakeQuantity', 'timeToBakeUnit', targetTemperatureQuantity', 'targetTemperatureUnit' keys from request_data
                if (inputContainer in w._pobjects) and (oven in w._pobjects) and (destinationContainer in w._pobjects):
                    doAction = True
                    ccd = {'op': "bake", 'inputContainer': inputContainer, 'oven': oven, "destinationContainer": destinationContainer}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"thingBaked": inputContainer, "outputDestinationContainer": destinationContainer, "kitchenStateOut": cwd}
            else:
                with updating:
                    retq["response"] = {"thingBaked": None, "outputDestinationContainer": None, "kitchenStateOut": cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-sprinkle",methods=['POST'])
    def to_sprinkle():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                inputState = request_data.get("kitchenStateIn")
                sws = request_data.get("setWorldState")
                if sws and (inputState is not None):
                    cgr = inputState
                inputTargetContainer = request_data["object"]
                inputToppingContainer = request_data["toppingContainer"]
                if (inputTargetContainer in w._pobjects) and (inputToppingContainer in w._pobjects):
                    doAction = True
                    ccd = {'op': "sprinkle", 'inputTargetContainer': inputTargetContainer, "inputToppingContainer": inputToppingContainer}
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"sprinkledObject": inputTargetContainer, "kitchenStateOut": cwd}
            else:
                with updating:
                    retq["response"] = {"sprinkledObject": None, "kitchenStateOut": cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
    @flask.route("/abe-sim-command/to-wait",methods=['POST'])
    def to_wait():
        global cwd, cgr, ccd
        retq = {'status': 'ok', 'response': ''}
        try:
            doAction = False
            with updating:
                request_data = request.get_json(force=True)
                frames = request_data.get("frames")
                if frames:
                    doAction = True
                    ccd = {'op': "wait", 'frames': frames}
            print("WAIT?", doAction, ccd)
            if doAction:
                with executingAction:
                    executingAction.wait()
                with updating:
                    retq["response"] = {"kitchenStateOut": cwd}
        except KeyError:
            return 'missing entries from state data', 400
        except SyntaxError:
            return 'ill-formed json for command', 400
        return json.dumps(retq)
        
    flask.run(port=54321, debug=True, use_reloader=False)

flaskThread = threading.Thread(target=thread_function_flask, args=())
flaskThread.start()

def handleINT(signum, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, handleINT)
signal.signal(signal.SIGTERM, handleINT)

waitingFor = 0

while True:
    with updating:
        if cgr is not None:
            w.greatReset(cgr)
            cgr = None
        cwd = w.worldDump()
        if ccd is not None:
            w._pobjects["abe"].setBodyProperty("fn", "done", False)
            if ('highlight' == ccd['op']) or ('remove_highlight' == ccd['op']):
               if 'highlight' == ccd['op']:
                   if ccd['text']:
                       DebugText(w._pobjects[ccd['item']], ccd['text'])
                   if ccd['label']:
                       DebugCapsule(w._pobjects[ccd['item']], ccd['label'])
               else:
                   for dobj in list(w._pobjects[ccd['item']]._debugObjects):
                       dobj.remove()
               w._pobjects["abe"].setBodyProperty("fn", "done", True)
               with executingAction:
                   executingAction.notify_all()
            elif 'fetch' == ccd['op']:
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
                cabinet = w._pobjects["kitchenCabinet"]
                g._processes = {}
                g._commandProcess = garden.Process(coherence=[procs.ProportionedItem(item, amount, store),procs.ItemOnLocation(item,cabinet),procs.ParkedArms(agent)])
                placeCamera(item)
            elif 'transfer' == ccd['op']:
                item = w._pobjects[ccd['item']]
                store = w._pobjects[ccd['destination']]
                agent = w._pobjects[list(w._ontoTypes["agent"])[0]]
                cabinet = w._pobjects["kitchenCabinet"]
                g._processes = {}
                g._commandProcess = garden.Process(coherence=[procs.TransferredContents(item,store),procs.ItemOnLocation(item,cabinet),procs.ParkedArms(agent)])
            elif 'mix' == ccd['op']:
                item = w._pobjects[ccd['item']]
                tool = w._pobjects[ccd['tool']]
                agent = w._pobjects[list(w._ontoTypes["agent"])[0]]
                counter = w._pobjects["counterTop"]
                cabinet = w._pobjects["kitchenCabinet"]
                g._processes = {}
                substance = procs.getMixedSubstance(item)
                g._commandProcess = garden.Process(coherence=[procs.MixedContents(item, tool, substance),procs.ItemOnLocation(tool,cabinet),procs.ParkedArms(agent)])
                placeCamera(item)
            elif 'shape' == ccd['op']:
                sourceContainer = w._pobjects[ccd['sourceContainer']]
                particlesPerShape = 4 ## TODO
                destinationContainer = w._pobjects[ccd['destinationContainer']]
                g._commandProcess = garden.Process(coherence=[procs.ShapedStuffInto(sourceContainer, DoughClump, particlesPerShape, destinationContainer)])
            elif 'line' == ccd['op']:
                inputContainer = w._pobjects[ccd['inputContainer']]
                lining = w._pobjects[ccd['lining']]
                g._commandProcess = garden.Process(coherence=[procs.LinedContainer(inputContainer, lining)])
            elif 'bake' == ccd['op']:
                inputContainer = w._pobjects[ccd['inputContainer']]
                oven = w._pobjects[ccd['oven']]
                destinationContainer = w._pobjects[ccd['destinationContainer']]
                g._commandProcess = garden.Process(coherence=[procs.BakedContents(inputContainer, oven, destinationContainer)])
            elif 'sprinkle' == ccd['op']:
                inputTargetContainer = w._pobjects[ccd['inputTargetContainer']]
                inputToppingContainer = w._pobjects[ccd['inputToppingContainer']]
                cabinet = w._pobjects["kitchenCabinet"]
                g._commandProcess = garden.Process(coherence=[procs.SprinkledContents(inputTargetContainer,inputToppingContainer),procs.ItemOnLocation(inputToppingContainer,cabinet)])
            elif 'wait' == ccd['op']:
                waitingFor = ccd['frames']
            ccd = None
        w.update()
        if 0 < waitingFor:
            waitingFor = waitingFor - 1
            if 0 >= waitingFor:
                with executingAction:
                    executingAction.notify_all()
        if (g._commandProcess is not None) and (0 < len(g._commandProcess._coherence)):
            bodyProcs = g.updateGarden()
            if all([x.isFulfilled() for x in g._commandProcess._coherence]):
                if g._commandProcess is not None:
                    g._commandProcess = None
                    cwd = w.worldDump()
                    with executingAction:
                        executingAction.notify_all()
            else:
                ##print("bps", bodyProcs)
                [x.bodyAction() for x in bodyProcs]
    if not isAMac:
        time.sleep(1.0/240.0)

