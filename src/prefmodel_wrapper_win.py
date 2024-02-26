import os
import json
import requests
import copy
import sys
import time
from threading import Thread

from TTS.api import TTS
#import sounddevice as sd
#sd.default.samplerate = 16000

import pyttsx3

import owlready2
from flask import Flask, request

# TODO: consider using ROS' smach package to implement the state machine(s)
#import smach

nluServer = "http://localhost:5005/model/parse"

utteranceSemantics = {}

doTheSpeaking = False

SIDX_START = 0
SIDX_BEGIN_BRING = 1
SIDX_ASK_PREFERENCE = 2
SIDX_ASK_PREFERENCE_SPECIFICS = 3
SIDX_ASK_ALTERNATIVE = 4
SIDX_STATE_ALTERNATIVE = 5
SIDX_CHECK_GOAL = 6

interactionState = {"fn_sidx": None, "entities": {}}
feasible = {}
speaker = {"agent": "Alice", "tts": None}
aliases = {"coffee": "dCoffee", "cup of coffee": "dCoffee", 
           "cafe au lait": "dCafeAuLait", "coffee with milk": "dCafeAuLait", "cup of coffee with milk": "dCafeAuLait", 
           "tea": "dTea", "cup of tea": "dTea",
           "drink": "dDrinkable", "dDrink": "dDrinkable", "something to drink": "dDrinkable", 
           "food": "dEdible", "something to eat": "dEdible", "some food": "dEdible",
           "vegan food": "dVeganFood",
           "chili sin carne": "dChiliSinCarne",
           "chili con carne": "dChiliConCarne"}
reification2Alias = {"dCafeAuLait": "cafe au lait", "dTea": "tea"}

reification2ObjectName = {"coffee": "coffee",
                          "dCoffee": "coffee",
                          "dCafeAuLait": "cafeAuLait",
                          "dBlackCoffee": "blackCoffee",
                          "dTea": "tea",
                          "dChiliConCarne": "chiliConCarne",
                          "dChiliSinCarne": "chiliSinCarne"}

basePath = os.path.dirname(os.path.abspath(__file__))
wavPath = os.path.join(basePath, "tmp.wav")
onto = {"owl": None}

def initOnto():
    onto["owl"] = owlready2.get_ontology(os.path.join(basePath, "SOMA_preferences.owl")).load()
    owlready2.sync_reasoner(infer_property_values=True)

def unAlias(s):
    return aliases.get(s,s)
def unPrefix(s):
    idx = s.find(":")
    if -1 == idx:
        return s
    return s[idx+1:]
def requestSay(s):
    speaker["tts"].save_to_file(s, wavPath)
    speaker["tts"].runAndWait()
    if doTheSpeaking:
        speaker["tts"].say(s)
        speaker["tts"].runAndWait()
    #wav = speaker["tts"]._speaker.tts(s, speaker=speaker["tts"]._speaker.speakers[0], language=speaker["tts"]._speaker.languages[0])    
    interactionState["requestedWAV"] = wavPath
def requestPerformAction(action, parameters, context=None):
    if context is None:
        context = ""
    item = reification2ObjectName.get(aliases.get(parameters.get("Item")),"")
    beneficiary = parameters.get("BeneficiaryRole","")
    requestSay("%s. Ok, I will bring %s to %s." % (context, item, beneficiary))
    interactionState["requestedObject"] = item
def queryPreferencesAnalogous(frame, agent, item, wide=False, blacklist=None):
    # Find topmost agent's preference that uses an item that specializes what the item in description also specializes 
    if blacklist is None:
        blacklist = []
    blacklist = set([unAlias(x) for x in blacklist])
    item = unAlias(item)
    r = list(owlready2.default_world.as_rdflib_graph().query_owlready(
                '''
                prefix dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#> 
                prefix soma: <http://www.ease-crc.org/ont/SOMA.owl#>
                SELECT DISTINCT ?i ?oe
                WHERE
                {
                    ?oe soma:encapsulates ?sd .
                    ?sd dul:defines ?rf .
                    ?sd dul:defines ?rb .
                    ?sd dul:defines ?ri .
                    ?rf dul:classifies soma:%s .
                    ?rb dul:classifies soma:%s .
                    ?ri dul:classifies ?i .
                    ?i dul:specializes ?a .
                    soma:%s dul:specializes ?a .
                    ?rf rdf:type soma:FrameRole .
                    ?rb rdf:type soma:BeneficiaryRole .
                    ?ri rdf:type soma:Item .
                    ?p rdf:type soma:Preference .
                    ?pd rdf:type soma:PreferenceOrder .
                    ?pd dul:describes ?p .
                    ?pd soma:orders ?oe .
                    ?p dul:isQualityOf soma:%s .
                }
                ''' % (frame, agent, item, agent)))
    if (0 == len(r)) or (0 == len(r[0])):
        return None
    mapO2I = {x[1].name: x[0] for x in r}
    mapI2S = {x[0].name: set(mapO2I[y.name].name for y in x[1].follows)for x in r}
    candidates = [x for x,s in mapI2S.items() if (0 == len(s.difference(blacklist))) and (x not in blacklist)]
    if 0 == len(candidates):
        return None
    return candidates[0]
def queryPreferencesCompatible(frame, agent, item):
    # Are the agent's preferences compatible with what is described?
    item = unAlias(item)
    pref = queryPreferencesAnalogous(frame, agent, item, wide=True)
    print("  QPC", item, pref)
    if pref is not None:
        r = list(owlready2.default_world.as_rdflib_graph().query_owlready('''
                    prefix dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#> 
                    prefix soma: <http://www.ease-crc.org/ont/SOMA.owl#>
                    SELECT ?x
                    WHERE
                        {?x soma:disjointReification soma:%s}
                    ''' % (item)))
        print("    ", r)
        if (pref in [unPrefix(x[0]._name) for x in r if 0 < len(x)]):
            return False
    return True
def queryReificationsSpecifiers(item):
    # Find which reifications specialize another
    r = list(owlready2.default_world.as_rdflib_graph().query_owlready('''
                prefix dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#> 
                prefix soma: <http://www.ease-crc.org/ont/SOMA.owl#>
                SELECT DISTINCT ?x
                WHERE
                    {?x rdf:type soma:Reification .
                     ?x dul:specializes soma:%s}
                ''' % unAlias(item)))
    if 0 == len(r):
        return []
    return [unPrefix(x[0]._name) for x in r if 0 < len(x)]
def assertPreferences(frame, beneficiary, item, sups):
    # add an ordered element that encapsulates a situation description for (frame, beneficiary, item)
    # connect the element to the preference of the beneficiary
    # look for set S of ordered elements which encapsulate sds of form (frame, beneficiary, x in sups)
    # add a follows relation from the new ordered element to all x in S
    item = unAlias(item)
    sups = set([unAlias(x) for x in sups])
    r = list(owlready2.default_world.as_rdflib_graph().query_owlready('''
                prefix dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#> 
                prefix soma: <http://www.ease-crc.org/ont/SOMA.owl#>
                SELECT DISTINCT ?pd ?p
                WHERE
                {
                    ?pd rdf:type soma:PreferenceOrder .
                    ?p rdf:type soma:Preference .
                    ?p dul:isDescribedBy ?pd .
                    ?p dul:isQualityOf soma:%s .
                }''' % beneficiary))
    if (0 < len(r)) and (0 < len(r[0])):
        pd = r[0][0]
        p = r[0][1]
    else:
        pd = onto["owl"]["PreferenceOrder"]()
        p = onto["owl"]["Preference"]()
        onto["owl"][beneficiary].hasQuality.append(p)
        pd.describes.append(p)
    roe = list(owlready2.default_world.as_rdflib_graph().query_owlready('''
                prefix dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#> 
                prefix soma: <http://www.ease-crc.org/ont/SOMA.owl#>
                SELECT DISTINCT ?ooe ?i
                WHERE
                {
                    ?p dul:isQualityOf soma:%s .
                    ?p rdf:type soma:Preference .
                    ?pd rdf:type soma:PreferenceOrder .
                    ?pd dul:describes ?p .
                    ?pd soma:orders ?ooe .
                    ?ooe rdf:type soma:OrderedElement .
                    ?ooe soma:encapsulates ?osd .
                    ?osd rdf:type dul:Description .
                    ?orf rdf:type soma:FrameRole .
                    ?orb rdf:type soma:BeneficiaryRole .
                    ?ori rdf:type soma:Item .
                    ?osd dul:defines ?orf .
                    ?osd dul:defines ?orb .
                    ?osd dul:defines ?ori .
                    ?orf dul:classifies soma:%s .
                    ?orb dul:classifies soma:%s .
                    ?ori dul:classifies ?i .
                }
                ''' % (beneficiary, frame, beneficiary)))
    roe = [x[0] for x in roe if x[1].name in sups]
    noe = onto["owl"]["OrderedElement"]()
    nsd = [x for x in onto["owl"].classes() if "Description" == x.name][0]()
    nrf = onto["owl"]["FrameRole"]()
    nrb = onto["owl"]["BeneficiaryRole"]()
    nri = onto["owl"]["Item"]()
    noe.encapsulates.append(nsd)
    nsd.defines.append(nrf)
    nsd.defines.append(nrb)
    nsd.defines.append(nri)
    nrf.classifies.append(onto["owl"][frame])
    nrb.classifies.append(onto["owl"][beneficiary])
    nri.classifies.append(onto["owl"][item])
    pd.orders.append(noe)
    for e in roe:
        noe.follows.append(e)
    owlready2.sync_reasoner(infer_property_values=True)
def verbalizeBeneficiary(b):
    if speaker["agent"] == b:
        return "you"
    return b

def fn_sidx_start(iS, gS):
    intent =  gS["dialog/UserIntent"]
    if "AssertSpeaker" == intent:
        agent = gS.get("dialog/AgentRole")
        if agent is None:
            requestSay("I did not catch that, could you repeat please?")
            iS["fn_sidx"] = fn_sidx_start
            iS["entities"] = {}
            return False
        else:
            speaker["agent"] = agent
            requestSay("Hello, %s" % agent)
            iS["fn_sidx"] = fn_sidx_start
            iS["entities"] = {}
            return False
    if "Transporting" == intent:
        beneficiary = gS.get("dialog/BeneficiaryRole")
        item = gS.get("dialog/Item")
        destination = gS.get("dialog/Destination")
        if (item is None) or ((destination is None) and (beneficiary is None)):
            requestSay("I did not understand that, could you repeat the command please?")
            return False
        if beneficiary is None:
            print("  BNone", {k[len("dialog/"):]:v for k, v in gS.items() if k.startswith("dialog/")})
            requestPerformAction("Transporting", {k[len("dialog/"):]:v for k, v in gS.items() if k.startswith("dialog/")})
            iS["fn_sidx"] = fn_sidx_start
            iS["entities"] = {}
            return False
        else:
            iS["entities"] = {"beneficiary": beneficiary, "item": item}
            iS["fn_sidx"] = fn_sidx_begin_bring
            return True
    if "AssertFeasibility" == intent:
        item = gS.get("dialog/Item")
        if item is None:
            requestSay("I did not understand that, can you repeat please?")
            iS["fn_sidx"] = fn_sidx_start
            iS["entities"] = {}
            return False
        else:
            feasible[aliases.get(item)] = True
            requestSay("I understand we have %s in stock." % item)
            return False
    if "AssertUnfeasibility" == intent:
        item = gS.get("dialog/Item")
        if item is None:
            requestSay("I did not understand that, can you repeat please?")
            iS["fn_sidx"] = fn_sidx_start
            iS["entities"] = {}
            return False
        else:
            feasible[aliases.get(item)] = False
            requestSay("I understand, we are out of %s" % item)
            return False
    if "AssertTransportGoal" == intent:
        beneficiary = gS.get("dialog/BeneficiaryRole")
        item = gS.get("dialog/Item")
        destination = gS.get("dialog/Destination")
        if (item is None) or ((destination is None) and (beneficiary is None)):
            requestSay("I did not understand that, could you repeat please?")
            return False
        if beneficiary is None:
            iS["fn_sidx"] = fn_sidx_start
            iS["entities"] = {}
            requestSay("Ok.")
            return False
        else:
            iS["entities"] = {"beneficiary": beneficiary, "item": item}
            iS["fn_sidx"] = fn_sidx_check_goal
            return True
    requestSay("I did not understand that, let's start from the beginning.")
    return False

def fn_sidx_begin_bring(iS, gS):
    pref = queryPreferencesAnalogous("dReceiveForConsumption", iS["entities"]["beneficiary"], iS["entities"]["item"])
    if pref is None:
        requestSay("What would %s like?" % verbalizeBeneficiary(iS["entities"]["beneficiary"]))
        iS["fn_sidx"] = fn_sidx_ask_preference
        return False
    else:
        specs = [x for x in queryReificationsSpecifiers(pref) if pref != x]
        if 0 < len(specs):
            if 1 == len(specs):
                requestSay("Would %s like %s? or perhaps something else?" % (verbalizeBeneficiary(iS["entities"]["beneficiary"]), specs[0]))
                iS["fn_sidx"] = fn_sidx_ask_preference
                iS["entities"]["asked"] = [specs[0]]
                return False
            else:
                requestSay("Would %s like %s or %s or perhaps something else?", (verbalizeBeneficiary(iS["entities"]["beneficiary"]), specs[0], specs[1]))
                iS["fn_sidx"] = fn_sidx_ask_preference
                iS["entities"]["asked"] = [specs[0], specs[1]]
                return False
        else:
            if feasible.get(pref, True):
                print("  Feasible", {k[len("dialog/"):]:v for k, v in gS.items() if k.startswith("dialog/")})
                requestPerformAction("Transporting", {k[len("dialog/"):]:v for k, v in gS.items() if k.startswith("dialog/")})
                iS["fn_sidx"] = fn_sidx_start
                iS["entities"] = {}
                return False
            else:
                bl = [pref]
                while True:
                    pref = queryPreferencesAnalogous("dReceiveForConsumption", iS["entities"]["beneficiary"], iS["entities"]["item"], blacklist=bl)
                    if (pref is None) or (feasible.get(pref, True)):
                        break
                    bl.append(pref)
                if pref is None:
                    requestSay("We don't have %s anymore, what would %s like instead?" % (bl[0], verbalizeBeneficiary(iS["entities"]["beneficiary"])))
                    iS["fn_sidx"] = fn_sidx_ask_preference
                    iS["entities"]["blist"] = bl
                    return False
                else:
                    #requestSay("We don't have %s anymore, but we have %s instead so I will bring %s that." % (bl[0], pref, verbalizeBeneficiary(iS["entities"]["beneficiary"])))
                    gS["dialog/Item"] = reification2Alias.get(pref)
                    print("  Replacement", {k[len("dialog/"):]:v for k, v in gS.items() if k.startswith("dialog/")})
                    requestPerformAction("Transporting", {k[len("dialog/"):]:v for k, v in gS.items() if k.startswith("dialog/")}, context = "We don't have %s anymore, but we have %s instead." % (bl[0], pref, verbalizeBeneficiary(iS["entities"]["beneficiary"])))
                    iS["fn_sidx"] = fn_sidx_start
                    iS["entities"] = {}
                    return False

def fn_sidx_ask_preference(iS, gS):
    intent =  gS["dialog/UserIntent"]
    sups = iS["entities"].get("blist",[])
    asked = iS["entities"].get("asked",[])
    if "AssertPreference" == intent:
        beneficiary = gS.get("dialog/BeneficiaryRole")
        item = gS.get("dialog/Item")
        if (beneficiary is None) or (item is None):
            requestSay("I did not understand that, could you repeat please?")
            return False
        else:
            assertPreferences("dReceiveForConsumption", beneficiary, item, sups)
            if beneficiary != iS["entities"]["beneficiary"]:
                requestSay("Thank you, but what would %s like?" % (verbalizeBeneficiary(iS["entities"]["beneficiary"])))
                return False
            else:
                iS["entities"]["item"] = item
                iS["fn_sidx"] = fn_sidx_begin_bring
                return True
    if "Agreement" == intent:
        if 1 == len(asked):
            iS["entities"]["item"] = asked[0]
            iS["fn_sidx"] = fn_sidx_begin_bring
            return True
        elif 0 == len(asked):
            requestSay("Sorry, I did not quite catch that, what would %s like?" % verbalizeBeneficiary(iS["entities"]["beneficiary"]))
            return False
        else:
            requestSay("Did I understand right -- is %s ok?" % asked[0])
            iS["entities"]["asked"] = [asked[0]]
            return False
    requestSay("I did not understand that, let's start from the beginning.")
    iS["fn_sidx"] = fn_sidx_start
    iS["entities"]
    return False

def fn_sidx_check_goal(iS, gS):
    print("  CheckGoal", iS, gS)
    if not queryPreferencesCompatible("dReceiveForConsumption", iS["entities"]["beneficiary"], iS["entities"]["item"]):
        requestSay("%s will not like that, they prefer %s." % (iS["entities"]["beneficiary"], queryPreferencesAnalogous("dReceiveForConsumption", iS["entities"]["beneficiary"], iS["entities"]["item"])))
    else:
        requestSay("Ok.")
    iS["fn_sidx"] = fn_sidx_start
    iS["entities"] = {}
    return False
    

def step(interactionState, utteranceSemantics):
    return interactionState["fn_sidx"](interactionState, utteranceSemantics)

'''
RASA wrapper for NLU -> dm state (intent, entities) -> BLACKBOARD -> PREFMODEL

PREFMODEL -> actspec -> ABESIM/CRAM
          -> question -> RASA wrapper/TTS


"bring me coffee" -> intent bring etc. -> prefmodel: query preference:
    what is preference?
    is it possible to be more specific?
Questions:
"what would you like?"
"would you like X or Y?"
    intent: state preference

"we are out of X, what would you like?"
"we are out of X, would Y be ok?"


intent: state task
response: check task against KB

prefmodel chooses:
    act or question

choosing act: send request to ABESIM/CRAM
choosing question: send request to TTS





States of interaction:

1) Start
    upon receiving bring intent switch to 2
    upon receiving intent statement switch to 7
2) Have [bring] command
    query prefmodel, if known, suffspec [and feasible] act and switch to 1 elif
        not known pref, switch to 3, elif
        known but unspec pref, switch to 4, elif
        known but unfeasible pref, no known alternative, switch to 5, elif
        known but unfeasible pref, known alternative, switch to 6
3) Inquire preference (e.g. "What would you like to drink?")
    upon receiving pref.statement intent, update KB and crstate and switch to 2
4) Inquire preference spec (e.g. "Would you like cafe au lait or black coffee?")
    upon receiving pref.statement intent, update KB and crstate and switch to 2
5) Inquire alternative (e.g. "We're out of coffee, what would you like instead?")
    upon receiving pref.statement intent, update KB and crstate and switch to 2
6) State alternative (e.g. "We're out of coffee so is it ok if I bring you tea?")
    upon receiving pref.statement intent, update KB and crstate and switch to 2
    upon receiving agreement, switch to 2
7) Check goal
    query prefmodel for compatibility between stated goal and known preferences
        disjoint descriptions: emit warning, switch to 1 else
        switch to 1

State of interaction:
    state id/flag
    entities:
        type of action being discussed: robot brings something to user (speaker) | someone (speaker) brings something to someone else
        entities involved in action:
            agent
            beneficiary
            item_requested
            item_to_bring

'''

# Connect to Unity Whisper, so change msg here from a ROS msg to whatever we need.
# ALSO, need to connect to RASA NLU
def onStateUpdate(text, interactionState, utteranceSemantics):
    req = {"text": text}
    r = requests.post(nluServer, data=bytes(json.dumps(req), "utf-8"))
    response = json.loads(r.text)
    if True:
        interactionState["requestedWAV"] = ""
        interactionState["requestedObject"] = ""
        utteranceSemantics.clear()
        utteranceSemantics["dialog/UserIntent"] = response["intent"]["name"]
        for e in response['entities']:
            utteranceSemantics["dialog/" + e.get("role", "UndefinedRole")] = e.get("value", "UnparsedEntity")
        if utteranceSemantics.get("dialog/BeneficiaryRole") in ["I", "i", "me"]:
            utteranceSemantics["dialog/BeneficiaryRole"] = speaker["agent"]
        print("  ", utteranceSemantics)
        procState = True
        while procState:
            procState = step(interactionState, utteranceSemantics)

def thread_function_flask():
    flask = Flask(__name__)
    @flask.route("/preferences/tell", methods=['POST'])
    def tell():
        print("Got told.")
        try:
            msg = request.get_json(force=True)
        except SyntaxError:
            return json.dumps({'wav': '', 'object': ''}), requests.status_codes.codes.BAD_REQUEST
        print("  ", msg)
        text = msg.get("text", "")
        onStateUpdate(text, interactionState, utteranceSemantics)
        return json.dumps({'wav': interactionState["requestedWAV"], "object": interactionState["requestedObject"]}), requests.status_codes.codes.ALL_OK
    flask.run(port=44444, debug=True, use_reloader=False)

if __name__ == '__main__':
    if 1 < len(sys.argv):
        doTheSpeaking = True
    try:
        interactionState["fn_sidx"] = fn_sidx_start
        # Connect to TEXT input from Unity Whisper asset -- it must trigger a callback
        # Connect to AbeSim : via POST request in requestPerformAction
        # Connect to TTS
        #model_name = TTS.list_models()[0]
        #speaker["tts"] = TTS(model_name)
        speaker["tts"] = pyttsx3.init()
        speaker["tts"].setProperty('rate', 150)
        speaker["tts"].setProperty('volume',1.0)
        initOnto()
        flaskThread = Thread(target=thread_function_flask, args=())
        flaskThread.start()
        print("Preference model started.")
        while True:
            pass
    except Exception as e:
        print(e)



'''
 def foo(s):
    r = requests.post(command, bytes(json.dumps({"text":s}),"utf-8"))
    return json.loads(r.text)
'''