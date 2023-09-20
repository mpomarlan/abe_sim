import json
import os
import requests
import sys
import time

crOp = 0
if 1 < len(sys.argv):
    crOp = int(sys.argv[1])

pathTemplate = "./dbgWS/bsWS{}.log"

commands = [
        ["http://localhost:54321/abe-sim-command/to-preheat-oven", {'oven': 'oven', 'quantity': 200, 'unit': 'C', 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-fry", {'thingToFry': 'fryingPan', 'stoveToFryOn': 'oven', 'heatingMode': 'medium', "timeToFryQuantity": 1, "timeToFryUnit": "min", 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-preheat-oven", {'oven': 'oven', 'quantity': 150, 'unit': 'C', 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'cuttingBoard', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-place", {'object': 'bacon', 'container': 'cuttingBoard', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-place", {'object': 'fryingPan', 'container': 'kitchenCabinet', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'largeBowl1', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-cut", {'object': 'bacon', 'cuttingTool': 'cookingKnife', 'cutPattern': 'dice', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-place", {'object': ["ChoppedCookedBacon"], 'container': 'largeBowl1', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-place", {'object': 'broccoli', 'container': 'cuttingBoard', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-cut", {'object': 'broccoli', 'cuttingTool': 'cookingKnife', 'cutPattern': 'dice', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-place", {'object': ["ChoppedBroccoli"], 'container': 'largeBowl1', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-place", {'object': 'cuttingBoard', 'container': 'kitchenCabinet', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'mediumBowl1', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-peel", {'inputIngredient': 'potato', 'peelingTool': 'cookingKnife', 'containerForPeels': 'mediumBowl1', 'containerForPeeledIngredient': 'largeBowl1', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-mash", {'inputIngredient': 'largeBowl1', 'mashingTool': 'masher', 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-place", {'object': 'cookingKnife', 'container': 'kitchenCabinet', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-place", {'object': 'mediumBowl1', 'container': 'kitchenCabinet', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'mediumBowl2', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-portion", {'containerWithIngredient': 'mayonnaiseJar', 'targetContainer': 'mediumBowl2', 'quantity': 134, 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-portion", {'containerWithIngredient': 'ciderVinegarBottle', 'targetContainer': 'mediumBowl2', 'quantity': 134, 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-mix", {'containerWithInputIngredients': 'mediumBowl2', 'mixingTool': 'whisk', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-transfer", {'containerWithInputIngredients': 'mediumBowl2', 'targetContainer': 'largeBowl1', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-mingle", {'containerWithInputIngredients': 'largeBowl1', 'minglingTool': 'whisk', 'kitchenStateIn': None, 'setWorldState': False}],
        ["http://localhost:54321/abe-sim-command/to-refrigerate", {'containerWithIngredients': 'largeBowl1', 'refrigerator': 'fridge', 'setWorldState': False}],
#        ["http://localhost:54321/abe-sim-command/to-preheat-oven", {'oven': 'oven', 'quantity': 200, 'unit': 'C', 'setWorldState': False}, None, "./mWS1.log"],
#        ["http://localhost:54321/abe-sim-command/to-uncover", {'object': 'mediumBowl1', 'setWorldState': False}, "./mWS1.log", "./mWS2.log"],
#        ["http://localhost:54321/abe-sim-command/to-cover", {'object': 'mediumBowl1', 'cover': 'mediumBowlLid1', 'setWorldState': False}, "./mWS2.log", "./mWS3.log"],
#        ["http://localhost:54321/abe-sim-command/to-refrigerate", {'containerWithIngredients': 'mediumBowl2', 'refrigerator': 'fridge', 'setWorldState': False}, "./mWS3.log", "./mWS4.log"],
#        ["http://localhost:54321/abe-sim-command/to-fry", {'thingToFry': 'fryingPan', 'stoveToFryOn': 'oven', 'heatingMode': 'medium', "timeToFryQuantity": 1, "timeToFryUnit": "min", 'setWorldState': False}, "./mWS4.log", "./mWS5.log"],
#        ["http://localhost:54321/abe-sim-command/to-mash", {'inputIngredient': 'mediumBowl3', 'mashingTool': 'masher', 'setWorldState': False}, "./mWS5.log", "./mWS6.log"],
    ]

pathIn = None
crOutFile = 1
if 0 < crOp:
    pathIn = pathTemplate.format(crOp)
    crOutFile = crOp + 1

if True:#crOp < 7:
    for k, e in enumerate(commands[crOp:]):
        command, req = e
        pathOut = pathTemplate.format(crOutFile)
        crOutFile += 1
        kSI = None
        if (0 == k) and (pathIn is not None):
            kSI = json.loads(open(pathIn).read())
            req['setWorldState'] = True
        req['kitchenStateIn'] = kSI
        if (not command.endswith("to-place")) or (isinstance(req["object"], str)):
            r = requests.post(command, data=bytes(json.dumps(req), "utf-8"))
            response = json.loads(r.text)
        else:
            dws = {'kitchenStateIn': '?kitchen-state-1'}
            r = requests.post("http://localhost:54321/abe-sim-command/to-get-kitchen", data=bytes(json.dumps(dws), "utf-8"))
            worldState = json.loads(r.text)['response']['?kitchen-state-1']
            toPlace = [x for x in worldState.keys() if (worldState[x]['type'] in req["object"])]# and ('kitchenCounter' == worldState[x]['at'])]
            for e in toPlace:
                req["object"] = e
                r = requests.post(command, data=bytes(json.dumps(req), "utf-8"))
                response = json.loads(r.text)
                req["setWorldState"] = False
        print(response)
        if (pathOut is not None) and ('response' in response) and ('kitchenStateOut' in response['response']):
            with open(pathOut, "w") as outfile:
                _ = outfile.write("%s\n" % json.dumps(response['response']['kitchenStateOut']))
        pathIn = pathOut

sys.exit()

