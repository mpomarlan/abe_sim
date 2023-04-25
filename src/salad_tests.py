import json
import os
import requests
import sys
import time

crOp = 0
if 1 < len(sys.argv):
    crOp = int(sys.argv[1])

commands = [
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'largeBowl', 'kitchenStateIn': None, 'setWorldState': False}, None, "./sWS1.log"],
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'redOnion', 'kitchenStateIn': None, 'setWorldState': False}, "./sWS1.log", "./sWS2.log"],
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'broccoli', 'kitchenStateIn': None, 'setWorldState': False}, "./sWS2.log", "./sWS3.log"],
        ["http://localhost:54321/abe-sim-command/to-cut", {'object': 'bacon', 'cuttingTool': 'cookingKnife', 'cutPattern': 'dice', 'kitchenStateIn': None, 'setWorldState': False}, "./sWS3.log", "./sWS4.log"],
        ["http://localhost:54321/abe-sim-command/to-cut", {'object': 'fetaCheese', 'cuttingTool': 'cookingKnife', 'cutPattern': 'dice', 'kitchenStateIn': None, 'setWorldState': False}, "./sWS4.log", "./sWS5.log"],
        ["http://localhost:54321/abe-sim-command/to-cut", {'object': 'broccoli', 'cuttingTool': 'cookingKnife', 'cutPattern': 'dice', 'kitchenStateIn': None, 'setWorldState': False}, "./sWS5.log", "./sWS6.log"],
        ["http://localhost:54321/abe-sim-command/to-cut", {'object': 'redOnion', 'cuttingTool': 'cookingKnife', 'cutPattern': 'dice', 'kitchenStateIn': None, 'setWorldState': False}, "./sWS6.log", "./sWS7.log"]
    ]

if crOp < 7:
    for k, e in enumerate(commands[crOp:]):
        command, req, pathIn, pathOut = e
        kSI = None
        if (0 == k) and (pathIn is not None):
            kSI = json.loads(open(pathIn).read())
            req['setWorldState'] = True
        req['kitchenStateIn'] = kSI
        r = requests.post(command, data=bytes(json.dumps(req), "utf-8"))
        response = json.loads(r.text)
        print(response)
        if (pathOut is not None) and ('response' in response) and ('kitchenStateOut' in response['response']):
            with open(pathOut, "w") as outfile:
                _ = outfile.write("%s\n" % json.dumps(response['response']['kitchenStateOut']))

dgr = {'kitchenStateIn': json.loads(open('sWS7.log').read())}
r = requests.post("http://localhost:54321/abe-sim-command/to-set-kitchen", data=bytes(json.dumps(dgr), "utf-8"))
response = json.loads(r.text)

dws = {'kitchenStateIn': '?kitchen-state-1'}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-kitchen", data=bytes(json.dumps(dws), "utf-8"))
worldState = json.loads(r.text)['response']['?kitchen-state-1']

toPlace = [x for x in worldState.keys() if (worldState[x]['type'] in ['ChoppedBroccoli', 'ChoppedRedOnion', 'FetaCheeseCube', 'ChoppedBacon']) and ('kitchenCounter' == worldState[x]['at'])]

print(toPlace)
for e in toPlace:
    dws = {'kitchenStateIn': '?kitchen-state-1'}
    r = requests.post("http://localhost:54321/abe-sim-command/to-get-kitchen", data=bytes(json.dumps(dws), "utf-8"))
    worldState = json.loads(r.text)['response']['?kitchen-state-1']
    if (e not in worldState) or ('kitchenCounter' != worldState[e]['at']):
        continue
    r = requests.post("http://localhost:54321/abe-sim-command/to-place", data=bytes(json.dumps({'object': e, 'container': 'largeBowl'}), "utf-8"))

sys.exit()

