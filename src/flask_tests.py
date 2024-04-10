import json
import os
import requests
import sys
import time

'''
### Command _: get current simulation time
req = {}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-time", data=bytes(json.dumps(req), "utf-8"))
response = json.loads(r.text)['response']
print("SIMULATION TIME:", response)

### Command _: cancel any ongoing action. Robot will also drop anything it carries and will stop trying to clopen anything.
req = {"smart": False}
r = requests.post("http://localhost:54321/abe-sim-command/to-cancel", data=bytes(json.dumps(req), "utf-8"))
response = json.loads(r.text)['response']
print("CANCELLATION:", response)

def cancelAbeSmart(smart = False):
    req = {'smart': smart}
    r = requests.post("http://localhost:54321/abe-sim-command/to-cancel", data=bytes(json.dumps(req), "utf-8"))
    response = json.loads(r.text)['response']
    print("CANCELLATION:", response)
    
### Command _: fetch mediumBowl3 to kitchen counter
def fetch():
    req= {'object': 'mediumBowl3', 'kitchenStateIn': None, 'setWorldState': False}
    r = requests.post("http://localhost:54321/abe-sim-command/to-fetch", data=bytes(json.dumps(req), "utf-8"))
    response = json.loads(r.text)['response']
    print("FETCHING:", response)

### Command _: go to specified pose. Not specifying pose will yield an error message.
req = {}
r = requests.post("http://localhost:54321/abe-sim-command/to-go-to-pose", data=bytes(json.dumps(req), "utf-8"))
response = json.loads(r.text)['response']
print("GO TO POSE (parameters missing):", response)

### Command _: go to specified pose. Note: returns immediately, does not wait for robot to get there.
req = {"position":[0,-2], "yaw": -1.57}
r = requests.post("http://localhost:54321/abe-sim-command/to-go-to-pose", data=bytes(json.dumps(req), "utf-8"))
response = json.loads(r.text)['response']
print("GO TO POSE (parameters ok):", response)
time.sleep(5)

### Command 1: get the kitchen state
dws = {'kitchenStateIn': '?kitchen-state-1'}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-kitchen", data=bytes(json.dumps(dws), "utf-8"))
worldState = json.loads(r.text)['response']
print(worldState)

### Command _: go to specified pose. Note: returns immediately, does not wait for robot to get there.
req = {"position":[0,2], "yaw": 1.57}
r = requests.post("http://localhost:54321/abe-sim-command/to-go-to-pose", data=bytes(json.dumps(req), "utf-8"))
response = json.loads(r.text)['response']
print("GO TO POSE (parameters ok):", response)
time.sleep(5)

### Command 2: set the kitchen state
dgr = {'kitchenStateIn': worldState["?kitchen-state-1"]}
r = requests.post("http://localhost:54321/abe-sim-command/to-set-kitchen", data=bytes(json.dumps(dgr), "utf-8"))
response = json.loads(r.text)
print("SET KITCHEN STATE:", response)

### Command _: go to specified pose. Note: returns immediately, does not wait for robot to get there.
req = {"position":[0,2], "yaw": 1.57}
r = requests.post("http://localhost:54321/abe-sim-command/to-go-to-pose", data=bytes(json.dumps(req), "utf-8"))
response = json.loads(r.text)['response']
print("GO TO POSE (parameters ok):", response)
time.sleep(5)

### Command _: get updates. A more concise WorldDump.
req = {}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-state-updates", data=bytes(json.dumps(req), "utf-8"))
response = json.loads(r.text)['response']
print("GET STATE UPDATE:", response)

### Command 3: get location
gc = {'availableLocation': '?available-bowl', 'type': 'MediumBowl', 'kitchen': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-location", data=bytes(json.dumps(gc), "utf-8"))
response = json.loads(r.text)
print(response)
'''

crOp = 0
if 1 < len(sys.argv):
    crOp = int(sys.argv[1])

commands = [
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'mediumBowl1', 'kitchenStateIn': None, 'setWorldState': False}, None, "./WS1.log"],
        ["http://localhost:54321/abe-sim-command/to-portion", {'containerWithIngredient': 'sugarBag', 'targetContainer': 'mediumBowl1', 'quantity': 134, 'kitchenStateIn': None, 'setWorldState': False}, "./WS1.log", "./WS2.log"],
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'mediumBowl2', 'kitchenStateIn': None, 'setWorldState': False}, "./WS2.log", "./WS3.log"],
        ["http://localhost:54321/abe-sim-command/to-portion", {'containerWithIngredient': 'butterBag', 'targetContainer': 'mediumBowl2', 'quantity': 134, 'kitchenStateIn': None, 'setWorldState': False}, "./WS3.log", "./WS4.log"],
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'mediumBowl3', 'kitchenStateIn': None, 'setWorldState': False}, "./WS4.log", "./WS5.log"],
        ["http://localhost:54321/abe-sim-command/to-transfer", {'containerWithInputIngredients': 'mediumBowl1', 'targetContainer': 'mediumBowl3', 'kitchenStateIn': None, 'setWorldState': False}, "./WS5.log", "./WS6.log"],
        ["http://localhost:54321/abe-sim-command/to-transfer", {'containerWithInputIngredients': 'mediumBowl2', 'targetContainer': 'mediumBowl3', 'kitchenStateIn': None, 'setWorldState': False}, "./WS6.log", "./WS7.log"],
        ["http://localhost:54321/abe-sim-command/to-mix", {'containerWithInputIngredients': 'mediumBowl3', 'mixingTool': 'whisk', 'kitchenStateIn': None, 'setWorldState': False}, "./WS7.log", "./WS8.log"],
        ["http://localhost:54321/abe-sim-command/to-fetch", {'object': 'bakingTray', 'kitchenStateIn': None, 'setWorldState': False}, "./WS8.log", "./WS9.log"],
        ["http://localhost:54321/abe-sim-command/to-line", {'bakingTray': 'bakingTray', 'bakingPaper': 'bakingSheet', 'kitchenStateIn': None, 'setWorldState': False}, "./WS9.log", "./WS10.log"],
        ["http://localhost:54321/abe-sim-command/to-portion-and-arrange", {'containerWithDough': 'mediumBowl3', 'destination': 'bakingTray', 'kitchenStateIn': None, 'setWorldState': False}, "./WS10.log", "./WS11.log"],
        ["http://localhost:54321/abe-sim-command/to-bake", {'thingToBake': 'bakingTray', 'inputDestinationContainer': 'kitchenCounter', 'oven': 'oven', 'kitchenStateIn': None, 'setWorldState': False}, "./WS11.log", "./WS12.log"],
        ["http://localhost:54321/abe-sim-command/to-sprinkle", {'object': 'bakingTray', 'toppingContainer': 'sugarBag', 'kitchenStateIn': None, 'setWorldState': False}, "./WS12.log", "./WS13.log"]
    ]

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

sys.exit()

### Command 4: fetch bowl to countertop
dpo = {'object': 'mediumBowl1', 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-fetch", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 5: proportion 134g of sugarBag in mediumBowl1
dpo = {'containerWithIngredient': 'sugarBag', 'targetContainer': 'mediumBowl1', 'quantity': 134, 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-portion", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

sys.exit()

### Command 6: proportion 134g of butterBag in mediumBowl2
dpo = {'containerWithIngredient': 'butterBag', 'targetContainer': 'mediumBowl2', 'quantity': 226, 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-portion", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 7: fetch bowl to countertop
dpo = {'object': 'mediumBowl3', 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-fetch", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 8: transfer bowl1 contents to bowl3
dpo = {'containerWithInputIngredients': 'mediumBowl1', 'targetContainer': 'mediumBowl3', 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-transfer", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 9: transfer bowl2 contents to bowl3
dpo = {'containerWithInputIngredients': 'mediumBowl2', 'targetContainer': 'mediumBowl3', 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-transfer", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

# ####
# #### Command 1: get the kitchen state
# #dws = {'kitchen': '?kitchen-state-1'}
# #r = requests.post("http://localhost:54321/abe-sim-command/to-get-kitchen", data=bytes(json.dumps(dws), "utf-8"))
# #worldState = json.loads(r.text)['response']
# ##print(worldState)
# #with open("dbg23.log", "w") as outfile:
# #    outfile.write(json.dumps(worldState["?kitchen-state-1"]))
# #
# #worldState = json.loads(open("dbg23.log").read())
# #### Command 2: set the kitchen state
# #dgr = {'kitchenStateIn': worldState}
# #r = requests.post("http://localhost:54321/abe-sim-command/to-set-kitchen", data=bytes(json.dumps(dgr), "utf-8"))
# #response = json.loads(r.text)
# #print(response)

### Command 10: mixing
dpo = {'containerWithInputIngredients': 'mediumBowl3', 'mixingTool': 'whisk', 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-mix", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 11: lining
dpo = {'bakingTray': 'bakingTray1', 'bakingPaper': 'bakingSheet1', 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-line", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 12: shaping
dpo = {'containerWithDough': 'mediumBowl3', 'destination': 'bakingTray1', 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-shape", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 0: NOP
dgr = {'frames': 1000}
r = requests.post("http://localhost:54321/abe-sim-command/to-wait", data=bytes(json.dumps(dgr), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 13: baking
dpo = {'thingToBake': 'bakingTray1', 'oven': 'kitchenStove', 'inputDestinationContainer': 'counterTop', 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-bake", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 14: sprinkling
dpo = {'object': 'bakingTray1', 'toppingContainer': 'sugarShaker', 'kitchenStateIn': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-sprinkle", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

