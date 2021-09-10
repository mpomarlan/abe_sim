import requests
import json

### Command 1: get the kitchen state
dws = {'kitchen': '?kitchen-state-1'}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-kitchen", data=bytes(json.dumps(dws), "utf-8"))
worldState = json.loads(r.text)['response']
print(worldState)

### Command 2: set the kitchen state
dgr = {'kitchen-input-state': worldState["?kitchen-state-1"]}
r = requests.post("http://localhost:54321/abe-sim-command/to-set-kitchen", data=bytes(json.dumps(dgr), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 3: get location
gc = {'available-location': '?available-countertop', 'type': 'CounterTop', 'kitchen': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-location", data=bytes(json.dumps(gc), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 4: fetch bowl to countertop
dpo = {'object': 'mediumBowl1', 'kitchenInputState': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-fetch", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

