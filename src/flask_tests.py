import requests
import json

### Command 1: get the kitchen state
dws = {'kitchen': '?kitchen-state-1'}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-kitchen", data=bytes(json.dumps(dws), "utf-8"))
worldState = json.loads(r.text)['response']
print(worldState)

### Command 2: set the kitchen state
dgr = {'kitchenInputState': worldState["?kitchen-state-1"]}
r = requests.post("http://localhost:54321/abe-sim-command/to-set-kitchen", data=bytes(json.dumps(dgr), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 3: get location
gc = {'availableLocation': '?available-countertop', 'type': 'CounterTop', 'kitchen': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-location", data=bytes(json.dumps(gc), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 4: fetch bowl to countertop
dpo = {'object': 'mediumBowl1', 'kitchenInputState': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-fetch", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 5: proportion 134g of sugarBag in mediumBowl1
dpo = {'inputContainer': 'sugarBag', 'newContainer': 'mediumBowl1', 'amount': 134, 'kitchenInputState': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-portion", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 6: proportion 134g of butterBag in mediumBowl2
dpo = {'inputContainer': 'butterBag', 'newContainer': 'mediumBowl2', 'amount': 226, 'kitchenInputState': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-portion", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)

### Command 7: fetch bowl to countertop
dpo = {'object': 'mediumBowl3', 'kitchenInputState': None, 'setWorldState': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-fetch", data=bytes(json.dumps(dpo), "utf-8"))
response = json.loads(r.text)
print(response)


