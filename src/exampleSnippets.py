##Command 1 start
import requests
import json

dhi = {'op': 'hi', 'args': {}}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dhi), "utf-8"))
print(r)
print(r.text)
##Command 1 end

##Command 2 start
dws = {'op': 'rws'}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dws), "utf-8"))
worldState = json.loads(r.text)['response']
worldState.keys()

dws = {'kitchen': '?kitchen-state-1'}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-kitchen", data=bytes(json.dumps(dws), "utf-8"))
worldState = json.loads(r.text)['response']
worldState.keys()

##Command 2 end

go = {'available-location': '?available-oven', 'kitchen': None, 'type': 'Oven', 'set-world-state': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-get-location", data=bytes(json.dumps(go), "utf-8"))
response = json.loads(r.text)
response

##Command 3 start
dpo = {'op': 'placeon', 'args': {'object': 'bakingTray1', 'destination': 'counterTop1'}}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dpo), "utf-8"))


dpo = {'object': 'bakingTray1', 'kitchen-input-state': None, 'set-world-state': False}
r = requests.post("http://localhost:54321/abe-sim-command/to-fetch", data=bytes(json.dumps(dpo), "utf-8"))

##Command 3 end

##Command 4 start
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dws), "utf-8"))
worldState1 = json.loads(r.text)['response']
objs = worldState1['worldState']
butterNames = [x for x in objs.keys() if ('props' in objs[x]) and ('type' in objs[x]['props']) and ("['Butter']" == objs[x]['props']['type'])]
tempMap = {'hot': [], 'cold': []}
for n in butterNames:
    print(n, objs[n]["props"]["temperature"])
    if 10 < objs[n]["props"]["temperature"]:
        tempMap['hot'].append(n)
    else:
        tempMap['cold'].append(n)

dpo = {'op': 'placeon', 'args': {'object': tempMap['cold'][0], 'destination': 'table.002'}}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dpo), "utf-8"))
##Command 4 end

##Command 5 start
dws = {'op': 'rws'}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dws), "utf-8"))
worldState2 = json.loads(r.text)['response']
worldState2.keys()
objs = worldState2['worldState']
butterNames = [x for x in objs.keys() if ('props' in objs[x]) and ('type' in objs[x]['props']) and ("['Butter']" == objs[x]['props']['type'])]
tempMap = {'hot': [], 'cold': []}
for n in butterNames:
    print(n, objs[n]["props"]["temperature"])
    if 10 < objs[n]["props"]["temperature"]:
        tempMap['hot'].append(n)
    else:
        tempMap['cold'].append(n)

dpo = {'op': 'placeon', 'args': {'object': tempMap['hot'][0], 'destination': 'table.002'}}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dpo), "utf-8"))
##Command 5 end

##Command 6 start
default = json.loads(open('./default_scene.json').read())
start = json.loads(open('./bake_start_scene.json').read())['objProps']

dsws = {'args': default}
r = requests.post("http://localhost:54321/abe-sim-command/to-set-kitchen", data=bytes(json.dumps(dsws), "utf-8"))

##Command 6 end

