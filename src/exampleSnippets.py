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
##Command 2 end

##Command 3 start
dpo = {'op': 'placeon', 'args': {'object': 'bottleOil', 'destination': 'table.000'}}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dpo), "utf-8"))
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
dsws = {'op': 'sws', 'args': worldState}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dsws), "utf-8"))
##Command 6 end

