import argparse
import os
import sys
import json

def json2Triples(data, eliminateRootS=False, nameProperty=None):
    obligations = []
    retq = []
    todoPOST = []
    todoPRIO = [(None, data)]
    while todoPRIO:
        dependent, cr = todoPRIO.pop()
        obligation = {"payload": cr, "dependencies": {}, "metDependencies": set(), "dependent": dependent, "path": "/root/"}
        obligations.append(obligation)
        if dependent is not None:
            obligationSup, key = dependent
            obligationSup["dependencies"][key] = obligation
            obligation["path"] = obligationSup["path"] + str(key)
            if not obligation["path"].endswith("/"):
                obligation["path"] += ("/")
        if isinstance(cr, list):
            for k, v in enumerate(cr):
                todoPRIO.append(((obligation, k), v))
        elif isinstance(cr, dict):
            for k, v in cr.items():
                todoPRIO.append(((obligation, k), v))
    for o in obligations:
        if o["dependencies"].keys() == o["metDependencies"]:
            todoPOST.append(o)
    while todoPOST:
        cr = todoPOST.pop()
        path = cr.get("path", "/root/")
        dependencies = cr.get("dependencies", {})
        dependent = cr.get("dependent")
        if dependent is not None:
            obligation, key = dependent
            obligation["metDependencies"].add(key)
            if obligation["dependencies"].keys() == obligation["metDependencies"]:
                todoPOST.append(obligation)
        payload = cr.get("payload")
        if payload is None:
            cr["value"] = None
            continue
        elif (payload in [True, False]) or (type(payload) in [int,float,str]):
            cr["value"] = payload
        elif isinstance(payload, list):
            cr["value"] = path
            for k, _ in enumerate(payload):
                if dependencies[k]["value"] is not None:
                    retq.append((path, "member_%d" % k, dependencies[k]["value"]))
        elif isinstance(payload, dict):
            cr["value"] = path
            for k in payload.keys():
                if dependencies[k]["value"] is not None:
                    retq.append((path, k, dependencies[k]["value"]))
    if eliminateRootS:
        retq = [x for x in retq if "/root/" != x[0]]
    if nameProperty is not None:
        #names2Paths = {}
        paths2Names = {}
        for e in retq:
            if nameProperty == e[1]:
                #if e[2] not in names2Paths:
                #    names2Paths[e[2]] = []
                #names2Paths[e[2]].append(e[0])
                if e[0] not in paths2Names:
                    paths2Names[e[0]] = []
                paths2Names[e[0]].append(e[2])
        aux = []
        #for k, v in paths2Names.items():
        #    if 1 < len(v):
        #        for e in v[1:]:
        #            aux.append((v[0], "http://www.w3.org/2002/07/owl#sameAs", e))
        for e in retq:
            s = e[0]
            o = e[2]
            if s in paths2Names:
                s = paths2Names[s][0]
            ### TODO: here, some extra filtering is needed on e[1]: it should be a data property so that we can be sure not to replace actual data with an assumed URI
            if o in paths2Names:
                o = paths2Names[o][0]
            aux.append((s, e[1], o))
        retq = aux
    retq = sorted(retq)
    return retq

if "__main__" == __name__:
    parser = argparse.ArgumentParser(prog='json2Triples', description='Convert json files (e.g. abesim world dumps) into lists of triples', epilog='Text after help string goes here.')
    parser.add_argument('-i', '--inputFile', default="", help='Input json file to convert.')
    parser.add_argument('-o', '--outputFile', default="", help="Output file to write to.")
    parser.add_argument('-r', '--eliminateRootS', default=False, action="store_true", help='Do not return triples where the s component is /root/ i.e. where the s is the top-level json object.')
    parser.add_argument('-n', '--nameProperty', help='This property is used to define names for paths. Optional.')
    arguments = parser.parse_args()
    inputFile = arguments.inputFile
    if "" == inputFile:
        print("Must provide an input file")
    outputFile = arguments.outputFile
    if "" == outputFile:
        outputFile = inputFile[:inputFile.find(".")] + ".rdf"
    eliminateRootS = arguments.eliminateRootS
    nameProperty = arguments.nameProperty
    if "" == nameProperty:
        nameProperty = None
    data = json.loads(open(inputFile).read())
    res = json2Triples(data, eliminateRootS=eliminateRootS, nameProperty=nameProperty)
    with open(outputFile, "w") as outfile:
        for e in res:
            outfile.write("%s\n" % json.dumps(e))

