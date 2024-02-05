import curses
import json
import os
import time

from subprocess import Popen, PIPE, CREATE_NEW_CONSOLE
import subprocess
import multiprocessing
import queue
import threading
from websockets.sync.client import connect

# 'COLOR_BLACK', 'COLOR_BLUE', 'COLOR_CYAN', 'COLOR_GREEN', 'COLOR_MAGENTA', 'COLOR_RED', 'COLOR_WHITE', 'COLOR_YELLOW'
# k: {"type": "G", "description": {"goal": "pickedItem", "hand": hand, "item": item}, "children": [], "previousStatus": None

PAIR_ENTITY = 1
PAIR_PROPERTY = 2
PAIR_TRUE = 3
PAIR_FALSE = 4
PAIR_NULL = 5
PAIR_TREE = 6
descMap = {"G": "goal", "P": "process"}

basePath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../")
treeSrcURL = "ws://localhost:54322"

treeQueue = queue.LifoQueue()

def depthTraversal(tree):
    # Assumes root is at node index 0
    depthMap = {"0":0}
    parentMap = {"0":None}
    todo = ["0"]
    traversal = []
    lineMap = {}
    lastLineMap = {}
    while todo:
        cr = todo.pop()
        lineMap[cr] = len(lineMap)
        traversal.append(cr)
        for e in reversed(tree[cr].get("children",[])):
            e = str(e)
            todo.append(e)
            parentMap[e] = cr
            depthMap[e] = depthMap[cr] + 1
    for k in tree.keys():
        if k in lineMap:
            lastLineMap[k] = lineMap[k]
            chs = tree[k].get("children",[])
            if 0 < len(chs):
                lastLineMap[k] = lineMap[str(chs[-1])]
    return traversal, depthMap, parentMap, lastLineMap

def displayTree(cstr, tree):
    traversal, depthMap, parentMap, lastLineMap = depthTraversal(tree)
    activeNodes = []
    #cstr.addstr(str(traversal)+"\n")
    #cstr.addstr(str(depthMap)+"\n")
    #cstr.addstr(str(parentMap)+"\n")
    #cstr.addstr(str(lastLineMap)+"\n")
    retq = ""
    for l,k in enumerate(traversal):
        cdepth = depthMap[k]
        branchStr = " "*cdepth
        branchStrS = " "*cdepth
        for e in activeNodes:
            edepth = depthMap[e]
            if l == lastLineMap[e]:
                nodeCh = b"\xe2\x94\x94".decode("utf-8")
                nodeChS = "\\"
            elif edepth+1==cdepth:
                nodeCh = b"\xe2\x94\x9c".decode("utf-8")
                nodeChS = "+"
            else:
                nodeCh = b"\xe2\x94\x82".decode("utf-8")
                nodeChS = "|"
            branchStr = branchStr[:edepth] + nodeCh + branchStr[edepth+1:]
            branchStrS = branchStrS[:edepth] + nodeChS + branchStrS[edepth+1:]
        activeNodes.append(k)
        activeNodes = [x for x in activeNodes if l < lastLineMap[x]]
        node = tree[k]
        cstr.addstr(branchStr, curses.color_pair(PAIR_TREE)|curses.A_BOLD)
        retq += branchStrS
        cstr.addstr("%s:%s "%(node["type"], node["description"][descMap[node["type"]]]), curses.color_pair(PAIR_ENTITY)|curses.A_BOLD)
        retq += "%s:%s "%(node["type"], node["description"][descMap[node["type"]]])
        cstr.addstr(">> ", curses.color_pair(PAIR_TREE)|curses.A_BOLD)
        retq += ">> "
        props = sorted([(k,v) for k,v in node["description"].items() if ("goal" != k) and ("process" != k)])
        for k,p in enumerate(props):
            cstr.addstr(p[0], curses.color_pair(PAIR_PROPERTY))
            cstr.addstr(": ", curses.color_pair(PAIR_TREE))
            cstr.addstr(str(p[1]), curses.color_pair(PAIR_ENTITY))
            retq += p[0] + ": " + str(p[1])
            if k < len(props)-1:
                cstr.addstr(",", curses.color_pair(PAIR_TREE))
                retq += ","
            cstr.addstr(" ", curses.color_pair(PAIR_TREE))
            retq += " "
        cstr.addstr(": ", curses.color_pair(PAIR_TREE)|curses.A_BOLD)
        retq += ": "
        prevStatus = node.get("previousStatus")
        if prevStatus is None:
            cstr.addstr("None", curses.color_pair(PAIR_NULL)|curses.A_BOLD)
            retq += "None"
        elif prevStatus is False:
            cstr.addstr("False", curses.color_pair(PAIR_FALSE)|curses.A_BOLD)
            retq += "False"
        elif prevStatus is True:
            cstr.addstr("True", curses.color_pair(PAIR_TRUE)|curses.A_BOLD)
            retq += "True"
        cstr.addstr("\n")
        retq += "\n"
    return retq

def treeReception():
    with connect(treeSrcURL) as websocket:
        while True:
            msg = websocket.recv()
            treeQueue.put(msg)

def dbgmain():
    threadReception = threading.Thread(target=lambda : treeReception(), daemon=True)
    threadReception.start()
    while True:
        print("TREEVIZ: current process garden\n")
        msg = treeQueue.get()
        # clear queue
        while not treeQueue.empty():
            treeQueue.get()
            treeQueue.task_done()
        tree = json.loads(msg.decode("utf-8"))
        print(str(tree))
        treeQueue.task_done()

def main(cscr):
    #cscr.resize(80,140)
    cscr.timeout(0)
    cscr.clear()
    cscr.scrollok(True)
    curses.init_pair(PAIR_ENTITY, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(PAIR_PROPERTY, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(PAIR_TRUE, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(PAIR_FALSE, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(PAIR_NULL, curses.COLOR_MAGENTA, curses.COLOR_BLACK)
    curses.init_pair(PAIR_TREE, curses.COLOR_WHITE, curses.COLOR_BLACK)
    #tree = {"0": {"type": "G", "description": {"goal": "Grasped", "item": "cup"}, "previousStatus": False, "children": [1,3]}, "1": {"type": "P", "description": {"process": "Grasping", "item": "cup"}, "previousStatus": None, "children": [2,4]}, "2": {"type": "P", "description": {"process": "Approaching", "item": "cup"}, "previousStatus": None, "children": []}, "4": {"type": "P", "description": {"process": "Reaching", "item": "cup"}, "previousStatus": None, "children": [5]}, "3": {"type": "P", "description": {"process": "BlaBla", "item": "cup"}, "previousStatus": None, "children": []}, "5": {"type": "P", "description": {"process": "FooFoo", "item": "cup"}, "previousStatus": None, "children": []}}
    tree = {}
    treeStr = ""
    threadReception = threading.Thread(target=lambda : treeReception(), daemon=True)
    threadReception.start()
    cscr.addstr("TREEVIZ: current process garden\n")
    cscr.refresh()
    lastCh = " "
    while True:
        cscr.clear()
        cscr.addstr("TREEVIZ: current process garden %s\n" % lastCh)
        addPause=True
        if not treeQueue.empty():
            msg = treeQueue.get()
            # clear queue
            while not treeQueue.empty():
                treeQueue.get()
                treeQueue.task_done()
            tree = json.loads(msg.decode("utf-8"))
            treeQueue.task_done()
        else:
            addPause=True
        #cscr.addstr("  got messages\n")
        #cscr.addstr("%s\n" % str(tree.keys()))
        #cscr.addstr("  %s\n" % json.dumps(tree))
        try:
            if "0" in tree:
                treeStr = displayTree(cscr, tree)
        except Exception as e:
            treeStr = "Exception:" + str(e)
            cscr.addstr()
        ch = cscr.getch()
        if -1 != ch:
            #lastCh = str(ch)
            if 32 == ch:
                fileName = "pg_%s.txt" % time.asctime().replace(" ", "_").replace(":","_")
                with open(os.path.join(basePath,fileName),"w") as outfile:
                    _ = outfile.write("%s\n" % treeStr)
            #else:
            #    cscr.do_command(ch)
        cscr.refresh()
        if addPause:
            time.sleep(0.01)

if "__main__" == __name__:
    curses.wrapper(main)
    #dbgmain()

