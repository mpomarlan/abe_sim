#!/usr/bin/python

import os 
from os.path import expanduser

ownPath = os.path.dirname(os.path.realpath(__file__))
home = expanduser("~")
morseDir = os.path.join(home,".morse")
morseConfig = os.path.join(morseDir,"config")

if not os.path.isfile(morseConfig):
    if not os.path.isdir(morseDir):
        os.mkdir(morseDir)
    with open(morseConfig,"w") as outfile:
        outfile.write("[sites]\nabe_sim=%s\n" % ownPath)
else:
    abeLine = "abe_sim=%s" % ownPath
    txt = open(morseConfig).read()
    lines = txt.splitlines()
    with open(morseConfig,"w") as outfile:
        for l in lines:
            if ("abe_sim" == l.strip()[0:7]):
                print("WARNING: encountered a previous definition of abe_sim: \n\t%s" % l)
                continue
            outfile.write("%s\n" % l)
            if "[sites]" == l.strip():
                outfile.write("%s\n" % abeLine)
                

