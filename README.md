# abe_sim
A little agent simulation in MORSE, to be used in a MUHAI pipeline to evaluate semspecs. (Very much a work in progress now.)

# Installation

## Dependencies

To try the current stuff out, you will first need to [install MORSE ](https://www.openrobots.org/morse/doc/stable/user/installation.html) and pip-install trimesh and python-fcl. For example, assuming you have pip3 as a command to install packages for a python3.4 or newer distribution, you can run:

```
sudo pip3 install rtree trimesh python-fcl
```

Warning: because of API changes, newer versions of Blender will not work with MORSE. This can create problems on newer versions of Linux. For instance, at the moment installing on Ubuntu 20.04 will not work via the debian, but 18.04 should.

To see whether your configuration is proper, try this:

```
morse check
```

This will report errors such as the python version assumed by Blender and Morse being different (they must not be for the system to work). You can also use it to see whether the Blender version will be a problem. In the output from the check, look for the Blender version: it must be 2.79 or less. **Blender version 2.80 and above will not work with Morse, regardless of what morse check says.**

## Setting up a workspace

Create a MORSE folder in your home directory. This is where you will store the abe_sim, and some other code. The list of commands shows you how you might do this on a Linux system, including setting up a link to the schemasim package so that you can access it when running abe_sim. The last two lines are there to register the abe_sim simulation with MORSE so you can run it.

```
mkdir MORSE
cd ./MORSE
git clone https://github.com/mpomarlan/abe_sim
git clone https://github.com/mpomarlan/schemasim
ln -s ../../schemasim/schemasim ./abe_sim/src/schemasim
cd ./abe_sim
register.py
```

# Running the simulation

Once the dependencies and the repositories above are installed/cloned, go to the folder in which you have placed the repositories (in the commands above, that would be the MORSE folder). Once there, run

```
morse run abe_sim
```

and you should see the simulated environment appear in a blender game window. You can move the blue agent with the arrow keys, and the camera with wasd and ctrl+mouse move.

However we'd like to have the agent figure some stuff out without us having to guide it always. To do that, open a terminal and go to the repository folder, and then in its src folder (if you followed the commands above, go to MORSE/abe_sim/src). Once there, start a python3 instance and run the following code:

```python
from abe_sim.brain.midbrain import Midbrain
from pymorse import Morse
simu = None
while True:
    try:
        simu = Morse()
        break
    except OSError:
        continue

base = simu.robot.base
hands = simu.robot.hands
head = simu.robot.head
pose = simu.robot.pose
world = simu.robot.worlddump
mb = Midbrain(head, hands, base, pose, world, simu)
mb.updateNavigationMap()
mb.startOperations()

objSchemas = mb.getObjectSchemas()

from schemasim.schemas.l11_functional_control import Support

def placeTrajectorOnSupport(trajector, supporter):
    trajSchema = objSchemas[trajector].unplace(mb.sim3D)
    destspec = [Support(supporter=objSchemas[supporter],supportee=trajSchema), trajSchema]
    mb.carryObject(trajector, destspec)

placeTrajectorOnSupport('whisk3', 'counterTop1')
```

What you should see is that, after some time to start up its "brain", the agent is going to move towards one of the cupboards, pick up a whisk, and deliver it to the large counter to one side of the kitchen. We can also place some other object, for example mediumBowl3, on the counterTop1 (but ONLY AFTER the agent is finished with the whisk!):

```
placeTrajectorOnSupport('mediumBowl3', 'counterTop1')
```

We could also have a look at some other auxiliary structures the agent maintains about the world. One such structure is the navigation map, which the agent uses to navigate around obstacles. We can have a look at how this map looks like:

```python
print(mb.cellMap)
```

You will see a grid of either "." (meaning, this square cell of the grid is free) or "#" (the cell is occupied and should not be walked through).

NOTE: the navigation grid is somewhat course, and encourages the agent to stay away from furniture objects. If you move the agent around using the keys, it is possible that you place it inside a cell it considers occupied on its navigation grid, and won't be able to get out using its navigation function. If you suspect this is the case, manually move the agent to a free area such as the center and resume giving it navigation goals afterwards.

Speaking of objects, what objects are there in the environment? We can obtain a printout of objects and some data about them, such as types and positions, using

```python
mb.listObjects()
```

This will give you a lot of text in the console, but supposing you want that information in a variable that you can use later in your program, you should run instead

```
objs = mb.cerebellum._retrieveObjects()
```

objs is a dictionary, where they keys are object names and the values are dictionaries containing information about these objects, such as their position, type, mesh filename and so on.

# Communicating with the simulation using HTTP requests

This section supposes that you have the abe_sim running, and a python instance for the agent's brain (the code above, up to and including the call to startOperations).

HTTP post requests can be sent from many programs and using many programming languages; in this section, we will give an example in python of how to communicate with the simulation.

So, start another python instance. Run this code:

```
import requests
import json

dhi = {'op': 'hi', 'args': {}}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dhi), "utf-8"))
print(r)
print(r.text)
```

Assuming everything went right, you should see the following as a printout of the text produced by the request:

```
{'status': 'ok', 'response': 'hi!'}
```

Let's look at the structure of the request. First, it is a POST request, sending a json dictionary with two keys: 'op' (the command) and 'args' (the arguments). What arguments are needed depend on the command; the 'hi' command needs no arguments, and the 'args' key can even be ommitted entirely.

'hi' is a very simple command: you can use it to check the server is running. Note the address and port, and the path to send the POST request to.

There are other commands you can try however. For example, you can get a record of the current world state.

```
dws = {'op': 'rws'}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dws), "utf-8"))
worldState = json.loads(r.text)['response']
```

'rws' (or 'retrieveworldstate') is a command that takes no arguments but returns a lot of data. You can try printing the worldState -- that's a lot of lines!

<!-- Another interesting command is to tell the robot to put an object on another. We'll try to do this with one of the "butter" clumps in the scene. These are made of particles which will try to stick to each other, stronger if they are cold, less so if they are hot. You can see what happens when you try to manipulate the clumps with the robot. But first we need to figure out what names these clumps have (because of how they are made up, i.e., because which particles are in which clump may change, these names will change):

```
objs = worldState['world-state']
butterNames = [x for x in objs.keys() if ('props' in objs[x]) and ('type' in objs[x]['props']) and ("['Butter']" == objs[x]['props']['type'])]
for n in butterNames:
    print(n, objs[n]["props"]["temperature"])
```

The code above is selecting which objects in the scene are of type butter, and then prints the temperatures of each of these objects. Let's say that 'ButterParticle.007' is the name of one of the butter clumps, then we could use

```
dpo = {'op': 'placeon', 'args': {'object': 'ButterParticle.007', 'destination': 'table.002'}}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dpo), "utf-8"))
```

to have the robot place the butter on one of the tables. Try this on both clumps -- they will behave very differently! -->

Finally, we can try to reset the simulated world to some state it had in the past. We have such a stored state -- the one we queried way at the beginning of this section -- so let's try this:

```
dsws = {'op': 'sws', 'args': worldState}
r = requests.post("http://localhost:54321/abe-sim-command", data=bytes(json.dumps(dsws), "utf-8"))
```

If you now look into the simulation window, you should see all objects and the robot returned to their original locations.
