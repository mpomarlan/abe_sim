# abe_sim
A little agent simulation in MORSE, to be used in a MUHAI pipeline to evaluate semspecs. (Very much a work in progress now.)

To try the current stuff out, you will first need to [install MORSE ](https://www.openrobots.org/morse/doc/stable/user/installation.html) and pip-install trimesh.

Once these dependencies are installed, and this repository cloned, go to the folder in which you have cloned it (not the repository folder itself, but its parent folder). Once there, run

```
morse run abe_sim
```

and you should see the simulated environment appear in a blender game window. You can move the blue agent with the arrow keys, and the camera with wasd and ctrl+mouse move.

However we'd like to have the agent figure some stuff out without us having to guide it always. To do that, open a terminal and go to the repository folder, and then in its src folder. Once there, start a python3 instance and run the following code:

```python
from abe_sim.brain.midbrain import Midbrain
from pymorse import Morse
simu = Morse()
base = simu.robot.base
hands = simu.robot.hands
head = simu.robot.head
pose = simu.robot.pose
world = simu.robot.worlddump
mb = Midbrain(head, hands, base, pose, world, simu)
mb.updateNavigationMap()
```

So far, not much seems to have happened but we created the agent's "brain" and had it create a navigation map; this is what it will use to navigate around obstacles. We can have a look at how this map looks like:

```python
for line in reversed(mb.cellMap.passables):
    s = ""
    for c in line:
        if c:
            s = s + "."
        else:
            s = s + "#"
    print(s)
```

You will see a grid of either "." (meaning, this square cell of the grid is free) or "#" (the cell is occupied and should not be walked through).

So far though, the agent's brain is sleeping, so to speak. Lets wake it up, and while we're at it, let's retrieve what position it thinks it is at.

```python
mb.startOperations()
mb.cerebellum.currentPosition("base")
```

If you move the agent a little with the keyboard, you should see a different position if you call currentPosition("base") again.

Now that the agent's brain is awake though, we shouldn't often need to move it with the keyboard anymore; instead, we give it a target position to get to and let it decide how to move around.

```python
mb.navigateToPosition(-3.5, -4.5, 0)
```

You should see a message immediately after you call this function, telling you whether the goal you set is reachable and if so, that the agent is now on its way. A while later, another message should appear, telling you that the agent arrived at its destination or close enough.

Maybe we want to bring it back to the start position again now:

```python
mb.navigateToPosition(0, 0, 0)
```

And we can keep giving other goals and see how it moves. If you're quick enough, you can give the agent a goal while it is heading for another; it will drop its old target and go immediately to the new one.

NOTE: the navigation grid is somewhat course, and encourages the agent to stay away from furniture objects. If you move the agent around using the keys, it is possible that you place it inside a cell it considers occupied on its navigation grid, and won't be able to get out using its navigation function. If you suspect this is the case, manually move the agent to a free area such as the center and resume giving it navigation goals afterwards.

Speaking of objects, what objects are there in the environment? We can obtain a list of objects and some data about them, such as types and positions, using

```python
mb.listObjects()
```

