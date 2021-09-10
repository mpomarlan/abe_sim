# abe_sim
A little agent simulation in pybullet, to be used in a MUHAI pipeline to evaluate semspecs. (Very much a work in progress now.)


# Installation

## Dependencies
You will need python3.4 or newer. 

Aside from standard libraries, pybullet and flask are the only other direct dependencies. It should be possible to simply pip install them:
 
```
sudo pip3 install pybullet flask
```

## Usage of OpenGL
Support for hardware accelerated rendering is inconsistent on some Linux distributions, so if you encounter problems you should opt for pybullet's software-only "tiny" renderer. To do this, make sure the world creation line in the runBrain.py file reads:

```
w = World(pybulletOptions = "--opengl2")
```

This should be safe on most systems; on the other hand there are reports the software renderer does not work on Mac. If this is the case for your system, try replacing the world creation line in the runBrain.py file with:

```
w = World(pybulletOptions = "")
```

# Running the simulation

Once the dependencies are installed, go to the folder in which you have cloned this repository and open a terminal. In the terminal, run:

```
cd ./src
python3 ./runBrain.py
```

This will start a pybullet simulation, and you should also see a window visualizing the simulation.

In the same terminal window, open up a new tab. It should already be in the src folder of this repository clone, but if it is not cd into the src folder and then you can run:

```
./flask_tests.py
```

to test the communication with the simulation via HTTP.
