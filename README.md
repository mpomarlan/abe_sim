# abe_sim
A little agent simulation in PyBullet, to be used in a MUHAI pipeline to evaluate semspecs. (Very much a work in progress now.)

# Installation

## Dependencies

Python version 3.4 or newer is required.

To try the current stuff out, you will first need to [install PyBullet ](https://www.openrobots.org/morse/doc/stable/user/installation.html). For example, assuming you have pip3 as a command to install packages for a python3.4 or newer distribution, you can run:

```
sudo pip3 install pybullet
```

Installing pybullet will involve compiling the bullet library and, depending on your OS, have various dependencies. If you already do C/C++ development it is likely you have those installed already. If error messages appear during the pip install, it may be worth searching online for guides about installing pybullet. A few such guides are listed below:

- Windows: [Deepak Jogi: How to install PyBullet in Windows](https://deepakjogi.medium.com/how-to-install-pybullet-physics-simulation-in-windows-e1f16baa26f6)
- Linux: [GeeksForGeeks: How to install PyBullet in Linux](https://www.geeksforgeeks.org/how-to-install-pybullet-package-in-python-on-linux/)
- MacOS: todo

A note on running PyBullet on MacOS: we have gotten it to run fairly reliably. However, some python support is rudimentary on newer M1 chips. Proceed with lowered expectations.

Other abe_sim dependencies are python packages that are often already installed or, in any case, of lower installation complexity:

- flask
- numpy
- pyyaml

# Running the simulation

Once you have installed pybullet and any other dependecies, and cloned the repository, you can go to the src subfolder and run

```
python runBrain.py -h
```

will print out a help message about command line parameters. To run the default scene with a GUI at the highest quality setting recommended for your OS, run

```
python runBrain.py -g
```

If instead you wish to run the scene without any graphics rendering, use

```
python runBrain.py
```

Assuming you use a GUI, you should see the simulated environment appear in a PyBullet window; pressing g while this window is selected toggles the side panels.

At this point the simulated agent is ready to receive commands via HTTP POST requests. Have a look at the wiki for which commands are available and what arguments they take; the flaskTests.py also contains examples on how to call most of the available commands.
