# Acknowledgement:
This work was funded by the European MUHAI project (Horizon 2020 research and innovation program) under grant agreement number 951846. We thank Katrin Beuls, Paul van Eecke, Jens Nevens, Luc Steels, Robert Porzel for the insightful discussions that contributed to this work.
# Application domain:
Physics Simulation, Natural language processing.
# Citation:
@inproceedings{nevens-etal-2024-benchmark,
    title = "A Benchmark for Recipe Understanding in Artificial Agents",
    author = "Nevens, Jens  and
      de Haes, Robin  and
      Ringe, Rachel  and
      Pomarlan, Mihai  and
      Porzel, Robert  and
      Beuls, Katrien  and
      van Eecke, Paul",
    editor = "Calzolari, Nicoletta  and
      Kan, Min-Yen  and
      Hoste, Veronique  and
      Lenci, Alessandro  and
      Sakti, Sakriani  and
      Xue, Nianwen",
    booktitle = "Proceedings of the 2024 Joint International Conference on Computational Linguistics, Language Resources and Evaluation (LREC-COLING 2024)",
    month = may,
    year = "2024",
    address = "Torino, Italia",
    publisher = "ELRA and ICCL",
    url = "https://aclanthology.org/2024.lrec-main.3",
    pages = "22--42",
}
# Code of Conduct:
Link to the code of conduct of the project
# Code repository:
[https://github.com/muhai-project/abe_sim](https://github.com/muhai-project/abe_sim)
# Contact:
Mihai Pomarlan: mpomarlan@yahoo.co.uk
# Contribution guidelines:
Text indicating how to contribute to this code repository
# Contributors:
Contributors to a software component (email addresses)
# Creation date:
17-01-2021
# Description:
A little agent simulation in PyBullet, to be used in a MUHAI pipeline to evaluate semspecs. (Very much a work in progress now.)
# DockerFile:
Build file(s) to create a Docker image for the target software
# Documentation:
[https://github.com/mpomarlan/abe_sim/wiki](https://github.com/mpomarlan/abe_sim/wiki)
# Download URL:
[https://github.com/muhai-project/abe_sim](https://github.com/muhai-project/abe_sim)
# DOI:
Digital Object Identifier associated with the software (if any). DOIs associated with publications will also be detected.
# Executable examples:
Jupyter notebooks ready for execution (e.g., files, or through myBinder/colab links)
# FAQ:
Frequently asked questions about a software component
# Forks count:
4
# Forks url:
[https://github.com/mpomarlan/abe_sim](https://github.com/mpomarlan/abe_sim)
[https://github.com/RRachelRR/abe_sim](https://github.com/RRachelRR/abe_sim)
(some forks private or stale)
# Full name:
Abe_Sim (muhai-project/abe_sim)
# Full title:
If the repository is a short name, we will attempt to extract the longer version of the repository name
# Images:
Images used to illustrate the software component
# Installation instructions:
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
- requests
- sparqlwrapper

# Invocation:
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

# Issue tracker:
[https://github.com/mpomarlan/abe_sim/issues](https://github.com/mpomarlan/abe_sim/issues)
# Keywords:
simulation environment, simulated robot
# License:
BSD 2-clause
# Logo:
Main logo used to represent the target software component
# Name:
Abe_Sim
# Ontologies:
URL and path to the ontology files present in the repository, e.g., https://w3id.org/mira/ontology/
# Owner:
https://github.com/muhai-project
# Owner type:
Organization
# Package distribution:
Links to package sites like pypi in case the repository has a package available.
# Programming languages:
python 3
# Related papers:
URL to possible related papers within the repository stated within the readme file (from Arxiv)
# Releases (GitHub only):
Pointer to the available versions of a software component. For each release, somef will track its description, author, name, date of publication, date of creation, the link to the html page of the release, the id of the release and a link to the tarball zip and code of the release
# Repository Status:
Repository status as it is described in repostatus.org.
# Requirements:
- flask
- numpy
- pybullet
- pyyaml
- requests
- sparqlwrapper
# Support:
Guidelines and links of where to obtain support for a software component
# Stargazers count:
Total number of stargazers of the project
# Scripts: Snippets of code contained in the repository
# Support channels:
Help channels one can use to get support about the target software component
# Usage examples:
Assumptions and considerations recorded by the authors when executing a software component, or examples on how to use it
# Workflows:
URL and path to the workflow files present in the repository
