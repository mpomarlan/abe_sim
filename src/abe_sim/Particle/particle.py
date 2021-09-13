import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr

class Particle(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./particle.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"particle": {"graspable": True, "substance": self._substanceName()}, "": {"type": self._ontoTypeName()}, "fn": {"graspingradius": 0.2, "substancelink": "particle"}}
    def _ontoTypeName(self):
        return "particle"
    def _substanceName(self):
        return "particle"
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "particle", graspingRadius=self._customStateVariables["fn"]["graspingradius"])
        self._dynamics = {"graspable": graspable}

class SugarParticle(Particle):
    def _ontoTypeName(self):
        return "sugarparticle"
    def _substanceName(self):
        return "sugar"

