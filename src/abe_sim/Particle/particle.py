import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr
import abe_sim.dynamics.mixing as mx

class Particle(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, self._urdfName())
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"particle": {"graspable": True, "substance": self._substanceName()}, "": {"type": self._ontoTypeName()}, "fn": {"ingredient": True, "graspingradius": 0.2, "substancelink": "particle"}}
    def _ontoTypeName(self):
        return "particle"
    def _substanceName(self):
        return "particle"
    def _urdfName(self):
        return "./particle.urdf"
    def _particleDynamics(self):
        return {}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "particle", graspingRadius=self._customStateVariables["fn"]["graspingradius"])
        aux = self._particleDynamics()
        aux["graspable"] = graspable
        self._dynamics = aux

class SugarParticle(Particle):
    def _urdfName(self):
        return "./sugarparticle.urdf"
    def _ontoTypeName(self):
        return "sugarparticle"
    def _substanceName(self):
        return "sugar"
    def _particleDynamics(self):
        return {"mixing": mx.Mixing(self._world, self, "particle", mixingRadius=0.08, inputSubstances=set(["sugar","butter"]), outputSubstance="sweetbutter", nextURDF="./sweetbutterparticle.urdf", mixDecrement=1, mixSpeed=0.1, maxMixing=500, mixKey="mixing", nextMixDynamics=None, nearTypeFilter=Particle)}

class ButterParticle(Particle):
    def _urdfName(self):
        return "./butterparticle.urdf"
    def _ontoTypeName(self):
        return "butterparticle"
    def _substanceName(self):
        return "butter"
    def _particleDynamics(self):
        return {"mixing": mx.Mixing(self._world, self, "particle", mixingRadius=0.08, inputSubstances=set(["sugar","butter"]), outputSubstance="sweetbutter", nextURDF="./sweetbutterparticle.urdf", mixDecrement=1, mixSpeed=0.1, maxMixing=500, mixKey="mixing", nextMixDynamics=None, nearTypeFilter=Particle)}

class SweetButterParticle(Particle):
    def _urdfName(self):
        return "./sweetbutterparticle.urdf"
    def _ontoTypeName(self):
        return "sweetbutterparticle"
    def _substanceName(self):
        return "sweetbutter"
    def _particleDynamics(self):
        return {}

