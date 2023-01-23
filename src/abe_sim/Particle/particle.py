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
        self._customStateVariables = {"particle": {"graspable": True, "substance": self._substanceName(), "mixing": 500}, "type": self._ontoTypeName(), "fn": {"maxmixing": 500, "ingredient": True, "graspingradius": 0.2, "mixingradius": 0.1, "substancelink": "particle"}}
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
        cDir = os.path.dirname(os.path.abspath(__file__))
        return {"mixing": mx.Mixing(self._world, self, "particle", mixingRadius=self._customStateVariables["fn"]["mixingradius"], inputSubstances=set(["sugar","butter"]), outputSubstance="sweetbutter", nextURDF=os.path.join(cDir, "./sweetbutterparticle.urdf"), mixDecrement=1, mixSpeed=0.1, maxMixing=self._customStateVariables["fn"]["maxmixing"], mixKey="mixing", nextMixDynamics=None, nearTypeFilter=Particle)}

class ButterParticle(Particle):
    def _urdfName(self):
        return "./butterparticle.urdf"
    def _ontoTypeName(self):
        return "butterparticle"
    def _substanceName(self):
        return "butter"
    def _particleDynamics(self):
        cDir = os.path.dirname(os.path.abspath(__file__))
        return {"mixing": mx.Mixing(self._world, self, "particle", mixingRadius=self._customStateVariables["fn"]["mixingradius"], inputSubstances=set(["sugar","butter"]), outputSubstance="sweetbutter", nextURDF=os.path.join(cDir, "./sweetbutterparticle.urdf"), mixDecrement=1, mixSpeed=0.1, maxMixing=self._customStateVariables["fn"]["maxmixing"], mixKey="mixing", nextMixDynamics=None, nearTypeFilter=Particle)}

class FlourParticle(Particle):
    def _urdfName(self):
        return "./flourparticle.urdf"
    def _ontoTypeName(self):
        return "flourparticle"
    def _substanceName(self):
        return "flour"
    def _particleDynamics(self):
        cDir = os.path.dirname(os.path.abspath(__file__))
        return {"mixing": mx.Mixing(self._world, self, "particle", mixingRadius=self._customStateVariables["fn"]["mixingradius"], inputSubstances=set(["flour","butter"]), outputSubstance="dough", nextURDF=os.path.join(cDir, "./doughparticle.urdf"), mixDecrement=1, mixSpeed=0.1, maxMixing=self._customStateVariables["fn"]["maxmixing"], mixKey="mixing", nextMixDynamics=None, nearTypeFilter=Particle)}
    
class AlmondFlourParticle(Particle):
    def _urdfName(self):
        return "./almondflourparticle.urdf"
    def _ontoTypeName(self):
        return "almondflourparticle"
    def _substanceName(self):
        return "almondflour"
    def _particleDynamics(self):
        cDir = os.path.dirname(os.path.abspath(__file__))
        return {"mixing": mx.Mixing(self._world, self, "particle", mixingRadius=self._customStateVariables["fn"]["mixingradius"], inputSubstances=set(["flour","butter"]), outputSubstance="dough", nextURDF=os.path.join(cDir, "./doughparticle.urdf"), mixDecrement=1, mixSpeed=0.1, maxMixing=self._customStateVariables["fn"]["maxmixing"], mixKey="mixing", nextMixDynamics=None, nearTypeFilter=Particle)}
       
class VanillaExtractParticle(Particle):
    def _urdfName(self):
        return "./vanillaextract_particle.urdf"
    def _ontoTypeName(self):
        return "vanillaextractparticle"
    def _substanceName(self):
        return "vanillaextract"
    def _particleDynamics(self):
        cDir = os.path.dirname(os.path.abspath(__file__))
        return {"mixing": mx.Mixing(self._world, self, "particle", mixingRadius=self._customStateVariables["fn"]["mixingradius"], inputSubstances=set(["flour","butter"]), outputSubstance="dough", nextURDF=os.path.join(cDir, "./doughparticle.urdf"), mixDecrement=1, mixSpeed=0.1, maxMixing=self._customStateVariables["fn"]["maxmixing"], mixKey="mixing", nextMixDynamics=None, nearTypeFilter=Particle)}

class AlmondExtractParticle(Particle):
    def _urdfName(self):
        return "./almondextract_particle.urdf"
    def _ontoTypeName(self):
        return "almondextractparticle"
    def _substanceName(self):
        return "almondextract"
    def _particleDynamics(self):
        cDir = os.path.dirname(os.path.abspath(__file__))
        return {"mixing": mx.Mixing(self._world, self, "particle", mixingRadius=self._customStateVariables["fn"]["mixingradius"], inputSubstances=set(["flour","butter"]), outputSubstance="dough", nextURDF=os.path.join(cDir, "./doughparticle.urdf"), mixDecrement=1, mixSpeed=0.1, maxMixing=self._customStateVariables["fn"]["maxmixing"], mixKey="mixing", nextMixDynamics=None, nearTypeFilter=Particle)}

class SweetButterParticle(Particle):
    def _urdfName(self):
        return "./sweetbutterparticle.urdf"
    def _ontoTypeName(self):
        return "sweetbutterparticle"
    def _substanceName(self):
        return "sweetbutter"
    def _particleDynamics(self):
        return {}

