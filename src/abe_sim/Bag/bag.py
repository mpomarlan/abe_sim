import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr
import abe_sim.dynamics.pourportioning as pp
import abe_sim.Particle.particle as prt

class Bag(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, self._urdfName())
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"bag": {"graspable": True, "pourPortioningDelay": 0}, "": {"type": self._ontoTypeName()}, "fn": {"dripaxis": [0,1,0], "refpt": [0, 0.089, 0.115], "graspingradius": 0.27, "portiontype": self._portionTypeName()}}
    def _portionTypeName(self):
        return "particle"
    def _ontoTypeName(self):
        return "bag"
    def _urdfName(self):
        return "./bag.urdf"
    def _getParticleType(self):
        return prt.Particle
    def _getPortionArgs(self):
        return []
    def _getPortionKWArgs(self):
        return {}
    def _customInitDynamicModels(self):
        graspable = gr.Grasping(self._world, self, "bag", graspingRadius=self._customStateVariables["fn"]["graspingradius"])
        pourPortioning = pp.PourPortioning(self._world, self, "bag", dripAxis=self._customStateVariables["fn"]["dripaxis"], dripPoint=self._customStateVariables["fn"]["refpt"], particleType=self._getParticleType(), maxDelay=80, portionArgs=self._getPortionArgs(), portionKWArgs=self._getPortionKWArgs())
        self._dynamics = {"graspable": graspable, "pourPortioning": pourPortioning}


class SugarBag(Bag):
    def _portionTypeName(self):
        return "sugarparticle"
    def _ontoTypeName(self):
        return "sugarBag"
    def _urdfName(self):
        return "./sugarbag.urdf"
    def _getParticleType(self):
        return prt.SugarParticle

class ButterBag(Bag):
    def _portionTypeName(self):
        return "butterparticle"
    def _ontoTypeName(self):
        return "butterBag"
    def _urdfName(self):
        return "./butterbag.urdf"
    def _getParticleType(self):
        return prt.ButterParticle

