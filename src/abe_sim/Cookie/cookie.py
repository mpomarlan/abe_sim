import os
import abe_sim.pobject as pob
import abe_sim.dynamics.grasping as gr
import abe_sim.dynamics.baking as bk
#import abe_sim.dynamics.mixing as mx

class DoughClump(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, self._urdfName())
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        cDir = os.path.dirname(os.path.abspath(__file__))
        sugarToppedURDF = os.path.join(cDir, "./sugarsprinkledcookie.urdf")
        self._customStateVariables = {"doughclump": {"graspable": True}, "": {"type": self._ontoTypeName()}, "fn": {"toppedurdfs": {"sugar": sugarToppedURDF}, "ingredient": True, "graspingradius": 0.3, "bakeable": True, "bakedurdf": os.path.join(cDir, "./cookie.urdf"), "bakinghp": 4000}}
    def _ontoTypeName(self):
        return "doughclump"
    def _urdfName(self):
        return "./doughclump.urdf"
    def _customInitDynamicModels(self):
        self._dynamics = {"graspable": gr.Grasping(self._world, self, "doughclump", graspingRadius=self._customStateVariables["fn"]["graspingradius"]), "bakeable": bk.Baking(self._world, self)}

class Cookie(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, self._urdfName())
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        cDir = os.path.dirname(os.path.abspath(__file__))
        sugarToppedURDF = os.path.join(cDir, "./sugarsprinkledcookie.urdf")
        self._customStateVariables = {"doughclump": {"graspable": True}, "": {"type": self._ontoTypeName()}, "fn": {"toppedurdfs": {"sugar": sugarToppedURDF}, "ingredient": True, "graspingradius": 0.3}}
    def _ontoTypeName(self):
        return "cookie"
    def _urdfName(self):
        return "./cookie.urdf"
    def _customInitDynamicModels(self):
        self._dynamics = {"graspable": gr.Grasping(self._world, self, "doughclump", graspingRadius=self._customStateVariables["fn"]["graspingradius"])}

class SugarSprinkledCookie(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, self._urdfName())
        self._useMaximalCoordinates = False
        self._useFixedBase = False
        self._customStateVariables = {"doughclump": {"graspable": True}, "": {"type": self._ontoTypeName()}, "fn": {"ingredient": True, "graspingradius": 0.3}}
    def _ontoTypeName(self):
        return "cookie"
    def _urdfName(self):
        return "./sugarsprinkledcookie.urdf"
    def _customInitDynamicModels(self):
        self._dynamics = {"graspable": gr.Grasping(self._world, self, "doughclump", graspingRadius=self._customStateVariables["fn"]["graspingradius"])}

