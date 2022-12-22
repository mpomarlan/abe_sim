import os
import abe_sim.pobject as pob

class KitchenSink(pob.PObject):
    def _customInitPreLoad(self, *args, **kwargs):
        cDir = os.path.dirname(os.path.abspath(__file__))
        self._urdf = os.path.join(cDir, "./kitchensink.urdf")
        self._useMaximalCoordinates = False
        self._useFixedBase = True
        self._customStateVariables = {"type": "kitchensink", "fn": {"dfltype": "dfl:kitchen_sink.n.wn.artifact"}}

