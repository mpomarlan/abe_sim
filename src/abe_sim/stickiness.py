import math
import pybullet

from abe_sim.world import getDictionaryEntry, stubbornTry

def updateStickiness(name, customDynamicsAPI):
    w = customDynamicsAPI["leetHAXXOR"]()
    at = w.at((name,))
    if at is None:
        return None
    mass = w.getObjectProperty((name,), "mass")
    stickiness = w._kinematicTrees[name].get("fn", {}).get("stickiness", {}).get("stickiness") or 0.93
    position, _, velocity, _ = w.getKinematicData((name,))
    force = [-mass*w._sfr*stickiness*velocity[0], -mass*w._sfr*stickiness*velocity[1], 0]
    w.applyExternalForce((name,), force, position, inWorldFrame=True)

