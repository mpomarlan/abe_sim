##### Init sim, place objects
import pybullet as p
import time
from abe_sim.world import World
w = World(pybulletOptions = "--opengl2")
from abe_sim.Abe.abe import Abe
from abe_sim.Fridge.fridge import Fridge
from abe_sim.CookingKnife.cookingknife import CookingKnife
from abe_sim.Bread.bread import Bread, BreadSlice

a = w.addPObjectOfType("abe", Abe, [0,0,0], [0,0,0,1])
f = w.addPObjectOfType("fridge", Fridge, [-0.25, -2, 0.65], [0,0,0,1])
c = w.addPObjectOfType("knife", CookingKnife, [0, 0.4, 0.45], [0, 0, 0, 1])
b = w.addPObjectOfType("bread", Bread, [0.6, 0.23, 0.45], [0,0,0.707,0.707])


#### Grab knife and "cut" from the bread
a.applyRigidBodyControls([{"jointTargets": {"hand_left_y_to_hand_left_z": (-0.27,0,1)}, "+constraints": [], "-constraints": []}])
for k in range(100):
    w.update()

a._customStateVariables["hand_left_roll"]["grasping"] = ["knife"]
a.applyRigidBodyControls([{"jointTargets": {"hand_left_y_to_hand_left_z": (-0.035,0,1), "hand_left_x_to_hand_left_y": (0,0,1)}, "+constraints": [], "-constraints": []}])
for k in range(100):
    w.update()

a.applyRigidBodyControls([{"jointTargets": {"world_to_base_x": (0,0,1), "hand_left_base_to_hand_left_x": (0.75,0,1)}, "+constraints": [], "-constraints": []}])
for k in range(100):
    w.update()
    time.sleep(0.1)

a.applyRigidBodyControls([{"jointTargets": {"world_to_base_x": (0,0,1), "hand_left_base_to_hand_left_x": (0,0,1)}, "+constraints": [], "-constraints": []}])
for k in range(100):
    w.update()
    time.sleep(0.1)


#### Pull open the fridge doors
a.applyRigidBodyControls([{"jointTargets": {"hand_right_x_to_hand_right_y": (-0.5,0,1), "base_x_to_base_y": (0,0,1)}, "+constraints": [], "-constraints": []}])
for k in range(200):
    w.update()

a._customStateVariables["hand_right_roll"]["pulling"] = True
a.applyRigidBodyControls([{"jointTargets": {"hand_right_x_to_hand_right_y": (-0.0,0,1), "base_x_to_base_y": (0,0,1)}, "+constraints": [], "-constraints": []}])
for k in range(100):
    w.update()
    time.sleep(0.1)

