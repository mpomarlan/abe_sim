from morse.builder import *
from abe_sim.builder.robots import Abe

from morse.core import blenderapi

import time

import math


robot = Abe()
robot.translate(1.0, 0.0, 0.0)
robot.add_default_interface('socket')

#env = Environment('abe_sim/environments/abe_sim.blend', fastmode = False)
env = Environment('abe_sim/environments/abe_simkitchen_default.blend', fastmode = False)
#env.set_camera_location([0.0, -9.0, 9.5])
env.set_camera_location([-10.0, 0.0, 10.5])
#env.set_camera_rotation([37.566*math.pi/180.0,0,0])
env.set_camera_rotation([37.566*math.pi/180.0,0,-90.0*math.pi/180.0])

