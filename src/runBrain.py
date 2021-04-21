from abe_sim.brain.midbrain import Midbrain
from pymorse import Morse
simu = None
while True:
    try:
        simu = Morse()
        break
    except OSError:
        continue

base = simu.robot.base
hands = simu.robot.hands
head = simu.robot.head
pose = simu.robot.pose
world = simu.robot.worlddump
mb = Midbrain(head, hands, base, pose, world, simu)
mb.updateNavigationMap()
mb.startOperations()
