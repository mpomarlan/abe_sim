from abe_sim.world import World
w = World(pybulletOptions = "--opengl2")

description = {'name': 'box', 'simtype': 'ktree', 'fn': {'filename': './box.urdf'}}
w.addObject(description)

spunBox = {'boxSpun': {'angularVelocity': (-3.614007241618348e-18, 1.9590460348413238e-19, 0.9674725625427731), 'name': 'boxSpun', 'fn': {'filename': './box.urdf', 'links': {'box': {'_previousNonzeroMass': 1.0}}}, 'jointAppliedTorques': {}, 'type': 'owl:Thing', 'jointPositions': {}, 'customStateVariables': {}, 'linearVelocity': (0.0, 0.0, 0.0), 'orientation': (0.609112148332597, -0.06052100668431846, 0.7345065021891456, -0.29295016085578723), 'simtype': 'ktree', 'jointReactionForces': {}, 'jointVelocities': {}, 'position': (1.0, 0.0, 0.0), 'at': None}}['boxSpun']
w.addObject(spunBox)


rigidConstraint = {'name': 'constraint', 'simtype': 'kcon', 'fn': {'parent': 'box', 'parentLink': 'box', 'child': 'boxSpun', 'childLink': 'box', 'jointType': 'fixed'}}
w.addObject(rigidConstraint)

