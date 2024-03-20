import json
import re

import requests
import time
from world import World
from constants import *


def extract_abe_world_state_objects():
    world_state_objects = []
    try:
        with open(WORLD_STATE_FILE_PATH, 'r') as world_state_file:
            data = json.load(world_state_file)
            for obj_name, obj_data in data.items():
                if obj_data.get('simtype') == 'ktree':
                    fn_values = obj_data.get('fn', {})
                    characteristics = [key.lower() for key, value in fn_values.items() if
                                       isinstance(value, bool) and value is True]
                    obj_dict = {
                        'name': obj_name,
                        'type': obj_data.get('type'),
                        'characteristics': set(characteristics),
                        'at': obj_data.get('at'),
                        'immobile': obj_data.get('immobile')
                    }

                    grasping = obj_data.get('customStateVariables', {}).get('grasping', {}).get('actuallyGrasping', {})
                    grasping_left = grasping.get("hand_left", [])
                    grasping_right = grasping.get("hand_right", [])
                    if grasping_left is not None:
                        obj_dict['holding_left'] = grasping_left
                    if grasping_right is not None:
                        obj_dict['holding_right'] = grasping_right
                    # TODO: here also add keys for properties such as closed or open and on or off when they are
                    #  added in abe_sim

                    world_state_objects.append(obj_dict)

        return world_state_objects
    except FileNotFoundError:
        print("File not found.")


def compute_pddl_init_state(abe_world_state_objects):
    pddl_init_predicates = []
    pddl_objects = []
    for abe_object in abe_world_state_objects:
        name = abe_object['name']
        if name.lower() in ABE_ROBOT_NAMES and ('holding_left' in abe_object or 'holding_right' in abe_object):
            holding_left = abe_object.get('holding_left', [])
            holding_right = abe_object.get('holding_right', [])
            for item in holding_left:
                pddl_init_predicates.append(f"(holding-left {item})")
            for item in holding_right:
                pddl_init_predicates.append(f"(holding-right {item})")
        inferred_type = infer_pddl_type(abe_object)
        if inferred_type is not None:
            pddl_objects.append(f"{name} - {inferred_type}")

            location = abe_object['at']
            if location is not None and location not in ABE_ROBOT_NAMES:
                pddl_init_predicates.append(f"(at {name} {location})")

            # TODO: if type device, clopenablestorage, fridge  see if it is closed or open
            #  if type device, see if on or off.
            #  Now assume that everything is off and closed
            if inferred_type in [PDDL_FRIDGE_TYPE, PDDL_DEVICE_TYPE, PDDL_CLOPENABLE_STORAGE_TYPE]:
                pddl_init_predicates.append(f"(closed {name})")
            if inferred_type is PDDL_DEVICE_TYPE:
                pddl_init_predicates.append(f"(off {name})")

            if abe_object.get('immobile') is True:
                pddl_init_predicates.append(f"(immobile {name})")

    pddl_objects_string = "\n".join(pddl_objects)
    init_predicates_string = "\n".join(pddl_init_predicates)
    return f"""
    (:objects
      {pddl_objects_string}
    )
    (:init
      {init_predicates_string}
    )
      """


def compute_pddl_problem(world):
    abe_world_state_objects = world.worldDump()
    init_state = compute_pddl_init_state(abe_world_state_objects)
    return f"(define {PDDL_PROBLEM_HEADER} {init_state} {PDDL_GOAL})"


def infer_pddl_type(abe_world_object):
    abe_type = abe_world_object['name'].lower()
    characteristics = abe_world_object['characteristics']

    if ABE_GRASPABLE_CHARACTERISTIC in characteristics:
        if characteristics & ABE_PERISHABLE_CHARACTERISTICS or "fresh" in abe_type:
            return PDDL_PERISHABLE_TYPE
        if characteristics & ABE_NONPERISHABLE_CHARACTERISTICS:
            return PDDL_NONPERISHABLE_TYPE
        if characteristics & ABE_UTENSIL_CHARACTERISTICS or "fork" in abe_type or "spoon" in abe_type:
            return PDDL_UTENSIL_TYPE
        if characteristics & ABE_VESSEL_CHARACTERISTICS or "pot" in abe_type or "pan" in abe_type:
            return PDDL_VESSEL_TYPE
    if abe_type == PDDL_FRIDGE_TYPE:
        return abe_type
    if "oven" in abe_type or "microwave" in abe_type or "foodprocessor" in abe_type:
        return PDDL_DEVICE_TYPE
    if "kitchencabinet" in abe_type:
        return PDDL_CLOPENABLE_STORAGE_TYPE
    if "kitchencounter" in abe_type:
        return PDDL_NOTCLOPENABLE_STORAGE_TYPE
    if ABE_CONTAINER_CHARACTERISTIC in characteristics:
        return PDDL_CONTAINER_TYPE
    return None


def extract_command(match):
    command = match[0]
    parameters = match[1].strip().split()
    return command, parameters


def handle_abort_commands(world: World):
    with open(DOMAIN_FILE_PATH, 'r') as domain_file:

        print('Defining PDDL problem dynamically from world state...')
        problem = compute_pddl_problem(world)
        print(f'PDDL problem definition complete.')

        req_body = {
            "domain": domain_file.read(),
            "problem": problem
        }

        print(problem)

        print('Starting planner...')
        start = time.time()
        solve_request_url = requests.post(f"{PDDL_PLANNER_URL}/package/lama-first/solve", json=req_body).json()
        response = requests.post(PDDL_PLANNER_URL + solve_request_url['result'])

        while response.json().get("status", "") == 'PENDING':
            # Query the result every 0.5 seconds while the job is executing
            response = requests.post(PDDL_PLANNER_URL + solve_request_url['result'])
            time.sleep(0.5)

        end = time.time()
        print(f'Planner finished in {round(end - start, 3)} seconds.')

        response = json.loads(response.content)
        result = response['result']
        if response['status'] != 'ok' or result is None or result['stderr'] != '' or result['output'] == {}:
            print('No plan found. Abe cannot cancel gracefully.')
            return

        plan = result['output']['sas_plan']
        steps = map(extract_command, re.findall(r'\((\S+)\s+(.*)\)', plan))
        # TODO: output is always lower case. Make the names match with names from world
        print(steps)
