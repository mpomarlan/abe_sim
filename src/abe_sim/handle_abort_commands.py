import json
import re
import time

import inflection
import requests

from abe_sim.constants import *
from abe_sim.dbpedia_utils import DBPEDIA_PERISHABLE_FOODS, DBPEDIA_UTENSILS, DBPEDIA_VESSELS
from abe_sim.world import World


def camel_to_snake(word: str):
    if word is None:
        return None
    return inflection.underscore(word)


def snake_to_camel(word: str):
    if word is None:
        return None
    return inflection.camelize(word, uppercase_first_letter=False)


def extract_abe_world_state_objects(objects):
    world_state_objects = []
    for obj_name, obj_data in objects.items():
        if obj_data.get('simtype') == 'ktree':
            fn_values = obj_data.get('fn', {})
            characteristics = [key.lower() for key, value in fn_values.items() if
                               isinstance(value, bool) and value is True]
            obj_dict = {
                'name': camel_to_snake(obj_name),
                'type': camel_to_snake(obj_data.get('type')),
                'characteristics': set(characteristics),
                'at': camel_to_snake(obj_data.get('at')),
                'immobile': obj_data.get('immobile')
            }

            grasping = obj_data.get('customStateVariables', {}).get('grasping', {}).get('actuallyGrasping', {})
            if not grasping:
                world_state_objects.append(obj_dict)
                continue
            grasping_left = next(iter(grasping.get("hand_left", [])), None)
            grasping_right = next(iter(grasping.get("hand_right", [])), None)
            if grasping_left:
                obj_dict['holding_left'] = list(map(lambda x: camel_to_snake(x), grasping_left[0]))
            if grasping_right:
                obj_dict['holding_right'] = list(map(lambda x: camel_to_snake(x), grasping_right[0]))
            # TODO: here also add keys for properties such as closed or open and on or off when they are
            #  added in abe_sim

            world_state_objects.append(obj_dict)

    return world_state_objects


def compute_pddl_init_state(abe_world_state_objects, relevant_world_state_objects):
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

            if name in relevant_world_state_objects:
                pddl_init_predicates.append(f"(relevant {name})")

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


def compute_pddl_problem(world, relevantEntities):
    abe_world_state_objects = extract_abe_world_state_objects(world.worldDump())
    relevant_world_state_objects = set([x["name"] for x in abe_world_state_objects]).intersection(relevantEntities)
    init_state = compute_pddl_init_state(abe_world_state_objects, relevant_world_state_objects)
    return f"(define {PDDL_PROBLEM_HEADER} {init_state} {PDDL_GOAL})"


def infer_pddl_type(abe_world_object):
    abe_name = abe_world_object['name'].lower()
    abe_type = abe_world_object['type'].lower()
    characteristics = abe_world_object['characteristics']

    if ABE_GRASPABLE_CHARACTERISTIC in characteristics:
        if characteristics & ABE_PERISHABLE_CHARACTERISTICS or "fresh" in abe_name \
                or abe_type in DBPEDIA_PERISHABLE_FOODS:
            return PDDL_PERISHABLE_TYPE
        if characteristics & ABE_NONPERISHABLE_CHARACTERISTICS:
            return PDDL_NONPERISHABLE_TYPE
        if characteristics & ABE_UTENSIL_CHARACTERISTICS or "fork" in abe_name or "spoon" in abe_name \
                or abe_type in DBPEDIA_UTENSILS:
            return PDDL_UTENSIL_TYPE
        if characteristics & ABE_VESSEL_CHARACTERISTICS or "pot" in abe_name or "pan" in abe_name \
                or abe_type in DBPEDIA_VESSELS:
            return PDDL_VESSEL_TYPE
    if abe_name == PDDL_FRIDGE_TYPE or abe_type == PDDL_FRIDGE_TYPE:
        return PDDL_FRIDGE_TYPE
    if abe_name == PDDL_TRASHCAN_TYPE or abe_type == PDDL_TRASHCAN_TYPE:
        return PDDL_TRASHCAN_TYPE
    if "peel" in abe_type or "seed" in abe_type:
        return PDDL_DISPOSABLE_TYPE
    if "oven" in abe_name or "microwave" in abe_name or "food_processor" in abe_name:
        return PDDL_DEVICE_TYPE
    if "kitchen_cabinet" in abe_name:
        return PDDL_CLOPENABLE_STORAGE_TYPE
    if "kitchen_counter" in abe_name:
        return PDDL_NOTCLOPENABLE_STORAGE_TYPE
    if ABE_CONTAINER_CHARACTERISTIC in characteristics:
        return PDDL_CONTAINER_TYPE
    return None


def extract_command(match):
    command = match[0]
    parameters = match[1].strip().split()
    return command, parameters


def handle_abort_commands(world: World, relevantEntities: set):
    with open(DOMAIN_FILE_PATH, 'r') as domain_file:

        print('Defining PDDL problem dynamically from world state...')
        problem = compute_pddl_problem(world, relevantEntities)
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
            return None

        plan = result['output']['sas_plan']
        steps = list(map(extract_command, re.findall(r'\((\S+)\s+(.*)\)', plan)))
        steps.pop()  # last element of plan contains the cost information, not actually a step
        retq = [(action, list(map(lambda x: snake_to_camel(x), params))) for action, params in steps]
        print(retq)
        return retq
