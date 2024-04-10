import os

BASE_PATH = os.path.dirname(__file__)
DOMAIN_FILE_PATH = BASE_PATH + "\\..\\pddl\\domain.pddl"
WORLD_STATE_FILE_PATH = BASE_PATH + "\\..\\WS1.log"

PDDL_PLANNER_URL = "https://solver.planning.domains:5001"
PDDL_PROBLEM_HEADER = "(problem abe_cancel_problem) (:domain abe_cancel)"
PDDL_GOAL = """
   (:goal
        (and
            (forall
                (?pf - perishable)
                (safe-perishable ?pf)
            )
            (forall
                (?npf - nonperishable)
                (safe-nonperishable ?npf)
            )
            (forall
                (?cl - clopenable)
                (safe-clopenable ?cl)
            )
            (forall
                (?dev - device)
                (safe-device ?dev)
            )
            (forall
                (?ves - vessel)
                (safe-vessel ?ves)
            )
            (forall
                (?ut - utensil)
                (safe-utensil ?ut)
            )
            (forall
                (?dis - disposable)
                (safe-disposable ?dis)
            )
        )
    )"""
PDDL_FRIDGE_TYPE = 'fridge'
PDDL_DEVICE_TYPE = 'device'
PDDL_VESSEL_TYPE = 'vessel'
PDDL_UTENSIL_TYPE = 'utensil'
PDDL_PERISHABLE_TYPE = 'perishable'
PDDL_NONPERISHABLE_TYPE = 'nonperishable'
PDDL_CLOPENABLE_STORAGE_TYPE = 'clopenablestorage'
PDDL_NOTCLOPENABLE_STORAGE_TYPE = 'notclopenablestorage'
PDDL_CONTAINER_TYPE = 'container'
PDDL_TRASHCAN_TYPE = 'trash_can'
PDDL_DISPOSABLE_TYPE = 'disposable'

ABE_ROBOT_NAMES = {'abe', 'bea'}
ABE_UTENSIL_CHARACTERISTICS = {'cancut', 'canmash', 'canpeel', 'canskim', 'canspread'}
ABE_VESSEL_CHARACTERISTICS = {'coverable'}
ABE_PERISHABLE_CHARACTERISTICS = {'bakeable', 'consumable', 'crackable', 'cuttable', 'fryable', 'mashable',
                                  'mingleable', 'mixable', 'mixmakeable', 'peelable', 'seedable', 'skimmable',
                                  'sliceable'}
ABE_NONPERISHABLE_CHARACTERISTICS = {'pourable'}
ABE_GRASPABLE_CHARACTERISTIC = 'graspable'
ABE_CONTAINER_CHARACTERISTIC = 'cancontain'
IMMOBILE_CHARACTERISTIC = 'immobile'
