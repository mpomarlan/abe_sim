(define (domain abe_cancel)

    ;remove requirements that are not needed
    (:requirements :strips :typing :conditional-effects :negative-preconditions :derived-predicates :equality :disjunctive-preconditions)

    (:types
        locatable - object
        container utensil food disposable - locatable
        vessel storage trash_can - container
        clopenable notclopenablestorage - storage
        device fridge clopenablestorage - clopenable
        perishable nonperishable - food
    )

    ; un-comment following line if constants are needed
    ;(:constants )

    (:predicates
        (relevant ?obj - object)
        (at ?obj - locatable ?loc - locatable)
        (holding-left ?obj - locatable)
        (holding-right ?obj - locatable)
        (robot-can-grasp)
        (immobile ?gr - locatable)
        (open ?c - clopenable)
        (closed ?c - clopenable)
        (on ?c - device)
        (off ?c - device)
        (safe-perishable ?p - perishable)
        (safe-nonperishable ?np - nonperishable) ; in clopenablestorage sau notclopenablestorage sau chiar si tranzitiv in fridge (doar in cazul in care se afla intr-un vessel care contine alimente perisabile)
        (safe-clopenable ?clo - clopenable) ; closed
        (safe-device ?dev - device) ; off, safe-clopenable, nu exista food care se afla (tranzitiv) in device, nu exista vessel, nu exista utensils
        (safe-vessel ?ves - vessel) ;ori in fridge (in caz ca (,) contine alimente perisabile) ori in storage, ori in notclopenablestorage
        (safe-utensil ?ut - utensil) ; intr-un clopenablestorage sau notclopenablestorage (de exemplu dulap, masa, etc...)
        (safe-disposable ?disp - disposable) ; intr-un clopenablestorage sau notclopenablestorage (de exemplu dulap, masa, etc...)
        (transitive-at ?obj - locatable ?loc - locatable)

    )

    (:action move
        :parameters (?gr - locatable ?src - container ?dest - storage)
        :precondition (and
            (robot-can-grasp)
            (not (immobile ?gr))
            (at ?gr ?src)
        )
        :effect (and
            (not (at ?gr ?src))
            (at ?gr ?dest)
        )
    )

    (:action put
        :parameters (?gr - locatable ?dest - notclopenablestorage)
        :precondition (and
            (or
                (holding-left ?gr)
                (holding-right ?gr))
            (not (immobile ?gr))
        )
        :effect (and
            (when
                (holding-left ?gr)
                (not (holding-left ?gr))
            )
            (when
                (holding-right ?gr)
                (not (holding-right ?gr))
            )
            (at ?gr ?dest)
        )
    )

    (:action close
        :parameters (?clo - clopenable)
        :precondition (and
            (robot-can-grasp)
            (open ?clo)
        )

        :effect (and
            (not (open ?clo))
            (closed ?clo))
    )

    (:action turnoff
        :parameters (?dev - device)
        :precondition (and
            (robot-can-grasp)
            (on ?dev)
        )
        :effect (and
            (not (on ?dev))
            (off ?dev)
        )
    )

    (:derived
        (transitive-at ?st - locatable ?loc - locatable)
        (or
            (at ?st ?loc)
            (exists
                (?other - container)
                (and
                    (at ?st ?other)
                    (transitive-at ?other ?loc)
                )
            )
        )
    )

    (:derived
        (safe-perishable ?p - perishable) ; transitively in fridge
        (exists
            (?fr - fridge)
            (transitive-at ?p ?fr)
        )
    )

    (:derived
        (safe-nonperishable ?np - nonperishable) ; in clopenablestorage sau notclopenablestorage sau chiar si tranzitiv in fridge (doar in cazul in care se afla intr-un vessel care contine alimente perisabile)
        (or
            (exists
                (?st - clopenablestorage)
                (transitive-at ?np ?st)
            )
            (exists
                (?st - notclopenablestorage)
                (transitive-at ?np ?st)
            )
            (exists
                (?fr - fridge)
                (and
                    (transitive-at ?np ?fr)
                    (exists
                        (?ves - vessel)
                        (and
                            (transitive-at ?np ?ves)
                            (exists
                                (?p - perishable)
                                (transitive-at ?p ?ves)
                            )
                        )

                    )
                )
            )
        )
    )

    (:derived
        (safe-clopenable ?clo - clopenable) ; closed
        (closed ?clo)
    )

    (:derived
        (safe-device ?dev - device) ; off, safe-clopenable, nu exista food care se afla (tranzitiv) in device, nu exista vessel, nu exista utensils
        (and
            (off ?dev)
            (safe-clopenable ?dev)
        )
    )

    (:derived
        (safe-vessel ?ves - vessel) ;ori in fridge (in caz ca (,) contine alimente perisabile) ori in storage, ori in notclopenablestorage
        (or
            (and
                (not (exists
                        (?p - perishable)
                        (at ?p ?ves)
                    )
                )
                (or
                    (exists
                        (?st - clopenablestorage)
                        (at ?ves ?st)
                    )
                    (exists
                        (?st - notclopenablestorage)
                        (at ?ves ?st)
                    )
                )
            )

            (exists
                (?p - perishable)
                (and
                    (at ?p ?ves)
                    (exists
                        (?fr - fridge)
                        (at ?ves ?fr)
                    )
                )
            )
        )
    )

    (:derived
        (safe-utensil ?ut - utensil) ; intr-un clopenablestorage sau notclopenablestorage (de exemplu dulap, masa, etc...)
        (or
            (exists
                (?st - clopenablestorage)
                (transitive-at ?ut ?st)
            )
            (exists
                (?st - notclopenablestorage)
                (transitive-at ?ut ?st)
            )
        )
    )

    (:derived
        (safe-disposable ?disp - disposable) ; intr-un clopenablestorage sau notclopenablestorage (de exemplu dulap, masa, etc...)
        (exists
            (?can - trash_can)
            (at ?disp ?can)
        )
    )

    (:derived
        (robot-can-grasp)
        (and
            (not (exists
                    (?obj - locatable)
                    (holding-left ?obj)
                )
            )

            (not (exists
                    (?obj - locatable)
                    (holding-right ?obj)
                )
            )
        )
    )
)