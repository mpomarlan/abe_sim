(in-package :abesimtests)

(defun place-all (ptype dest)
  (mapcar (lambda (objname)
            (request-to-place objname dest))
          (loop for k being hash-key
                    using (hash-value v)
                    of (cdr (request-get-kitchen "ks"))
                    when (equal ptype (gethash "type" v))
                      collect k)))

(defun test-broccoli-salad ()
  ;; TODO: load appropriate initial WS
  (request-to-preheat-oven "oven" 200 "C")
  (request-to-fry "fryingPan" "oven" "medium" 1 "min")
  (request-to-preheat-oven "oven" 150 "C")
  (request-to-fetch "cuttingBoard")
  (request-to-place "bacon" "cuttingBoard")
  (request-to-place "fryingPan" "kitchenCabinet")
  (request-to-fetch "largeBowl1")
  (request-to-cut "bacon" "cookingKnife" "dice")
  (place-all "ChoppedCookedBacon" "largeBowl1")
  (request-to-place "broccoli" "cuttingBoard")
  (request-to-cut "broccoli" "cookingKnife" "dice")
  (place-all "ChoppedBroccoli" "largeBowl1")
  (request-to-place "cuttingBoard" "kitchenCabinet")
  (request-to-fetch "mediumBowl1")
  (request-to-peel "potato" "cookingKnife" "mediumBowl1" "largeBowl1")
  (request-to-mash "largeBowl1" "masher")
  (request-to-place "cookingKnife" "kitchenCabinet")
  (request-to-place "mediumBowl1" "kitchenCabinet")
  (request-to-fetch "mediumBowl2")
  (request-to-portion "mayonnaiseJar" "mediumBowl2" 134)
  (request-to-portion "ciderVinegarBottle" "mediumBowl2" 134)
  (request-to-mix "mediumBowl2" "whisk")
  (request-to-transfer "mediumBowl2" "largeBowl1")
  (request-to-mingle "largeBowl1" "whisk")
  (request-to-refrigerate "largeBowl1" "fridge" 4 "C"))

(defun test-almond-cookies ()
  ;; TODO: load appropriate initial WS
  (request-to-fetch "mediumBowl1")
  (request-to-portion "sugarBag" "mediumBowl1" 134)
  (request-to-fetch "mediumBowl2")
  (request-to-portion "butterBag" "mediumBowl2" 134)
  (request-to-fetch "mediumBowl3")
  (request-to-transfer "mediumBowl1" "mediumBowl3")
  (request-to-transfer "mediumBowl2" "mediumBowl3")
  (request-to-mix "mediumBowl3" "whisk")
  (request-to-fetch "bakingTray")
  (request-to-line "bakingTray" "bakingSheet")
  (request-to-portion-and-arrange "mediumBowl3" "bakingTray") ;;
  (request-to-bake "bakingTray" "oven" "kitchenCounter")
  (request-to-sprinkle "bakingTray" "sugarBag"))