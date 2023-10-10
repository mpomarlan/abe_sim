(in-package :abesimtests)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;                                 utils                                                    ;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *kitchen-host* "http://127.0.0.1:54321")

(defun send-request (route json &key (host *kitchen-host*) (connection-timeout 3600))
  "Send curl request and returns the answer."
  (let* ((url (concatenate 'string *kitchen-host* route))
         (response (dex:post (concatenate 'string *kitchen-host* route)
                             :headers '(("content-type" . "application/json"))
                             :content json
                             :read-timeout 2000)))
    (com.inuoe.jzon:parse response)))

(defun alist-hashmap (alist)
  (let ((h (make-hash-table :test #'equal)))
    (mapcar (lambda (x)
              (setf (gethash (car x) h)
                    (cdr x)))
            alist)
    h))

(defun encode-request (alist)
    (com.inuoe.jzon:stringify (alist-hashmap alist)))

(defun symbol-keyword (x)
  (intern (string-upcase x) :keyword))

(defun cdrassoc (key alist)
  (cdr (assoc key alist)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;                                 commands                                                 ;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-get-kitchen ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-get-kitchen (kitchen-state-variable)
  (let ((response (send-request "/abe-sim-command/to-get-kitchen"
                                (encode-request `(("kitchenStateIn" . ,kitchen-state-variable))))))
    (when response (handler-case  (cons kitchen-state-variable (gethash kitchen-state-variable (gethash "response" response)))
                     (error (e) (format t
                                        "Error in response from abe-sim api service (route: to-get-kitchen): ~S.~&"
                                        e))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-set-kitchen  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-set-kitchen (kitchen-state-in)
  (gethash  "response"
            (send-request "/abe-sim-command/to-set-kitchen"
                          (encode-request `(("kitchenStateIn" . ,kitchen-state-in))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-cancel ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-cancel ()
  (let ((time (gethash "response"
                       (send-request "/abe-sim-command/to-cancel"
                                     (encode-request `())))))
    time))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-get-time ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-get-time ()
  (let ((time (gethash "response"
                       (send-request "/abe-sim-command/to-get-time"
                                     (encode-request `())))))
    time))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-wait ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-wait (frames-to-wait)
  (gethash "response"
           (send-request "/abe-sim-command/to-wait"
                         (encode-request `(("frames" . ,frames-to-wait))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-cut ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-cut (object cutting-tool cutting-pattern &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-cut"
                         (encode-request `(("object"         . ,object)
                                           ("cuttingTool"    . ,cutting-cool)
                                           ("cuttingPattern" . ,cutting-pattern)
                                           ("kitchenStateIn" . ,kitchen-state-in)
                                           ("setWorldState"  . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-refrigerate ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-refrigerate (container-with-ingredients refrigerator cooling-quantity cooling-unit &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-refrigerate"
                         (encode-request `(("containerWithIngredients" . ,container-with-ingredients)
                                           ("refrigerator"             . ,refrigerator)
                                           ("coolingQuantity"          . ,cooling-quantity)
                                           ("coolingUnit"              . ,cooling-unit)
                                           ("kitchenStateIn"           . ,kitchen-state-in)
                                           ("setWorldState"            . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-flour ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-flour (container-to-flour ingredient-to-flour-with &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-flour"
                         (encode-request `(("containerToFlour"      . ,container-to-flour)
                                           ("ingredientToFlourWith" . ,ingredient-to-flour-with)
                                           ("kitchenStateIn"        . ,kitchen-state-in)
                                           ("setWorldState"         . ,(not (not kitchen-state-in)))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-grease ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-grease (container-to-grease ingredient-to-grease-with &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-grease"
                         (encode-request `(("containerToGrease"      . ,container-to-grease)
                                           ("ingredientToGreaseWith" . ,ingredient-to-grease-with)
                                           ("kitchenStateIn"         . ,kitchen-state-in)
                                           ("setWorldState"          . ,(not (not kitchen-state-in)))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-sprinkle ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-sprinkle (object topping-container &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-sprinkle"
                         (encode-request `(("object"           . ,object)
                                           ("toppingContainer" . ,topping-container)
                                           ("kitchenStateIn"   . ,kitchen-state-in)
                                           ("setWorldState"    . ,(not (not kitchen-state-in)))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-bake ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-bake (thing-to-bake oven input-destination-container &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-bake"
                         (encode-request `(("thingToBake"               . ,thing-to-bake)
                                           ("oven"                      . ,oven)
                                           ("inputDestinationContainer" . ,input-destination-container)
                                           ("kitchenStateIn"            . ,kitchen-state-in)
                                           ("setWorldState"             . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-boil ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-boil (thing-to-boil stove heatingMode time-to-boil-amount time-to-boil-unit destination &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-boil"
                         (encode-request `(("thingToBoil"        . ,thing-to-boil)
                                           ("stoveToBoilOn"      . ,stove)
                                           ("heatingMode"        . ,heatingMode)
                                           ("timeToBoilQuantity" . ,time-to-boil-amount)
                                           ("timeToBoilUnit"     . ,time-to-boil-unit)
                                           ("destination"        . ,destination)
                                           ("kitchenStateIn"     . ,kitchen-state-in)
                                           ("setWorldState"      . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-melt ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-melt (container-with-input-ingredients melting-tool &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-melt"
                         (encode-request `(("containerWithInputIngredients" . ,container-with-input-ingredients)
                                           ("melting-tool"                  . ,oven)
                                           ("kitchenStateIn"            . ,kitchen-state-in)
                                           ("setWorldState"             . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-wash ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-wash (thing-to-wash sink destination &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-wash"
                         (encode-request `(("thingToWash"               . ,thing-to-wash)
                                           ("sink"                      . ,sink)
                                           ("inputDestinationContainer" . ,destination)
                                           ("kitchenStateIn"            . ,kitchen-state-in)
                                           ("setWorldState"             . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-line ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-line (baking-tray baking-paper &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-line"
                         (encode-request `(("bakingTray"     . ,baking-tray)
                                           ("bakingPaper"    . ,baking-paper)
                                           ("kitchenStateIn" . ,kitchen-state-in)
                                           ("setWorldState"  . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-mix  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-mix (container-with-input-ingredients mixing-tool &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-mix"
                         (encode-request `(("containerWithInputIngredients" . ,container-with-input-ingredients)
                                           ("mixingTool"                    . ,mixing-tool)
                                           ("kitchenStateIn"                . ,kitchen-state-in)
                                           ("setWorldState"                 . ,(not (not kitchen-state-in))))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-beat  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-beat (container-with-input-ingredients beating-tool &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-beat"
                         (encode-request `(("containerWithInputIngredients" . ,container-with-input-ingredients)
                                           ("beatingTool"                   . ,beating-tool)
                                           ("kitchenStateIn"                . ,kitchen-state-in)
                                           ("setWorldState"                 . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-transfer ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-transfer (container-with-input-ingredients target-container &optional (kitchen-state-in nil)) ;
  (gethash "response"
           (send-request "/abe-sim-command/to-transfer"
                         (encode-request `(("containerWithInputIngredients" . ,container-with-input-ingredients)
                                           ("targetContainer"               . ,target-container)
                                           ("kitchenStateIn"                . ,kitchen-state-in)
                                           ("setWorldState"                 . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-portion ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-portion (container-with-ingredient target-container quantity &optional (kitchen-state-in nil))
  (gethash "response" 
           (send-request "/abe-sim-command/to-portion"
                         (encode-request `(("containerWithIngredient" . ,container-with-ingredient)
                                           ("targetContainer"         . ,target-container)
                                           ("quantity"                . ,quantity)
                                           ("kitchenStateIn"          . ,kitchen-state-in)
                                           ("setWorldState"           . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-fetch ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-fetch (object &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-fetch"
                         (encode-request `(("object"         . ,object)
                                           ("kitchenStateIn" . ,kitchen-state-in)
                                           ("setWorldState"  . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-get-location ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-get-location (available-location-variable type &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-get-location"
                         (encode-request `(("availableLocation" . ,available-location-variable)
                                           ("type"              . ,type)
                                           ("kitchenStateIn"    . ,kitchen-state-in)
                                           ("setWorldState"     . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-leave-for-time ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-leave-for-time (container amount unit &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-leave-for-time"
                         (encode-request `(("containerWithIngredients" . ,container)
                                           ("coolingQuantity"          . ,amount)
                                           ("timeUnit"                 . ,unit)
                                           ("kitchenStateIn"           . ,kitchen-state-in)
                                           ("setWorldState"            . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-bring-to-temperature ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-bring-to-temperature (container amount unit &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-bring-to-temperature"
                         (encode-request `(("containerWithIngredients" . ,container)
                                           ("temperatureQuantity"      . ,amount)
                                           ("temperatureUnit"          . ,unit)
                                           ("kitchenStateIn"           . ,kitchen-state-in)
                                           ("setWorldState"            . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-preheat-oven ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-preheat-oven (oven amount unit &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-preheat-oven"
                         (encode-request `(("oven"           . ,oven)
                                           ("amount"         . ,amount)
                                           ("unit"           . ,unit)
                                           ("kitchenStateIn" . ,kitchen-state-in)
                                           ("setWorldState"  . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-fry ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-fry (thingToFry stoveToFryOn heatingMode timeToFryAmount timeToFryUnit &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-fry"
                         (encode-request `(("thingToFry"      . ,thingToFry)
                                           ("stoveToFryOn"    . ,stoveToFryOn)
                                           ("heatingMode"     . ,heatingMode)
                                           ("timeToFryAmount" . ,timeToFryAmount)
                                           ("timeToFryUnit"   . ,timeToFryUnit)
                                           ("kitchenStateIn"  . ,kitchen-state-in)
                                           ("setWorldState"   . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-place ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-place (object container &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-place"
                         (encode-request `(("object"         . ,object)
                                           ("container"      . ,container)
                                           ("kitchenStateIn" . ,kitchen-state-in)
                                           ("setWorldState"  . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-cover ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-cover (object cover &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-cover"
                         (encode-request `(("object"         . ,object)
                                           ("cover"          . ,cover)
                                           ("kitchenStateIn" . ,kitchen-state-in)
                                           ("setWorldState"  . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-uncover ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-uncover (object &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-uncover"
                         (encode-request `(("object"         . ,object)
                                           ("kitchenStateIn" . ,kitchen-state-in)
                                           ("setWorldState"  . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-peel ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-peel (inputIngredient peelingTool containerForPeels containerForPeeledIngredient &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-peel"
                         (encode-request `(("inputIngredient"              . ,inputIngredient)
                                           ("peelingTool"                  . ,peelingTool)
                                           ("containerForPeels"            . ,containerForPeels)
                                           ("containerForPeeledIngredient" . ,containerForPeeledIngredient)
                                           ("kitchenStateIn"               . ,kitchen-state-in)
                                           ("setWorldState"                . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-seed ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-seed (inputIngredient seedingTool containerForSeeds containerForSeededIngredient &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-seed"
                         (encode-request `(("inputIngredient"              . ,inputIngredient)
                                           ("seedingTool"                  . ,seedingTool)
                                           ("containerForSeeds"            . ,containerForSeeds)
                                           ("containerForSeededIngredient" . ,containerForSeededIngredient)
                                           ("kitchenStateIn"               . ,kitchen-state-in)
                                           ("setWorldState"                . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-flatten ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-flatten (portion flatteningTool &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-flatten"
                         (encode-request `(("portion"        . ,portion)
                                           ("flatteningTool" . ,flatteningTool)
                                           ("kitchenStateIn" . ,kitchen-state-in)
                                           ("setWorldState"  . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-grind ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-grind (inputIngredient grindingTool &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-grind"
                         (encode-request `(("containerWithIngredientsToBeGround" . ,inputIngredient)
                                           ("grindingTool"                       . ,grindingTool)
                                           ("kitchenStateIn"                     . ,kitchen-state-in)
                                           ("setWorldState"                      . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-mash ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-mash (inputIngredient mashingTool &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-mash"
                         (encode-request `(("inputIngredient" . ,inputIngredient)
                                           ("mashingTool"     . ,mashingTool)
                                           ("kitchenStateIn"  . ,kitchen-state-in)
                                           ("setWorldState"   . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-mingle ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-mingle (containerWithInputIngredients minglingTool &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-mingle"
                         (encode-request `(("containerWithInputIngredients" . ,containerWithInputIngredients)
                                           ("minglingTool"                  . ,minglingTool)
                                           ("kitchenStateIn"                . ,kitchen-state-in)
                                           ("setWorldState"                 . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-refrigerate ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-refrigerate (container amount unit &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-refrigerate"
                         (encode-request `(("containerWithIngredients" . ,container)
                                           ("refrigerator"             . ,refrigerator)
                                           ("amount"                   . ,amount)
                                           ("unit"                     . ,unit)
                                           ("kitchenStateIn"           . ,kitchen-state-in)
                                           ("setWorldState"            . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-portion-and-arrange ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-portion-and-arrange (containerWithDough destination &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-portion-and-arrange"
                         (encode-request `(("containerWithDough" . ,containerWithDough)
                                           ("destination"        . ,destination)
                                           ("kitchenStateIn"     . ,kitchen-state-in)
                                           ("setWorldState"      . ,(not (not kitchen-state-in))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-shape ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun request-to-shape (portions shape &optional (kitchen-state-in nil))
  (gethash "response"
           (send-request "/abe-sim-command/to-shape"
                         (encode-request `(("portions"       . ,portions)
                                           ("shape"          . ,shape)
                                           ("kitchenStateIn" . ,kitchen-state-in)
                                           ("setWorldState"  . ,(not (not kitchen-state-in))))))))
