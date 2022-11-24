(ql:quickload :drakma)
(ql:quickload :cl-json)
(ql:quickload :dexador)

;;https://github.com/marijnh/ST-JSON ?

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
                                           :timeout 2000)))
    (cl-json::decode-json-from-string response)))


(defun symbol-keyword (x)
  (intern (string-upcase x) :keyword))

(defun cdrassoc (key alist)
  (cdr (assoc key alist)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;                                 commands                                                 ;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-wait ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-wait (frames)
  (cdr (assoc :response
              (send-request "/abe-sim-command/to-wait"
                            (cl-json:encode-json-to-string `((:frames . ,frames)))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-sprinkle ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-sprinkle (object topping-container )
  (cdr (assoc :response
              (send-request "/abe-sim-command/to-sprinkle"
                            (cl-json:encode-json-to-string `((:object . ,object)
                                                             (:topping-container . ,topping-container)
                                                             (:kitchen-State-In . nil)
                                                             (:set-World-State . nil)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-bake ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-bake (thing-to-bake oven input-destination-container)
  (cdr (assoc :response
              (send-request "/abe-sim-command/to-bake"
                            (cl-json:encode-json-to-string `((:thing-to-bake . ,thing-to-bake)
                                                             (:oven . ,oven)
                                                             (:input-destination-container . ,input-destination-container)
                                                             (:kitchen-State-In . nil)
                                                             (:set-World-State . nil)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-shape ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-shape (container-with-dough destination)
  (cdr (assoc :response
              (send-request "/abe-sim-command/to-shape"
                            (cl-json:encode-json-to-string `((:container-with-dough . ,container-with-dough)
                                                             (:destination . ,destination)
                                                             (:kitchen-State-In . nil)
                                                             (:set-World-State . nil)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-line ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-line (baking-tray baking-paper)
  (cdr (assoc :response
              (send-request "/abe-sim-command/to-line"
                            (cl-json:encode-json-to-string `((:baking-tray . ,baking-tray)
                                                             (:baking-paper . ,baking-paper)
                                                             (:kitchen-State-In . nil)
                                                             (:set-World-State . nil)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-mix  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-mix (container-with-input-ingredients mixing-tool)
  (cdr (assoc :response
              (send-request "/abe-sim-command/to-mix"
                            (cl-json:encode-json-to-string `((:container-with-input-ingredients . ,container-with-input-ingredients)
                                                             (:mixing-tool . ,mixing-tool)
                                                             (:kitchen-State-In . nil)
                                                             (:set-World-State . nil)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-transfer ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-transfer (container-with-input-ingredients target-container)
  (cdr (assoc :response
              (send-request "/abe-sim-command/to-transfer"
                            (cl-json:encode-json-to-string `((:container-with-input-ingredients . ,container-with-input-ingredients)
                                                             (:target-container . ,target-container)
                                                             (:kitchen-State-In . nil)
                                                             (:set-World-State . nil)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-portion ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-portion (container-with-ingredient target-container quantity)
  (cdr (assoc :response (send-request
   "/abe-sim-command/to-portion"
  
               (cl-json:encode-json-to-string `((:container-with-ingredient . ,container-with-ingredient)
                                                (:target-container . ,target-container)
                                                (:quantity . ,quantity)
                                                (:kitchen-State-In . nil)
                                                (:set-World-State . nil)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-fetch ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-fetch (object) 
  (cdr (assoc :response
              (send-request "/abe-sim-command/to-fetch"
                            (cl-json:encode-json-to-string `((:object . ,object)
                                                             (:kitchen-State-In . nil)
                                                             (:set-World-State . nil) ))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-get-location ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-get-location (available-location-variable type)
  (cdr (assoc :response
              (send-request "/abe-sim-command/to-get-location"
                            (cl-json:encode-json-to-string `((:available-location . ,available-location-variable)
                                                             (:type . ,type)
                                                             (:kitchen-state-in . nil)
                                                             (:set-world-state . nil)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-get-kitchen ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-get-kitchen (kitchen-state-variable)
  (let ((response (send-request "/abe-sim-command/to-get-kitchen"
                                (cl-json:encode-json-to-string `((:kitchen-state-in . ,kitchen-state-variable))))))
    (when response (handler-case  (assoc (symbol-keyword kitchen-state-variable) (cdr (assoc :response response)))
                     (error (e) (format t
                                        "Error in response from abe-sim api service (route: to-get-kitchen): ~S.~&"
                                        e))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; to-set-kitchen  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun to-set-kitchen ( kitchen-state-in )
  (cdr (assoc  :response
               (send-request "/abe-sim-command/to-set-kitchen"
                             (cl-json:encode-json-to-string `((:kitchen-state-in . ,kitchen-state-in)))))))

 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;                             execute commands                                             ;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;; command 1: get the kitchen state
(defparameter  world-state (cdr (to-get-kitchen "?kitchen-state-1" )))
world-state

;;; command 2: set the kitchen state
(defparameter response-set-state (to-set-kitchen world-state))
response-set-state

;;;command 3: get location
(defparameter response-get-location (to-get-location "?available-countertop" "CounterTop"))
response-get-location

;; command 4: fetch bowl to countertop
;; {'object': 'mediumBowl1', 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-fetch-bowl (to-fetch "mediumBowl1"))
response-fetch-bowl

;; command 5: proportion 134g of sugarBag in mediumBowl1
;;  {'containerWithIngredient': 'sugarBag', 'targetContainer': 'mediumBowl1', 'quantity': 134, 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-sugar-in-bowl (to-portion "sugarBag" "mediumBowl1" 134))

;; command 6: proportion 134g of butterBag in mediumBowl2
;; {'containerWithIngredient': 'butterBag', 'targetContainer': 'mediumBowl2', 'quantity': 226, 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-butter-in-bowl (to-portion "butterBag" "mediumBowl2" 134))

;; Command 7: fetch bowl to countertop
(defparameter response-fetch-bowl-3 (to-fetch "mediumBowl3"))
response-fetch-bowl-3

;; Command 8: transfer bowl1 contents to bowl3
;; {'containerWithInputIngredients': 'mediumBowl1', 'targetContainer': 'mediumBowl3', 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-transfer-1 (to-transfer "mediumBowl1" "mediumBowl3"))
response-transfer-1

;; {'object': 'mediumBowl1', 'kitchenStateIn': None, 'setWorldState': None} LISP
;; {'object': 'mediumBowl1', 'kitchenStateIn': None, 'setWorldState': False} py

;; Command 9: transfer bowl2 contents to bowl3
;; {'containerWithInputIngredients': 'mediumBowl2', 'targetContainer': 'mediumBowl3', 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-transfer-2 (to-transfer  "mediumBowl2" "mediumBowl3"))
response-transfer-1

;; Command 10: mixing
;; {'containerWithInputIngredients': 'mediumBowl3', 'mixingTool': 'whisk', 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-mixing-1 (to-mix "mediumBowl3" "whisk"))
response-mixing-1

;; Command 11: lining
;; {'bakingTray': 'bakingTray1', 'bakingPaper': 'bakingSheet1', 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-lining-1 (to-line "bakingTray1" "bakingSheet1"))


;; Command 12: shaping
;; {'containerWithDough': 'mediumBowl3', 'destination': 'bakingTray1', 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-shape-1 (to-shape "mediumBowl3" "bakingTray1"))

;; TO WAIT
;; {'frames': 1000}
(defparameter response-wait-1 (to-wait 1000))

;; Command 13: baking
;; {'thingToBake': 'bakingTray1', 'oven': 'kitchenStove', 'inputDestinationContainer': 'counterTop', 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-bake-1 (to-bake "bakingTray1" "kitchenStove" "counterTop"))


;; Command 14: sprinkling
;; {'object': 'bakingTray1', 'toppingContainer': 'sugarShaker', 'kitchenStateIn': None, 'setWorldState': False}
(defparameter response-sprinkling (to-sprinkle "bakingTray1" "sugarShaker"))
