(ql:quickload :drakma)
(ql:quickload :cl-json)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defparameter *kitchen-host* "http://127.0.0.1:54321")

(defun send-request (route json &key (host *kitchen-host*) (connection-timeout 3600))
  "Send curl request and returns the answer."
  (let* ((url (concatenate 'string host route))
         (response (drakma:http-request url
                                        :method :post
                                        :content-type "application/json"
                                        :content json
                                        :connection-timeout connection-timeout)))
    (when response (handler-case (cl-json:decode-json-from-string response)
                     (error (e)
                       (format t "Error in response from spacy API service [nlp-tools penelope-interface]: ~S.~&" e))))))

(defun symbol-keyword (x)
  (intern (string-upcase x) :keyword))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; commands

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



(defun to-sprinkle ( object-assoc-list)
  (send-request "/abe-sim-command/to-sprinkle" (cl-json:encode-json-to-string object-assoc-list)))

(defun to-bake ( object-assoc-list)
  (send-request "/abe-sim-command/to-bake" (cl-json:encode-json-to-string object-assoc-list)))

(defun to-shape ( object-assoc-list)
  (send-request "/abe-sim-command/to-shape" (cl-json:encode-json-to-string object-assoc-list)))

(defun to-line ( object-assoc-list)
  (send-request "/abe-sim-command/to-line" (cl-json:encode-json-to-string object-assoc-list)))

(defun to-mix ( object-assoc-list)
  (send-request "/abe-sim-command/to-mix" (cl-json:encode-json-to-string object-assoc-list)))

(defun to-transfer (container-with-ingredients target-container)
  (send-request "/abe-sim-command/to-transfer"
                (cl-json:encode-json-to-string `((:container-with-ingredients . ,container-with-ingredients)
                                                 (:target-container . ,target-container)
                                                 (:kitchen-State-In . nil)
                                                 (:set-World-State . nil)))))

(defun to-portion (container-with-ingredient target-container quantity)
  (send-request
   "/abe-sim-command/to-portion"
   (cl-json:encode-json-to-string `((:container-with-ingredient . ,container-with-ingredient)
                                    (:target-container . ,target-container)
                                    (:quantity . ,quantity)
                                    (:kitchen-State-In . nil)
                                    (:set-World-State . nil)))))

(defun to-fetch (object) 
  (send-request "/abe-sim-command/to-fetch" (cl-json:encode-json-to-string `((:object . ,object)
                                                                             (:kitchen-State-In . nil)
                                                                             (:set-World-State . nil) ))))

(defun to-get-location (available-location-variable type)
  (send-request
   "/abe-sim-command/to-get-location"
   (cl-json:encode-json-to-string `((:available-location . ,available-location-variable)
                                    (:type . ,type)
                                    (:kitchen-state-in . nil)
                                    (:set-world-state . nil)))))


(defun to-get-kitchen ( kitchen-state-variable)
  (let ((response (send-request "/abe-sim-command/to-get-kitchen" (cl-json:encode-json-to-string `((:kitchen . ,kitchen-state-variable))))))
    (when response (handler-case
                       (assoc (symbol-keyword  kitchen-state-variable) (cdr (assoc :response response)))
                     (error (e) (format t "Error in response from abe-sim api service (route: to-get-kitchen): ~S.~&" e))))))



(defun to-set-kitchen ( kitchen-state-in )
   (send-request "/abe-sim-command/to-set-kitchen" (cl-json:encode-json-to-string `((:kitchen-state-in . ,kitchen-state-in)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; execute commands


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;; command 1: get the kitchen state
(defparameter  world-state (cdr (to-get-kitchen "?kitchen-state-1" )))
world-state
(assoc :abe world-state)
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
