(ql:quickload :drakma)
(ql:quickload :cl-json)

; encode json
;(json:encode-json '#( ((kitchen . "aaaa") )))


(defparameter *kitchen-host* "http://127.0.0.1:54321")

(defun send-request (route json &key (host *kitchen-host*) (connection-timeout 20))
  "Send curl request and returns the answer."
  (let* ((url (string-append host route))
         (response (drakma:http-request url
                                        :method :post
                                        :content-type "application/json"
                                        :content json
                                        :connection-timeout connection-timeout)))
    (when response (handler-case (cl-json:decode-json-from-string response)
                     (error (e)
                       (format t "Error in response from spacy API service [nlp-tools penelope-interface]: ~S.~&" e))))))

;(send-request "/abe-sim-command/to-get-kitchen" (cl-json:encode-json-to-string `((:kitchen . "aaaaa" ))))

; send request and show the string as json string again
;(cl-json:encode-json-to-string (send-request "/abe-sim-command/to-get-kitchen" (cl-json:encode-json-to-string `((:kitchen . "aaaaa" )))))



; encode an association list to a json string
;(cl-json:encode-json-to-string `((:kitchen . "aaaaa" )))

;retrieve key from association list
;(cdr (assoc (intern "a" :keyword) '((:a . 1 ))))



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

(defun to-transfer ( object-assoc-list)
  (send-request "/abe-sim-command/to-transfer" (cl-json:encode-json-to-string object-assoc-list)))

(defun to-portion ( object-assoc-list)
  (send-request "/abe-sim-command/to-portion" (cl-json:encode-json-to-string object-assoc-list)))

(defun to-fetch ( object-assoc-list)
  (send-request "/abe-sim-command/to-fetch" (cl-json:encode-json-to-string object-assoc-list)))

(defun to-get-location ( object-assoc-list)
  (send-request "/abe-sim-command/to-get-location" (cl-json:encode-json-to-string object-assoc-list)))

(defun to-get-kitchen ( kitchen-state-variable)
  (let ((response (send-request "/abe-sim-command/to-get-kitchen" (cl-json:encode-json-to-string `((:kitchen . ,kitchen-state-variable))))))
    (when response (handler-case
                       (assoc (intern (string-upcase kitchen-state-variable) :keyword) (cdr (assoc :response response)))
                     (error (e) (format t "Error in response from abe-sim api service (route: to-get-kitchen): ~S.~&" e))))))


(defun to-set-kitchen ( kitchen-state-in )
   (send-request "/abe-sim-command/to-set-kitchen" (cl-json:encode-json-to-string `((:kitchen-state-in . ,kitchen-state-in)))))

;;; command 1: get the kitchen state
(defparameter  world-state (cdr (to-get-kitchen "?kitchen-state-1" )))


;;; command 2: set the kitchen state
(defparameter response-set-state (to-set-kitchen world-state))

;;;command 3: get location
;(defparameter response-get-location (to-get-location '((:available-location . "?available-countertop")(:type . "CounterTop") (:kitchen-State-In . nil) (:set-World-State . nil)) ))



;response-get-location

(cl-json:encode-json-to-string (cl-json:decode-json-from-string "{\"a\":[1,2]}"))

world-state


