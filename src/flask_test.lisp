(ql:quickload :drakma)
(ql:quickload :cl-json)

(json:encode-json
  '#( ((kitchen . "aaaa") )))


(drakma:http-request "http://lisp.org/")

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

(send-request "/abe-sim-command/to-get-kitchen" (cl-json:encode-json-to-string `((:kitchen . "aaaaa" ))))

(cl-json:encode-json-to-string 
(send-request "/abe-sim-command/to-get-kitchen" (cl-json:encode-json-to-string `((:kitchen . "aaaaa" )))))


(cl-json:encode-json-to-string `((:kitchen . "aaaaa" )))

;retrieve key from association list
(cdr (assoc (intern "a" :keyword) '((:a . 1 ))))

(defun get-kitchen-state (varname)
  (send-request "/abe-sim-command/to-get-kitchen" (cl-json:encode-json-to-string `((:kitchen . ,varname )))))

(get-kitchen-state "bbbb")