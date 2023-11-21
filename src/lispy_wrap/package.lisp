(in-package :cl-user)

(defpackage :abesimtests
  (:use :cl-user
        :common-lisp)
  (:export #:test-almond-cookies
           #:test-broccoli-salad)
  (:shadow )
  (:documentation "Tests of the AbeSim HTTP Post interface from Lisp."))
