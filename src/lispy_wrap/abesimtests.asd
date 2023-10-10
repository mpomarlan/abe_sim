(defsystem #:abesimtests
  :version "0.0.1"
  :description "A set of tests for communicating to AbeSim from lisp."
  :author "Mihai Pomarlan"
  :license "BSD 2-clause"
  :depends-on (#:com.inuoe.jzon)
  :components ((:file "vr-requests")
               (:file "simtests" :depends-on ("vr-requests"))))
