# Dependencies

To run the tests you will need to have the com.inuoe.jzon package available. It can be found here:

[https://github.com/Zulu-Inuoe/jzon](https://github.com/Zulu-Inuoe/jzon)

You will then need some environment to work with common lisp, for example SLIME in emacs.

# Loading packages

Both abesimtests and com.inuoe.jzon should be on the package path so that quicklisp can access them. One way to do this is to run:

```lisp
(push "path/to/com.inuoe.jzon" quicklisp:*local-project-directories*)
(push "path/to/abesimtests" quicklisp:*local-project-directories*)
```

In both cases, the path should be that of a directory containing the asd file belonging to the package. Once this is done, run:

```lisp
(quicklisp:quickload "abesimtests")
```

# Running tests

First, make sure there is a terminal in which the runBrain.py script is running. Then, you may call either of the test functions:

```lisp
(abesimtests:test-almond-cookies)
(abesimtests:test-broccoli-salad)
```

