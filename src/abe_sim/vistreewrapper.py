import os
import subprocess
import sys

cmd = "\"" + sys.executable + "\" \"%s\"" % str(os.path.join(os.path.dirname(os.path.abspath(__file__)), "vistree.py"))
visProc = subprocess.Popen(cmd, subprocess.PIPE)
while True:
    pass