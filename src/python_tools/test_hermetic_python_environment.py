import sys

# a basic test to make sure we're not running the system python interpreter
assert "/usr/bin/python" not in sys.executable
