from enum import Enum

class ActionArgument(str, Enum):
    build = "build"
    test = "test"
    run = "run"
