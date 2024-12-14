import software.python_bindings as tbots_cpp
from software.py_constants import MAX_ROBOT_IDS_PER_SIDE

def CommunicationManager():
    def __init__(self):
        self.primitive_senders: List[Union[None, tbots_cpp.PrimitiveSender]] = [None] * MAX_ROBOT_IDS_PER_SIDE
