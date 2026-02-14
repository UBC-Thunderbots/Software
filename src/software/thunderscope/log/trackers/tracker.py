from software.thunderscope.proto_unix_io import ProtoUnixIO
from typing import Callable, Any, Optional

class Tracker:
  def __init__(self, callback: Optional[Callable[[Any], None]] = None, buffer_size: int = 5):
    self.callback = callback
    self.buffer_size = buffer_size
    
  def set_proto_unix_io(self, proto_unix_io: ProtoUnixIO) -> None:
    raise Exception("Not Implemented, please use the appropriate subclass!")
    
  def refresh(self) -> None:
    raise Exception("Not Implemented, please use the appropriate subclass!")