from abc import abstractmethod
from dataclass import dataclass
from proto.import_all_protos import *

@dataclass
class IEvent:
  
  @abstractmethod
  def get_timestamp(self) -> float:
    raise NotImplementedError("Please use the appropriate subclass of event!")
  
  @abstractmethod
  def to_csv_row(self):
    raise NotImplementedError("Please use the appropriate subclass of event!")
  
  @abstractmethod
  def from_csv_row(self):
    raise NotImplementedError("Please use the appropriate subclass of event!")