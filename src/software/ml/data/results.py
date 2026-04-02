from abc import abstractmethod
from software.evaluation.events.event import IEvent

class IResult:
  @abstractmethod
  def update_result(self, event: IEvent) -> None:
    raise NotImplementedError("Please use the appropriate subclass!")