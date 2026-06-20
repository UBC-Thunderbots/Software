from abc import abstractmethod
from software.evaluation.logs.log_interface import TimestampedEvalLog


class IResult:
    @abstractmethod
    def update_result(self, eval_log: TimestampedEvalLog) -> None:
        raise NotImplementedError("Please use the appropriate subclass!")
