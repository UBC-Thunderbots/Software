from abc import abstractmethod, ABC
from dataclasses import dataclass
from proto.import_all_protos import *
from software.thunderscope.time_provider import time_provider_instance
from typing import Iterator, Any


class IEvalLog(ABC):
    @abstractmethod
    @staticmethod
    def get_num_cols() -> int:
        """Gets the number of columns present in this log"""
        raise NotImplementedError("Please use the appropriate subclass of log!")

    @abstractmethod
    def to_array(self) -> list[Any]:
        """Converts this log to an array of elements"""
        raise NotImplementedError("Please use the appropriate subclass of log!")

    def to_csv_row(self):
        """
        Converts this log into a Comma Separated Values string
        
        :return: a string of values separated by commas
        """
        row_array = self.to_array()
        assert len(row_array) == self.get_num_cols()

        return ",".join(row_array)

    @abstractmethod
    @staticmethod
    def from_csv_row(row_iter: Iterator[str], **kwargs) -> IEvalLog | None:
        """
        Converts a CSV row into an instance of this log
        
        :param row_iter: an iterator representing a csv row, which returns elements one by one
        :param **kwargs: any extra arguments needed for this log not present in the csv row
        """
        raise NotImplementedError("Please use the appropriate subclass of log!")


@dataclass
class TimestampedEvalLog(IEvalLog):
    timestamp: float

    def __post_init__(self):
        """Sets the timestamp from the singleton"""
        self.timestamp = time_provider_instance.elapsed_time_ns()

    def get_timestamp(self) -> float:
        """Get this log's timestamp"""
        return self.timestamp

    @staticmethod
    def get_num_cols() -> int:
        return 1

    @override
    def to_array(self) -> list[Any]:
        return [self.timestamp]

    @staticmethod
    @override
    def from_csv_row(row_iter: Iterator[str], **kwargs) -> TimestampedEvalLog | None:
        return TimestampedEvalLog(timestamp=float(next(row_iter)))
