from __future__ import annotations
from abc import abstractmethod, ABC
from dataclasses import dataclass, field
from proto.import_all_protos import *
from typing import Iterator, Any, override
from software.thunderscope.time_provider import time_provider_instance


class IEvalLog(ABC):
    @classmethod
    @abstractmethod
    def get_num_cols(cls) -> int:
        """Gets the number of columns present in this log"""
        raise NotImplementedError("Please use the appropriate subclass of log!")

    @abstractmethod
    def to_array(self) -> list[Any]:
        """Converts this log to an array of elements"""
        raise NotImplementedError("Please use the appropriate subclass of log!")

    def to_csv_row(self):
        """Converts this log into a Comma Separated Values string

        :return: a string of values separated by commas
        """
        row_array = self.to_array()
        assert len(row_array) == type(self).get_num_cols()

        return ",".join([str(elem) for elem in row_array])

    @staticmethod
    @abstractmethod
    def from_csv_row(row_iter: Iterator[str], **kwargs) -> IEvalLog | None:
        """Converts a CSV row into an instance of this log

        :param row_iter: an iterator representing a csv row, which returns elements one by one
        :param **kwargs: any extra arguments needed for this log not present in the csv row
        """
        raise NotImplementedError("Please use the appropriate subclass of log!")


@dataclass
class TimestampedEvalLog(IEvalLog):
    timestamp: float = field(default_factory=time_provider_instance.elapsed_time_ns)

    def get_timestamp(self) -> float:
        """Get this log's timestamp"""
        return self.timestamp

    @classmethod
    @override
    def get_num_cols(cls) -> int:
        return 1

    @override
    def to_array(self) -> list[Any]:
        return [self.timestamp]

    @staticmethod
    @override
    def from_csv_row(row_iter: Iterator[str], **kwargs) -> TimestampedEvalLog | None:
        return TimestampedEvalLog(timestamp=float(next(row_iter)))
