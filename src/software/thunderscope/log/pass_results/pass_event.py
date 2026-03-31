from software.thunderscope.log.trackers.tracked_event import (
    TrackedEvent,
    event_to_csv_row as base_event_to_csv_row,
)
from dataclasses import dataclass
from enum import auto, StrEnum
from proto.import_all_protos import *


class PassResultType(StrEnum):
    """Enum for the different types of pass results we want to log"""

    RESULT_1S = auto()
    RESULT_5S = auto()
    RESULT_10S = auto()


@dataclass
class TrackedPassResult:
    """Class representing a Pass we are tracking
    Contains the pass event itself within the nested TrackedEvent
    along with information about the pass itself and its score
    """

    pass_event: TrackedEvent
    pass_result_type: PassResultType
    pass_: Pass
    score: float


def result_to_csv_row(pass_result: TrackedPassResult) -> str:
    base_csv_str = base_event_to_csv_row(pass_result.pass_event)

    pass_row = [
        pass_result.pass_result_type,
        pass_result.pass_.passer_point.x_meters,
        pass_result.pass_.passer_point.y_meters,
        pass_result.pass_.receiver_point.x_meters,
        pass_result.pass_.receiver_point.y_meters,
        pass_result.pass_.pass_speed_m_per_s,
        pass_result.score,
    ]

    return ",".join([str(elem) for elem in pass_row]) + "," + base_csv_str
