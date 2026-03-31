from software.thunderscope.log.trackers.tracked_event import (
    TrackedEvent,
    event_to_csv_row as base_event_to_csv_row,
)
from dataclasses import dataclass
from enum import auto, StrEnum
from proto.import_all_protos import *


class PassEventType(StrEnum):
    """Enum for the different types of pass result events we want to log"""

    RESULT_1S = auto()
    RESULT_5S = auto()
    RESULT_10S = auto()


@dataclass
class TrackedPassEvent:
    tracked_event: TrackedEvent
    pass_event_type: PassEventType
    pass_: Pass
    score: float


def event_to_csv_row(pass_event: TrackedPassEvent) -> str:
    base_csv_str = base_event_to_csv_row(pass_event.tracked_event)

    pass_row = [
        pass_event.pass_event_type,
        pass_event.pass_.passer_point.x_meters,
        pass_event.pass_.passer_point.y_meters,
        pass_event.pass_.receiver_point.x_meters,
        pass_event.pass_.receiver_point.y_meters,
        pass_event.pass_.pass_speed_m_per_s,
        pass_event.score,
    ]

    return ",".join([str(elem) for elem in pass_row]) + "," + base_csv_str
