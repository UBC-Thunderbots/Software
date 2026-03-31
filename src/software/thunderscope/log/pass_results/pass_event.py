from software.thunderscope.log.trackers.tracked_event import (
    TrackedEvent, EventType,
    event_to_csv_row as base_event_to_csv_row,
    get_event_from_world, Team
)
from dataclasses import dataclass
from enum import auto, StrEnum
from proto.import_all_protos import *


class PassResultType(StrEnum):
    """Enum for the different types of pass results we want to log"""
    RESULT_PRE = auto()

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

def pass_vis_to_csv_row(pass_features: PassFeatures, team: Team) -> str:
    pass_event = get_event_from_world(
        world_msg=pass_features.world_state,
        event_type=EventType.PASS,
        from_team=team,
        for_team=team
    )
    
    pass_result = TrackedPassResult(
        pass_event=pass_event,
        pass_result_type=PassResultType.RESULT_PRE,
        pass_=pass_features.pass_,
        score=pass_features.score
    )
    
    return result_to_csv_row(pass_result)
    

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
