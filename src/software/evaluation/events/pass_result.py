from software.evaluation.events.event import IEvent
from software.evaluation.events.tracked_event import TrackedEvent, EventType
from dataclasses import dataclass
from enum import IntEnum
from proto.import_all_protos import *


class PassResultType(IntEnum):
    """Enum for the different types of pass results we want to log"""

    RESULT_1S = 1
    RESULT_5S = 5
    RESULT_10S = 10
    RESULT_20S = 20
    RESULT_30S = 30
    RESULT_0S = 0


@dataclass
class TrackedPassResult(IEvent):
    """Class representing a Pass we are tracking
    Contains the pass event itself within the nested TrackedEvent
    along with information about the pass itself and its score
    """
    pass_event: TrackedEvent
    pass_result_type: PassResultType
    pass_: Pass
    
    @staticmethod
    def from_world_and_pass(
      pass_: Pass, pass_result_type: PassResultType, world_msg: World, event_type: EventType, team: Team
    ) -> TrackedPassResult:
      pass_event = TrackedEvent.from_world(
        world_msg=world_msg, event_type=EventType.PASS, from_team=team, for_team=team
      )
      
      return TrackedPassResult(
        pass_event=pass_event,
        pass_=pass_,
        pass_result_type=pass_result_type
      )
      
    @override
    def get_timestamp(self) -> float:
       return self.pass_event.get_timestamp()
    
    @override
    def to_csv_row(self):
      pass_event_csv = self.pass_event.to_csv_row()
      
      pass_row = [
        self.pass_result_type,
        self.pass_.passer_point.x_meters,
        self.pass_.passer_point.y_meters,
        self.pass_.receiver_point.x_meters,
        self.pass_.receiver_point.y_meters,
        self.pass_.pass_speed_m_per_s
      ]
      
      return ",".join([str(elem) for elem in pass_row]) + "," + pass_event_csv