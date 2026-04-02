from dataclasses import dataclass
from enum import StrEnum, auto
from proto.import_all_protos import *
from typing import Any
from software.evaluation.events.event import IEvent
from software.evaluation.events.world_state import WorldState

class EventType(StrEnum):
    """Enum for the different types of events we want to track"""

    PASS = auto()
    SHOT_ON_GOAL = auto()
    ENEMY_SHOT_ON_GOAL = auto()
    SHOT_BLOCKED = auto()
    FRIENDLY_POSSESSION_START = auto()
    FRIENDLY_POSSESSION_END = auto()
    ENEMY_POSSESSION_START = auto()
    ENEMY_POSSESSION_END = auto()
    GAME_START = auto()
    GAME_END = auto()
    GOAL_SCORED = auto()
    YELLOW_CARD = auto()
    RED_CARD = auto()


class Team(StrEnum):
    """The teams present in the game"""

    BLUE = auto()
    YELLOW = auto()

@dataclass
class TrackedEvent(IEvent):
    """Represents a single event being tracked, where and for whom the event is, and the game state at the time of the event"""

    event_type: EventType
    from_team: Team
    for_team: Team
    world_state: WorldState
    
    @staticmethod
    def from_world(
        world_msg: World, event_type: EventType, from_team: Team, for_team: Team
    ) -> TrackedEvent:
        """
        Creates a TrackedEvent from a world protobuf message

        :param world_msg: the world object containing the state of the game
        :param event_type: the type of event being recorded
        :param from_team: the team that the event is coming from
        :param for_team: the team that the event is for
        :return: a fully populated TrackedEvent including ball and robot states
        """
        world_state = WorldState.from_world(world_msg=world_msg)

        return TrackedEvent(
            event_type=event_type,
            from_team=from_team,
            for_team=for_team,
            world_state=world_state
        )
        
    @override
    def get_timestamp(self) -> float:
        return self.world_state.get_timestamp()
    
    @override
    def to_csv_row(self):
        world_csv = self.world_state.to_csv_row()
        
        row = [self.event_type.value, self.from_team.value, self.for_team.value]
        
        return ",".join(row) + "," + world_csv