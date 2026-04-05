from __future__ import annotations
from dataclasses import dataclass
from enum import StrEnum, auto
from proto.import_all_protos import *
from typing import Any, override
from software.evaluation.logs.log_interface import TimestampedEvalLog
from software.evaluation.logs.world_state_log import WorldStateLog


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
class EventLog(TimestampedEvalLog):
    """Represents a single event being tracked, where and for whom the event is, and the game state at the time of the event"""

    event_type: EventType
    from_team: Team
    for_team: Team
    world_state_log: WorldStateLog

    num_cols = TimestampedEvalLog.get_num_cols() + 2 + WorldStateLog.get_num_cols()

    @staticmethod
    def from_world(
        world_msg: World, event_type: EventType, from_team: Team, for_team: Team
    ) -> EventLog:
        """Creates an EventLog from a world protobuf message

        :param world_msg: the world object containing the state of the game
        :param event_type: the type of event being recorded
        :param from_team: the team that the event is coming from
        :param for_team: the team that the event is for
        :return: a fully populated EventLog including world state
        """
        world_state_log = WorldStateLog.from_world(world_msg=world_msg)

        return EventLog(
            timestamp=0,
            event_type=event_type,
            from_team=from_team,
            for_team=for_team,
            world_state_log=world_state_log,
        )

    @override    
    @classmethod
    def get_num_cols(cls) -> int:
        return EventLog.num_cols

    @override
    def to_array(self) -> list[Any]:
        return [
            self.event_type.value,
            self.from_team.value,
            self.for_team.value,
        ] + self.world_state_log.to_array()

    @staticmethod
    @override
    def from_csv_row(row_iter: Iterator[str]) -> EventLog | None:
        """Parses a full CSV row into an EventLog."""
        # 1. Handle Timestamp (inherited from TimestampedEvalLog)
        timestamp = float(next(row_iter))

        # 2. Parse Enums/Metadata
        event_type = EventType(next(row_iter))
        from_team = Team(next(row_iter))
        for_team = Team(next(row_iter))

        # 3. Delegate the remaining iterator to WorldStateLog
        world_state = WorldStateLog.from_csv_row(row_iter)

        if not world_state:
            return None

        return EventLog(
            timestamp=timestamp,
            event_type=event_type,
            from_team=from_team,
            for_team=for_team,
            world_state_log=world_state,
        )
