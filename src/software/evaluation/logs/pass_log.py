from __future__ import annotations
from software.evaluation.logs.log_interface import (
    TimestampedEvalLog,
    count_primitive_fields,
)
from software.evaluation.logs.event_log import EventLog, EventType
from dataclasses import dataclass
from enum import IntEnum
from proto.import_all_protos import *
import uuid
from typing import override


class PassLogType(IntEnum):
    """Enum for the different types of pass results we want to log

    Each value consists of a label and an interval in seconds
    """

    RESULT_1S = 1
    RESULT_5S = 5
    RESULT_10S = 10
    RESULT_20S = 20
    RESULT_30S = 30
    RESULT_0S = 0


@dataclass
class PassLog(TimestampedEvalLog):
    """Class representing a Pass we are tracking
    Contains the pass event itself within the nested EventLog
    along with information about the pass itself
    """

    # since the same pass is logged multiple times,
    # we assign an id to make it easier to correlate
    pass_id: uuid.UUID

    pass_event: EventLog
    pass_log_type: PassLogType
    pass_: Pass

    num_cols: int = (
        EventLog.get_num_cols() + 2 + count_primitive_fields(Pass.DESCRIPTOR)
    )

    def get_pass_start_point(self) -> list[float]:
        """Returns the current pass start position as a [float, float] array
        represnting x, y coordinates
        """
        return [self.pass_.passer_point.x_meters, self.pass_.passer_point.y_meters]

    def get_pass_end_point(self) -> list[float]:
        """Returns the current pass end position as a [float, float] array
        represnting x, y coordinates
        """
        return [self.pass_.receiver_point.x_meters, self.pass_.receiver_point.y_meters]

    def get_pass_speed(self) -> float:
        return self.pass_.pass_speed_m_per_s

    @classmethod
    @override
    def get_num_cols(cls) -> int:
        return PassLog.num_cols

    @staticmethod
    def from_world_and_pass(
        pass_id: uuid.UUID,
        pass_: Pass,
        pass_result_type: PassLogType,
        world_msg: World,
        team: Team,
    ) -> PassLog:
        pass_event = EventLog.from_world(
            world_msg=world_msg,
            event_type=EventType.PASS,
            from_team=team,
            for_team=team,
        )

        return PassLog(
            timestamp=pass_event.get_timestamp(),
            pass_id=pass_id,
            pass_event=pass_event,
            pass_=pass_,
            pass_log_type=pass_result_type,
        )

    @override
    def to_array(self):
        pass_event_array = self.pass_event.to_array()

        pass_array = (
            [self.pass_id, self.pass_log_type]
            + self.get_pass_start_point()
            + self.get_pass_end_point()
            + [self.pass_.pass_speed_m_per_s]
        )

        return pass_event_array + pass_array

    @staticmethod
    @override
    def from_csv_row(row_iter: Iterator[str]) -> PassLog | None:
        # 1. Delegate to EventLog to parse the event and world state
        pass_event = EventLog.from_csv_row(row_iter)

        if not pass_event:
            return None

        # 2. Parse Pass ID
        pass_id = uuid.UUID(next(row_iter))

        # 3. Parse PassLogType (Enum)
        pass_result_type = PassLogType(int(next(row_iter)))

        # 4. Parse the Pass message fields
        pass_msg = Pass(
            passer_point=Point(
                x_meters=float(next(row_iter)), y_meters=float(next(row_iter))
            ),
            receiver_point=Point(
                x_meters=float(next(row_iter)), y_meters=float(next(row_iter))
            ),
            pass_speed_m_per_s=float(next(row_iter)),
        )

        return PassLog(
            pass_id=pass_id,
            timestamp=pass_event.get_timestamp(),
            pass_event=pass_event,
            pass_log_type=pass_result_type,
            pass_=pass_msg,
        )
