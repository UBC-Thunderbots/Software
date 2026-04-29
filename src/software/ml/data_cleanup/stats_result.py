from dataclasses import dataclass, field
from software.evaluation.logs.log_interface import IEvalLog
from software.ml.data_cleanup.result_interface import IResult
from software.evaluation.logs.event_log import EventLog, Team, EventType
from software.evaluation.logs.log_interface import IEvalLog
from typing import cast, Any, List, override
import software.python_bindings as tbots_cpp
from proto.import_all_protos import *


@dataclass
class StatsResult(IResult, IEvalLog):
    friendly_team: Team
    score: int = 0
    enemy_score: int = 0
    yellow_cards: int = 0
    red_cards: int = 0
    has_possession: bool | None = None
    shots_on_net: int = 0
    ball_in_enemy_half: bool = False
    passes: int = 0
    blocked_enemy_shots: int = 0

    _field: tbots_cpp.Field = field(init=False, repr=False, default=None)

    @classmethod
    def get_num_cols(cls) -> int:
        """Returns the number of elements in the to_array output (9 fields)"""
        return 9

    def to_array(self) -> List[Any]:
        return [
            self.score,
            self.enemy_score,
            self.yellow_cards,
            self.red_cards,
            self.has_possession,
            self.shots_on_net,
            self.ball_in_enemy_half,
            self.passes,
            self.blocked_enemy_shots,
        ]

    def _is_in_enemy_half(self, point_to_check: List[float]):
        if self._field is None:
            self._field = tbots_cpp.Field.createSSLDivisionBField()

        point = tbots_cpp.Point(point_to_check[0], point_to_check[1])

        return tbots_cpp.contains(self._field.enemyHalf(), point)

    @override
    def update_result(self, eval_log: IEvalLog) -> None:
        tracked_event = cast(EventLog, eval_log)

        for_friendly = tracked_event.for_team == self.friendly_team

        self.ball_in_enemy_half = self._is_in_enemy_half(
            tracked_event.world_state_log.ball_state.get_position()
        )

        match tracked_event.event_type:
            case EventType.PASS if for_friendly:
                self.passes += 1

            case EventType.SHOT_ON_GOAL if for_friendly:
                self.shots_on_net += 1

            case EventType.GOAL_SCORED:
                if for_friendly:
                    self.score += 1
                else:
                    self.enemy_score += 1

            case EventType.SHOT_BLOCKED if for_friendly:
                self.blocked_enemy_shots += 1

            case EventType.YELLOW_CARD if for_friendly:
                self.yellow_cards += 1

            case EventType.RED_CARD if for_friendly:
                self.red_cards += 1

            case EventType.FRIENDLY_POSSESSION_START:
                self.has_possession = True
            case EventType.FRIENDLY_POSSESSION_END:
                self.has_possession = None

            case EventType.ENEMY_POSSESSION_START:
                self.has_possession = False
            case EventType.ENEMY_POSSESSION_END:
                self.has_possession = None

            case EventType.GAME_END | EventType.GAME_START:
                self.has_possession = None
                self.ball_in_enemy_half = False
                self.passes = 0
                self.score = 0
                self.enemy_score = 0
                self.yellow_cards = 0
                self.red_cards = 0
                self.shots_on_net = 0
                self.blocked_enemy_shots = 0
    
    @staticmethod
    def _parse_bool(val: str) -> bool | None:
        val = val.strip().lower()
        if val in ("none", ""): return None
        return val == "true"

    @staticmethod
    def from_csv_row(row_iter: Iterator[str], friendly_team: Team) -> "StatsResult":
        """
        Converts a CSV row back into a StatsResult instance.
        """

        return StatsResult(
            friendly_team=friendly_team,
            score=int(next(row_iter)),
            enemy_score=int(next(row_iter)),
            yellow_cards=int(next(row_iter)),
            red_cards=int(next(row_iter)),
            # Handling 'None' string or empty values for Optional bool
            has_possession=cls._parse_bool(next(row_iter)),
            shots_on_net=int(next(row_iter)),
            ball_in_enemy_half=next(row_iter).lower() == 'true',
            passes=int(next(row_iter)),
            blocked_enemy_shots=int(next(row_iter))
        )
