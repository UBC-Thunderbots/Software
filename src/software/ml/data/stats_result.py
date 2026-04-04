from dataclasses import dataclass, field
from software.evaluation.events.event import IEvalLog
from software.ml.data.result_interface import IResult
from software.evaluation.events.tracked_event import EventLog, Team, EventType
from typing import cast, Any, List
import software.python_bindings as tbots_cpp


@dataclass
class StatsResult(IResult):
    friendly_team: Team
    score: int = 0
    enemy_score: int = 0
    yellow_cards: int = 0
    red_cards: int = 0
    has_possession: bool | None = None
    shots_on_net: int = 0
    ball_in_enemy_half: bool = False
    passes: int = 0

    _field: tbots_cpp.Field = field(init=False, repr=False)

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
        ]

    def _is_in_enemy_half(self, point_to_check: Point):
        if not self._field:
            self._field = tbots_cpp.Field.createSSLDivisionBField()

        point = tbots_cpp.Point(point_to_check.x_meters, point_to_check.y_meters)

        return tbots_cpp.contains(self._field.enemyHalf(), point)

    @override
    def update_result(self, eval_log: IEvalLog) -> None:
        tracked_event = cast(EventLog, eval_log)

        for_friendly = tracked_event.for_team == self.friendly_team

        self.ball_in_enemy_half = self._is_in_enemy_half(
            tracked_event.world_state.ball_state.global_position
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

            case EventType.GAME_END:
                self.has_possession = False
                self.ball_in_enemy_half = False
