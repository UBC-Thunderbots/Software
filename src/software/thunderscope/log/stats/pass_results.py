from software.thunderscope.log.trackers import (
    PossessionTracker,
    PassTracker,
    TrackerBuilder,
    RefereeTracker,
)
from software.thunderscope.proto_unix_io import ProtoUnixIO
from datetime import datetime
from dataclasses import dataclass
from software.thunderscope.constants import PassResultsConstants
import os


@dataclass
class PassLog:
    pass_: Pass
    timestamp: datetime
    friendly_score: int
    enemy_score: int


class PassResultsTracker:
    """Class to track the results of any passes taken
    i.e looking at if our position in the game got better or worse
    after certain time intervals
    """

    FRIENDLY_GOAL_SCORE = 10
    ENEMY_GOAL_SCORE = -FRIENDLY_GOAL_SCORE
    FRIENDLY_POSSESSION_SCORE = 2
    ENEMY_POSSESSION_SCORE = -FRIENDLY_POSSESSION_SCORE
    NEUTRAL_SCORE = 0

    # the time intervals to log results for after each pass
    # so after a pass, wait X seconds and then log game state
    INTERVALS = [1, 5, 10]

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
    ):
        self.friendly_colour_yellow = friendly_colour_yellow

        self.tracker = (
            TrackerBuilder(proto_unix_io=proto_unix_io)
            .add_tracker(PassTracker, callback=self._add_pass_timestamp)
            .add_tracker(PossessionTracker, callback=self._update_friendly_possession)
            .add_tracker(
                RefereeTracker,
                callback=self._update_scores_friendly,
                friendly_color_yellow=self.friendly_colour_yellow,
            )
            .add_tracker(
                RefereeTracker,
                callback=self._update_scores_friendly,
                friendly_color_yellow=(not self.friendly_colour_yellow),
            )
        )

        self.pass_times_map: dict[int, list[PassLog]] = {
            interval: [] for interval in self.INTERVALS
        }

        self.is_friendly_possession: bool | None = False
        self.friendly_score = 0
        self.enemy_score = 0

        self.pass_results_file_map = {}

    def setup(self):
        pass_results_dir = PassResultsConstants.PASS_RESULTS_DIRECTORY_PATH

        # create all directories in path if they doesn't exist
        os.makedirs(os.path.dirname(pass_results_dir), exist_ok=True)

        for interval in self.INTERVALS:
            self.pass_results_file_map[interval] = open(
                os.path.join(
                    pass_results_dir,
                    PassResultsConstants.PASS_RESULTS_FILE_NAME_TEMPLATE.format(
                        interval=interval
                    ),
                )
            )

    def cleanup(self):
        for interval in self.INTERVALS:
            if self.pass_results_file_map[interval]:
                self.pass_results_file_map[interval].close()

    def _update_friendly_possession(self, is_friendly_possession: bool | None) -> None:
        self.is_friendly_possession = is_friendly_possession

    def _update_scores_friendly(self, friendly_score: int, *_) -> None:
        self.friendly_score = friendly_score

    def _update_scores_enemy(self, enemy_score: int, *_) -> None:
        self.enemy_score = enemy_score

    def _add_pass_timestamp(self, pass_: Pass) -> None:
        self.pass_times_map[self.INTERVALS[0]].append(
            PassLog(
                pass_=pass_,
                timestamp=datetime.now(),
                friendly_score=self.friendly_score,
                enemy_score=self.enemy_score,
            )
        )

    def refresh(self) -> None:
        """Refreshes the kick tracker so we stay up to date on new passes"""
        self.tracker.refresh()

    def _log_pass_result(self, logged_pass: PassLog, interval: int) -> None:
        pass_score = self._get_pass_score(logged_pass)

        self._log_pass_result_to_file(
            file=self.pass_results_file_map[interval],
            pass_=logged_pass.pass_,
            score=pass_score,
        )

    def _get_pass_score(self, logged_pass: PassLog) -> int:
        if self.friendly_score > logged_pass.friendly_score:
            return self.FRIENDLY_GOAL_SCORE

        if self.enemy_score > logged_pass.enemy_score:
            return self.ENEMY_GOAL_SCORE

        if self.is_friendly_possession:
            return self.FRIENDLY_POSSESSION_SCORE
        elif self.is_friendly_possession is False:
            return self.ENEMY_POSSESSION_SCORE

        return self.NEUTRAL_SCORE

    def _log_pass_result_to_file(self, file, pass_: Pass, score: int) -> None:
        pass_result_string = PassResultsConstants.PASS_RESULTS_TEMPLATE.format(
            pass_start_x=pass_.passer_point.x_meters,
            pass_start_y=pass_.passer_point.y_meters,
            pass_end_x=pass_.receiver_point.x_meters,
            pass_end_y=pass_.receiver_point.y_meters,
            speed=pass_.pass_speed_m_per_s,
            score=score,
        )

        file.write(pass_result_string)

    def _update_pass_timestamps(self):
        for idx, interval in enumerate(self.INTERVALS):
            pass_timestamps = self.pass_times_map[idx]

            time_now = datetime.now()

            while (
                pass_timestamps
                and (time_now - pass_timestamps[0].timestamp).total_seconds() > interval
            ):
                pass_with_timestamp = pass_timestamps.pop(0)

                self._log_pass_result(pass_with_timestamp, interval)

                if idx < len(self.INTERVALS) - 1:
                    self.pass_times_map[self.INTERVALS[idx + 1]].append(
                        pass_with_timestamp
                    )
