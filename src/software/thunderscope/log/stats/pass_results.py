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
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
import software.python_bindings as tbots_cpp


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

    PASS_RESULTS_TEMPLATE = (
        "{pass_start_x},{pass_start_y},"
        "{pass_end_x},{pass_end_y},"
        "{speed},"
        "{score}\n"
    )

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
    ):
        """Initializes the pass resuidxlts tracker

        :param proto_unix_io: the proto unix io to use
        :param friendly_colour_yellow: if the friendly color is yellow or not
        :param buffer_size: buffer size to use
        """
        self.friendly_colour_yellow = friendly_colour_yellow

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        proto_unix_io.register_observer(World, self.world_buffer)

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
            interval: [] for interval in PassResultsConstants.INTERVALS_S
        }

        self.is_friendly_possession: bool | None = False
        self.friendly_score = 0
        self.enemy_score = 0

        self.pass_results_file_map = {}

        self.world = None

    def setup(self):
        """Creates the relevant directories and a csv file for each of the
        intervals in INTERVALS
        """
        pass_results_dir = PassResultsConstants.PASS_RESULTS_DIRECTORY_PATH

        # create all directories in path if they doesn't exist
        os.makedirs(os.path.dirname(pass_results_dir), exist_ok=True)

        for interval in PassResultsConstants.INTERVALS_S:
            file_path = os.path.join(
                pass_results_dir,
                PassResultsConstants.PASS_RESULTS_FILE_NAME_TEMPLATE.format(
                    interval=interval
                ),
            )

            is_new_file = not os.path.exists(file_path)

            self.pass_results_file_map[interval] = open(
                file_path,
                "a",
            )

            # write the headers first if the file doesn't already exist
            if is_new_file:
                self.pass_results_file_map[interval].write(
                    self._get_pass_result_headers()
                )
                self.pass_results_file_map[interval].flush()

    def cleanup(self):
        """Flushes content and closes all the files for all intervals"""
        for interval in PassResultsConstants.INTERVALS_S:
            if self.pass_results_file_map[interval]:
                self.pass_results_file_map[interval].flush()
                self.pass_results_file_map[interval].close()

    def _update_friendly_possession(self, is_friendly_possession: bool | None) -> None:
        self.is_friendly_possession = is_friendly_possession

    def _update_scores_friendly(self, friendly_score: int, *_) -> None:
        self.friendly_score = friendly_score

    def _update_scores_enemy(self, enemy_score: int, *_) -> None:
        self.enemy_score = enemy_score

    def _add_pass_timestamp(self, pass_: Pass) -> None:
        """Adds the given pass, the current timestamp, and the current scores to the lowest interval's list
        :param pass_: the pass to add
        """
        # TODO: use world timestamp time instead of datetime time
        self.pass_times_map[PassResultsConstants.INTERVALS_S[0]].append(
            PassLog(
                pass_=pass_,
                timestamp=datetime.now(),
                friendly_score=self.friendly_score,
                enemy_score=self.enemy_score,
            )
        )

    def refresh(self) -> None:
        """Refreshes the tracker so we stay up to date on new passes
        and checks to see if any passes are older than their interval
        """
        world_msg = self.world_buffer.get(block=False, return_cached=True)

        if world_msg:
            self.world = tbots_cpp.World(world_msg)

        self.tracker.refresh()

        self._update_pass_timestamps()

    def _log_pass_result(self, logged_pass: PassLog, interval_s: int) -> None:
        """For an already recorded pass, calculates and logs its score for the given interval
        i.e after <interval> seconds following the pass

        :param logged_pass: a pass that already occurred that we want to find the score for
        :param interval_s: how long (in seconds) it has been after the pass
        """
        pass_score = self._get_pass_score(logged_pass)

        self._log_pass_result_to_file(
            file=self.pass_results_file_map[interval_s],
            pass_=logged_pass.pass_,
            score=pass_score,
        )

    def _get_pass_score(self, logged_pass: PassLog) -> int:
        """For the given logged pass, get the score based on the current game state
        If the friendly / enemy scores at the time of pass are different
        Or if possession has changed, return the corresponding score
        Else, return the neutral score

        :param logged_pass: the pass to score
        :return: a single integer score for the pass
        """
        if self.friendly_score > logged_pass.friendly_score:
            return PassResultsConstants.FRIENDLY_GOAL_SCORE

        if self.enemy_score > logged_pass.enemy_score:
            return PassResultsConstants.ENEMY_GOAL_SCORE

        if self.is_friendly_possession:
            return PassResultsConstants.FRIENDLY_POSSESSION_SCORE
        elif self.is_friendly_possession is False:
            return PassResultsConstants.ENEMY_POSSESSION_SCORE

        return PassResultsConstants.NEUTRAL_SCORE

    def _get_pass_result_headers(self):
        return self.PASS_RESULTS_TEMPLATE.replace("{", "").replace("}", "")

    def _log_pass_result_to_file(self, file, pass_: Pass, score: int) -> None:
        """Logs a single pass's result to the given file handle

        :param file: the file handle to write to
        :param pass_: the pass to log
        :param score: the score for the given pass
        """
        pass_result_string = self.PASS_RESULTS_TEMPLATE.format(
            pass_start_x=pass_.passer_point.x_meters,
            pass_start_y=pass_.passer_point.y_meters,
            pass_end_x=pass_.receiver_point.x_meters,
            pass_end_y=pass_.receiver_point.y_meters,
            speed=pass_.pass_speed_m_per_s,
            score=score,
        )

        file.write(pass_result_string)
        file.flush()

    def _update_pass_timestamps(self):
        """For all currently logged passes, check if the interval they belong to has passed
        If so, log their score to the corresponding file
        And move them to the next interval if exists
        """
        for idx, interval in enumerate(PassResultsConstants.INTERVALS_S):
            pass_timestamps = self.pass_times_map[interval]

            # TODO: use world timestamp time instead of datetime time
            time_now = datetime.now()

            while (
                pass_timestamps
                and (time_now - pass_timestamps[0].timestamp).total_seconds() > interval
            ):
                pass_with_timestamp = pass_timestamps.pop(0)
                print(
                    f"Pass {pass_with_timestamp.pass_} is older than interval {interval}"
                )

                self._log_pass_result(pass_with_timestamp, interval)

                if idx < len(PassResultsConstants.INTERVALS_S) - 1:
                    self.pass_times_map[
                        PassResultsConstants.INTERVALS_S[idx + 1]
                    ].append(pass_with_timestamp)
