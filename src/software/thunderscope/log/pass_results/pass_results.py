from software.thunderscope.log.trackers import (
    PossessionTracker,
    TrackerBuilder,
    RefereeTracker,
)
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.constants import PassResultsConstants
import os
from proto.import_all_protos import *
from software.thunderscope.log.trackers.tracked_event import (
    Team as TeamEnum,
    EventType,
    TrackedEvent,
)
from software.thunderscope.log.pass_results.pass_event_tracker import (
    PassEventTracker,
    event_to_csv_row,
)
from typing import Callable, Any
import queue


class PassData:
    """Class to track pass data:

    1. Considered passes that we sampled and their scores
    2. Game results for actually taken passes after specific intervals
    """

    EVENT_BUFFER_SIZE = 100

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
    ):
        """Initializes the pass results tracker

        :param proto_unix_io: the proto unix io to use
        :param friendly_colour_yellow: if the friendly color is yellow or not
        :param buffer_size: buffer size to use
        """
        self.friendly_colour_yellow = friendly_colour_yellow

        # track pass results to this queuea
        self.pass_result_queue = queue.Queue(self.EVENT_BUFFER_SIZE)

        self.pass_tracker = PassEventTracker(
            proto_unix_io=proto_unix_io,
            from_team=(
                TeamEnum.YELLOW if self.friendly_colour_yellow else TeamEnum.BLUE
            ),
            event_queue=self.pass_result_queue,
            buffer_size=buffer_size,
        )

        # track our overall performance to this queue
        self.performance_queue = queue.Queue(self.EVENT_BUFFER_SIZE)

        self.performance_tracker = (
            TrackerBuilder(
                proto_unix_io=proto_unix_io,
                from_team=self._get_team(True),
                event_queue=self.performance_queue,
                buffer_size=buffer_size,
            )
            .add_tracker(PossessionTracker)
            .add_tracker(
                RefereeTracker,
                friendly_color_yellow=self.friendly_colour_yellow,
            )
        )

        # track the enemy's performance to the same queue
        self.enemy_performance_tracker = TrackerBuilder(
            proto_unix_io=proto_unix_io,
            from_team=self._get_team(False),
            event_queue=self.performance_queue,
            buffer_size=buffer_size,
        ).add_tracker(
            RefereeTracker,
            friendly_color_yellow=(not self.friendly_colour_yellow),
        )

        self.events_file_path = os.path.join(
            PassResultsConstants.PASS_RESULTS_DIRECTORY_PATH,
            PassResultsConstants.PASS_RESULTS_FILE_NAME,
        )
        self.events_file_handle = None

    def _get_team(is_friendly: bool) -> TeamEnum:
        return (
            TeamEnum.YELLOW
            if (self.friendly_colour_yellow == is_friendly)
            else TeamEnum.BLUE
        )

    def setup(self):
        """Creates the relevant directories and a csv file for each of the
        intervals in INTERVALS
        """
        # create temp stats directory if it doesn't exist
        os.makedirs(os.path.dirname(self.events_file_path), exist_ok=True)

        self.events_file_handle = open(self.events_file_path, "a")

    def cleanup(self):
        """Flushes content and closes all the files for all intervals"""
        if self.events_file_handle:
            self.events_file_handle.flush()
            self.events_file_handle.close()

    def refresh(self) -> None:
        """Refreshes the tracker so we stay up to date on new passes
        and checks to see if any passes are older than their interval
        """
        self.pass_tracker.refresh()
        self.performance_tracker.refresh()

        if not self.events_file_handle:
            return

        self._check_queue(self.performance_queue, self._handle_performance_event)
        self._check_queue(self.pass_result_queue, self.self._log_pass_result)

    def _check_queue(queue: queue.Queue, callback: Callable[[Any], None]) -> None:
        while not queue.empty():
            try:
                # Get item without blocking
                event = queue.get_nowait()

                self.callback(event)
            except queue.Empty:
                return

    def _handle_performance_event(event: TrackedEvent) -> None:
        from_friendly = event.from_team == self._get_team(True)

        if event.event_type == EventType.GOAL_SCORED:
            if from_friendly:
                self.curr_performance += PassResultsConstants.FRIENDLY_GOAL_SCORE
            else:
                self.curr_performance += PassResultsConstants.ENEMY_GOAL_SCORE
        elif event.event_type == EventType.YELLOW_CARD and from_friendly:
            self.curr_performance += PassResultsConstants.FRIENDLY_YELLOW_CARD_SCORE
        elif event.event_type == EventType.RED_CARD and from_friendly:
            self.curr_performance += PassResultsConstants.FRIENDLY_RED_CARD_SCORE
        elif event.event_type == EventType.FRIENDLY_POSSESSION_START:
            self.curr_performance += PassResultsConstants.FRIENDLY_POSSESSION_SCORE
        elif event.event_type == EventType.FRIENDLY_POSSESSION_END:
            self.curr_performance -= PassResultsConstants.FRIENDLY_POSSESSION_SCORE
        elif event.event_type == EventType.ENEMY_POSSESSION_START:
            self.curr_performance += PassResultsConstants.ENEMY_POSSESSION_SCORE
        elif event.event_type == EventType.ENEMY_POSSESSION_END:
            self.curr_performance -= PassResultsConstants.ENEMY_POSSESSION_SCORE

        self.pass_tracker.set_score(self.curr_performance)

    def _log_pass_result(event: TrackedPassEvent):
        if not self.events_file_handle:
            return

        try:
            csv_row = event_to_csv_row(event=event)
            self.events_file_handle.write(csv_row + "\n")
            self.events_file_handle.flush()

        except (IOError, FileNotFoundError, PermissionError):
            logging.warning("Failed to write event to file")
