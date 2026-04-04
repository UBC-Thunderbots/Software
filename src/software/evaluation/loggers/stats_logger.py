import os

from software.evaluation.trackers import (
    PossessionTracker,
    ShotTracker,
    TrackerBuilder,
    RefereeTracker,
    GoalieTracker,
)
from dataclasses import dataclass
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.constants import RuntimeManagerConstants
from software.evaluation.logs.event_log import Team as TeamEnum, EventLog
import logging
from proto.import_all_protos import *
import queue


@dataclass
class FSStats:
    """Stats for how well a FullSystem is performing"""

    num_yellow_cards: int = 0
    num_red_cards: int = 0
    num_scores: int = 0

    num_shots_on_net: int = 0
    num_enemy_shots_blocked: int = 0


class StatsLogger:
    # From GoalieTacticConfig
    INCOMING_SHOT_MIN_VELOCITY = 0.2

    EVENT_BUFFER_SIZE = 100

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
        record_enemy_stats: bool = False,
    ):
        """Initializes the FullSystem Stats Tracker

        :param friendly_colour_yellow: if the friendly colour is yellow
        :param buffer_size: the buffer size for protocol buffers
        :param record_enemy_stats: if this should record both friendly and enemy stats or just friendly
        """
        self.friendly_colour_yellow = friendly_colour_yellow

        self.events_file_path = os.path.join(
            RuntimeManagerConstants.RUNTIME_EVENTS_DIRECTORY_PATH,
            RuntimeManagerConstants.RUNTIME_EVENTS_FILE,
        )
        # initialized in setup()
        self.events_file_handle = None

        self.event_queue = queue.Queue(self.EVENT_BUFFER_SIZE)

        self.tracker = (
            TrackerBuilder(
                proto_unix_io=proto_unix_io,
                from_team=(
                    TeamEnum.YELLOW if self.friendly_colour_yellow else TeamEnum.BLUE
                ),
                event_queue=self.event_queue,
                buffer_size=buffer_size,
            )
            .add_tracker(ShotTracker)
            .add_tracker(PossessionTracker)
            .add_tracker(
                RefereeTracker, friendly_color_yellow=self.friendly_colour_yellow
            )
            .add_tracker(GoalieTracker, for_friendly=True)
        )

        self.record_enemy_stats = record_enemy_stats
        if self.record_enemy_stats:
            self.enemy_tracker = (
                TrackerBuilder(
                    proto_unix_io=proto_unix_io,
                    from_team=(
                        TeamEnum.YELLOW
                        if self.friendly_colour_yellow
                        else TeamEnum.BLUE
                    ),
                    for_team=(
                        TeamEnum.BLUE
                        if self.friendly_colour_yellow
                        else TeamEnum.YELLOW
                    ),
                    event_queue=self.event_queue,
                    buffer_size=buffer_size,
                )
                .add_tracker(
                    RefereeTracker,
                    friendly_color_yellow=(not self.friendly_colour_yellow),
                )
                .add_tracker(GoalieTracker, for_friendly=False)
            )

    def refresh(self) -> None:
        """Refreshes the events for the game so far"""
        self.tracker.refresh()

        if not self.events_file_handle:
            return

        while not self.event_queue.empty():
            try:
                # Get item without blocking
                event = self.event_queue.get_nowait()

                self._write_event_to_file(event)
            except queue.Empty:
                return

    def __enter__(self):
        """Sets up the file resources for logging
        Creates any missing directories and stores the file handle
        """
        # create temp stats directory if it doesn't exist
        os.makedirs(os.path.dirname(self.events_file_path), exist_ok=True)

        self.events_file_handle = open(self.events_file_path, "a")

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Writes all logs back to file, and cleans up any created file resources after logging"""
        if self.events_file_handle:
            self.events_file_handle.flush()
            self.events_file_handle.close()

    def _write_event_to_file(self, event: EventLog) -> None:
        """Write the given stats to the given file

        :param event: the event to write
        """
        if not self.events_file_handle:
            return

        try:
            csv_row = event.to_csv_row()
            self.events_file_handle.write(csv_row + "\n")
            self.events_file_handle.flush()

        except (IOError, FileNotFoundError, PermissionError):
            logging.warning("Failed to write event to file")
