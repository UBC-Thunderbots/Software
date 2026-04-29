from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.constants import PassResultsConstants
import os
from proto.import_all_protos import *
from software.evaluation.logs.event_log import (
    Team as TeamEnum,
)
from software.evaluation.trackers.pass_log_tracker import (
    PassLogTracker,
)
from software.evaluation.logs.pass_log import (
    PassLog,
)
import queue


class PassLogger:
    """Class to track passes.

    i.e When a pass happens, we want to log the game state and the pass itself
    As well as the game state at certain intervals after the pass
    to see the outcomes of the pass
    """

    EVENT_BUFFER_SIZE = 100

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        friendly_colour_yellow: bool,
        out_file_name: str | None = None,
        buffer_size: int = 5,
    ):
        """Initializes the pass results tracker

        :param proto_unix_io: the proto unix io to use
        :param friendly_colour_yellow: if the friendly color is yellow or not
        :param out_file_name: name of file to write pass results to. 
                              If None, uses the value from constants
        :param buffer_size: buffer size to use
        """
        self.friendly_colour_yellow = friendly_colour_yellow

        # track pass results to this queue
        self.pass_result_queue = queue.Queue(self.EVENT_BUFFER_SIZE)

        self.pass_tracker = PassLogTracker(
            proto_unix_io=proto_unix_io,
            from_team=(
                TeamEnum.YELLOW if self.friendly_colour_yellow else TeamEnum.BLUE
            ),
            event_queue=self.pass_result_queue,
            buffer_size=buffer_size,
        )

        self.events_file_path = os.path.join(
            PassResultsConstants.PASS_RESULTS_DIRECTORY_PATH,
            PassResultsConstants.PASS_RESULTS_FILE_NAME if out_file_name is None else out_file_name,
        )
        self.events_file_handle = None

    def _get_team(self, is_friendly: bool) -> TeamEnum:
        """Gets the correct Team enum value for either the current friendly or enemy team

        :param is_friendly: whether to return the team for friendly or enemy
        :return: the corresponding Team Enum value
        """
        return (
            TeamEnum.YELLOW
            if (self.friendly_colour_yellow == is_friendly)
            else TeamEnum.BLUE
        )

    def __enter__(self):
        """Creates any missing directories and opens an append file handle"""
        # create temp stats directory if it doesn't exist
        os.makedirs(os.path.dirname(self.events_file_path), exist_ok=True)

        self.events_file_handle = open(self.events_file_path, "a")

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Flushes content and closes the log file"""
        if self.events_file_handle:
            self.events_file_handle.flush()
            self.events_file_handle.close()

    def refresh(self) -> None:
        """Refreshes the tracker so we stay up to date on new passes
        and checks to see if any passes are older than their interval
        """
        self.pass_tracker.refresh()

        if not self.events_file_handle:
            return

        while not self.pass_result_queue.empty():
            try:
                # Get item without blocking
                pass_log = self.pass_result_queue.get_nowait()

                self._log_pass(pass_log)
            except queue.Empty:
                return

    def _log_pass(self, pass_log: PassLog):
        """Logs a single pass to file

        :param result: the result to log
        """
        if not self.events_file_handle:
            return

        try:
            csv_row = pass_log.to_csv_row()
            self.events_file_handle.write(csv_row + "\n")
            self.events_file_handle.flush()

        except (IOError, FileNotFoundError, PermissionError):
            pass
