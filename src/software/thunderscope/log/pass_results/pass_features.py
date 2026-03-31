from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from proto.import_all_protos import *
from software.thunderscope.constants import PassResultsConstants
import os
from software.thunderscope.log.pass_results.pass_event import pass_vis_to_csv_row
from software.thunderscope.log.trackers.tracked_event import Team

class PassSamples:
    """Class to track samples passes from the pass generator and their scores
    """

    EVENT_BUFFER_SIZE = 100

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
    ):
      """Initializes the pass samples tracker

      :param proto_unix_io: the proto unix io to use
      :param buffer_size: buffer size to use
      """
      self.buffer_size = buffer_size
      self.friendly_colour_yellow = friendly_colour_yellow

      self.proto_unix_io = proto_unix_io
      self.sampled_passes_buffer = ThreadSafeBuffer(self.buffer_size, PassFeatures)
      self.proto_unix_io.register_observer(PassFeatures, self.sampled_passes_buffer)
      
      self.events_file_path = os.path.join(
        PassResultsConstants.PASS_RESULTS_DIRECTORY_PATH,
        PassResultsConstants.PASS_RESULTS_FILE_NAME,
      )
      self.events_file_handle = None
      
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
      sampled_pass = self.sampled_passes_buffer.get(block=False, return_cached=True)

      if sampled_pass is None:
          return
        
      self._log_sampled_pass(sampled_pass)
      
    def _log_sampled_pass(self, sampled_pass: PassFeatures):
      if not self.events_file_handle:
        return

      try:  
          csv_row = pass_vis_to_csv_row(sampled_pass, team=Team.YELLOW if self.friendly_colour_yellow else Team.BLUE)
          self.events_file_handle.write(csv_row + "\n")
          self.events_file_handle.flush()

      except (IOError, FileNotFoundError, PermissionError):
          pass
    