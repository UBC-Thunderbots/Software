import datetime

import software.python_bindings as tbots_cpp
from software.thunderscope.log.stats.trackers.kick_tracker import KickTracker
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class PassResultsTracker:
    """Class to track the results of any passes taken
    i.e looking at if our position in the game got better or worse
    after certain time intervals
    """

    def __init__(self, friendly_colour_yellow: bool, buffer_size: int = 5):
        self.friendly_colour_yellow = friendly_colour_yellow
        
        pass

    def refresh(self) -> None:
        """Refreshes the kick tracker so we stay up to date on new passes"""
        pass

    def record_pass_taken(self, pass_taken: Pass):
        pass
