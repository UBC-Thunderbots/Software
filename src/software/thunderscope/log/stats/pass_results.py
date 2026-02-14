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

        self.attacker_vis_buffer = ThreadSafeBuffer(buffer_size, AttackerVisualization)
        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

        self.passes_so_far = 0
        self.pass_times_1s = []
        self.pass_times_5s = []
        self.pass_times_10s = []

        # the callback is called when there's a new pass
        self.kick_tracker = KickTracker(pass_callback=self.record_pass_taken)

    def refresh(self) -> None:
        """Refreshes the kick tracker so we stay up to date on new passes"""
        attacker_vis_msg = self.attacker_vis_buffer.get(block=False)

        world_msg = self.world_buffer.get(block=False, return_cached=True)
        world = tbots_cpp.World(world_msg)

        self.kick_tracker.refresh(
            attacker_vis_msg=attacker_vis_msg, ball=world.ball(), field=world.field()
        )

    def record_pass_taken(self, pass_taken: Pass):
        self.pass_times_1s.append((datetime.datetime.now(), pass_taken))
        self.passes_so_far += 1
