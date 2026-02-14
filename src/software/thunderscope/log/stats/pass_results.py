from software.thunderscope.log.trackers import (
    PossessionTracker,
    PassTracker,
    TrackerBuilder,
    RefereeTracker,
    GoalieTracker,
)
from software.thunderscope.proto_unix_io import ProtoUnixIO

class PassResultsTracker:
    """Class to track the results of any passes taken
    i.e looking at if our position in the game got better or worse
    after certain time intervals
    """

    def __init__(self, 
        proto_unix_io: ProtoUnixIO, friendly_colour_yellow: bool, buffer_size: int = 5):
        self.friendly_colour_yellow = friendly_colour_yellow

        self.tracker = (
            TrackerBuilder(proto_unix_io=proto_unix_io)
            .add_tracker(PassTracker, callback=self._update_shot_count)
            .add_tracker(PossessionTracker, callback=self._update_posession)
            .add_tracker(
                RefereeTracker,
                callback=self._update_referee_info_friendly,
                friendly_color_yellow=self.friendly_colour_yellow,
            )
        )

    def refresh(self) -> None:
        """Refreshes the kick tracker so we stay up to date on new passes"""
        pass

    def record_pass_taken(self, pass_taken: Pass):
        pass
