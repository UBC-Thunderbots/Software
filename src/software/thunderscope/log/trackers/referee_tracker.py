from typing import override, Callable
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO


class RefereeTracker(Tracker):
    def __init__(
        self,
        friendly_color_yellow: bool,
        callback: Callable[[int, int, int], None],
        buffer_size: int = 5,
    ):
        super().__init__(callback, buffer_size)

        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee)

        self.friendly_color_yellow = friendly_color_yellow

    @override
    def set_proto_unix_io(self, proto_unix_io: ProtoUnixIO) -> None:
        super().set_proto_unix_io(
            proto_unix_io,
            [
                (Referee, self.referee_buffer),
            ],
        )

    @override
    def refresh(self):
        """Refresh and update the callback with the latest referee information"""
        refree_msg = self.referee_buffer.get(block=False, return_cached=True)

        if not refree_msg:
            return

        if refree_msg.HasField("yellow" if self.friendly_colour_yellow else "blue"):
            team_info = (
                refree_msg.yellow if self.friendly_colour_yellow else refree_msg.blue
            )

            num_goals = 0
            num_yellow_cards = 0
            num_red_cards = 0

            if team_info.HasField("score"):
                num_goals = team_info.score

            if team_info.HasField("yellow_cards"):
                num_yellow_cards = team_info.yellow_cards

            if team_info.HasField("red_cards"):
                num_red_cards = team_info.red_cards

            if self.callback:
                self.callback(num_goals, num_yellow_cards, num_red_cards)
