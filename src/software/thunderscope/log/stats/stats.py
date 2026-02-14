from software.thunderscope.log.stats.fullsystem_stats import FullSystemStats
from software.thunderscope.proto_unix_io import ProtoUnixIO
from proto.visualization_pb2 import AttackerVisualization
from proto.import_all_protos import *


class Stats:
    """This class is a wrapper for all Statistics related operations we want to do with FullSystem or Thunderscope"""

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        friendly_color_yellow: bool = False,
        record_enemy_stats: bool = False,
        buffer_size: int = 5,
    ):
        self.proto_unix_io = proto_unix_io

        self.fs_stats = FullSystemStats(
            friendly_colour_yellow=friendly_color_yellow,
            buffer_size=buffer_size,
            record_enemy_stats=record_enemy_stats,
        )

        for arg in [
            (AttackerVisualization, self.fs_stats.attacker_vis_buffer),
            (Referee, self.fs_stats.referee_buffer),
            (World, self.fs_stats.world_buffer),
        ]:
            proto_unix_io.register_observer(*arg)

    def refresh(self):
        self.fs_stats.refresh()
