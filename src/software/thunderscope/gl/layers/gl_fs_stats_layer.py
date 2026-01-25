import os
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from dataclasses import dataclass
import software.python_bindings as tbots_cpp
from software.thunderscope.constants import RuntimeManagerConstants
import logging


@dataclass
class FSStats:
    self.num_yellow_cards: int = 0
    self.num_red_cards: int = 0
    self.num_scores: int = 0
    self.num_shots_on_net: int = 0
    self.latest_shot_on_net: tbots_cpp.Point = None
    self.num_enemy_shots_blocked: int = 0


class GlFSStatsLayer(GLLayer):
    def __init__(
        self,
        name: str,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
        record_enemy_stats: bool = False,
    ):
        super().__init__(name)

        self.friendly_colour_yellow = friendly_colour_yellow

        self.attacker_vis_buffer = ThreadSafeBuffer(buffer_size, AttackerVisualization)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee)

        self.stats = FSStats()

        self.record_enemy_stats = record_enemy_stats
        if self.record_enemy_stats:
            self.enemy_stats = FSStats()

    @override
    def refresh_graphics(self) -> None:
        """Refreshes the stats for the game so far"""
        attacker_vis_msg = self.attacker_vis_buffer.get(block=False, return_cached=True)

        if attacker_vis_msg.HasField("shot"):
            if attacker_vis_msg.shot.HasField("shot_origin"):
                shot_origin = attacker_vis_msg.shot.shot_origin
                shot_origin_point = tbots_cpp.Point(
                    shot_origin.x_meters, shot_origin.y_meters
                )

                if (
                    self.latest_shot_on_net is None
                    or self.latest_shot_on_net != shot_origin_point
                ):
                    self.latest_shot_on_net = shot_origin_point
                    self.num_shots_on_net += 1

        refree_msg = self.referee_buffer.get(block=False, return_cached=True)

        if refree_msg.HasField("yellow" if self.friendly_colour_yellow else "blue"):
            self._record_referee_stats(
                refree_msg.yellow if self.friendly_colour_yellow else refree_msg.blue,
                self.stats,
            )

        if self.record_enemy_stats and refree_msg.HasField(
            "blue" if self.friendly_colour_yellow else "yellow"
        ):
            self._record_referee_stats(
                refree_msg.blue if self.friendly_colour_yellow else refree_msg.yellow,
                self.enemy_stats,
            )

    def _record_referee_stats(self, team_info: TeamInfo, stats: FSStats) -> None:
        if team_info.HasField("score"):
            self.stats.num_scores = team_info.score

        if team_info.HasField("yellow_cards"):
            self.stats.num_yellow_cards = team_info.yellow_cards

        if team_info.HasField("red_cards"):
            self.stats.num_red_cards = team_info.red_cards

    def __del__(self):
        stats_file_name = (
            RuntimeManagerConstants.RUNTIME_ENEMY_STATS_FILE
            if self.friendly_colour_yellow
            else RuntimeManagerConstants.RUNTIME_FRIENDLY_STATS_FILE
        )

        self._write_stats_to_file(stats_file_name, self.stats)

        if self.record_enemy_stats:
            enemy_stats_file_name = (
                RuntimeManagerConstants.RUNTIME_FRIENDLY_FROM_ENEMY_STATS_FILE
                if self.friendly_colour_yellow
                else RuntimeManagerConstants.RUNTIME_ENEMY_FROM_FRIENDLY_STATS_FILE
            )

            self._write_stats_to_file(enemy_stats_file_name, self.enemy_stats)

    def _write_stats_to_file(self, stats: FSStats, file_name: str) -> None:
        file_path = os.path.join(
            RuntimeManagerConstants.RUNTIME_STATS_DIRECTORY_PATH, file_name
        )

        try:
            # formatted as key-value pairs in TOML
            stats_to_write = (
                f'{RuntimeManagerConstants.RUNTIME_STATS_SCORE_KEY} = "{stats.num_scores}"\n'
                f'{RuntimeManagerConstants.RUNTIME_STATS_RED_CARDS_KEY} = "{stats.num_red_cards}"\n'
                f'{RuntimeManagerConstants.RUNTIME_STATS_YELLOW_CARDS_KEY} = "{stats.num_yellow_cards}"\n'
                f'{RuntimeManagerConstants.RUNTIME_STATS_SHOTS_ON_NET} = "{stats.num_shots_on_net}"\n'
                f'{RuntimeManagerConstants.RUNTIME_STATS_SHOTS_BLOCKED} = "{stats.num_enemy_shots_blocked}"'
            )

            # create temp stats directory if it doesn't exist
            os.makedirs(os.path.dirname(file_path), exist_ok=True)

            with open(file_path, "w") as stats_file:
                stats_file.write(stats_to_write)
        except (FileNotFoundError, PermissionError):
            logging.warning(f"Failed to write TOML FS stats file at: {file_path}")
