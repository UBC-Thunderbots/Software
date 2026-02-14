import os

from software.thunderscope.log.trackers import (
    PossessionTracker,
    ShotTracker,
    TrackerBuilder,
    RefereeTracker,
    GoalieTracker,
)
from dataclasses import dataclass
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.constants import RuntimeManagerConstants
import logging


@dataclass
class FSStats:
    """Stats for how well a FullSystem is performing"""

    num_yellow_cards: int = 0
    num_red_cards: int = 0
    num_scores: int = 0

    num_shots_on_net: int = 0
    num_enemy_shots_blocked: int = 0


class FullSystemStats:
    # From GoalieTacticConfig
    INCOMING_SHOT_MIN_VELOCITY = 0.2

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

        # True if friendly had the last possession, False if enemy
        # None if neither
        self.last_possession_friendly: bool | None = None

        self.stats = FSStats()

        # the python __del__ destructor isn't called reliably
        # so printing this at the start instead
        print(f"\n\n\n##### Writing FS Stats to {self._get_stats_file()}#####\n\n\n")

        self.tracker = (
            TrackerBuilder(proto_unix_io=proto_unix_io)
            .add_tracker(ShotTracker, callback=self._update_shot_count)
            .add_tracker(PossessionTracker, callback=self._update_posession)
            .add_tracker(
                RefereeTracker,
                callback=self._update_referee_info_friendly,
                friendly_color_yellow=self.friendly_colour_yellow,
            )
            .add_tracker(
                GoalieTracker,
                callback=self._update_goalie_shot_friendly,
                for_friendly=True,
            )
        )

        self.record_enemy_stats = record_enemy_stats
        if self.record_enemy_stats:
            self.enemy_stats = FSStats()
            self.tracker = self.tracker.add_tracker(
                RefereeTracker,
                callback=self._update_referee_info_enemy,
                friendly_color_yellow=(not self.friendly_colour_yellow),
            ).add_tracker(
                GoalieTracker,
                callback=self._update_goalie_shot_enemy,
                for_friendly=False,
            )
            print(
                f"\n\n\n##### Writing Enemy FS Stats to {self._get_enemy_stats_file()}#####\n\n\n"
            )

    def refresh(self) -> None:
        """Refreshes the stats for the game so far"""
        self.tracker.refresh()

        self._flush_stats()

    def _update_shot_count(self, _: Shot):
        self.stats.num_shots_on_net += 1

    def _update_posession(self, friendly_posession: bool | None):
        self.last_possession_friendly = not friendly_posession

    def _update_referee_info_friendly(
        self, num_goals: int, num_yellow_cards: int, num_red_cards: int
    ) -> None:
        self.stats.num_scores = num_goals
        self.stats.num_yellow_cards = num_yellow_cards
        self.stats.num_red_cards = num_red_cards

    def _update_referee_info_enemy(
        self, num_goals: int, num_yellow_cards: int, num_red_cards: int
    ) -> None:
        self.enemy_stats.num_scores = num_goals
        self.enemy_stats.num_yellow_cards = num_yellow_cards
        self.enemy_stats.num_red_cards = num_red_cards

    def _update_goalie_shot_friendly(
        self, is_shot_incoming: bool, last_shot_incoming: bool
    ) -> None:
        if not is_shot_incoming and last_shot_incoming:
            self.stats.num_enemy_shots_blocked += 1

        if is_shot_incoming and not last_shot_incoming:
            self.enemy_stats.num_shots_on_net += 1

    def _update_goalie_shot_enemy(
        self, is_shot_incoming: bool, last_shot_incoming: bool
    ) -> None:
        if not is_shot_incoming and last_shot_incoming:
            self.enemy_stats.num_enemy_shots_blocked += 1

    def _get_stats_file(self):
        return os.path.join(
            RuntimeManagerConstants.RUNTIME_STATS_DIRECTORY_PATH,
            RuntimeManagerConstants.RUNTIME_ENEMY_STATS_FILE
            if self.friendly_colour_yellow
            else RuntimeManagerConstants.RUNTIME_FRIENDLY_STATS_FILE,
        )

    def _get_enemy_stats_file(self):
        return os.path.join(
            RuntimeManagerConstants.RUNTIME_STATS_DIRECTORY_PATH,
            RuntimeManagerConstants.RUNTIME_FRIENDLY_FROM_ENEMY_STATS_FILE
            if self.friendly_colour_yellow
            else RuntimeManagerConstants.RUNTIME_ENEMY_FROM_FRIENDLY_STATS_FILE,
        )

    def _flush_stats(self):
        """Write the current stats to disk"""
        stats_file_name = self._get_stats_file()

        self._write_stats_to_file(self.stats, stats_file_name)

        if self.record_enemy_stats:
            enemy_stats_file_name = self._get_enemy_stats_file()

            self._write_stats_to_file(self.enemy_stats, enemy_stats_file_name)

    def _write_stats_to_file(self, stats: FSStats, file_path: str) -> None:
        """Write the given stats to the given file

        :param stats: the stats to write
        :param file_path: the file to write to
        """
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
