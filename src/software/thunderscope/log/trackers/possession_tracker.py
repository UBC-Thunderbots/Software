from software.thunderscope.log.trackers.tracker import Tracker
from typing import override, Callable
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from proto.import_all_protos import *
import software.python_bindings as tbots_cpp


class PossessionTracker(Tracker):
    def __init__(self, callback: Callable[[bool | None], None], buffer_size: int = 5):
        super().__init__(callback, buffer_size)

        self.world_buffer = ThreadSafeBuffer(self.buffer_size, World)

    @override
    def set_proto_unix_io(self, proto_unix_io: ProtoUnixIO) -> None:
        super().set_proto_unix_io(proto_unix_io, [
            (World, self.world_buffer),
        ])

    @override
    def refresh(self):
        """Refresh and update the callback with the latest ball possession"""
        world_msg = self.world_buffer.get(block=False, return_cached=True)

        if world_msg is None:
            return

        world = tbots_cpp.World(world_msg)

        if self.callback:
            self.callback(
                self._check_posession_for_friendly(
                    world.friendlyTeam(), world.enemyTeam(), world.ball().position()
                )
            )

    def _check_posession_for_friendly(
        self,
        friendly_team: tbots_cpp.Team,
        enemy_team: tbots_cpp.Team,
        ball_position: tbots_cpp.Point,
    ) -> bool | None:
        """Check for if the friendly team has possession of the ball
        True if they do, False if enemy team has possession, None if neither

        :param friendly_team: the friendly team
        :param enemy_team: the enemy team
        :param ball_position: the current ball position
        :return: True / False / None depending on which team has possession
        """
        if self._check_posession_for_team(friendly_team, ball_position):
            return True

        if self._check_posession_for_team(enemy_team, ball_position):
            return False

        return None

    def _check_posession_for_team(
        self, team: tbots_cpp.Team, ball_position: tbots_cpp.Point
    ) -> bool:
        """Check if the given team has possession of the ball

        :param team: the team to check
        :param ball_position: the current ball position
        :return: True if the team has possession, False otherwise
        """
        for robot in team.getAllRobots():
            if robot.isNearDribbler(ball_position):
                return True

        return False
