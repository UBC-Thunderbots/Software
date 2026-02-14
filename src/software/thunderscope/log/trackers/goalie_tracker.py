from software.thunderscope.log.trackers.tracker import Tracker
from typing import Callable, override
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
import software.python_bindings as tbots_cpp


class GoalieTracker(Tracker):
    def __init__(
        self,
        for_friendly: bool,
        callback: Callable[[bool, bool], None],
        buffer_size: int = 5,
    ):
        super().__init__(callback, buffer_size)

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

        self.for_friendly = for_friendly

        self.is_shot_incoming = False

    @override
    def set_proto_unix_io(self, proto_unix_io: ProtoUnixIO) -> None:
        super().set_proto_unix_io(
            proto_unix_io,
            [
                (World, self.world_buffer),
            ],
        )

    @override
    def refresh(self):
        """Refresh and update the callback with the latest referee information"""
        world_msg = self.world_buffer.get(block=False, return_cached=True)

        if not world_msg:
            return

        world = tbots_cpp.World(world_msg)

        latest_is_shot_incoming = self._is_goal_shot_incoming(
            world.ball(), world.field(), for_friendly=self.for_friendly
        )

        if self.callback:
            self.callback(latest_is_shot_incoming, self.is_shot_incoming)

        self.is_shot_incoming = latest_is_shot_incoming

    def _is_goal_shot_incoming(
        self, ball: tbots_cpp.Ball, field: tbots_cpp.Field, for_friendly: bool
    ) -> bool:
        """Follows similar logic to goalie_fsm.cpp
        Checks if the ball velocity intersects with the goal line given
        and if the ball is moving fast enough in the right direction
        and if the ball is in the correct half of the field
        and if the last possession was by the correct team
        to consider it a shot on goal

        :param ball: the current state of the ball
        :param field: the current state of the field
        :param for_friendly: if we should check for a goal shot on the friendly or enemy side
        :return:
        """
        ball_position = ball.position()
        ball_velocity = ball.velocity()
        ball_ray = tbots_cpp.Ray(ball_position, ball_velocity)
        goal_segment = (
            tbots_cpp.Segment(
                field.friendlyGoalpostPos()
                + tbots_cpp.Vector(0, -ROBOT_MAX_RADIUS_METERS),
                field.friendlyGoalpostNeg()
                + tbots_cpp.Vector(0, ROBOT_MAX_RADIUS_METERS),
            )
            if for_friendly
            else tbots_cpp.Segment(
                field.enemyGoalpostPos()
                + tbots_cpp.Vector(0, -ROBOT_MAX_RADIUS_METERS),
                field.enemyGoalpostNeg() + tbots_cpp.Vector(0, ROBOT_MAX_RADIUS_METERS),
            )
        )

        shot_incoming = (
            len(tbots_cpp.intersection(ball_ray, goal_segment)) != 0
            and ball_velocity.length() > self.MIN_SHOT_SPEED
            and (
                field.pointInFriendlyHalf(ball_position)
                if for_friendly
                else field.pointInEnemyHalf(ball_position)
            )
            and (ball_velocity.x() <= 0 if for_friendly else ball_velocity.x() >= 0)
        )

        return shot_incoming
