from software.thunderscope.log.trackers.tracker import Tracker
from typing import Callable, override
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
import software.python_bindings as tbots_cpp
from software.py_constants import ROBOT_MAX_RADIUS_METERS
from software.thunderscope.log.trackers.tracked_event import EventType

class GoalieTracker(Tracker):
    """Tracker to track new shots on goal"""

    # tune these values to reduce noise in what is considered a kick
    # higher values exclude noise such as dribbling or small movements of the ball
    # but can exclude real kicks
    MIN_SHOT_SPEED = 2.0

    def __init__(self, proto_unix_io: ProtoUnixIO, out_file_path: str, for_friendly: bool, buffer_size: int = 5):
        """Initializes the Goalie tracker

        :param for_friendly: if we should track shots on goal for the friendly or enemy team
        :param proto_unix_io: the proto unix io to get the game state from
        :param out_file_path: the file to write tracked events to               
        :param buffer_size: buffer size for the tracker's io
        """
        super().__init__(proto_unix_io, out_file_path, buffer_size)

        self.for_friendly = for_friendly

        self.is_shot_incoming = False

    @override
    def refresh(self):
        """Refresh and log any new shots on goal"""
        super().refresh()
        
        if self.cached_world is None:
            return

        latest_is_shot_incoming = self._is_goal_shot_incoming(
            self.cached_world.ball(), self.cached_world.field(), for_friendly=self.for_friendly
        )

        self._log_incoming_shots(latest_is_shot_incoming)

        self.is_shot_incoming = latest_is_shot_incoming
        
    def _log_incoming_shots(self, new_shot_incoming):
        event_type = None
        
        if not new_shot_incoming and self.is_shot_incoming:
            event_type = EventType.SHOT_BLOCKED
        
        if new_shot_incoming and not self.is_shot_incoming:
            event_type = EventType.ENEMY_SHOT_ON_GOAL
                
        if not event_type:
            return
        
        self.write_event(event_type=event_type)            

    def _get_goal_shot_region(
        self, field: tbots_cpp.Field, for_friendly: bool
    ) -> tbots_cpp.Rectangle:
        """Returns the corresponding defense area (friendly or enemy) expanded by the shorter side's length on all sides
        for the area the ball should at least enter to be considered a shot on goal

        :param field: the current field
        :param for_friendly: if we should get the area for the friendly or enemy side
        :return: an expanded defense area
        """
        defense_area = (
            field.friendlyDefenseArea() if for_friendly else field.enemyDefenseArea()
        )

        expanded_defense_area = defense_area.expand(defense_area.xLength())
        return expanded_defense_area

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
            and tbots_cpp.contains(
                self._get_goal_shot_region(field, for_friendly), ball_position
            )
            and (ball_velocity.x() <= 0 if for_friendly else ball_velocity.x() >= 0)
        )

        return shot_incoming
