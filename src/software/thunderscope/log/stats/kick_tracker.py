from typing import Optional

import software.python_bindings as tbots_cpp
from proto.visualization_pb2 import AttackerVisualization
from proto.import_all_protos import *
from typing import Callable


class KickTracker:
    """Class for tracking the attacker's kicks
    (shots and passes)
    using the attacker visualizations and the state of the world
    """

    # We also do not have a 100% guaranteed way to know if an attacker has taken a kick (shot or pass)
    # Often times, the robot will consider many, very similar kicks consecutively WITHOUT taking any
    # so we don't want to count every kick if they're too close together
    # this angle is the minimum difference in direction a kick should be from the previous one to count as "different"
    # lower value -> more similar kicks are counted as different, adding noise
    # higher value -> legitimate kicks may be excluded since they seem "too similar"
    MIN_NEW_SHOT_ANGLE_DIFFERENCE_RAD = tbots_cpp.Angle.fromDegrees(10).toRadians()

    # tune these values to reduce noise in what is considered a "shot" to net
    # higher values exclude noise such as dribbling or passes
    # but can exclude real kicks
    MIN_SHOT_SPEED = 2.0

    # We do not have a 100% guaranteed way of knowing if a ball was kicked
    # so we use ball.hasBallBeenKicked
    # this angle is the maximum difference in angle between the expected kick direction and actual ball velocity
    # for it to count as a kick
    # lower value -> more sensitive, will exclude some valid kicks
    # higher value -> may not recognize a real kick if it differs too much from the expected kick direction
    MAX_KICK_ANGLE_DIFFERENCE = tbots_cpp.Angle.fromDegrees(5)

    def __init__(
        self,
        pass_callback: Callable[[Pass], None] = None,
        shot_callback: Callable[[Shot], None] = None,
    ):
        """Initialize the kick tracker
        :param pass_callback: an optional callback to call when there's a pass
        :param shot_callback: an optional callback to call when there's a shot
        """
        self.latest_shot_angle: tbots_cpp.Angle = tbots_cpp.Angle()
        self.shot_taken = False
        self.num_shots = 0

        self.latest_pass_angle: tbots_cpp.Angle = tbots_cpp.Angle()
        self.pass_taken = False
        self.num_passes = 0

        self.shot_callback = shot_callback
        self.pass_callback = pass_callback

    def refresh(
        self,
        attacker_vis_msg: AttackerVisualization,
        ball: tbots_cpp.Ball,
        field: tbots_cpp.Field,
    ) -> None:
        """Refreshes the kick tracker with the new attacker visualization
        and the latest state of the world

        :param attacker_vis_msg: the latest attacker visualization message
        :param ball: the latest ball state
        :param field: the field state
        """
        # check if the attacker has a new shot
        if attacker_vis_msg and attacker_vis_msg.HasField("shot"):
            new_shot_angle = self.get_new_kick_angle(
                attacker_vis_msg.shot.shot_origin,
                attacker_vis_msg.shot.shot_target,
                self.latest_shot_angle,
            )

            if new_shot_angle is not None:
                self.latest_shot_angle = new_shot_angle
                self.shot_taken = True

        # check if the attacker has a new pass
        if (
            attacker_vis_msg
            and attacker_vis_msg.HasField("pass_")
            and attacker_vis_msg.pass_committed
        ):
            new_pass_angle = self.get_new_kick_angle(
                attacker_vis_msg.pass_.passer_point,
                attacker_vis_msg.pass_.receiver_point,
                self.latest_pass_angle,
            )

            if new_pass_angle is not None:
                self.latest_pass_angle = new_pass_angle
                self.pass_taken = True

        # check if the shot chosen by the attacker has actually been taken
        # and additionally, if the ball is in the enemy half (reduces noise)
        if (
            not self.pass_taken
            and ball.hasBallBeenKicked(
                self.latest_pass_angle,
                self.MIN_SHOT_SPEED,
                self.MAX_KICK_ANGLE_DIFFERENCE,
            )
            and field.pointInEnemyHalf(ball.position())
        ):
            self.num_shots += 1
            self.shot_taken = True

            if self.shot_callback:
                self.shot_callback(attacker_vis_msg.shot)

        # check if the pass chosen by the attacker has actually been taken
        elif not self.pass_taken and ball.hasBallBeenKicked(
            self.latest_pass_angle,
            self.MIN_SHOT_SPEED,
            self.MAX_KICK_ANGLE_DIFFERENCE,
        ):
            self.num_passes += 1
            self.pass_taken = True

            if self.pass_callback:
                self.pass_callback(attacker_vis_msg.pass_)

    def get_new_kick_angle(
        self, origin: Point, target: Point, latest_angle: tbots_cpp.Angle
    ) -> Optional[tbots_cpp.Angle]:
        """For a kick with the given origin and target, return the new kick angle
        IF it is different enough from the latest angle so far
        If not, return None

        :param origin: the origin of the kick
        :param target: the target of the kick
        :param latest_angle: the latest kick on record to compare against
        :return: the new kick angle or None
        """
        kick_origin = tbots_cpp.Point(
            origin.x_meters,
            origin.y_meters,
        )
        kick_target = tbots_cpp.Point(
            target.x_meters,
            target.y_meters,
        )
        new_kick_angle = tbots_cpp.Vector(
            kick_target.x() - kick_origin.x(), kick_target.y() - kick_origin.y()
        ).orientation()

        if (
            abs((new_kick_angle - latest_angle).toRadians())
            > self.MIN_NEW_SHOT_ANGLE_DIFFERENCE_RAD
        ):
            return new_kick_angle

        return None
