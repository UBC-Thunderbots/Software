from typing import Optional

import software.python_bindings as tbots_cpp
from proto.visualization_pb2 import AttackerVisualization
from proto.import_all_protos import *
from typing import Callable, Any, override
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.log.trackers.tracker import Tracker
from software.thunderscope.proto_unix_io import ProtoUnixIO

class KickTracker(Tracker):
    """Base Class for tracking the attacker's kicks
    using the attacker visualizations and the state of the world
    """

    # We also do not have a 100% guaranteed way to know if an attacker has taken a kick (shot or pass)
    # Often times, the robot will consider many, very similar kicks consecutively WITHOUT taking any
    # so we don't want to count every kick if they're too close together
    # this angle is the minimum difference in direction a kick should be from the previous one to count as "different"
    # lower value -> more similar kicks are counted as different, adding noise
    # higher value -> legitimate kicks may be excluded since they seem "too similar"
    MIN_NEW_SHOT_ANGLE_DIFFERENCE_RAD = tbots_cpp.Angle.fromDegrees(10).toRadians()

    # tune these values to reduce noise in what is considered a kick
    # higher values exclude noise such as dribbling or small movements of the ball
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
        kick_callback: Optional[Callable[[Any], None]] = None,
        buffer_size: int = 5
    ):
        """Initialize the kick tracker
        :param pass_callback: an optional callback to call when there's a pass
        :param shot_callback: an optional callback to call when there's a shot
        """
        super().__init__(kick_callback, buffer_size)      
        self.latest_kick_angle: tbots_cpp.Angle = tbots_cpp.Angle()
        self.kick_taken = False
        self.num_kicks = 0
        
        self.attacker_vis_buffer = ThreadSafeBuffer(self.buffer_size, AttackerVisualization)
        self.world_buffer = ThreadSafeBuffer(self.buffer_size, World)
        
    @override
    def set_proto_unix_io(self, proto_unix_io: ProtoUnixIO) -> None:
        for arg in [
            (AttackerVisualization, self.attacker_vis_buffer),
            (World, self.world_buffer),
        ]:
            proto_unix_io.register_observer(*arg)

    def _get_new_kick_angle(
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
    
    def refresh(
        self
    ) -> None:
        """
        Refreshes the tracker by getting the current state of the world
        and the latest attacker visualization
        """
        attacker_vis_msg = self.attacker_vis_buffer.get(block=False)

        if not attacker_vis_msg:
            return
        
        world_msg = self.world_buffer.get(block=False, return_cached=True)
        
        if world_msg is None:
            return
        world = tbots_cpp.World(world_msg)
        
        self._refresh_kicks(attacker_vis_msg, world)        
        
    def _refresh_kicks(self, attacker_vis_msg: AttackerVisualization, world: tbots_cpp.World) -> None:
        raise Exception("Not Implemented, please use the appropriate subclass!")
    
class PassTracker(KickTracker):
    def __init__(self, pass_callback: Callable[[Pass], None] = None, buffer_size: int = 5):
        super().__init__(pass_callback, buffer_size)
       
    @override 
    def _refresh_kicks(
        self, attacker_vis_msg: AttackerVisualization, world: tbots_cpp.World
    ) -> None:
        """Refreshes the pass tracker with the new attacker visualization
        and the latest state of the world

        :param attacker_vis_msg: the latest attacker visualization message
        :param world: the current world state
        """        
        # check if the attacker has a new pass
        if (
            attacker_vis_msg
            and attacker_vis_msg.HasField("pass_")
            and attacker_vis_msg.pass_committed
        ):
            new_pass_angle = self._get_new_kick_angle(
                attacker_vis_msg.pass_.passer_point,
                attacker_vis_msg.pass_.receiver_point,
                self.latest_kick_angle,
            )

            if new_pass_angle is not None:
                self.latest_kick_angle = new_pass_angle
                self.kick_taken = True
                
        ball = world.ball()
        
        # check if the pass chosen by the attacker has actually been taken
        if not self.kick_taken and ball.hasBallBeenKicked(
            self.latest_kick_angle,
            self.MIN_SHOT_SPEED,
            self.MAX_KICK_ANGLE_DIFFERENCE,
        ):
            self.num_passes += 1
            self.pass_taken = True

            if self.callback:
                self.callback(attacker_vis_msg.pass_)

class ShotTracker(KickTracker):
    def __init__(self, shot_callback: Callable[[Shot], None] = None, buffer_size: int = 5):
        super().__init__(shot_callback, buffer_size)
       
    @override 
    def _refresh_kicks(
        self,
        attacker_vis_msg: AttackerVisualization,
        world: tbots_cpp.World
    ) -> None:
        """Refreshes the shot tracker with the new attacker visualization
        and the latest state of the world

        :param attacker_vis_msg: the latest attacker visualization message
        :param world: the current world state
        """
        # check if the attacker has a new shot
        if attacker_vis_msg and attacker_vis_msg.HasField("shot"):
            new_shot_angle = self._get_new_kick_angle(
                attacker_vis_msg.shot.shot_origin,
                attacker_vis_msg.shot.shot_target,
                self.latest_kick_angle,
            )

            if new_shot_angle is not None:
                self.latest_kick_angle = new_shot_angle
                self.kick_taken = True
                
        ball = world.ball()
        field = world.field()
                
        # check if the shot chosen by the attacker has actually been taken
        # and additionally, if the ball is in the enemy half (reduces noise)
        if (
            not self.kick_taken
            and ball.hasBallBeenKicked(
                self.latest_kick_angle,
                self.MIN_SHOT_SPEED,
                self.MAX_KICK_ANGLE_DIFFERENCE,
            )
            and field.pointInEnemyHalf(ball.position())
        ):
            self.num_shots += 1
            self.shot_taken = True

            if self.callback:
                self.callback(attacker_vis_msg.shot)