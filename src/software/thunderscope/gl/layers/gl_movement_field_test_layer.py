import math
from pyqtgraph.Qt.QtCore import Qt
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent
from proto.import_all_protos import *
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.logger.logger import create_logger
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

logger = create_logger(__name__)


class GLMovementFieldTestLayer(GLLayer):

    def __init__(
        self, name: str, fullsystem_io: ProtoUnixIO, buffer_size: int = 5
    ) -> None:
        """Initialize the GLMovementLayer

        :param name:            The displayed name of the layer
        :param buffer_size:     The buffer size we have
        :param fullsystem_io:   The fullsystem protounix io
        """
        super().__init__(name)

        self.world_buffer: ThreadSafeBuffer = ThreadSafeBuffer(buffer_size, World)
        self.fullsystem_io: ProtoUnixIO = fullsystem_io
        self.friendly_robot_on_field = []

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """move to the point clicked

        :param event: The event
        """
        if not self.visible():
            return

        if not event.mouse_event.modifiers() == Qt.KeyboardModifier.ShiftModifier:
            return

        point = event.point_in_scene
        logger.info(f"I am clicking on point: {point}")
        self.move_to_point(point)

    def move_to_point(self, point):
        """move to a point

        :param point: the point we are commanding the robot to move to
        """

        if len(self.friendly_robot_on_field) == 0:
            logger.warning("There are no friendly robots on the field.")
            return

        robot_id = self.friendly_robot_on_field[0]

        point = Point(x_meters=point.x(), y_meters=point.y())
        move_tactic = MoveTactic(
            destination=point,
            dribbler_mode=DribblerMode.OFF,
            final_orientation=Angle(radians=-math.pi / 2),
            ball_collision_type=BallCollisionType.AVOID,
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )

        assign_tactic = AssignedTacticPlayControlParams()
        assign_tactic.assigned_tactics[robot_id].move.CopyFrom(move_tactic)

        self.fullsystem_io.send_proto(AssignedTacticPlayControlParams, assign_tactic)
        logger.info(f"I am moving moving robot: {robot_id} to  {point}")

    def refresh_graphics(self):
        """Updating the world cache"""
        world = self.world_buffer.get(block=False, return_cached=False)

        if world is None: 
            return 

        if world.friendly_team: 
            self.friendly_robot_on_field = [robot.id for robot in world.friendly_team.team_robots]

