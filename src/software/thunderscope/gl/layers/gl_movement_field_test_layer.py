import math
from pyqtgraph.Qt.QtCore import Qt
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent
from proto.import_all_protos import *
from software.thunderscope.gl.layers.gl_world_layer import tbots_cpp
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.logger.logger import create_logger
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

logger = create_logger(__name__)


class GLMovementFieldTestLayer(GLLayer):
    def __init__(
        self, name: str, fullsystem_io: ProtoUnixIO, buffer_size: int = 5
    ) -> None:
        """Initialize the GLMovementFieldTestLayer

        :param name:            The displayed name of the layer
        :param buffer_size:     The buffer size we have
        :param fullsystem_io:   The fullsystem protounix io
        """
        super().__init__(name)

        self.world_buffer: ThreadSafeBuffer = ThreadSafeBuffer(buffer_size, World)
        self.fullsystem_io: ProtoUnixIO = fullsystem_io
        self.selected_robot_id = 0
        self.cached_team: tbots_cpp.Team = None
        self.is_selected = False

    def select_closest_robot(self, point):
        """Find the closest robot to a point

        :param point: the reference point to find the closest robot
        """
        if not self.cached_team:
            logger.warning("No vision data received")
            return

        closest_robot = self.cached_team.getNearestRobot(
            tbots_cpp.Point(point.x(), point.y())
        )
        if closest_robot is None:
            logger.warning(
                "No robots found. Are you sure there is friendly robot on field?"
            )
            return

        self.selected_robot_id = closest_robot.id()
        self.is_selected = True

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Move to the point clicked.
        If Shift+Alt+Control is pressed, clicking selects a robot based on the closest point in scene.
        If Shift+Alt is pressed, clicking moves a robot to the point in scene.

        :param event: The event
        """
        if not self.visible():
            return

        if not event.mouse_event.modifiers() & Qt.KeyboardModifier.AltModifier:
            return

        point = event.point_in_scene
        if event.mouse_event.modifiers() & Qt.KeyboardModifier.ControlModifier:
            # Shift+Alt+Control is pressed
            self.select_closest_robot(point)
        else:
            # Shift+Alt is pressed
            self.move_to_point(point)

    def move_to_point(self, point):
        """Move to a point

        :param point: the point we are commanding the robot to move to
        """
        # whether the selected_robot_index would cause index out of range issues
        if not self.is_selected:
            logger.warning("No robot selected to be moved")
            return

        robot_id = self.selected_robot_id

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

    def refresh_graphics(self):
        """Updating the world cache"""
        world = self.world_buffer.get(block=False, return_cached=False)

        if world is None:
            return

        self.cached_team = tbots_cpp.Team(world.friendly_team)
