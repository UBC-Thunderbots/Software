import math
from pyqtgraph.Qt.QtCore import Qt
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent
from proto.import_all_protos import *
from software.thunderscope.gl.layers.gl_world_layer import tbots_cpp
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.logger.logger import create_logger
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from typing import override


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

        # State for drag-to-orient movement
        self.is_dragging_to_orient = False
        self.target_point = None
        self.current_orientation = -math.pi / 2

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

    @override
    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Handle mouse press events.
        If Shift+Alt+Control is pressed, clicking selects a robot based on the closest point in scene.
        If Shift+Alt is pressed, clicking starts a drag-to-orient movement sequence.

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
            # Shift+Alt is pressed - start drag-to-orient sequence
            if not self.is_selected:
                logger.warning("No robot selected to be moved")
                return

            self.target_point = point
            self.current_orientation = -math.pi / 2
            self.is_dragging_to_orient = True

    def move_to_point(self, point, orientation: float = None):
        """Move to a point with an optional orientation

        :param point: the point we are commanding the robot to move to
        :param orientation: the final orientation in radians (defaults to -π/2)
        """
        # whether the selected_robot_index would cause index out of range issues
        if not self.is_selected:
            logger.warning("No robot selected to be moved")
            return

        robot_id = self.selected_robot_id

        if orientation is None:
            orientation = -math.pi / 2

        point = Point(x_meters=point.x(), y_meters=point.y())
        move_tactic = MoveTactic(
            destination=point,
            dribbler_mode=DribblerMode.OFF,
            final_orientation=Angle(radians=orientation),
            ball_collision_type=BallCollisionType.AVOID,
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )

        assign_tactic = AssignedTacticPlayControlParams()
        assign_tactic.assigned_tactics[robot_id].move.CopyFrom(move_tactic)

        self.fullsystem_io.send_proto(AssignedTacticPlayControlParams, assign_tactic)

    @override
    def mouse_in_scene_dragged(self, event: MouseInSceneEvent) -> None:
        """Handle mouse drag events to update orientation.

        :param event: The event
        """
        if not self.visible():
            return

        if not self.is_dragging_to_orient or self.target_point is None:
            return

        # Calculate the angle from the target point to the current mouse position
        dx = event.point_in_scene.x() - self.target_point.x()
        dy = event.point_in_scene.y() - self.target_point.y()

        # Calculate angle using atan2 (y, x)
        self.current_orientation = math.atan2(dy, dx)

    @override
    def mouse_in_scene_released(self, event: MouseInSceneEvent) -> None:
        """Handle mouse release events to finalize movement with the calculated orientation.

        :param event: The event
        """
        if not self.visible():
            return

        if not self.is_dragging_to_orient or self.target_point is None:
            return

        # Move to the target point with the calculated orientation
        self.move_to_point(self.target_point, self.current_orientation)

        # Reset drag-to-orient state
        self.is_dragging_to_orient = False
        self.target_point = None
        self.current_orientation = -math.pi / 2

    @override
    def refresh_graphics(self):
        """Updating the world cache"""
        world = self.world_buffer.get(block=False, return_cached=False)

        if world is None:
            return

        self.cached_team = tbots_cpp.Team(world.friendly_team)
