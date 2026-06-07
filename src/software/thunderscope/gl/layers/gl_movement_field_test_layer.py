import math
from pyqtgraph.Qt.QtCore import Qt
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent
from proto.import_all_protos import *
from software.thunderscope.gl.layers.gl_world_layer import tbots_cpp
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.logger.logger import create_logger
from software.thunderscope.constants import Colors
from software.thunderscope.gl.graphics.gl_robot_outline import GLRobotOutline
from software.thunderscope.gl.graphics.gl_line_strip import GLLineStrip
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from typing import override


logger = create_logger(__name__)


class GLMovementFieldTestLayer(GLLayer):
    needs_mouse_movement_tracking = True

    DEFAULT_ORIENTATION = -math.pi / 2
    ORIENTATION_LINE_LENGTH = 0.3

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
        self.cached_world: World | None = None
        self.is_selected = False

        # State for drag-to-orient movement
        self.is_dragging_to_orient = False
        self.target_point = None
        self.current_orientation = self.DEFAULT_ORIENTATION

        self.selected_robot_graphic = GLRobotOutline(
            parent_item=self, outline_color=Colors.SELECTED_ROBOT_OUTLINE
        )
        self.preview_robot_graphic = GLRobotOutline(
            parent_item=self, outline_color=Colors.DESIRED_ROBOT_LOCATION_OUTLINE
        )
        self.preview_orientation_graphic = GLLineStrip(
            parent_item=self, outline_color=Colors.DESIRED_ROBOT_LOCATION_OUTLINE
        )
        self.selected_robot_graphic.hide()
        self.preview_robot_graphic.hide()
        self.preview_orientation_graphic.hide()

    def _get_selected_robot(self, cached_world: World | None, selected_robot_id: int):
        """Return the proto robot matching the selected robot ID, or None.

        :param cached_world: The world snapshot to search
        :param selected_robot_id: The ID of the robot to find
        """
        if cached_world is None:
            return None
        return next(
            (
                robot
                for robot in cached_world.friendly_team.team_robots
                if robot.id == selected_robot_id
            ),
            None,
        )

    def select_closest_robot(self, point):
        """Find the closest robot to a point

        :param point: the reference point to find the closest robot
        """
        team = tbots_cpp.Team(self.cached_world.friendly_team)
        if not team:
            logger.warning("No vision data received")
            return

        closest_robot = team.getNearestRobot(tbots_cpp.Point(point.x(), point.y()))
        if closest_robot is None:
            logger.warning(
                "No robots found. Are you sure there is friendly robot on field?"
            )
            return

        self.selected_robot_id = closest_robot.id()
        self.is_selected = True

        # Set the selected robot's orientation as current orientation
        robot = self._get_selected_robot(self.cached_world, self.selected_robot_id)
        if robot is not None:
            self.current_orientation = robot.current_state.global_orientation.radians

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
            self.is_dragging_to_orient = True

    def move_to_point(self, point, orientation: float = None):
        """Move to a point with an optional orientation

        :param point: the point we are commanding the robot to move to
        :param orientation: the final orientation in radians
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

    @override
    def mouse_in_scene_moved(self, event: MouseInSceneEvent) -> None:
        """Handle mouse moved events to update robot position preview.

        :param event: The event
        """
        if not self.visible() or not self.is_selected:
            self.target_point = None
            return

        if self.is_dragging_to_orient:
            return

        if (
            event.mouse_event.modifiers() & Qt.KeyboardModifier.AltModifier
            and not event.mouse_event.modifiers() & Qt.KeyboardModifier.ControlModifier
        ):
            # Shift+Alt is pressed - mouse point is the current target position
            self.target_point = event.point_in_scene
        else:
            self.target_point = None

    def _draw_selection(self):
        """Renders an outline around the currently selected robot."""
        robot = (
            self._get_selected_robot(self.cached_world, self.selected_robot_id)
            if self.is_selected
            else None
        )
        if robot is None:
            self.selected_robot_graphic.hide()
            return

        self.selected_robot_graphic.set_position(
            robot.current_state.global_position.x_meters,
            robot.current_state.global_position.y_meters,
        )
        self.selected_robot_graphic.set_orientation(
            math.degrees(robot.current_state.global_orientation.radians)
        )
        self.selected_robot_graphic.show()

    def _draw_preview(self):
        """Renders a robot outline at the target position and a line indicating orientation."""
        if self.target_point is None:
            self.preview_robot_graphic.hide()
            self.preview_orientation_graphic.hide()
            return

        x = self.target_point.x()
        y = self.target_point.y()
        angle = self.current_orientation

        self.preview_robot_graphic.set_position(x, y)
        self.preview_robot_graphic.set_orientation(math.degrees(angle))
        self.preview_robot_graphic.show()

        end_x = x + math.cos(angle) * self.ORIENTATION_LINE_LENGTH
        end_y = y + math.sin(angle) * self.ORIENTATION_LINE_LENGTH

        self.preview_orientation_graphic.set_points([[x, y], [end_x, end_y]])
        self.preview_orientation_graphic.show()

    @override
    def refresh_graphics(self):
        """Updating the world cache"""
        world = self.world_buffer.get(block=False, return_cached=False)

        if world is not None:
            self.cached_world = world

        self._draw_selection()
        self._draw_preview()
