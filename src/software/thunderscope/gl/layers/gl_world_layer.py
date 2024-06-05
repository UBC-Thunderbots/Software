from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.opengl import *

import math
import numpy as np

import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.constants import (
    Colors,
    DepthValues,
    SPEED_SEGMENT_SCALE,
    DEFAULT_EMPTY_FIELD_WORLD,
    is_field_message_empty,
    SIMULATION_SPEEDS,
    LINE_WIDTH,
    CustomGLOptions,
)

from typing import Dict, Tuple

from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_rect import GLRect
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
from software.thunderscope.gl.graphics.gl_robot import GLRobot
from software.thunderscope.gl.graphics.gl_sphere import GLSphere
from software.thunderscope.gl.graphics.gl_goal import GLGoal

from software.networking.unix.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO

from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLWorldLayer(GLLayer):
    """GLLayer that visualizes the world and vision data"""

    def __init__(
        self,
        name: str,
        simulator_io: ProtoUnixIO,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
    ) -> None:
        """Initialize the GLWorldLayer

        :param name: The displayed name of the layer
        :param simulator_io: The simulator io communicate with the simulator
        :param friendly_colour_yellow: Is the friendly_colour_yellow?
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)
        self.setDepthValue(DepthValues.FOREGROUND_DEPTH)

        self.simulator_io = simulator_io
        self.friendly_colour_yellow = friendly_colour_yellow

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)
        self.robot_status_buffer = ThreadSafeBuffer(buffer_size, RobotStatus)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee, False)
        self.simulation_state_buffer = ThreadSafeBuffer(buffer_size, SimulationState)
        self.cached_world = World()
        # fields to store the team from the cached world state as a dict
        self._cached_friendly_team = {}
        self._cached_enemy_team = {}
        self.cached_robot_status = {}

        self.key_pressed = {}
        self.accepted_keys = [
            Qt.Key.Key_Control,
            Qt.Key.Key_I,
            Qt.Key.Key_Space,
            Qt.Key.Key_Shift,
            Qt.Key.Key_Up,
            Qt.Key.Key_Down,
        ]
        for key in self.accepted_keys:
            self.key_pressed[key] = False

        self.display_robot_ids = True
        self.display_speed_lines = True
        self.is_playing = True
        self.simulation_speed = 1.0

        self.ball_velocity_vector = None
        self.point_in_scene_picked = None

        self.friendly_defense_area_graphic = GLRect(
            parent_item=self, outline_color=Colors.FIELD_LINE_COLOR
        )
        self.enemy_defense_area_graphic = GLRect(
            parent_item=self, outline_color=Colors.FIELD_LINE_COLOR
        )
        self.field_lines_graphic = GLRect(
            parent_item=self, outline_color=Colors.FIELD_LINE_COLOR
        )
        self.field_outer_boundary_graphic = GLRect(
            parent_item=self, outline_color=Colors.FIELD_LINE_LIGHTER_COLOR
        )
        self.halfway_line_graphic = GLPolygon(
            parent_item=self, outline_color=Colors.FIELD_LINE_LIGHTER_COLOR,
        )
        self.goal_to_goal_line_graphic = GLPolygon(
            parent_item=self, outline_color=Colors.FIELD_LINE_LIGHTER_COLOR,
        )
        self.field_center_circle_graphic = GLCircle(
            parent_item=self, outline_color=Colors.FIELD_LINE_COLOR
        )
        self.friendly_goal_graphic = GLGoal(parent_item=self, color=Colors.GOAL_COLOR)
        self.enemy_goal_graphic = GLGoal(parent_item=self, color=Colors.GOAL_COLOR)
        self.ball_graphic = GLSphere(
            parent_item=self, radius=BALL_MAX_RADIUS_METERS, color=Colors.BALL_COLOR
        )
        self.ball_kick_velocity_graphic = GLPolygon(
            parent_item=self, outline_color=Colors.SPEED_VECTOR_COLOR
        )

        self.friendly_defense_area_graphic.setDepthValue(DepthValues.BACKGROUND_DEPTH)
        self.enemy_defense_area_graphic.setDepthValue(DepthValues.BACKGROUND_DEPTH)
        self.field_lines_graphic.setDepthValue(DepthValues.BACKGROUND_DEPTH)
        self.field_outer_boundary_graphic.setDepthValue(DepthValues.BACKGROUND_DEPTH)
        self.halfway_line_graphic.setDepthValue(DepthValues.BACKGROUND_DEPTH)
        self.goal_to_goal_line_graphic.setDepthValue(DepthValues.BACKGROUND_DEPTH)
        self.field_center_circle_graphic.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.friendly_robot_graphics = ObservableList(self._graphics_changed)
        self.enemy_robot_graphics = ObservableList(self._graphics_changed)
        self.friendly_robot_id_graphics = ObservableList(self._graphics_changed)
        self.enemy_robot_id_graphics = ObservableList(self._graphics_changed)
        self.breakbeam_graphics = ObservableList(self._graphics_changed)
        self.auto_kick_graphics = ObservableList(self._graphics_changed)
        self.auto_chip_graphics = ObservableList(self._graphics_changed)
        self.speed_line_graphics = ObservableList(self._graphics_changed)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        """Detect when a key has been pressed

        :param event: The event

        """
        self.key_pressed[event.key()] = True

        if event.key() == QtCore.Qt.Key.Key_I:
            self.display_robot_ids = not self.display_robot_ids
        elif event.key() == QtCore.Qt.Key.Key_S:
            self.display_speed_lines = not self.display_speed_lines

        # If user is holding ctrl + space, send a command to simulator to pause the gameplay
        if (
            self.key_pressed[QtCore.Qt.Key.Key_Control]
            and self.key_pressed[QtCore.Qt.Key.Key_Space]
        ):
            self.toggle_play_state()

        if (
            self.key_pressed[QtCore.Qt.Key.Key_Control]
            and self.key_pressed[QtCore.Qt.Key.Key_Up]
        ):
            self.increment_sim_speed()

        if (
            self.key_pressed[QtCore.Qt.Key.Key_Control]
            and self.key_pressed[QtCore.Qt.Key.Key_Down]
        ):
            self.decrement_sim_speed()

    def increment_sim_speed(self) -> None:
        """Increment the simulation speed to the next fastest speed, if there's one"""
        curr_sim_speed_index = SIMULATION_SPEEDS.index(self.simulation_speed)
        new_simulation_speed = SIMULATION_SPEEDS[max(0, curr_sim_speed_index - 1)]
        self.set_simulation_speed(new_simulation_speed)

    def decrement_sim_speed(self) -> None:
        """Decrement the simulation speed to the previous fastest speed, if there's one"""
        curr_sim_speed_index = SIMULATION_SPEEDS.index(self.simulation_speed)
        new_simulation_speed = SIMULATION_SPEEDS[
            min(len(SIMULATION_SPEEDS) - 1, curr_sim_speed_index + 1)
        ]
        self.set_simulation_speed(new_simulation_speed)

    def toggle_play_state(self) -> bool:
        """
        Pauses the simulated gameplay and toggles the play state
        Calls all callback functions with the new play state
        :return: the current play state
        """
        simulator_state = SimulationState(
            is_playing=not self.is_playing, simulation_speed=self.simulation_speed
        )
        self.is_playing = not self.is_playing

        self.simulator_io.send_proto(SimulationState, simulator_state)

        return self.is_playing

    def set_simulation_speed(self, speed: float) -> None:
        """
        Sets the speed of the simulator
        :param speed: the new speed to set
        """
        self.simulation_speed = speed
        simulator_state = SimulationState(
            is_playing=self.is_playing, simulation_speed=self.simulation_speed
        )
        self.simulator_io.send_proto(SimulationState, simulator_state)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        """Detect when a key has been released

        :param event: The event

        """
        self.key_pressed[event.key()] = False

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Detect that the mouse was pressed and picked a point in the 3D scene
        
        :param event: The event
        
        """
        if not event.mouse_event.modifiers() == Qt.KeyboardModifier.ShiftModifier:
            return

        self.point_in_scene_picked = self._invert_position_if_defending_negative_half(
            event.point_in_scene
        )

        # Send a command to the simulator to move the ball to the picked point
        world_state = WorldState()
        world_state.ball_state.CopyFrom(
            BallState(
                global_position=Point(
                    x_meters=self.point_in_scene_picked.x(),
                    y_meters=self.point_in_scene_picked.y(),
                )
            )
        )
        self.simulator_io.send_proto(WorldState, world_state)

    def mouse_in_scene_dragged(self, event: MouseInSceneEvent) -> None:
        """Detect that the mouse was dragged within the 3D scene
        
        :param event: The event
        
        """
        if not event.mouse_event.modifiers() == Qt.KeyboardModifier.ShiftModifier:
            return

        if not self.point_in_scene_picked:
            return

        # User picked a point in the 3D scene and is now dragging it across the scene
        # to apply a velocity on the ball (i.e. kick it).
        # We create a velocity vector that is proportional to the distance the
        # mouse has moved away from the ball.
        self.ball_velocity_vector = (
            self.point_in_scene_picked
            - self._invert_position_if_defending_negative_half(event.point_in_scene)
        )

        # Cap the maximum kick speed
        if self.ball_velocity_vector.length() > BALL_MAX_SPEED_METERS_PER_SECOND:
            self.ball_velocity_vector.normalize()
            self.ball_velocity_vector *= BALL_MAX_SPEED_METERS_PER_SECOND

    def mouse_in_scene_released(self, event: MouseInSceneEvent) -> None:
        """Detect that the mouse was released after picking a point in the 3D scene
        
        :param event: The event
        
        """
        if not event.mouse_event.modifiers() == Qt.KeyboardModifier.ShiftModifier:
            return

        if not self.point_in_scene_picked or not self.ball_velocity_vector:
            return

        if self._should_invert_coordinate_frame():
            self.ball_velocity_vector = -self.ball_velocity_vector

        # Send a command to the simulator to give the ball the specified
        # velocity (i.e. kick it)

        world_state = WorldState()
        world_state.ball_state.CopyFrom(
            BallState(
                global_position=Point(
                    x_meters=self.point_in_scene_picked.x(),
                    y_meters=self.point_in_scene_picked.y(),
                ),
                global_velocity=Vector(
                    x_component_meters=self.ball_velocity_vector.x(),
                    y_component_meters=self.ball_velocity_vector.y(),
                ),
            )
        )

        self.ball_velocity_vector = None
        self.simulator_io.send_proto(WorldState, world_state)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        self.cached_world = self.world_buffer.get(block=False, return_cached=True)

        # if not receiving worlds, just render an empty field
        if is_field_message_empty(self.cached_world.field):
            self.cached_world = DEFAULT_EMPTY_FIELD_WORLD

        self.__update_field_graphics(self.cached_world.field)
        self.__update_goal_graphics(self.cached_world.field)
        self.__update_ball_graphics(self.cached_world.ball.current_state)

        self._cached_friendly_team = {
            robot.id: (
                robot.current_state.global_position.x_meters,
                robot.current_state.global_position.y_meters,
                robot.current_state.global_orientation.radians,
            )
            for robot in self.cached_world.friendly_team.team_robots
        }

        self._cached_enemy_team = {
            robot.id: (
                robot.current_state.global_position.x_meters,
                robot.current_state.global_position.y_meters,
                robot.current_state.global_orientation.radians,
            )
            for robot in self.cached_world.enemy_team.team_robots
        }

        self._update_robots_graphics()

        self.__update_robot_status_graphics()
        self.__update_auto_chip_or_kick_graphics()
        self.__update_speed_line_graphics()

        # Update internal simulation state
        simulation_state = self.simulation_state_buffer.get(
            block=False, return_cached=False
        )
        if simulation_state:
            self.is_playing = simulation_state.is_playing
            self.simulation_speed = simulation_state.simulation_speed

    def _update_robots_graphics(self) -> None:
        """
        Updates the GLGraphicsItems that display all robots (friendly and enemy team)
        """
        friendly_colour = (
            Colors.YELLOW_ROBOT_COLOR
            if self.friendly_colour_yellow
            else Colors.BLUE_ROBOT_COLOR
        )
        enemy_colour = (
            Colors.BLUE_ROBOT_COLOR
            if self.friendly_colour_yellow
            else Colors.YELLOW_ROBOT_COLOR
        )

        self.__update_robot_graphics(
            self._cached_friendly_team,
            friendly_colour,
            self.friendly_robot_graphics,
            self.friendly_robot_id_graphics,
        )
        self.__update_robot_graphics(
            self._cached_enemy_team,
            enemy_colour,
            self.enemy_robot_graphics,
            self.enemy_robot_id_graphics,
        )

    def __update_field_graphics(self, field: Field) -> None:
        """Update the GLGraphicsItems that display the field lines and markings
        
        :param field: The field proto

        """
        self.field_lines_graphic.set_dimensions(
            field.field_x_length, field.field_y_length
        )

        boundary_buffer = 2 * field.boundary_buffer_size
        self.field_outer_boundary_graphic.set_dimensions(
            field.field_x_length + boundary_buffer,
            field.field_y_length + boundary_buffer,
        )

        self.friendly_defense_area_graphic.set_dimensions(
            field.defense_x_length, field.defense_y_length
        )
        self.friendly_defense_area_graphic.set_position(
            -(field.field_x_length / 2) + (field.defense_x_length / 2), 0
        )

        self.enemy_defense_area_graphic.set_dimensions(
            field.defense_x_length, field.defense_y_length
        )
        self.enemy_defense_area_graphic.set_position(
            (field.field_x_length / 2) - (field.defense_x_length / 2), 0
        )

        self.halfway_line_graphic.set_points(
            [(0, -(field.field_y_length / 2)), (0, (field.field_y_length / 2))]
        )

        self.goal_to_goal_line_graphic.set_points(
            [(-(field.field_x_length / 2), 0), ((field.field_x_length / 2), 0)]
        )

        self.field_center_circle_graphic.set_radius(field.center_circle_radius)

    def __update_goal_graphics(self, field: Field) -> None:
        """Update the GLGraphicsItems that display the goals
        
        :param field: The field proto

        """
        if not field.goal_x_length or not field.goal_y_length:
            return

        self.friendly_goal_graphic.set_dimensions(
            field.goal_x_length, field.goal_y_length
        )
        self.friendly_goal_graphic.set_position(-field.field_x_length / 2, 0)
        self.friendly_goal_graphic.set_orientation(0)

        self.enemy_goal_graphic.set_dimensions(field.goal_x_length, field.goal_y_length)
        self.enemy_goal_graphic.set_position(field.field_x_length / 2, 0)
        self.enemy_goal_graphic.set_orientation(180)

    def __update_ball_graphics(self, ball_state: BallState) -> None:
        """Update the GLGraphicsItems that display the ball
        
        :param ball_state: The ball state proto

        """
        self.ball_graphic.set_position(
            ball_state.global_position.x_meters,
            ball_state.global_position.y_meters,
            ball_state.distance_from_ground,
        )

    def __update_robot_graphics(
        self,
        robots: Dict[int, Tuple[float, float, float]],
        color: QtGui.QColor,
        robot_graphics: ObservableList,
        robot_id_graphics: ObservableList,
    ) -> None:
        """Update the GLGraphicsItems that display the robots
        
        :param robots: a mapping of robot ids to a tuple containing x-coord, y-coord, and orientation
        :param color: The color of the robots
        :param robot_graphics: The ObservableList containing the robot graphics for this team
        :param robot_id_graphics: The ObservableList containing the robot ID graphics for this team

        """
        # Ensure we have the same number of graphics as robots
        robot_graphics.resize(len(robots), lambda: GLRobot())
        robot_id_graphics.resize(
            len(robots),
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 10, weight=700),
                color=Colors.PRIMARY_TEXT_COLOR,
            ),
        )

        for robot_graphic, robot_id_graphic, robot_id in zip(
            robot_graphics, robot_id_graphics, robots.keys(),
        ):
            # update the robot graphic with the robot state
            pos_x, pos_y, orientation = robots[robot_id]

            robot_graphic.set_position(pos_x, pos_y)
            robot_graphic.set_orientation(math.degrees(orientation))
            robot_graphic.setColor(color)
            robot_graphic.show()

            if self.display_robot_ids:
                robot_id_graphic.show()

                robot_id_graphic.setDepthValue(DepthValues.ABOVE_FOREGROUND_DEPTH)

                robot_id_graphic.setData(
                    text=str(robot_id),
                    pos=[
                        pos_x - (ROBOT_MAX_RADIUS_METERS / 2),
                        pos_y,
                        ROBOT_MAX_HEIGHT_METERS + 0.1,
                    ],
                )

            else:
                robot_id_graphic.hide()

    def __update_robot_status_graphics(self) -> None:
        """Update the robot status graphics"""

        # Get the robot status messages
        robot_statuses = {}
        while True:
            robot_status = self.robot_status_buffer.get(
                block=False, return_cached=False
            )
            if robot_status:
                robot_statuses[robot_status.robot_id] = robot_status
            else:
                break

        # Ensure we have the same number of graphics as robots
        self.breakbeam_graphics.resize(
            len(self.cached_world.friendly_team.team_robots),
            lambda: GLCircle(
                parent_item=self,
                radius=ROBOT_MAX_RADIUS_METERS / 2,
                outline_color=Colors.BREAKBEAM_TRIPPED_COLOR,
            ),
        )

        for breakbeam_graphic, robot in zip(
            self.breakbeam_graphics, self.cached_world.friendly_team.team_robots
        ):
            if (
                robot.id in robot_statuses
                and robot_statuses[robot.id].power_status.breakbeam_tripped
            ):
                breakbeam_graphic.show()

                breakbeam_graphic.setDepthValue(DepthValues.ABOVE_FOREGROUND_DEPTH)

                breakbeam_graphic.set_position(
                    robot.current_state.global_position.x_meters,
                    robot.current_state.global_position.y_meters,
                )
            else:
                breakbeam_graphic.hide()

    def __update_auto_chip_or_kick_graphics(self) -> None:
        """Update the auto kick and auto chip graphics"""

        # See which robots have auto kick or auto chip enabled
        auto_kick_robots = []
        auto_chip_robots = []
        primitive_set = self.primitive_set_buffer.get(block=False)

        for robot_id in primitive_set.robot_primitives:
            primitive = primitive_set.robot_primitives[robot_id]

            if primitive.HasField("move"):
                autochip_or_kick = primitive.move.auto_chip_or_kick

                if (
                    autochip_or_kick.HasField("autokick_speed_m_per_s")
                    and autochip_or_kick.autokick_speed_m_per_s > 0
                ):
                    auto_kick_robots.append(robot_id)
                elif (
                    autochip_or_kick.HasField("autochip_distance_meters")
                    and autochip_or_kick.autochip_distance_meters > 0
                ):
                    auto_chip_robots.append(robot_id)

        # Ensure we have the same number of graphics as robots
        self.auto_chip_graphics.resize(
            len(self.cached_world.friendly_team.team_robots),
            lambda: GLPolygon(
                outline_color=Colors.AUTO_CHIP_ENABLED_COLOR, line_width=LINE_WIDTH * 2
            ),
        )
        self.auto_kick_graphics.resize(
            len(self.cached_world.friendly_team.team_robots),
            lambda: GLPolygon(
                outline_color=Colors.AUTO_KICK_ENABLED_COLOR, line_width=LINE_WIDTH * 2
            ),
        )

        def update_polygon(polygon_graphic: GLPolygon, robot: tbots_cpp.Robot) -> None:
            """
            Update the polygon graphic with the robot's dribbler area
            :param polygon_graphic: The polygon graphic to update
            :param robot: The robot to get the dribbler area from
            """
            polygon_graphic.updateGLOptions(CustomGLOptions.OPAQUE_WITH_OUT_DEPTH_TEST)
            polygon_graphic.setDepthValue(DepthValues.ABOVE_FOREGROUND_DEPTH)

            dribble_area_polygon_points = (
                tbots_cpp.Robot(robot).dribblerArea().getPoints()
            )
            polygon_graphic.set_points(
                [
                    (
                        dribble_area_polygon_points[0].x(),
                        dribble_area_polygon_points[0].y(),
                    ),
                    (
                        dribble_area_polygon_points[3].x(),
                        dribble_area_polygon_points[3].y(),
                    ),
                ]
            )

        # Update the graphics
        for auto_chip_graphic, auto_kick_graphic, robot in zip(
            self.auto_chip_graphics,
            self.auto_kick_graphics,
            self.cached_world.friendly_team.team_robots,
        ):
            if robot.id in auto_chip_robots:
                auto_chip_graphic.show()
                auto_kick_graphic.hide()
                update_polygon(auto_chip_graphic, robot)

            elif robot.id in auto_kick_robots:
                auto_kick_graphic.show()
                auto_chip_graphic.hide()
                update_polygon(auto_kick_graphic, robot)

            else:
                auto_chip_graphic.hide()
                auto_kick_graphic.hide()

    def __update_speed_line_graphics(self) -> None:
        """Update the speed lines visualizing the robot and ball speeds"""

        # When the user is kicking the ball, show the kick velocity vector
        # as a speed line
        if self.ball_velocity_vector:

            ball_state = self.cached_world.ball.current_state
            velocity = self.ball_velocity_vector * SPEED_SEGMENT_SCALE

            self.ball_kick_velocity_graphic.show()
            self.ball_kick_velocity_graphic.set_points(
                [
                    (
                        ball_state.global_position.x_meters,
                        ball_state.global_position.y_meters,
                    ),
                    (
                        ball_state.global_position.x_meters + velocity.x(),
                        ball_state.global_position.y_meters + velocity.y(),
                    ),
                ]
            )

        else:
            self.ball_kick_velocity_graphic.hide()

        # Combining robots and ball into one list
        objects = []
        if self.display_speed_lines:
            objects = list(self.cached_world.friendly_team.team_robots)
            objects.append(self.cached_world.ball)

        # Ensure we have the same number of graphics as robots/balls
        self.speed_line_graphics.resize(
            len(objects), lambda: GLPolygon(outline_color=Colors.SPEED_VECTOR_COLOR),
        )

        for speed_line_graphic, object in zip(self.speed_line_graphics, objects):
            pos_x = object.current_state.global_position.x_meters
            pos_y = object.current_state.global_position.y_meters
            velocity = object.current_state.global_velocity
            speed_line_graphic.set_points(
                [
                    [pos_x, pos_y],
                    [
                        pos_x + velocity.x_component_meters * SPEED_SEGMENT_SCALE,
                        pos_y + velocity.y_component_meters * SPEED_SEGMENT_SCALE,
                    ],
                ]
            )

    def _should_invert_coordinate_frame(self) -> bool:
        """Our coordinate system always assumes that the friendly team is defending
        the negative half of the field.

        If we are defending the positive half, we invert the coordinate frame
        and render the inverted proto. 

        We can use the referee msg to determine if we are defending the positive
        or negative half of the field.

        :return: True if we should invert the coordinate frame, False otherwise

        """
        referee = self.referee_buffer.get(block=False)

        if (self.friendly_colour_yellow and not referee.blue_team_on_positive_half) or (
            not self.friendly_colour_yellow and referee.blue_team_on_positive_half
        ):
            return True
        return False

    def _invert_position_if_defending_negative_half(
        self, point: QtGui.QVector3D
    ) -> QtGui.QVector3D:
        """If we are defending the negative half of the field, we invert the coordinate frame
        for the mouse click/3D point picked to match up with the visualization.

        :param point: The point location in the 3D scene [x, y]
        :return: The inverted point location [x, y] (if needed to be inverted)

        """
        if self._should_invert_coordinate_frame():
            return QtGui.QVector3D(-point[0], -point[1], point[2])
        return point
