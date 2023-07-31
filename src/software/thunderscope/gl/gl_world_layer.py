from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.opengl import *

import math

from proto.import_all_protos import *
from software.py_constants import *
import software.python_bindings as geom
import software.thunderscope.constants as constants
from software.thunderscope.constants import Colors, SPEED_SEGMENT_SCALE

from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_rect import GLRect
from software.thunderscope.gl.graphics.gl_robot import GLRobot
from software.thunderscope.gl.graphics.gl_sphere import GLSphere
from software.thunderscope.gl.graphics.gl_goal import GLGoal

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from software.thunderscope.gl.gl_layer import GLLayer
from software.thunderscope.gl.helpers.extended_gl_view_widget import PointInSceneEvent


class GLWorldLayer(GLLayer):
    """GLLayer that visualizes the world and vision data"""

    # The maximum allowed velocity that the user can give the ball
    MAX_ALLOWED_KICK_SPEED_M_PER_S = 6.5

    def __init__(self, name: str, simulator_io, friendly_colour_yellow: bool, buffer_size: int = 5):
        """Initialize the GLWorldLayer

        :param name: The displayed name of the layer
        :param simulator_io: The simulator io communicate with the simulator
        :param friendly_colour_yellow: Is the friendly_colour_yellow?
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        GLLayer.__init__(self, name)

        self.simulator_io = simulator_io
        self.friendly_colour_yellow = friendly_colour_yellow

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.robot_status_buffer = ThreadSafeBuffer(buffer_size, RobotStatus)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee, False)
        self.cached_world = World()
        self.cached_robot_status = {}

        self.key_pressed = {}
        self.accepted_keys = [
            Qt.Key.Key_Control,
            Qt.Key.Key_I,
            Qt.Key.Key_Space,
            Qt.Key.Key_Shift,
        ]
        for key in self.accepted_keys:
            self.key_pressed[key] = False

        self.display_robot_id = False
        self.display_speed_lines = True
        self.is_playing = True

        self.ball_velocity_vector = None
        self.point_in_scene_picked = None

        self.graphics_list.register_graphics_group(
            "field_marking_rects", lambda: GLRect(color=Colors.FIELD_LINE_COLOR)
        )
        self.graphics_list.register_graphics_group(
            "field_outer_boundary_rect", lambda: GLRect(color=Colors.FIELD_LINE_LIGHTER_COLOR)
        )
        self.graphics_list.register_graphics_group(
            "field_marking_lines",
            lambda: GLLinePlotItem(color=Colors.FIELD_LINE_LIGHTER_COLOR),
        )
        self.graphics_list.register_graphics_group(
            "field_center_circle", lambda: GLCircle(color=Colors.FIELD_LINE_COLOR)
        )
        self.graphics_list.register_graphics_group(
            "goals", lambda: GLGoal(color=Colors.GOAL_COLOR)
        )
        self.graphics_list.register_graphics_group(
            "ball",
            lambda: GLSphere(radius=BALL_MAX_RADIUS_METERS, color=Colors.BALL_COLOR),
        )
        self.graphics_list.register_graphics_group(
            "robot_ids", 
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 10, weight=700), 
                color=Colors.PRIMARY_TEXT_COLOR
            )
        )
        self.graphics_list.register_graphics_group(
            "robot_status", lambda: GLCircle(color=Colors.BREAKBEAM_TRIPPED_COLOR)
        )
        self.graphics_list.register_graphics_group(
            "speed_lines", lambda: GLCircle(color=Colors.SPEED_VECTOR_COLOR)
        )
        self.graphics_list.register_graphics_group("robots", GLRobot)

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        """Detect when a key has been pressed

        :param event: The event

        """
        self.key_pressed[event.key()] = True

        if event.key() == QtCore.Qt.Key.Key_I:
            self.display_robot_id = not self.display_robot_id
        elif event.key() == QtCore.Qt.Key.Key_S:
            self.display_speed_lines = not self.display_speed_lines

        # If user is holding ctrl + space, send a command to simulator to pause the gameplay
        if (
            self.key_pressed[QtCore.Qt.Key.Key_Control]
            and self.key_pressed[QtCore.Qt.Key.Key_Space]
        ):

            simulator_state = SimulationState(is_playing=not self.is_playing)
            self.is_playing = not self.is_playing

            self.simulator_io.send_proto(SimulationState, simulator_state)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent):
        """Detect when a key has been released

        :param event: The event

        """
        self.key_pressed[event.key()] = False

    def mouse_in_scene_pressed(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_pressed event
        
        :param event: The event
        
        """
        self.point_in_scene_picked = self.__invert_position_if_defending_negative_half(
            event.point_in_scene
        )

        # Send a command to the simulator to move the ball to the picked point
        world_state = WorldState()
        world_state.ball_state.CopyFrom(
            BallState(
                global_position=Point(
                    x_meters=self.point_in_scene_picked[0],
                    y_meters=self.point_in_scene_picked[1],
                )
            )
        )
        self.simulator_io.send_proto(WorldState, world_state)

    def mouse_in_scene_dragged(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_dragged event
        
        :param event: The event
        
        """
        if not self.point_in_scene_picked:
            return

        # User picked a point in the 3D scene and is now dragging it across the scene
        # to apply a velocity on the ball (i.e. kick it).
        # We create a velocity vector that is proportional to the distance the
        # mouse has moved away from the ball.

        ball_position = geom.Vector(
            self.point_in_scene_picked[0], self.point_in_scene_picked[1]
        )

        self.ball_velocity_vector = (
            ball_position
            - self.__invert_position_if_defending_negative_half(
                geom.Vector(event.point_in_scene[0], event.point_in_scene[1])
            )
        )

        # Cap the maximum kick speed
        if (
            self.ball_velocity_vector.length()
            > GLWorldLayer.MAX_ALLOWED_KICK_SPEED_M_PER_S
        ):
            self.ball_velocity_vector = self.ball_velocity_vector.normalize(
                GLWorldLayer.MAX_ALLOWED_KICK_SPEED_M_PER_S
            )

    def mouse_in_scene_released(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_released event
        
        :param event: The event
        
        """
        if not self.point_in_scene_picked or not self.ball_velocity_vector:
            return

        if self.__should_invert_coordinate_frame():
            self.ball_velocity_vector = -self.ball_velocity_vector

        # Send a command to the simulator to give the ball the specified
        # velocity (i.e. kick it)

        world_state = WorldState()
        world_state.ball_state.CopyFrom(
            BallState(
                global_position=Point(
                    x_meters=self.point_in_scene_picked[0],
                    y_meters=self.point_in_scene_picked[1],
                ),
                global_velocity=Vector(
                    x_component_meters=self.ball_velocity_vector.x(),
                    y_component_meters=self.ball_velocity_vector.y(),
                ),
            )
        )

        self.ball_velocity_vector = None
        self.simulator_io.send_proto(WorldState, world_state)

    def _update_graphics(self):
        """Fetch and update graphics for the layer"""

        self.cached_world = self.world_buffer.get(block=False)

        self.__update_field_graphics(self.cached_world.field)
        self.__update_goal_graphics(self.cached_world.field)
        self.__update_ball_graphics(self.cached_world.ball.current_state)

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

        self.__update_robot_graphics(self.cached_world.friendly_team, friendly_colour)
        self.__update_robot_graphics(self.cached_world.enemy_team, enemy_colour)

        self.__update_robot_status_graphics()
        self.__update_speed_line_graphics()

    def __update_field_graphics(self, field: Field):
        """Update the GLGraphicsItems that display the field lines and markings
        
        :param field: The field proto

        """
        field_marking_rect_graphics = self.graphics_list.get_graphics(
            "field_marking_rects", 3
        )
        field_outer_boundary_rect = self.graphics_list.get_graphics(
            "field_outer_boundary_rect", 1
        )[0]
        field_marking_line_graphics = self.graphics_list.get_graphics(
            "field_marking_lines", 2
        )
        field_center_circle_graphic = self.graphics_list.get_graphics(
            "field_center_circle", 1
        )[0]

        # Outer field lines
        boundary_buffer = (2 * field.boundary_buffer_size)
        field_outer_boundary_rect.set_dimensions(
            field.field_x_length + boundary_buffer, field.field_y_length + boundary_buffer
        )
        field_marking_rect_graphics[0].set_dimensions(
            field.field_x_length, field.field_y_length
        )

        # Friendly defense area
        field_marking_rect_graphics[1].set_dimensions(
            field.defense_x_length, field.defense_y_length
        )
        field_marking_rect_graphics[1].set_position(
            -(field.field_x_length / 2) + (field.defense_x_length / 2), 0
        )

        # Enemy defense area
        field_marking_rect_graphics[2].set_dimensions(
            field.defense_x_length, field.defense_y_length
        )
        field_marking_rect_graphics[2].set_position(
            (field.field_x_length / 2) - (field.defense_x_length / 2), 0
        )

        # Halfway line
        field_marking_line_graphics[0].setData(
            pos=np.array(
                [[0, -(field.field_y_length / 2)], [0, (field.field_y_length / 2)]]
            ),
        )

        # Goal-to-goal line
        field_marking_line_graphics[1].setData(
            pos=np.array(
                [[-(field.field_x_length / 2), 0], [(field.field_x_length / 2), 0]]
            ),
        )

        # Center circle
        field_center_circle_graphic.set_radius(field.center_circle_radius)

    def __update_goal_graphics(self, field: Field):
        """Update the GLGraphicsItems that display the goals
        
        :param field: The field proto

        """
        goal_graphics = self.graphics_list.get_graphics("goals", 2)

        # Friendly goal
        goal_graphics[0].set_dimensions(field.goal_x_length, field.goal_y_length)
        goal_graphics[0].set_position(-field.field_x_length / 2, 0)
        goal_graphics[0].set_orientation(0)

        # Enemy goal
        goal_graphics[1].set_dimensions(field.goal_x_length, field.goal_y_length)
        goal_graphics[1].set_position(field.field_x_length / 2, 0)
        goal_graphics[1].set_orientation(180)

    def __update_ball_graphics(self, ball_state: BallState):
        """Update the GLGraphicsItems that display the ball
        
        :param ball_state: The ball state proto

        """
        # Update the ball graphic
        ball_graphic = self.graphics_list.get_graphics("ball", 1)[0]
        ball_graphic.set_position(
            ball_state.global_position.x_meters,
            ball_state.global_position.y_meters,
            ball_state.distance_from_ground,
        )

    def __update_robot_graphics(self, team: Team, color):
        """Update the GLGraphicsItems that display the robots
        
        :param team: The team proto
        :param color: The color of the robots

        """
        for robot_graphic, robot_id_graphic, robot in zip(
            self.graphics_list.get_graphics("robots", len(team.team_robots)),
            self.graphics_list.get_graphics("robot_ids", len(team.team_robots)),
            team.team_robots,
        ):
            robot_graphic.set_position(
                robot.current_state.global_position.x_meters,
                robot.current_state.global_position.y_meters,
            )
            robot_graphic.set_orientation(
                math.degrees(robot.current_state.global_orientation.radians)
            )
            robot_graphic.setColor(color)

            if self.display_robot_id:
                robot_id_graphic.show()
                robot_id_graphic.setData(
                    text=str(robot.id),
                    pos=[
                        robot.current_state.global_position.x_meters
                        + (ROBOT_MAX_RADIUS_METERS / 2),
                        robot.current_state.global_position.y_meters,
                        ROBOT_MAX_HEIGHT_METERS + 0.1,
                    ],
                )
            else:
                robot_id_graphic.hide()

    def __update_robot_status_graphics(self):
        """Update the robot status graphics"""
        self.cached_status = self.robot_status_buffer.get(block=False)

        for robot in self.cached_world.friendly_team.team_robots:
            if (
                self.cached_status.power_status.breakbeam_tripped is True
                and robot.id == self.cached_status.robot_id
            ):
                robot_status_graphic = self.graphics_list.get_graphics(
                    "robot_status", 1
                )[0]
                robot_status_graphic.set_radius(ROBOT_MAX_RADIUS_METERS / 2)
                robot_status_graphic.set_position(
                    robot.current_state.global_position.x_meters,
                    robot.current_state.global_position.y_meters,
                )

    def __update_speed_line_graphics(self):
        """Update the speed lines visualizing the robot and ball speeds"""

        # If user if trying to apply a velocity on the ball (i.e. kick it), visualize
        # the velocity vector
        if self.ball_velocity_vector:

            ball_state = self.cached_world.ball.current_state
            velocity = self.ball_velocity_vector * SPEED_SEGMENT_SCALE

            speed_line_graphic = self.graphics_list.get_graphics("speed_lines", 1)[0]
            speed_line_graphic.setData(
                pos=np.array(
                    [
                        [
                            ball_state.global_position.x_meters,
                            ball_state.global_position.y_meters,
                        ],
                        [
                            ball_state.global_position.x_meters + velocity.x(),
                            ball_state.global_position.y_meters + velocity.y(),
                        ],
                    ]
                ),
            )

        if self.display_speed_lines:

            # Combining robots and ball into one list so we can avoid
            # multiple calls to GraphicsList.get_graphics
            objects = list(self.cached_world.friendly_team.team_robots)
            objects.append(self.cached_world.ball)

            for speed_line_graphic, object in zip(
                self.graphics_list.get_graphics("speed_lines", len(objects)), objects
            ):
                pos_x = object.current_state.global_position.x_meters
                pos_y = object.current_state.global_position.y_meters
                velocity = object.current_state.global_velocity
                speed_line_graphic.setData(
                    pos=np.array(
                        [
                            [pos_x, pos_y],
                            [
                                pos_x
                                + velocity.x_component_meters * SPEED_SEGMENT_SCALE,
                                pos_y
                                + velocity.y_component_meters * SPEED_SEGMENT_SCALE,
                            ],
                        ]
                    ),
                )

    def __should_invert_coordinate_frame(self):
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

    def __invert_position_if_defending_negative_half(self, point):
        """If we are defending the negative half of the field, we invert the coordinate frame
        for the mouse click/3D point picked to match up with the visualization.

        :param point: The point location in the 3D scene [x, y]
        :return: The inverted point location [x, y] (if needed to be inverted)

        """
        if self.__should_invert_coordinate_frame():
            return [-point[0], -point[1]]

        return point
