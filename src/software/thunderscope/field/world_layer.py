import math

import software.python_bindings as tbots_cpp
import pyqtgraph as pg
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *

from proto.geometry_pb2 import Point, Segment
from software.py_constants import *
from software.thunderscope.constants import (
    BALL_HEIGHT_EFFECT_MULTIPLIER,
    LINE_WIDTH,
    SPEED_LINE_WIDTH,
    SPEED_SEGMENT_SCALE,
)
from software.thunderscope.constants import Colors
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

MAX_ALLOWED_KICK_SPEED_M_PER_S = 6.5


class WorldLayer(FieldLayer):
    def __init__(self, simulator_io, friendly_colour_yellow, buffer_size=5):
        """The WorldLayer

        :param simulator_io: The simulator io communicate with the simulator
        :param friendly_colour_yellow: Is the friendly_colour_yellow?
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        FieldLayer.__init__(self)

        self.simulator_io = simulator_io
        self.friendly_colour_yellow = friendly_colour_yellow

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.robot_status_buffer = ThreadSafeBuffer(buffer_size, RobotStatus)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee, False)
        self.cached_world = World()
        self.cached_status = {}

        self.ball_velocity_vector = None
        self.mouse_clicked = False

        self.key_pressed = {}
        self.display_robot_id = False

        self.is_playing = True

        self.accepted_keys = [Qt.Key.Key_Control, Qt.Key.Key_I, QtCore.Qt.Key.Key_Space]
        for key in self.accepted_keys:
            self.key_pressed[key] = False

        self.friendly_robot_id_text_items = {}
        self.enemy_robot_id_text_items = {}

    def keyPressEvent(self, event):
        """Detect when a key has been pressed (override)

        NOTE: function name format is different due to overriding the Qt function

        :param event: The event

        """
        self.key_pressed[event.key()] = True
        if event.key() == QtCore.Qt.Key.Key_I:
            self.display_robot_id = not self.display_robot_id

        # if user is holding ctrl + space, send a command to simulator to pause the gameplay
        if (
            self.key_pressed[QtCore.Qt.Key.Key_Control]
            and self.key_pressed[QtCore.Qt.Key.Key_Space]
        ):

            simulator_state = SimulationState(is_playing=not self.is_playing)
            self.is_playing = not self.is_playing

            self.simulator_io.send_proto(SimulationState, simulator_state)

    def keyReleaseEvent(self, event):
        """Detect when a key has been released (override)

        :param event: The event

        """
        self.key_pressed[event.key()] = False

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

    def __invert_mouse_position_if_defending_negative_half(self, mouse_click):
        """If we are defending the negative half of the field, we invert the coordinate frame
        for the mouse click to match up with the visualization.

        :param mouse_click: The mouse click location [x, y]
        :return: The inverted mouse click location [x, y] (if needed to be inverted)

        """
        if self.__should_invert_coordinate_frame():
            return [-mouse_click[0], -mouse_click[1]]

        return mouse_click

    def mouseMoveEvent(self, event):
        """Handle mouse movement

        NOTE: We want to be able to use the mouse to pan/zoom the field.
        But we also want to be able to click on a things to interact with them.

        We need to make sure that when we are not handling the mouse events,
        the super class receives them.

        :param event: The event

        """
        mouse_move_pos = [event.pos().x(), event.pos().y()]

        ball_position = tbots_cpp.Vector(
            self.cached_world.ball.current_state.global_position.x_meters
            * MILLIMETERS_PER_METER,
            self.cached_world.ball.current_state.global_position.y_meters
            * MILLIMETERS_PER_METER,
        )

        # If the control key is pressed and the mouse was clicked,
        # this means that we are dragging the mouse on the screen
        # to apply a velocity on the ball (to kick it). We create a
        # velocity vector that is proportional to the distance the
        # mouse has moved away from the ball.
        if self.key_pressed[Qt.Key.Key_Control] and self.mouse_clicked:
            self.ball_velocity_vector = ball_position - tbots_cpp.Vector(
                mouse_move_pos[0], mouse_move_pos[1]
            )

            # Cap the maximum kick speed
            if (
                self.ball_velocity_vector.length()
                > MAX_ALLOWED_KICK_SPEED_M_PER_S * MILLIMETERS_PER_METER
            ):
                self.ball_velocity_vector = self.ball_velocity_vector.normalize(
                    MAX_ALLOWED_KICK_SPEED_M_PER_S * MILLIMETERS_PER_METER
                )

        # This is key to not break the panning/zooming features
        else:
            super().mouseMoveEvent(event)

    def mousePressEvent(self, event):
        """Handle mouse clicks

        NOTE: We want to be able to use the mouse to pan/zoom the field.
        But we also want to be able to click on a things to interact with them.

        We need to make sure that when we are not handling the mouse events,
        the super class receives them.

        :param event: The event

        """
        self.mouse_clicked = True
        self.mouse_click_pos = [event.pos().x(), event.pos().y()]
        self.mouse_click_pos = self.__invert_mouse_position_if_defending_negative_half(
            self.mouse_click_pos
        )

        # determine whether a robot was clicked
        friendly_robot, enemy_robot = self.identify_robots(*self.mouse_click_pos)

        # If the user was holding ctrl, send a command to the simulator to move
        # the ball to the mouse click location.
        if self.key_pressed[Qt.Key.Key_Control]:
            world_state = WorldState()
            world_state.ball_state.CopyFrom(
                BallState(
                    global_position=Point(
                        x_meters=self.mouse_click_pos[0] / MILLIMETERS_PER_METER,
                        y_meters=self.mouse_click_pos[1] / MILLIMETERS_PER_METER,
                    )
                )
            )
            self.simulator_io.send_proto(WorldState, world_state)

        # This is key to not break the panning/zooming features
        else:
            super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        """Handle mouse clicks

        NOTE: We want to be able to use the mouse to pan/zoom the field.
        But we also want to be able to click on a things to interact with them.

        We need to make sure that when we are not handling the mouse events,
        the super class receives them.

        :param event: The event

        """
        self.mouse_clicked = False

        if self.ball_velocity_vector:

            if self.__should_invert_coordinate_frame():
                self.ball_velocity_vector = -self.ball_velocity_vector

            world_state = WorldState()
            world_state.ball_state.CopyFrom(
                BallState(
                    global_position=Point(
                        x_meters=self.mouse_click_pos[0] / MILLIMETERS_PER_METER,
                        y_meters=self.mouse_click_pos[1] / MILLIMETERS_PER_METER,
                    ),
                    global_velocity=Vector(
                        x_component_meters=self.ball_velocity_vector.x()
                        / MILLIMETERS_PER_METER,
                        y_component_meters=self.ball_velocity_vector.y()
                        / MILLIMETERS_PER_METER,
                    ),
                )
            )

            self.ball_velocity_vector = None
            self.simulator_io.send_proto(WorldState, world_state)

        else:
            super().mouseReleaseEvent(event)

    def identify_robots(self, mouse_x, mouse_y):
        """Identify which robot was clicked on the field

        :param mouse_x: The x position of the mouse click
        :param mouse_y: The y position of the mouse click
        
        """
        friendly_robot = self.identify_robot(
            mouse_x, mouse_y, self.cached_world.friendly_team.team_robots
        )
        enemy_robot = self.identify_robot(
            mouse_x, mouse_y, self.cached_world.enemy_team.team_robots
        )

        return friendly_robot, enemy_robot

    def identify_robot(self, mouse_x, mouse_y, team):
        """Identify which robot was clicked on the team

        :param mouse_x: The x position of the mouse click
        :param mouse_y: The y position of the mouse click
        :param team: The team of robots to iterate over

        """
        for robot_ in team:
            pos_x = robot_.current_state.global_position.x_meters
            pos_y = robot_.current_state.global_position.y_meters
            if (
                math.sqrt(
                    (pos_x - mouse_x / MILLIMETERS_PER_METER) ** 2
                    + (pos_y - mouse_y / MILLIMETERS_PER_METER) ** 2
                )
                <= ROBOT_MAX_RADIUS_MILLIMETERS / MILLIMETERS_PER_METER
            ):
                return robot_
        return None

    def draw_field(self, painter, field: Field):

        """Draw the field

        :param painter: The painter
        :param field: The field proto to draw

        """
        painter.setPen(pg.mkPen("w", width=LINE_WIDTH))

        # Draw Field Bounds
        painter.drawRect(
            QtCore.QRectF(
                -(field.field_x_length / 2) * MILLIMETERS_PER_METER,
                (field.field_y_length / 2) * MILLIMETERS_PER_METER,
                (field.field_x_length) * MILLIMETERS_PER_METER,
                -(field.field_y_length) * MILLIMETERS_PER_METER,
            )
        )

        # Draw Friendly Defense
        painter.drawRect(
            QtCore.QRectF(
                -(field.field_x_length / 2) * MILLIMETERS_PER_METER,
                (field.defense_y_length / 2) * MILLIMETERS_PER_METER,
                (field.defense_x_length) * MILLIMETERS_PER_METER,
                -(field.defense_y_length) * MILLIMETERS_PER_METER,
            )
        )

        # Draw Enemy Defense Area
        painter.drawRect(
            QtCore.QRectF(
                ((field.field_x_length / 2) - field.defense_x_length)
                * MILLIMETERS_PER_METER,
                (field.defense_y_length / 2) * MILLIMETERS_PER_METER,
                (field.defense_x_length) * MILLIMETERS_PER_METER,
                -(field.defense_y_length) * MILLIMETERS_PER_METER,
            )
        )

        # Draw Friendly Goal
        painter.drawRect(
            QtCore.QRectF(
                -(field.field_x_length / 2 + field.goal_x_length)
                * MILLIMETERS_PER_METER,
                (field.goal_y_length / 2) * MILLIMETERS_PER_METER,
                (field.goal_x_length) * MILLIMETERS_PER_METER,
                -(field.goal_y_length) * MILLIMETERS_PER_METER,
            )
        )

        # Draw Enemy Goal
        painter.drawRect(
            QtCore.QRectF(
                (field.field_x_length / 2) * MILLIMETERS_PER_METER,
                (field.goal_y_length / 2) * MILLIMETERS_PER_METER,
                (field.goal_x_length) * MILLIMETERS_PER_METER,
                -(field.goal_y_length) * MILLIMETERS_PER_METER,
            )
        )

        # Draw Centre Circle
        painter.drawEllipse(
            self.createCircle(Point(x_meters=0, y_meters=0), field.center_circle_radius)
        )

    def draw_team(self, painter, color, team, robot_id_map):

        """Draw the team with robot IDs

        :param painter: The painter
        :param color: The color of the robots
        :param team: The team proto to draw
        :param robot_id_map: map of robot_id -> text_item for the team being drawn

        """
        for robot in team.team_robots:

            # Draw robot ID
            if robot.id not in robot_id_map:
                robot_id_font = painter.font()
                robot_id_font.setPointSize(int(ROBOT_MAX_RADIUS_MILLIMETERS / 4))

                # setting a black background to keep ID visible over yellow robot
                robot_id_text = pg.TextItem(str(robot.id), fill=(0, 0, 0, 0))
                robot_id_text.setFont(robot_id_font)
                robot_id_map[robot.id] = robot_id_text
                robot_id_text.setParentItem(self)

            robot_id_map[robot.id].setPos(
                (robot.current_state.global_position.x_meters * MILLIMETERS_PER_METER)
                - ROBOT_MAX_RADIUS_MILLIMETERS,
                (robot.current_state.global_position.y_meters * MILLIMETERS_PER_METER)
                - ROBOT_MAX_RADIUS_MILLIMETERS,
            )
            robot_id_map[robot.id].setVisible(self.display_robot_id)

            # Draw robot
            painter.setPen(pg.mkPen(color))
            painter.setBrush(pg.mkBrush(color))
            self.drawRobot(
                robot.current_state.global_position,
                robot.current_state.global_orientation,
                painter,
            )

    def draw_ball_state(self, painter, ball_state: BallState):
        """Draw the ball

        :param painter: The painter
        :param ball_state: The ball state proto to draw

        """

        painter.setPen(pg.mkPen(Colors.BALL_COLOR, width=LINE_WIDTH))
        painter.setBrush(pg.mkBrush(Colors.BALL_COLOR))

        # Ball should get larger as the height of the ball increases
        ball_radius = BALL_MAX_RADIUS_METERS * (
            1 + BALL_HEIGHT_EFFECT_MULTIPLIER * ball_state.distance_from_ground
        )

        painter.drawEllipse(self.createCircle(ball_state.global_position, ball_radius))

        # If the mouse is being dragged on the screen, visualize
        # the ball velocity vector. The 0.5 scaling is abitrary
        if self.ball_velocity_vector:
            vector = self.ball_velocity_vector

            polyline = QtGui.QPolygon(
                [
                    QtCore.QPoint(
                        int(
                            MILLIMETERS_PER_METER * ball_state.global_position.x_meters
                        ),
                        int(
                            MILLIMETERS_PER_METER * ball_state.global_position.y_meters
                        ),
                    ),
                    QtCore.QPoint(
                        int(MILLIMETERS_PER_METER * ball_state.global_position.x_meters)
                        + vector.x(),
                        int(MILLIMETERS_PER_METER * ball_state.global_position.y_meters)
                        + vector.y(),
                    ),
                ]
            )

            painter.drawPolyline(polyline)

    def draw_robot_status(self, painter):
        """Draw the robot status

        :param painter: The painter

        """
        self.cached_status = self.robot_status_buffer.get(block=False)

        painter.setBrush(pg.mkBrush(None))
        painter.setPen(pg.mkPen("r", width=LINE_WIDTH))

        for robot in self.cached_world.friendly_team.team_robots:
            if (
                self.cached_status.power_status.breakbeam_tripped is True
                and robot.id == self.cached_status.robot_id
            ):
                painter.drawEllipse(
                    self.createCircle(
                        robot.current_state.global_position, ROBOT_MAX_RADIUS_METERS / 2
                    )
                )

    def draw_robot_speeds(self, painter):
        """Draw the robot speeds

        :param painter: The painter

        """
        painter.setPen(pg.mkPen(Colors.SPEED_VECTOR_COLOR, width=SPEED_LINE_WIDTH))

        for robot in self.cached_world.friendly_team.team_robots:
            velocity = robot.current_state.global_velocity
            start = robot.current_state.global_position
            end = Point(
                x_meters=start.x_meters
                + velocity.x_component_meters * SPEED_SEGMENT_SCALE,
                y_meters=start.y_meters
                + velocity.y_component_meters * SPEED_SEGMENT_SCALE,
            )
            speed_line = Segment(start=start, end=end)
            self.drawSegment(speed_line, painter)

    def draw_ball_speed(self, painter):
        """Draw the ball speed

        :param painter: The painter

        """
        painter.setPen(pg.mkPen(Colors.SPEED_VECTOR_COLOR, width=SPEED_LINE_WIDTH))

        ball = self.cached_world.ball
        velocity = ball.current_state.global_velocity
        start = ball.current_state.global_position
        end = Point(
            x_meters=start.x_meters + velocity.x_component_meters * SPEED_SEGMENT_SCALE,
            y_meters=start.y_meters + velocity.y_component_meters * SPEED_SEGMENT_SCALE,
        )
        speed_line = Segment(start=start, end=end)
        self.drawSegment(speed_line, painter)

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """
        self.cached_world = self.world_buffer.get(block=False)

        self.draw_field(painter, self.cached_world.field)
        self.draw_ball_state(painter, self.cached_world.ball.current_state)

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

        self.draw_team(
            painter,
            friendly_colour,
            self.cached_world.friendly_team,
            self.friendly_robot_id_text_items,
        )
        self.draw_team(
            painter,
            enemy_colour,
            self.cached_world.enemy_team,
            self.enemy_robot_id_text_items,
        )
        self.draw_robot_status(painter)
        self.draw_robot_speeds(painter)
        self.draw_ball_speed(painter)
