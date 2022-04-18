import math

import software.python_bindings as geom
import pyqtgraph as pg
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *

from software.thunderscope.colors import Colors
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.constants import (
    BALL_RADIUS,
    MM_PER_M,
    ROBOT_MAX_RADIUS_MM,
    LINE_WIDTH,
)
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

MAX_ALLOWED_KICK_SPEED = 6.5


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
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee)
        self.cached_world = World()
        self.cached_status = {}

        self.ball_velocity_vector = None
        self.mouse_clicked = False

        self.key_pressed = {}

        self.accepted_keys = [Qt.Key.Key_Control, Qt.Key.Key_I]
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

    def keyReleaseEvent(self, event):
        """Detect when a key has been released (override)

        :param event: The event

        """
        self.key_pressed[event.key()] = False

    def __should_invert_coordinate_frame(self):
        """Our coordinate system is defined such that we are on the negative half
        of the field.

        If we are defending the positive half, we invert the coordinate frame
        and render the inverted proto. 

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
        for the mouse click.

        :param mouse_click: The mouse click location [x, y]
        :return: The inverted mouse click location [x, y] (if needed to be inverted)

        """
        if self.__should_invert_coordinate_frame():
            return [-mouse_click[0], -mouse_click[1]]

        return mouse_click

    def mouseMoveEvent(self, event):
        """We want to be able to use the mouse to pan/zoom the field.
        But we also want to be able to click on a things to interact with them.

        We need to make sure that when we are not handling the mouse events,
        the super class receives them.

        :param event: The event

        """
        self.mouse_move_pos = [event.pos().x(), event.pos().y()]

        ball_state = self.cached_world.ball.current_state

        ball_position = geom.Vector(
            ball_state.global_position.x_meters * MM_PER_M,
            ball_state.global_position.y_meters * MM_PER_M,
        )

        if self.key_pressed[Qt.Key.Key_Control] and self.mouse_clicked:
            self.ball_velocity_vector = ball_position - geom.Vector(
                self.mouse_move_pos[0], self.mouse_move_pos[1]
            )

            if self.ball_velocity_vector.length() > MAX_ALLOWED_KICK_SPEED * MM_PER_M:
                self.ball_velocity_vector = self.ball_velocity_vector.normalize(
                    MAX_ALLOWED_KICK_SPEED * MM_PER_M
                )

        else:
            super().mouseMoveEvent(event)

    def mousePressEvent(self, event):
        """We want to be able to use the mouse to pan/zoom the field.
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

        if self.key_pressed[Qt.Key.Key_Control]:
            world_state = WorldState()
            world_state.ball_state.CopyFrom(
                BallState(
                    global_position=Point(
                        x_meters=self.mouse_click_pos[0] / MM_PER_M,
                        y_meters=self.mouse_click_pos[1] / MM_PER_M,
                    )
                )
            )
            self.simulator_io.send_proto(WorldState, world_state)

        else:
            super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        """We want to be able to use the mouse to pan/zoom the field.
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
                        x_meters=self.mouse_click_pos[0] / MM_PER_M,
                        y_meters=self.mouse_click_pos[1] / MM_PER_M,
                    ),
                    global_velocity=Vector(
                        x_component_meters=self.ball_velocity_vector.x() / MM_PER_M,
                        y_component_meters=self.ball_velocity_vector.y() / MM_PER_M,
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
                    (pos_x - mouse_x / MM_PER_M) ** 2
                    + (pos_y - mouse_y / MM_PER_M) ** 2
                )
                <= ROBOT_MAX_RADIUS_MM / MM_PER_M
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
                -(field.field_x_length / 2) * MM_PER_M,
                (field.field_y_length / 2) * MM_PER_M,
                (field.field_x_length) * MM_PER_M,
                -(field.field_y_length) * MM_PER_M,
            )
        )

        # Draw Friendly Defense
        painter.drawRect(
            QtCore.QRectF(
                -(field.field_x_length / 2) * MM_PER_M,
                (field.defense_y_length / 2) * MM_PER_M,
                (field.defense_x_length) * MM_PER_M,
                -(field.defense_y_length) * MM_PER_M,
            )
        )

        # Draw Enemy Defense Area
        painter.drawRect(
            QtCore.QRectF(
                ((field.field_x_length / 2) - field.defense_x_length) * MM_PER_M,
                (field.defense_y_length / 2) * MM_PER_M,
                (field.defense_x_length) * MM_PER_M,
                -(field.defense_y_length) * MM_PER_M,
            )
        )

        # Draw Centre Circle
        painter.drawEllipse(
            self.createCircle(0, 0, field.center_circle_radius * MM_PER_M)
        )

    def draw_team(self, painter, color, team, robot_id_map):

        """Draw the team with robot IDs

        :param painter: The painter
        :param color: The color of the robots
        :param team: The team proto to draw
        :param robot_id_map: map of robot_id -> text_item for the team being drawn

        """
        convert_degree = -16

        for robot in team.team_robots:

            if robot.id not in robot_id_map:
                robot_id_font = painter.font()
                robot_id_font.setPointSize(int(ROBOT_MAX_RADIUS_MM / 4))

                # setting a black background to keep ID visible over yellow robot
                robot_id_text = pg.TextItem(str(robot.id), fill=(0, 0, 0, 0))
                robot_id_text.setFont(robot_id_font)
                robot_id_map[robot.id] = robot_id_text
                robot_id_text.setParentItem(self)

            robot_id_map[robot.id].setPos(
                (robot.current_state.global_position.x_meters * MM_PER_M)
                - ROBOT_MAX_RADIUS_MM,
                (robot.current_state.global_position.y_meters * MM_PER_M)
                - ROBOT_MAX_RADIUS_MM,
            )
            robot_id_map[robot.id].setVisible(self.key_pressed[Qt.Key.Key_I])

            painter.setPen(pg.mkPen(color))
            painter.setBrush(pg.mkBrush(color))

            painter.drawChord(
                self.createCircle(
                    robot.current_state.global_position.x_meters * MM_PER_M,
                    robot.current_state.global_position.y_meters * MM_PER_M,
                    ROBOT_MAX_RADIUS_MM,
                ),
                int((math.degrees(robot.current_state.global_orientation.radians) + 45))
                * convert_degree,
                270 * convert_degree,
            )

    def draw_ball_state(self, painter, ball_state: BallState):
        """Draw the ball

        :param painter: The painter
        :param ball_state: The ball state proto to draw

        """

        painter.setPen(pg.mkPen(Colors.BALL_COLOR, width=LINE_WIDTH))
        painter.setBrush(pg.mkBrush(Colors.BALL_COLOR))

        painter.drawEllipse(
            self.createCircle(
                ball_state.global_position.x_meters * MM_PER_M,
                ball_state.global_position.y_meters * MM_PER_M,
                BALL_RADIUS,
            )
        )

        if self.ball_velocity_vector and self.mouse_clicked:
            vector = self.ball_velocity_vector * 0.5

            polyline = QtGui.QPolygon(
                [
                    QtCore.QPoint(
                        int(MM_PER_M * ball_state.global_position.x_meters),
                        int(MM_PER_M * ball_state.global_position.y_meters),
                    ),
                    QtCore.QPoint(
                        int(MM_PER_M * ball_state.global_position.x_meters)
                        + vector.x(),
                        int(MM_PER_M * ball_state.global_position.y_meters)
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

        # TODO draw unavailable robot capabilities
        for robot in self.cached_world.friendly_team.team_robots:
            if (
                self.cached_status.break_beam_status.ball_in_beam is True
                and robot.id == self.cached_status.robot_id
            ):
                painter.drawEllipse(
                    self.createCircle(
                        robot.current_state.global_position.x_meters * MM_PER_M,
                        robot.current_state.global_position.y_meters * MM_PER_M,
                        ROBOT_MAX_RADIUS_MM / 2,
                    )
                )

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
