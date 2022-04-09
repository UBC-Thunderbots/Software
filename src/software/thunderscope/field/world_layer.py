import math
import queue

import pyqtgraph as pg
from proto.ball_pb2 import Ball
from proto.team_pb2 import Robot, Team
from proto.vision_pb2 import BallState, RobotState
from proto.world_pb2 import Field, World
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt

from software.thunderscope.colors import Colors
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.constants import (
    BALL_RADIUS,
    MM_PER_M,
    ROBOT_MAX_RADIUS,
    UNIX_SOCKET_BASE_PATH,
)
from software.thunderscope.field.field_layer import FieldLayer


class WorldLayer(FieldLayer):
    def __init__(self, buffer_size=10):
        FieldLayer.__init__(self)
        self.cached_world = World()
        self.world_buffer = queue.Queue(buffer_size)
        self.setAcceptHoverEvents(True)
        self.setAcceptTouchEvents(True)
        self.pressed_CTRL = False
        self.pressed_M = False
        self.pressed_R = False
        self.mouse_clicked = False
        self.mouse_click_pos = [0, 0]
        self.mouse_hover_pos = [0, 0]  # might not need later, see hoverMoveEvent
        self.yellow_robot_id_text_items = {}
        self.blue_robot_id_text_items = {}

    def keyPressEvent(self, event):
        """Detect when a key has been pressed (override)
        Note: function name format is different due to overriding the Qt function

        :param event: The event

        """
        if event.key() == Qt.Key_R:
            # TODO (#2410) enter function to rotate the robot
            print("pressed R")
            self.pressed_R = True

        elif event.key() == Qt.Key_Control:
            # TODO (#2410) enter function to move the ball
            print("pressed CTRL")

        elif event.key() == Qt.Key_M:
            # TODO (#2410) enter function to move the robot
            print("pressed M")
            self.pressed_M = True

    # Note: the function name formatting is different but this can't be changed since it's overriding the built-in Qt function
    def keyReleaseEvent(self, event):
        """Detect when a key has been released (override)

        :param event: The event

        """
        if event.key() == Qt.Key_R:
            # TODO (#2410) exit function to rotate the robot
            print("released R")
            self.pressed_R = False

        elif event.key() == Qt.Key_Control:
            # TODO (#2410) exit function to move the ball
            self.pressed_CTRL = False
            print("released CTRL")

        elif event.key() == Qt.Key_M:
            # TODO (#2410) exit function to move the robot
            print("released M")
            self.pressed_M = False

    def hoverMoveEvent(self, event):
        """Detect where the mouse is hovering on the field (override)
        NOTE: Currently not used but may be useful in next part of (#2410)

        :param event: The event

        """
        self.mouse_hover_pos = [event.pos().x(), event.pos().y()]

    def mouseClickEvent(self, event):
        """Detect whether the mouse was clicked anywhere on the field (override)

        :param event: The event

        """
        # TODO (#2410) implement robot and ball interactivity through simulator, based on mouse and keyboard events

        # print the position of the mouse click
        print("x: " + str(event.pos().x() / MM_PER_M))
        print("y: " + str(event.pos().y() / MM_PER_M))

        self.mouse_clicked = True
        self.mouse_click_pos = [event.pos().x(), event.pos().y()]

        # determine whether a robot was clicked
        self.identify_robots(event.pos().x(), event.pos().y())

    def identify_robots(self, mouse_x, mouse_y):
        """Identify which robot was clicked on the field

        :param mouse_x: The x position of the mouse click
        :param mouse_y: The y position of the mouse click
        
        """
        self.identify_robot(
            mouse_x, mouse_y, self.cached_world.friendly_team.team_robots, "Friendly: "
        )
        self.identify_robot(
            mouse_x, mouse_y, self.cached_world.enemy_team.team_robots, "Enemy: "
        )

    def identify_robot(self, mouse_x, mouse_y, team, side):
        """Identify which robot was clicked on the team

        :param mouse_x: The x position of the mouse click
        :param mouse_y: The y position of the mouse click
        :param team: The team of robots to iterate over
        :param side: The label of the team, "Friendly" or "Enemy"

        """
        for robot_ in team:
            pos_x = robot_.current_state.global_position.x_meters
            pos_y = robot_.current_state.global_position.y_meters
            if (
                math.sqrt(
                    (pos_x - mouse_x / MM_PER_M) ** 2
                    + (pos_y - mouse_y / MM_PER_M) ** 2
                )
                <= ROBOT_MAX_RADIUS / MM_PER_M
            ):
                print(side)
                print(robot_.id)

    # Temporary draw function for testing purposes
    def draw_mouse_click_loc(self, painter):
        """Draw a circle indicating where the mouse was clicked on the field

        :param painter: The painter

        """
        painter.setPen(pg.mkPen("g", width=2))
        painter.drawEllipse(
            self.createCircle(
                self.mouse_click_pos[0], self.mouse_click_pos[1], BALL_RADIUS * 3,
            )
        )
        self.mouse_clicked = False

    def draw_field(self, painter, field: Field):

        """Draw the field

        :param painter: The painter
        :param field: The field proto to draw

        """
        painter.setPen(pg.mkPen("w", width=2))

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

    def draw_team(self, painter, color, team: Team, robot_id_map):

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
                robot_id_font.setPointSize(ROBOT_MAX_RADIUS / 7)
                # setting a black background to keep ID visible over yellow robot
                robot_id_text = pg.TextItem(
                    html='<span style="color: #FFF; background-color: #000">'
                    + str(robot.id)
                    + "</span>"
                )
                robot_id_map[robot.id] = robot_id_text
                robot_id_text.setParentItem(self)

            robot_id_map[robot.id].setPos(
                (robot.current_state.global_position.x_meters * MM_PER_M)
                - ROBOT_MAX_RADIUS,
                robot.current_state.global_position.y_meters * MM_PER_M,
            )

            painter.setPen(pg.mkPen(color))
            painter.setBrush(pg.mkBrush(color))

            painter.drawChord(
                self.createCircle(
                    robot.current_state.global_position.x_meters * MM_PER_M,
                    robot.current_state.global_position.y_meters * MM_PER_M,
                    ROBOT_MAX_RADIUS,
                ),
                int((math.degrees(robot.current_state.global_orientation.radians) + 45))
                * convert_degree,
                270 * convert_degree,
            )

    def draw_ball(self, painter, ball: Ball):
        """Draw the ball

        :param painter: The painter
        :param ball: The ball proto to draw

        """

        painter.setPen(pg.mkPen(Colors.BALL_COLOR))
        painter.setBrush(pg.mkBrush(Colors.BALL_COLOR))
        painter.drawEllipse(
            self.createCircle(
                ball.current_state.global_position.x_meters * MM_PER_M,
                ball.current_state.global_position.y_meters * MM_PER_M,
                BALL_RADIUS,
            )
        )

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        try:
            world = self.world_buffer.get_nowait()
        except queue.Empty as empty:
            world = self.cached_world

        self.cached_world = world
        self.draw_field(painter, world.field)
        self.draw_ball(painter, world.ball)

        # temporary function call for testing purposes
        self.draw_mouse_click_loc(painter)

        # TODO (#2399) Figure out which team color _we_ are and update the color
        # passed into the team.
        self.draw_team(
            painter,
            Colors.YELLOW_ROBOT_COLOR,
            world.friendly_team,
            self.yellow_robot_id_text_items,
        )
        self.draw_team(
            painter,
            Colors.BLUE_ROBOT_COLOR,
            world.enemy_team,
            self.blue_robot_id_text_items,
        )
