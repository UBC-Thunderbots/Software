import math
from typing import List

import software.python_bindings as geom
import pyqtgraph as pg
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt

from software.thunderscope.colors import Colors
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.constants import (
    BALL_RADIUS,
    MM_PER_M,
    ROBOT_MAX_RADIUS,
)
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

MAX_ALLOWED_KICK_SPEED = 6.5


class WorldLayer(FieldLayer):
    def __init__(self, simulator_io, friendly_colour_yellow, buffer_size=1):
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
        self.cached_world = World()

        self.ball_velocity_vector = None
        self.mouse_clicked = False

        self.pressed_CTRL = False
        self.pressed_M = False
        self.pressed_R = False

        self.mouse_click_pos = [0, 0]
        self.mouse_hover_pos = [0, 0]

    def keyPressEvent(self, event):
        """Detect when a key has been pressed (override)

        NOTE: function name format is different due to overriding the Qt function

        :param event: The event

        """
        if event.key() == Qt.Key.Key_R:
            self.pressed_R = True

        elif event.key() == Qt.Key.Key_Control:
            self.pressed_CTRL = True

        elif event.key() == Qt.Key.Key_M:
            self.pressed_M = True

    def keyReleaseEvent(self, event):
        """Detect when a key has been released (override)

        :param event: The event

        """
        if event.key() == Qt.Key.Key_R:
            self.pressed_R = False

        elif event.key() == Qt.Key.Key_Control:
            self.pressed_CTRL = False

        elif event.key() == Qt.Key.Key_M:
            self.pressed_M = False

    def mouseMoveEvent(self, event):
        """We want to be able to use the mouse to pan/zoom the field.
        But we also want to be able to click on a things to interact with them.

        We need to make sure that when we are not handling the mouse events,
        the super class receives them.

        :param event: The event

        """
        ball_state = self.cached_world.ball.current_state

        ball_position = geom.Vector(
            ball_state.global_position.x_meters * MM_PER_M,
            ball_state.global_position.y_meters * MM_PER_M,
        )

        if self.pressed_CTRL and self.mouse_clicked:
            self.ball_velocity_vector = ball_position - geom.Vector(
                event.pos().x(), event.pos().y()
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

        event.pos().x() / MM_PER_M
        event.pos().y() / MM_PER_M
        self.mouse_click_pos = [event.pos().x(), event.pos().y()]

        # determine whether a robot was clicked
        friendly_robot, enemy_robot = self.identify_robots(
            event.pos().x(), event.pos().y()
        )

        if self.pressed_CTRL:
            world_state = WorldState()
            world_state.ball_state.CopyFrom(
                BallState(
                    global_position=Point(
                        x_meters=event.pos().x() / MM_PER_M,
                        y_meters=event.pos().y() / MM_PER_M,
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
            world_state = WorldState()
            world_state.ball_state.CopyFrom(
                BallState(
                    global_position=Point(
                        x_meters=self.cached_world.ball.current_state.global_position.x_meters,
                        y_meters=self.cached_world.ball.current_state.global_position.y_meters,
                    ),
                    global_velocity=Vector(
                        x_component_meters=self.ball_velocity_vector.x() / MM_PER_M,
                        y_component_meters=self.ball_velocity_vector.y() / MM_PER_M,
                    ),
                )
            )

            self.simulator_io.send_proto(WorldState, world_state)
            self.ball_velocity_vector = None

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
                <= ROBOT_MAX_RADIUS / MM_PER_M
            ):
                return robot_
        return None

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

    def draw_robot_states(self, painter, color, robot_states: List[RobotState]):

        """Draw the robot states

        :param painter: The painter
        :param color: The color of the robots
        :param team: The team proto to draw

        """
        convert_degree = -16

        for robot_state in robot_states:

            painter.setPen(pg.mkPen(color))
            painter.setBrush(pg.mkBrush(color))

            # TODO (#2396) Draw the robot IDs of the robots
            painter.drawChord(
                self.createCircle(
                    robot_state.global_position.x_meters * MM_PER_M,
                    robot_state.global_position.y_meters * MM_PER_M,
                    ROBOT_MAX_RADIUS,
                ),
                int((math.degrees(robot_state.global_orientation.radians) + 45))
                * convert_degree,
                270 * convert_degree,
            )

    def draw_ball_state(self, painter, ball_state: BallState):
        """Draw the ball

        :param painter: The painter
        :param ball_state: The ball state proto to draw

        """

        painter.setPen(pg.mkPen(Colors.BALL_COLOR))
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
        self.draw_robot_states(
            painter,
            friendly_colour,
            [
                robot.current_state
                for robot in self.cached_world.friendly_team.team_robots
            ],
        )
        self.draw_robot_states(
            painter,
            enemy_colour,
            [robot.current_state for robot in self.cached_world.enemy_team.team_robots],
        )
