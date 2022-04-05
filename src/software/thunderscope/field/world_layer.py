import math
import queue
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


class WorldLayer(FieldLayer):
    def __init__(self, simulator_io, friendly_colour_yellow, buffer_size=1):
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
        self.simulator_io = simulator_io
        self.friendly_colour_yellow = friendly_colour_yellow

    def keyPressEvent(self, event):
        """Detect when a key has been pressed (override)
        Note: function name format is different due to overriding the Qt function

        :param event: The event

        """
        if event.key() == Qt.Key.Key_R:
            # TODO (#2410) enter function to rotate the robot
            print("pressed R")
            self.pressed_R = True

        elif event.key() == Qt.Key.Key_Control:
            # TODO (#2410) enter function to move the ball
            print("pressed CTRL")
            self.pressed_CTRL = True

        elif event.key() == Qt.Key.Key_M:
            # TODO (#2410) enter function to move the robot
            print("pressed M")
            self.pressed_M = True

    def __setup_robots(self, robot_locations, team_colour):
        """Initializes the world from a list of robot locations

        :param robot_locations: A list of robot locations (index is robot id)
        :param team_colour: The color (either "blue" or "yellow")

        """
        world_state = WorldState()

        for x, robot_map in enumerate(
            [world_state.blue_robots, world_state.yellow_robots]
        ):

            for robot_id, robot_location in enumerate(robot_locations):
                robot_map[robot_id].CopyFrom(
                    RobotState(
                        global_position=Point(
                            x_meters=-3 if x == 0 else 3, y_meters=robot_location.y()
                        ),
                        global_orientation=Angle(radians=0),
                        global_velocity=Vector(
                            x_component_meters=0, y_component_meters=0
                        ),
                        global_angular_velocity=AngularVelocity(radians_per_second=0),
                    )
                )

        return world_state

    # Note: the function name formatting is different but this can't be changed
    # since it's overriding the built-in Qt function
    def keyReleaseEvent(self, event):
        """Detect when a key has been released (override)

        :param event: The event

        """
        if event.key() == Qt.Key.Key_R:
            # TODO (#2410) exit function to rotate the robot
            print("released R")
            self.pressed_R = False

        elif event.key() == Qt.Key.Key_Control:
            # TODO (#2410) exit function to move the ball
            self.pressed_CTRL = False
            print("released CTRL")

        elif event.key() == Qt.Key.Key_M:
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
        x = event.pos().x() / MM_PER_M
        y = event.pos().y() / MM_PER_M

        self.mouse_clicked = True
        self.mouse_click_pos = [event.pos().x(), event.pos().y()]

        # determine whether a robot was clicked
        self.identify_robots(event.pos().x(), event.pos().y())

        if self.pressed_CTRL:
            # world_state = self.__setup_robots(
            # [geom.Point(-3, x) for x in range(-2, 3)], "blue"
            # )
            world_state = WorldState()
            world_state.ball_state.CopyFrom(
                BallState(global_position=Point(x_meters=x, y_meters=y,),)
            )
            print("HYELO")
            self.simulator_io.send_proto(WorldState, world_state)

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

    def draw_ball_state(self, painter, ball_state: BallState, ball_color):
        """Draw the ball

        :param painter: The painter
        :param ball_state: The ball state proto to draw

        """

        painter.setPen(pg.mkPen(ball_color))
        painter.setBrush(pg.mkBrush(ball_color))
        painter.drawEllipse(
            self.createCircle(
                ball_state.global_position.x_meters * MM_PER_M,
                ball_state.global_position.y_meters * MM_PER_M,
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
        self.draw_ball_state(painter, world.ball.current_state, Colors.BALL_COLOR)

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
            [robot.current_state for robot in world.friendly_team.team_robots],
        )
        self.draw_robot_states(
            painter,
            enemy_colour,
            [robot.current_state for robot in world.enemy_team.team_robots],
        )
