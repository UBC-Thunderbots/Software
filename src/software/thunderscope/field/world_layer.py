import pyqtgraph as pg
import math

from pyqtgraph.Qt import QtCore, QtGui
from proto.world_pb2 import World, Field
from proto.team_pb2 import Robot, Team
from proto.ball_pb2 import Ball
from proto.vision_pb2 import RobotState, BallState
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.constants import (
    ROBOT_MAX_RADIUS,
    BALL_RADIUS,
    MM_PER_M,
    UNIX_SOCKET_BASE_PATH,
    KEY_R,
    KEY_CTRL,
    KEY_M,
)
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.colors as colors

class WorldLayer(FieldLayer):

    def __init__(self):
        FieldLayer.__init__(self)
        self.world_receiver = ThreadedUnixListener(
            UNIX_SOCKET_BASE_PATH + World.DESCRIPTOR.full_name, World, max_buffer_size=1
        )
        self.cached_world = World()
        self.setAcceptHoverEvents(True)
        self.setAcceptTouchEvents(True)
        self.pressed_R = False
        self.pressed_CTRL = False
        self.pressed_M = False
        self.mouse_clicked = False
        self.mouse_click_pos = [0,0]
        self.mouse_hover_pos = [0,0]
        self.override_ball_loc = False
    
    def keyPressEvent(self, event):
        if event.key() == KEY_R:
            print("pressed R")
            self.pressed_R = True
            # enter function to rotate robot
        elif event.key() == KEY_CTRL:
            self.pressed_CTRL = True
            print("pressed CTRL")
            # enter function to move ball
        elif event.key() == KEY_M:
            print("pressed M")
            self.pressed_M = True
            # enter function to move robot
    
    def keyReleaseEvent(self, event):
        if event.key() == KEY_R:
            print("released R")
            self.pressed_R = False
            # enter function to rotate robot
        elif event.key() == KEY_CTRL:
            self.pressed_CTRL = False
            print("released CTRL")
            # enter function to move ball
        elif event.key() == KEY_M:
            print("released M")
            self.pressed_M = False
            # enter function to move robot

    def hoverMoveEvent(self, event):
        self.mouse_hover_pos = [event.pos().x(), event.pos().y()]
        # print(event.pos())

    def mouseClickEvent(self, event):
        print(event.pos())
        self.mouse_click_pos = [event.pos().x(), event.pos().y()]
        self.mouse_clicked = True
        if self.pressed_CTRL:
            self.override_ball_loc = True
        # see if there is a robot here and if there is, then "select" it
        # wait on a keypress event for what action to take
    
    def draw_mouse_click_loc(self, painter):
        """Draw a circle indicating where the mouse was clicked on the field

        :param painter: The painter

        """

        painter.setPen(pg.mkPen(colors.BLUE_ROBOT_COLOR))
        painter.setBrush(pg.mkBrush(colors.BLUE_ROBOT_COLOR))
        painter.drawEllipse(
            self.createCircle(
                self.mouse_click_pos[0], self.mouse_click_pos[1], BALL_RADIUS*3,
            )
        )
        self.mouse_clicked = False
    
    def draw_key_press_ctrl_mouse(self, painter):
        if self.pressed_CTRL and self.mouse_clicked:
            painter.setPen(pg.mkPen(colors.YELLOW_ROBOT_COLOR))
            painter.setBrush(pg.mkBrush(colors.YELLOW_ROBOT_COLOR))
            painter.drawEllipse(
                self.createCircle(
                    self.mouse_click_pos[0], self.mouse_click_pos[1], BALL_RADIUS*2,
                )
            )       
        self.mouse_clicked = False

    def draw_field(self, painter, field: Field):

        """Draw the field

        :param painter: The painter
        :param field: The field proto to draw

        """
        painter.setPen(pg.mkPen("w"))

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

    def draw_team(self, painter, color, team: Team):

        """Draw the team

        :param painter: The painter
        :param color: The color of the robots
        :param team: The team proto to draw

        """

        convert_degree = -16

        for robot in team.team_robots:

            painter.setPen(pg.mkPen(color))
            painter.setBrush(pg.mkBrush(color))

            # TODO (#2396) Draw the robot IDs of the robots
            painter.drawChord(
                self.createCircle(
                    robot.current_state.global_position.x_meters * MM_PER_M,
                    robot.current_state.global_position.y_meters * MM_PER_M,
                    ROBOT_MAX_RADIUS,
                ),
                (math.degrees(robot.current_state.global_orientation.radians) + 45)
                * convert_degree,
                270 * convert_degree,
            )

    def draw_ball(self, painter, ball: Ball):
        """Draw the ball

        :param painter: The painter
        :param ball: The ball proto to draw

        """
        if self.override_ball_loc and self.pressed_CTRL:
            painter.setPen(pg.mkPen(colors.BALL_COLOR))
            painter.setBrush(pg.mkBrush(colors.BALL_COLOR))
            painter.drawEllipse(
                self.createCircle(
                    self.mouse_click_pos[0],
                    self.mouse_click_pos[1],
                    BALL_RADIUS,
                )
            )
        else:
            painter.setPen(pg.mkPen(colors.BALL_COLOR))
            painter.setBrush(pg.mkBrush(colors.BALL_COLOR))
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

        world = self.world_receiver.maybe_pop()

        if not world:
            world = self.cached_world
        self.cached_world = world
        self.draw_field(painter, world.field)
        self.draw_ball(painter, world.ball)
        
        # temporary draw function - draws the last location where a mouseClickEvent occurrred
        # self.draw_mouse_click_loc(painter)

        self.draw_key_press_ctrl_mouse(painter)

        # TODO (#2399) Figure out which team color _we_ are and update the color
        # passed into the team.
        self.draw_team(painter, colors.YELLOW_ROBOT_COLOR, world.friendly_team)
        self.draw_team(painter, colors.BLUE_ROBOT_COLOR, world.enemy_team)
