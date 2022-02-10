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

        # Draw Centre Cicle
        painter.drawEllipse(
            self.createCircle(0, 0, field.center_circle_radius * MM_PER_M)
        )

    def draw_team(self, painter, color, team: Team):
        """Draw the team

        :param painter: The painter
        :param color: The color of the robots
        :param team: The team proto to draw

        """

        for robot in team.team_robots:

            painter.setPen(pg.mkPen(colors.ROBOT_ID_COLOR))

            painter.setPen(pg.mkPen(colors.ROBOT_ID_COLOR))
            robot_id_bounding_rect = QtCore.QRectF(robot.current_state.global_position.x_meters * MM_PER_M - (1 * ROBOT_MAX_RADIUS),
                                                   robot.current_state.global_position.y_meters * MM_PER_M + (2 * ROBOT_MAX_RADIUS),
                                                   ROBOT_MAX_RADIUS * MM_PER_M,
                                                   ROBOT_MAX_RADIUS * MM_PER_M)

            robot_id_font = painter.font()
            robot_id_font.setPointSize((robot_id_bounding_rect.width()) / (ROBOT_MAX_RADIUS * 4))
            painter.setFont(robot_id_font)

            painter.drawText(robot_id_bounding_rect, str(robot.id))

            # scaling_factor = 1.0 / (robot_id_bounding_rect.width() / (ROBOT_MAX_RADIUS * 2))
            # unsuccessful scaling attempts:
            # QtGui.QTransform(scaling_factor, 0, 0, -scaling_factor, 0, 0)
            # QtGui.QTransform.scale(scaling_factor, scaling_factor)

            #painter.scale(1.0, -1.0)

            """
            Working code:
            painter.setPen(pg.mkPen(colors.ROBOT_ID_COLOR))

            robot_id_font = painter.font()
            robot_id_font.setPointSize(robot_id_bounding_rect.width() / (ROBOT_MAX_RADIUS * 4))
            painter.setFont(robot_id_font)

            robot_id_bounding_rect = QtCore.QRectF(robot.current_state.global_position.x_meters * MM_PER_M - (1 * ROBOT_MAX_RADIUS),
                                                   robot.current_state.global_position.y_meters * MM_PER_M + (2 * ROBOT_MAX_RADIUS),
                                                   ROBOT_MAX_RADIUS * MM_PER_M,
                                                   ROBOT_MAX_RADIUS * MM_PER_M)
                                                   
            painter.drawText(robot_id_bounding_rect, str(robot.id))
            """

            painter.setPen(pg.mkPen(color))
            painter.setBrush(pg.mkBrush(color))

            # TODO (#2396) Draw the robot IDs of the robots
            # TODO (#2397) Draw the Orientation of the robots
            painter.drawEllipse(
                self.createCircle(
                    robot.current_state.global_position.x_meters * MM_PER_M,
                    robot.current_state.global_position.y_meters * MM_PER_M,
                    ROBOT_MAX_RADIUS,
                )
            )

    def draw_ball(self, painter, ball: Ball):
        """Draw the ball

        :param painter: The painter
        :param ball: The ball proto to draw

        """

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

        # TODO (#2399) Figure out which team color _we_ are and update the color
        # passed into the team.
        self.draw_team(painter, colors.YELLOW_ROBOT_COLOR, world.friendly_team)
        self.draw_team(painter, colors.BLUE_ROBOT_COLOR, world.enemy_team)
