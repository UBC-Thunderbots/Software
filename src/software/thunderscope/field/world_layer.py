import pyqtgraph as pg
import math

from pyqtgraph.Qt import QtCore, QtGui
from proto.world_pb2 import World, Field
from proto.team_pb2 import Robot, Team
from proto.ball_pb2 import Ball
from proto.vision_pb2 import RobotState, BallState
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.field.constants import ROBOT_MAX_RADIUS, BALL_RADIUS, MM_TO_M
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.colors as colors


class WorldLayer(FieldLayer):
    def __init__(self):
        FieldLayer.__init__(self)
        self.world_receiver = ThreadedUnixListener(
            "/tmp/tbots/TbotsProto.World", World, max_buffer_size=1
        )
        self.current_field = None
        self.cached_world = None

    def draw_field(self, painter, field: Field):

        # Draw Field Bounds
        painter.drawRect(
            QtCore.QRectF(
                -(field.field_x_length / 2) * MM_TO_M,
                (field.field_y_length / 2) * MM_TO_M,
                (field.field_x_length) * MM_TO_M,
                -(field.field_y_length) * MM_TO_M,
            )
        )

        # Draw Friendly Defense
        painter.drawRect(
            QtCore.QRectF(
                -(field.field_x_length / 2) * MM_TO_M,
                (field.defense_y_length / 2) * MM_TO_M,
                (field.defense_x_length) * MM_TO_M,
                -(field.defense_y_length) * MM_TO_M,
            )
        )

        # Draw Enemy Defense Area
        painter.drawRect(
            QtCore.QRectF(
                ((field.field_x_length / 2) - field.defense_x_length) * MM_TO_M,
                (field.defense_y_length / 2) * MM_TO_M,
                (field.defense_x_length) * MM_TO_M,
                -(field.defense_y_length) * MM_TO_M,
            )
        )

        # Draw Centre Cicle
        painter.drawEllipse(
            self.createCircle(0, 0, field.center_circle_radius * MM_TO_M)
        )

    def draw_team(self, painter, color, team: Team):

        for robot in team.team_robots:

            painter.setPen(pg.mkPen(color))
            painter.setBrush(pg.mkBrush(color))

            painter.drawEllipse(
                self.createCircle(
                    robot.current_state.global_position.x_meters * MM_TO_M,
                    robot.current_state.global_position.y_meters * MM_TO_M,
                    ROBOT_MAX_RADIUS,
                )
            )

    def draw_ball(self, painter, ball: Ball):

        painter.setPen(pg.mkPen(colors.BALL_COLOR))
        painter.setBrush(pg.mkBrush(colors.BALL_COLOR))

        painter.drawEllipse(
            self.createCircle(
                ball.current_state.global_position.x_meters * MM_TO_M,
                ball.current_state.global_position.y_meters * MM_TO_M,
                BALL_RADIUS,
            )
        )

    def paint(self, painter, option, widget):

        world = self.world_receiver.maybe_pop()

        if not world:
            world = self.cached_world

        self.cached_world = world
        field = world.field
        painter.setPen(pg.mkPen("w"))

        self.draw_field(painter, field)
        self.draw_team(painter, colors.BLUE_ROBOT_COLOR, world.friendly_team)
        self.draw_team(painter, colors.YELLOW_ROBOT_COLOR, world.enemy_team)
        self.draw_ball(painter, world.ball)
        self.current_field = field
