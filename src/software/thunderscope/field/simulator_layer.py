import queue

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
from software.thunderscope.field.world_layer import WorldLayer


class SimulatorLayer(FieldLayer):
    def __init__(self, simulator_io, buffer_size=1):
        FieldLayer.__init__(self)
        self.sim_world_state_buffer = queue.Queue(buffer_size)
        self.world_layer = WorldLayer(simulator_io)
        self.cached_sim_world_state = WorldState()

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        print("REDRAW")
        try:
            sim_world_state = self.sim_world_state_buffer.get_nowait()
        except queue.Empty as empty:
            sim_world_state = self.cached_sim_world_state

        self.cached_sim_world_state = sim_world_state
        self.world_layer.draw_ball_state(
            painter, sim_world_state.ball_state, Colors.SIM_BALL_COLOR
        )

        # TODO (#2399) Figure out which team color _we_ are and update the color
        # passed into the team.
        self.world_layer.draw_robot_states(
            painter, Colors.YELLOW_ROBOT_COLOR, sim_world_state.yellow_robots.values()
        )
        self.world_layer.draw_robot_states(
            painter, Colors.BLUE_ROBOT_COLOR, sim_world_state.blue_robots.values()
        )
