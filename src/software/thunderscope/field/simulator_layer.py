import software.python_bindings as geom
import pyqtgraph as pg
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt

from software.thunderscope.colors import Colors
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.python_constants import *
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.field.world_layer import WorldLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from extlibs.er_force_sim.src.protobuf.world_pb2 import (
    SimulatorState,
    SimBall,
    SimRobot,
)


class SimulatorLayer(FieldLayer):
    def __init__(self, friendly_colour_yellow, buffer_size=5):
        """Visualize the simulator

        :param friendly_colour_yellow: Is the friendly_colour_yellow?
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        FieldLayer.__init__(self)
        self.friendly_colour_yellow = friendly_colour_yellow
        self.simulator_state_buffer = ThreadSafeBuffer(buffer_size, SimulatorState)

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        sim_world_state = self.simulator_state_buffer.get(block=False)

        painter.setPen(pg.mkPen(Colors.SIM_BALL_COLOR))
        painter.setBrush(pg.mkBrush(Colors.SIM_BALL_COLOR))

        # Draw the ball from the simulator state
        painter.drawEllipse(
            self.createCircle(
                sim_world_state.ball.p_y * MILLIMETERS_PER_METER,
                -sim_world_state.ball.p_x * MILLIMETERS_PER_METER,
                BALL_MAX_RADIUS_MILLIMETERS,
            )
        )
