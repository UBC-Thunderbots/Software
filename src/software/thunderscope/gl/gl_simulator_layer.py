from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.opengl import *

from software.thunderscope.gl.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_ball import GLBall
from software.thunderscope.constants import Colors

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from extlibs.er_force_sim.src.protobuf.world_pb2 import (
    SimulatorState,
    SimBall,
    SimRobot,
)


class GLSimulatorLayer(GLLayer):
    """GLLayer that visualizes the simulator"""

    def __init__(self, friendly_colour_yellow, buffer_size=5):
        """Initialize the GLSimulatorLayer

        :param friendly_colour_yellow: Is the friendly_colour_yellow?
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        GLLayer.__init__(self)

        self.friendly_colour_yellow = friendly_colour_yellow
        self.simulator_state_buffer = ThreadSafeBuffer(buffer_size, SimulatorState)

        self.graphics_list.register_graphics_group(
            "ball", lambda: GLBall(color=Colors.SIM_BALL_COLOR)
        )

    def update_graphics(self):
        """Update the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems
        
        """
        # Clear all graphics in this layer if not visible
        if not self.isVisible():
            return self.graphics_list.get_changes()

        sim_world_state = self.simulator_state_buffer.get(block=False)

        ball_graphic = self.graphics_list.get_graphics("ball", 1)[0]

        # For some reason x and y are reversed?
        ball_graphic.set_position(
            sim_world_state.ball.p_y,
            -sim_world_state.ball.p_x,
            sim_world_state.ball.p_z,
        )

        return self.graphics_list.get_changes()
