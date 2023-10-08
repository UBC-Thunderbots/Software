from pyqtgraph.opengl import *

from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_sphere import GLSphere
from software.py_constants import BALL_MAX_RADIUS_METERS
from software.thunderscope.constants import Colors

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from extlibs.er_force_sim.src.protobuf.world_pb2 import SimulatorState


class GLSimulatorLayer(GLLayer):
    """GLLayer that visualizes the simulator"""

    def __init__(self, name: str, friendly_colour_yellow: bool, buffer_size: int = 5):
        """Initialize the GLSimulatorLayer

        :param name: The displayed name of the layer
        :param friendly_colour_yellow: Is the friendly_colour_yellow?
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        GLLayer.__init__(self, name)

        self.friendly_colour_yellow = friendly_colour_yellow
        self.simulator_state_buffer = ThreadSafeBuffer(buffer_size, SimulatorState)

        self.ball_graphic = GLSphere(
            parentItem=self, radius=BALL_MAX_RADIUS_METERS, color=Colors.SIM_BALL_COLOR
        )

    def refresh_graphics(self):
        """Update graphics in this layer"""

        sim_world_state = self.simulator_state_buffer.get(block=False)

        self.ball_graphic.set_position(
            sim_world_state.ball.p_y,
            -sim_world_state.ball.p_x,
            sim_world_state.ball.p_z,
        )
