import math
from typing import override

import proto.import_all_protos as protos
from extlibs.er_force_sim.src.protobuf.world_pb2 import SimulatorState
from pyqtgraph.Qt import QtGui
from software.py_constants import BALL_MAX_RADIUS_METERS
from software.thunderscope.constants import Colors, DepthValues, LINE_WIDTH
from software.thunderscope.gl.graphics.gl_robot_outline import GLRobotOutline
from software.thunderscope.gl.graphics.gl_sphere import GLSphere
from software.thunderscope.gl.helpers.observable_list import ObservableList
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

# The fused robots are the same size and, with realism off, the same pose.
# A larger outline just above the field stays visible around them.
GHOST_ROBOT_SCALE = 1.45
GHOST_Z_METERS = 0.02
GHOST_BALL_RADIUS_SCALE = 2.5


class GLSimulatorLayer(GLLayer):
    """GLLayer that visualizes the simulator"""

    def __init__(
        self, name: str, friendly_colour_yellow: bool, buffer_size: int = 5
    ) -> None:
        """Initialize the GLSimulatorLayer

        :param name: The displayed name of the layer
        :param friendly_colour_yellow: Is the friendly_colour_yellow?
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        """
        super().__init__(name)
        self.setDepthValue(DepthValues.ABOVE_FOREGROUND_DEPTH)

        self.friendly_colour_yellow = friendly_colour_yellow
        self.defending_positive_side = False
        self.simulator_state_buffer = ThreadSafeBuffer(buffer_size, SimulatorState)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, protos.Referee)

        self.ball_graphic = GLSphere(
            parent_item=self,
            radius=BALL_MAX_RADIUS_METERS * GHOST_BALL_RADIUS_SCALE,
            color=Colors.SIM_BALL_COLOR,
        )

        # Enables transparency
        self.ball_graphic.setGLOptions("translucent")

        self.blue_robot_graphics = ObservableList(self._graphics_changed)
        self.yellow_robot_graphics = ObservableList(self._graphics_changed)

    def _update_defending_side(self) -> None:
        """Match SensorFusion: flip only when this team defends the positive half."""
        referee = self.referee_buffer.get(block=False)
        if not referee.HasField("blue_team_on_positive_half"):
            return

        if self.friendly_colour_yellow:
            self.defending_positive_side = not referee.blue_team_on_positive_half
        else:
            self.defending_positive_side = referee.blue_team_on_positive_half

    def _to_display_frame(
        self, x: float, y: float, angle: float = 0.0
    ) -> tuple[float, float, float]:
        """Map a simulator-frame pose into the same frame as the fused World.

        :param x: The x coordinate in the simulator's vision frame
        :param y: The y coordinate in the simulator's vision frame
        :param angle: The orientation in radians
        :return: x, y, and angle in the frame the fused World is drawn in
        """
        if self.defending_positive_side:
            return -x, -y, angle + math.pi
        return x, y, angle

    def _make_robot_outline(
        self, outline_color: QtGui.QColor, fill_color: QtGui.QColor
    ) -> GLRobotOutline:
        """Create a solid outline around robot and a translucent fill.

        :param outline_color: The color of the outline
        :param fill_color: The color of the fill
        :return: The robot outline
        """
        graphic = GLRobotOutline(
            outline_color=outline_color,
            fill_color=fill_color,
            line_width=LINE_WIDTH * 2,
        )
        graphic.scale(GHOST_ROBOT_SCALE, GHOST_ROBOT_SCALE, 1, local=True)
        graphic.translate(0, 0, GHOST_Z_METERS)
        if graphic.fill_graphic is not None:
            graphic.fill_graphic.setGLOptions("translucent")
        return graphic

    def _update_robot_graphics(
        self,
        robots,
        graphics: ObservableList,
        outline_color: QtGui.QColor,
        fill_color: QtGui.QColor,
    ) -> None:
        """Draw one robot outline for each simulator robot.

        :param robots: The simulator robots to draw
        :param graphics: The ObservableList containing the robot graphics for this team
        :param outline_color: The color of the outline
        :param fill_color: The color of the fill
        """
        graphics.resize(
            len(robots),
            lambda: self._make_robot_outline(outline_color, fill_color),
        )

        for graphic, robot in zip(graphics, robots):
            x, y, angle = self._to_display_frame(robot.p_x, robot.p_y, robot.angle)
            graphic.set_position(x, y)
            graphic.set_orientation(math.degrees(angle))
            graphic.show()

    @override
    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""
        self._update_defending_side()
        sim_world_state = self.simulator_state_buffer.get(block=False)

        # X and Y reversed due to ER-Force's coordinate conventions
        ball_x, ball_y, _ = self._to_display_frame(
            sim_world_state.ball.p_y, -sim_world_state.ball.p_x
        )
        self.ball_graphic.set_position(ball_x, ball_y, sim_world_state.ball.p_z)

        self._update_robot_graphics(
            sim_world_state.blue_robots,
            self.blue_robot_graphics,
            Colors.BLUE_ROBOT_COLOR,
            Colors.SIM_BLUE_ROBOT_COLOR,
        )
        self._update_robot_graphics(
            sim_world_state.yellow_robots,
            self.yellow_robot_graphics,
            Colors.YELLOW_ROBOT_COLOR,
            Colors.SIM_YELLOW_ROBOT_COLOR,
        )
