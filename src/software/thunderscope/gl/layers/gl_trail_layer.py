
from pyqtgraph.opengl import *

import math
import time
from queue import Queue

from proto.world_pb2 import World
from proto.import_all_protos import *

from software.thunderscope.constants import Colors, DepthValues
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLTrailLayer(GLLayer):
    """GLLayer that visualizes trails from the navigator"""

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLTrailLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.world_buffer = ThreadSafeBuffer(
            buffer_size, World
        )
        self.trail_graphics_head = ObservableList(self._graphics_changed)
        TRAIL_DURATION_MILLI = 200

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        self.cached_world = self.world_buffer.get(block=False)
        self.__update_trail_graphics(self.cached_world.friendly_team)
        # self.__update_trail_graphics(self.cached_world.enemy_team)


    def __update_trail_graphics(self, team: Team) -> None:
        self.trail_graphics_head.resize(
            len(team.team_robots) * 1,
            lambda: GLPolygon(
                outline_color=Colors.DEFAULT_GRAPHICS_COLOR,
            ),
            )
        robot_trails = [Queue(maxsize=300) for robot in team.team_robots]
        # for trail in
        for trail_graphics_head, robot in zip(
                self.trail_graphics_head,
                team.team_robots,
        ):
            trail_graphics_head.set_points(
                [
                    [robot.current_state.global_position.x_meters,
                    robot.current_state.global_position.y_meters,]
                ]
            )


class RobotTrailHistory():
    def __init__(self, robot, x_pos:float=0, y_pos: float=0):
        self.robot = robot
        self.x_point = x_pos
        self.y_point = y_pos


