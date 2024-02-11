from pyqtgraph.opengl import *

from collections import deque
from proto.world_pb2 import World
from proto.import_all_protos import Team

from software.thunderscope.constants import Colors, DepthValues, TrailValues
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
        self._queuesExist = False
        self.maxTrailLength = TrailValues.DEFAULT_TRAIL_LENGTH
        self.trailSampleRate = TrailValues.DEFAULT_TRAIL_SAMPLING_RATE
        self.robot_trail_queues = None
        self.cached_world = World()

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        self.cached_world = self.world_buffer.get(block=False)

        # Ensures queues are created once upon first receival of world buffer
        if not self._queuesExist and self.cached_world is not None:
            self.robot_trail_queues = [
                deque([], self.maxTrailLength)
                for _ in self.cached_world.friendly_team.team_robots
            ]
            self._queuesExist = True

        if self._queuesExist:
            self.__update_trail_graphics(
                self.cached_world.friendly_team,
                self.robot_trail_queues,
                Colors.DEFAULT_GRAPHICS_COLOR,
            )

    def __update_trail_points(self, robot_queue, robot):

        robot_queue.append(
            robot.current_state.global_position
        )

    def __update_trail_graphics(self, team: Team, queues, color: Colors) -> None:
        self.trail_graphics_head.resize(
            len(team.team_robots) * 1,
            lambda: GLPolygon(
                outline_color=color,
            ),
        )
        # Update trail points
        for robot_trail, robot in zip(
                queues,
                team.team_robots,
        ):
            self.__update_trail_points(robot_trail, robot)

        # Draws trail graphics
        for trail_graphics_head, trail_queue in zip(
                self.trail_graphics_head,
                queues,
        ):
            trail_graphics_head.set_points(
                [[point.x_meters, point.y_meters] for point in trail_queue]
            )