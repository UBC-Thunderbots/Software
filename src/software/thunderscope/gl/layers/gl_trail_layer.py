from pyqtgraph.opengl import *

from collections import deque, defaultdict
from proto.world_pb2 import World
from proto.import_all_protos import Team, Robot

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

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.trail_graphics_head = ObservableList(self._graphics_changed)
        self.robot_trail_queues = defaultdict(lambda: deque([], self.max_trail_length))
        self.cached_world = World()

        self.max_trail_length = TrailValues.DEFAULT_TRAIL_LENGTH
        self.refresh_interval = TrailValues.DEFAULT_TRAIL_SAMPLING_RATE
        self.refresh_count = self.refresh_interval

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        self.cached_world = self.world_buffer.get(block=False, return_cached=False)

        if self.refresh_count <= 0 and self.cached_world is not None:
            self.refresh_count = self.refresh_interval
            for robot in self.cached_world.friendly_team.team_robots:
                self.__update_trail_points(robot)

            self.__update_trail_graphics(
                self.cached_world.friendly_team,
                self.robot_trail_queues,
                Colors.DEFAULT_GRAPHICS_COLOR,
            )

        self.refresh_count -= 1

    def __update_trail_points(self, robot: Robot) -> None:
        """Stores the robot's current global position in its trail history in the world

        :param robot: the given robot whose position is to be stored
        """
        self.robot_trail_queues[robot.id].append(robot.current_state.global_position)

    def __update_trail_graphics(
        self, team: Team, queues_dict: dict, color: Colors
    ) -> None:
        """Updates the onscreen visualizations of the past robot positions correlating to the
           given team.

        :param team: Team proto containing the current friendly team
        :param queues_dict: dictionary with the stored robot position histories
        :param color: color to render the trails in
        """

        self.trail_graphics_head.resize(
            len(team.team_robots), lambda: GLPolygon(outline_color=color,),
        )

        for trail_graphics_head, trail_queue in zip(
            self.trail_graphics_head, queues_dict,
        ):
            trail_graphics_head.set_points(
                [[point.x_meters, point.y_meters] for point in queues_dict[trail_queue]]
            )
