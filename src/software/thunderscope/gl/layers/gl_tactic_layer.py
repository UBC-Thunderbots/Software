from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *

import textwrap

from google.protobuf.json_format import MessageToDict

from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.constants import Colors

from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_rect import GLRect
from software.thunderscope.gl.graphics.gl_robot import GLRobot
from software.thunderscope.gl.graphics.gl_sphere import GLSphere

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from software.thunderscope.gl.layers.gl_layer import GLLayer


class GLTacticLayer(GLLayer):
    """GLLayer that visualizes tactics"""

    def __init__(self, name: str, buffer_size: int = 5):
        """Initialize the GLTacticLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        GLLayer.__init__(self, name)

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.play_info_buffer = ThreadSafeBuffer(buffer_size, PlayInfo, False)
        self.cached_world = World()

        self.graphics_list.register_graphics_group(
            "tactic_fsm_info",
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 8), color=Colors.SECONDARY_TEXT_COLOR
            ),
        )

    def _update_graphics(self):
        """Fetch and update graphics for the layer"""

        self.cached_world = self.world_buffer.get(block=False)
        play_info = self.play_info_buffer.get(block=False)
        play_info_dict = MessageToDict(play_info)

        self.__update_tactic_name_graphics(
            self.cached_world.friendly_team, play_info_dict
        )

    def __update_tactic_name_graphics(self, team: Team, play_info_dict):
        """Update the GLGraphicsItems that display tactic data
        
        :param team: The team proto
        :param play_info_dict: The dictionary containing play/tactic info

        """
        tactic_assignments = play_info_dict["robotTacticAssignment"]

        for tactic_fsm_info_graphic, robot in zip(
            self.graphics_list.get_graphics("tactic_fsm_info", len(team.team_robots)),
            team.team_robots,
        ):
            tactic_fsm_info_graphic.setData(
                text=textwrap.dedent(
                    f"""
                    {tactic_assignments[str(robot.id)]["tacticName"]} - 
                    {tactic_assignments[str(robot.id)]["tacticFsmState"]}
                    """
                ),
                pos=[
                    robot.current_state.global_position.x_meters + ROBOT_MAX_RADIUS_METERS + 0.05,
                    robot.current_state.global_position.y_meters,
                    ROBOT_MAX_HEIGHT_METERS + 0.1,
                ],
            )

            # Depth value of 1 ensures text is rendered over top other graphics 
            tactic_fsm_info_graphic.setDepthValue(1)