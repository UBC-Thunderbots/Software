from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.opengl import *

import textwrap

from google.protobuf.json_format import MessageToDict

from proto.import_all_protos import *
from software.py_constants import *
import software.thunderscope.constants as constants
from software.thunderscope.constants import Colors

from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_rect import GLRect
from software.thunderscope.gl.graphics.gl_robot import GLRobot
from software.thunderscope.gl.graphics.gl_sphere import GLSphere

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from software.thunderscope.gl.gl_layer import GLLayer


class GLTacticLayer(GLLayer):
    """GLLayer that visualizes tactics"""

    def __init__(self, buffer_size=5):
        """Initialize the GLTacticLayer

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        GLLayer.__init__(self)

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.play_info_buffer = ThreadSafeBuffer(buffer_size, PlayInfo, False)
        self.cached_world = World()

        self.graphics_list.register_graphics_group(
            "tactic_fsm_info",
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 6), color=Colors.SECONDARY_TEXT_COLOR
            ),
        )

    def update_tactic_name_graphics(self, team: Team, play_info_dict):
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
                    robot.current_state.global_position.x_meters,
                    robot.current_state.global_position.y_meters,
                    ROBOT_MAX_HEIGHT_METERS + 0.1,
                ],
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

        self.cached_world = self.world_buffer.get(block=False)
        play_info = self.play_info_buffer.get(block=False)
        play_info_dict = MessageToDict(play_info)

        self.update_tactic_name_graphics(
            self.cached_world.friendly_team, play_info_dict
        )

        return self.graphics_list.get_changes()
