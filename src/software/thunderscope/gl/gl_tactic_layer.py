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
from software.thunderscope.gl.graphics.gl_ball import GLBall

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

        self.tactic_text_graphics = []

    def updateTacticNameGraphics(self, team: Team, play_info_dict):
        """Update the GLGraphicsItems that display tactic data
        
        :param team: The team proto
        :param play_info_dict: The dictionary containing play/tactic info

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems

        """
        added_graphics, removed_graphics = self.setupGraphicsList(
            self.tactic_text_graphics,
            len(team.team_robots),
            lambda: GLTextItem(
                font=QtGui.QFont("Helvetica", 6), color=Colors.SECONDARY_TEXT_COLOR
            ),
        )

        tactic_assignments = play_info_dict["robotTacticAssignment"]

        for tactic_text_graphic, robot in zip(
            self.tactic_text_graphics, team.team_robots
        ):
            tactic_text_graphic.setData(
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

        return added_graphics, removed_graphics

    def updateGraphics(self):
        """Update the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems
        
        """
        if not self.isVisible():
            return ([], self.clearGraphicsList(self.tactic_text_graphics))

        added_graphics = []
        removed_graphics = []

        self.cached_world = self.world_buffer.get(block=False)
        play_info = self.play_info_buffer.get(block=False)
        play_info_dict = MessageToDict(play_info)

        (
            added_tactic_name_graphics,
            removed_tactic_name_graphics,
        ) = self.updateTacticNameGraphics(
            self.cached_world.friendly_team, play_info_dict,
        )

        added_graphics += added_tactic_name_graphics
        removed_graphics += removed_tactic_name_graphics

        return added_graphics, removed_graphics
