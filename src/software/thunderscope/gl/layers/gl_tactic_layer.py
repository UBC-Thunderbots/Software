from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *

import textwrap

from google.protobuf.json_format import MessageToDict

from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.constants import Colors, DepthValues

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from software.thunderscope.gl.layers.gl_layer import GLLayer

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLTacticLayer(GLLayer):
    """GLLayer that visualizes tactics"""

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLTacticLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)

        # Depth value of 1 ensures this layer is rendered over top other layers
        self.setDepthValue(DepthValues.SECONDARY_TEXT_DEPTH)

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.play_info_buffer = ThreadSafeBuffer(buffer_size, PlayInfo, False)
        self.cached_world = World()

        self.tactic_fsm_info_graphics = ObservableList(self._graphics_changed)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        self.cached_world = self.world_buffer.get(block=False)
        play_info = self.play_info_buffer.get(block=False)
        play_info_dict = MessageToDict(play_info)

        self.__update_tactic_name_graphics(
            self.cached_world.friendly_team, play_info_dict
        )

    def __update_tactic_name_graphics(self, team: Team, play_info_dict) -> None:
        """Update the GLGraphicsItems that display tactic data
        
        :param team: The team proto
        :param play_info_dict: The dictionary containing play/tactic info

        """
        tactic_assignments = play_info_dict["robotTacticAssignment"]

        # Ensure we have the same number of graphics as robots
        self.tactic_fsm_info_graphics.resize(
            len(team.team_robots),
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 8), color=Colors.SECONDARY_TEXT_COLOR
            ),
        )

        for tactic_fsm_info_graphic, robot in zip(
            self.tactic_fsm_info_graphics, team.team_robots,
        ):
            tactic_fsm_info_graphic.setData(
                text=textwrap.dedent(
                    f"""
                    {tactic_assignments[str(robot.id)]["tacticName"]} - 
                    {tactic_assignments[str(robot.id)]["tacticFsmState"]}
                    """
                ),
                pos=[
                    robot.current_state.global_position.x_meters
                    + ROBOT_MAX_RADIUS_METERS
                    + 0.05,
                    robot.current_state.global_position.y_meters,
                    ROBOT_MAX_HEIGHT_METERS + 0.1,
                ],
            )
