from typing import Optional

from google.protobuf.json_format import MessageToDict

from proto.import_all_protos import *
from software.thunderscope.constants import DepthValues
from software.thunderscope.gl.graphics.gl_label import GLLabel
from software.thunderscope.gl.helpers.observable_list import ObservableList
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class GLRefereeInfoLayer(GLLayer):
    """GLLayer that visualizes referee info"""

    REFEREE_COMMAND_PREFIX = "Command: "
    GAMESTATE_PREFIX = "Game State: "

    def __init__(self, name: str, buffer_size: int = 1) -> None:
        """Initialize the GLRefereeInfoLayer

        :param name: The displayed name of the layer
        :param buffer_size: the buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary.
        """
        super().__init__(name)
        self.setDepthValue(DepthValues.OVERLAY_DEPTH)
        self.referee_vis_buffer = ThreadSafeBuffer(buffer_size, Referee, False)

        self.referee_text_graphics = ObservableList(self._graphics_changed)

        # initialize the two text items to display
        self.gamestate_type_text: Optional[GLLabel] = None
        self.command_type_text: Optional[GLLabel] = None

    def refresh_graphics(self) -> None:
        """Update displays in the layer"""
        referee_proto = self.referee_vis_buffer.get(block=False, return_cached=False)
        if not referee_proto:
            return

        referee_msg_dict = MessageToDict(referee_proto)
        if not referee_msg_dict:
            return

        if not self.gamestate_type_text:
            self.gamestate_type_text = GLLabel(
                parent_item=self,
                offset=(-10, 50),
                text=GLRefereeInfoLayer.GAMESTATE_PREFIX + referee_msg_dict["stage"],
            )
            self.referee_text_graphics.append(self.gamestate_type_text)
        else:
            self.gamestate_type_text.set_text(
                GLRefereeInfoLayer.GAMESTATE_PREFIX + referee_msg_dict["stage"]
            )

        if not self.command_type_text:
            self.command_type_text = GLLabel(
                parent_item=self,
                offset=(-10, 70),
                text=GLRefereeInfoLayer.GAMESTATE_PREFIX + referee_msg_dict["command"],
            )
            self.referee_text_graphics.append(self.command_type_text)
        else:
            self.command_type_text.set_text(
                GLRefereeInfoLayer.REFEREE_COMMAND_PREFIX + referee_msg_dict["command"]
            )
