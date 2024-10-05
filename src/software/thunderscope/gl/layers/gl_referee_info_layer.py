from typing import Optional

from PyQt6 import QtGui
from google.protobuf.json_format import MessageToDict

from proto.import_all_protos import *
from software.thunderscope.constants import DepthValues, Colors
from software.thunderscope.gl.graphics.gl_circle import GLCircle
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
        self.ball_placement_vis_buffer = ThreadSafeBuffer(buffer_size, BallPlacementVisualization, False)

        self.referee_text_graphics = ObservableList(self._graphics_changed)

        self.placement_tolerance_graphic = GLCircle(
            parent_item=self,
            radius=BALL_PLACEMENT_TOLERANCE_RADIUS_METERS,
            outline_color=Colors.BALL_PLACEMENT_TOLERANCE_VISUALIZATION_COLOR,
        )

        self.placement_target_graphic = GLCircle(
            parent_item=self,
            radius=BALL_PLACEMENT_TOLERANCE_RADIUS_METERS,
            outline_color=Colors.BALL_PLACEMENT_TARGET_VISUALIZATION_COLOR,
        )

        self.robot_avoid_circle_graphic = GLCircle(
            parent_item=self,
            radius=BALL_PLACEMENT_ROBOT_AVOID_RADIUS_METERS,
            outline_color=Colors.BALL_PLACEMENT_ROBOT_AVOID_AREA_VISUALIZATION_COLOR,
        )
        self.robot_avoid_circle_graphic.setDepthValue(
            DepthValues.ABOVE_FOREGROUND_DEPTH
        )

        # initialize the two text items to display
        self.gamestate_type_text: Optional[GLLabel] = None
        self.command_type_text: Optional[GLLabel] = None

    def update_ball_placement(self):
        ball_placement_vis_proto = self.ball_placement_vis_buffer.get(
            block=False, return_cached=False
        )

        if ball_placement_vis_proto is None and not self.ball_placement_point_hidden:
            self.placement_tolerance_graphic.hide()
            self.placement_target_graphic.hide()
            self.robot_avoid_circle_graphic.hide()

            if self.ball_placement_countdown_graphic:
                self.ball_placement_countdown_graphic.hide()

            self.ball_placement_point_hidden = True
            self.shrink_target = True
            return
        elif ball_placement_vis_proto is not None:
            new_placement_point = ball_placement_vis_proto.ball_placement_point
            if self.ball_placement_point_hidden:
                self.ball_placement_point = new_placement_point

                self.placement_tolerance_graphic.set_position(
                    self.ball_placement_point.x_meters,
                    self.ball_placement_point.y_meters,
                )
                self.placement_tolerance_graphic.show()

                self.placement_target_graphic.set_position(
                    self.ball_placement_point.x_meters,
                    self.ball_placement_point.y_meters,
                )
                self.placement_target_graphic.set_radius(
                    BALL_PLACEMENT_TOLERANCE_RADIUS_METERS
                )
                self.placement_target_graphic.show()

                self.robot_avoid_circle_graphic.set_position(
                    self.ball_placement_point.x_meters,
                    self.ball_placement_point.y_meters,
                )
                self.robot_avoid_circle_graphic.show()

                self.ball_placement_countdown_graphic = GLTextItem(
                    parentItem=self,
                    font=QtGui.QFont("Roboto", 7, weight=700),
                    color=Colors.RED_TEXT_COLOR,
                )
                self.ball_placement_countdown_graphic.setData(
                    text=f"{BALL_PLACEMENT_TIME_LIMIT_S}s",
                    pos=[
                        self.ball_placement_point.x_meters
                        + BALL_PLACEMENT_ROBOT_AVOID_RADIUS_METERS / 2,
                        self.ball_placement_point.y_meters
                        + BALL_PLACEMENT_ROBOT_AVOID_RADIUS_METERS
                        + 0.1,
                        0,
                    ],
                )
                self.ball_placement_countdown_graphic.show()

                self.placement_start_time = time.time()
                self.ball_placement_point_hidden = False

            if self.shrink_target:
                self.placement_target_graphic.set_radius(
                    self.placement_target_graphic.radius - 0.01
                )
                self.shrink_target = self.placement_target_graphic.radius >= 0
            else:
                self.placement_target_graphic.set_radius(
                    self.placement_target_graphic.radius + 0.01
                )
                self.shrink_target = (
                        self.placement_target_graphic.radius
                        >= BALL_PLACEMENT_TOLERANCE_RADIUS_METERS
                )

            time_since_start = int(time.time() - self.placement_start_time)
            if time_since_start <= BALL_PLACEMENT_TIME_LIMIT_S:
                self.ball_placement_countdown_graphic.setData(
                    text=f"{BALL_PLACEMENT_TIME_LIMIT_S - time_since_start}s"
                )
            else:
                self.ball_placement_countdown_graphic.setData(text=f"{0}s")

    def update_text_info(self):
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

    def refresh_graphics(self) -> None:
        """Update displays in the layer"""
        self.update_text_info()
        self.update_ball_placement()
