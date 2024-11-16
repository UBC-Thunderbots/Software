from typing import Optional

from PyQt6 import QtGui
from google.protobuf.json_format import MessageToDict
from pyqtgraph.opengl.items.GLTextItem import GLTextItem

from proto.import_all_protos import *
from software.py_constants import *
import software.python_bindings as tbots_cpp
from software.thunderscope.constants import (
    DepthValues,
    Colors,
    THUNDERSCOPE_UI_FONT_NAME,
)
from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_label import GLLabel
from software.thunderscope.gl.helpers.observable_list import ObservableList
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class GLRefereeInfoLayer(GLLayer):
    """GLLayer that visualizes referee info"""

    REFEREE_COMMAND_PREFIX = "Command: "
    GAMESTATE_PREFIX = "Game State: "

    # outline color of the avoid area for ball placement
    BALL_PLACEMENT_ROBOT_AVOID_AREA_VISUALIZATION_COLOR = Colors.RED
    # outline color of the tolerance region for ball placement (when the ball is inside)
    BALL_PLACEMENT_TOLERANCE_VISUALIZATION_COLOR_ON = Colors.GREEN
    # outline color of the tolerance region for ball placement (when the ball is outside)
    BALL_PLACEMENT_TOLERANCE_VISUALIZATION_COLOR_OFF = Colors.RED
    # outline color of the target mark
    BALL_PLACEMENT_TARGET_VISUALIZATION_COLOR = Colors.FIELD_LINE_COLOR
    # text color for count down
    COUNT_DOWN_TEXT_COLOR = Colors.RED

    def __init__(self, name: str, buffer_size: int = 1) -> None:
        """Initialize the GLRefereeInfoLayer

        :param name: The displayed name of the layer
        :param buffer_size: the buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary.
        """
        super().__init__(name)
        self.setDepthValue(DepthValues.OVERLAY_DEPTH)
        self.referee_vis_buffer = ThreadSafeBuffer(buffer_size, Referee)
        self.ball_placement_vis_buffer = ThreadSafeBuffer(
            buffer_size, BallPlacementVisualization
        )
        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

        self.cached_world = None
        self.cached_referee_info = None

        self.referee_text_graphics = ObservableList(self._graphics_changed)

        self.placement_tolerance_graphic = GLCircle(
            parent_item=self,
            radius=BALL_PLACEMENT_TOLERANCE_RADIUS_METERS,
            outline_color=self.BALL_PLACEMENT_TOLERANCE_VISUALIZATION_COLOR_OFF,
        )

        self.placement_target_graphic = GLCircle(
            parent_item=self,
            radius=BALL_PLACEMENT_TOLERANCE_RADIUS_METERS,
            outline_color=self.BALL_PLACEMENT_TARGET_VISUALIZATION_COLOR,
        )

        self.robot_avoid_circle_graphic = GLCircle(
            parent_item=self,
            radius=BALL_PLACEMENT_ROBOT_AVOID_RADIUS_METERS,
            outline_color=self.BALL_PLACEMENT_ROBOT_AVOID_AREA_VISUALIZATION_COLOR,
        )

        # initialize the two text items to display
        self.gamestate_type_text: Optional[GLLabel] = None
        self.command_type_text: Optional[GLLabel] = None

        self.ball_placement_countdown_graphic: Optional[GLTextItem] = None

        self.ball_placement_point = None
        self.ball_placement_in_progress = False
        self.ball_placement_tolerance_circle = None
        self.shrink_target = True

    def __update_ball_placement(self) -> None:
        """Update ball placement visuals"""
        ball_placement_vis_proto = self.ball_placement_vis_buffer.get(
            block=False, return_cached=False
        )

        if not self.ball_placement_countdown_graphic:
            self.ball_placement_countdown_graphic = GLTextItem(
                parentItem=self,
                font=QtGui.QFont(THUNDERSCOPE_UI_FONT_NAME, 7, weight=700),
                color=self.COUNT_DOWN_TEXT_COLOR,
            )

        # if ball placement is in progress, update all the visuals
        if self.ball_placement_in_progress:
            self.__update_ball_placement_status(self.cached_world.ball.current_state)
            self.__update_target_visual()

        if ball_placement_vis_proto:
            new_placement_point = ball_placement_vis_proto.ball_placement_point
            if not self.ball_placement_in_progress:
                # initialize the visuals
                self.ball_placement_tolerance_circle = tbots_cpp.Circle(
                    tbots_cpp.createPoint(new_placement_point),
                    BALL_PLACEMENT_TOLERANCE_RADIUS_METERS,
                )
                self.__display_ball_placement_visuals(new_placement_point)
                self.ball_placement_in_progress = True
            self.ball_placement_point = new_placement_point
        elif self.ball_placement_in_progress:
            # finish ball placement visualization
            self.__hide_ball_placement_visuals()
            self.ball_placement_in_progress = False

    def __update_referee_info(self):
        """Update gamestate and command info text displays"""
        referee_proto = self.referee_vis_buffer.get(block=False, return_cached=False)
        if not referee_proto:
            return

        referee_msg_dict = MessageToDict(referee_proto)
        if not referee_msg_dict:
            return
        self.cached_referee_info = referee_msg_dict

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
        """Refresh all visuals for both ball placement and referee info"""
        self.cached_world = self.world_buffer.get(block=False, return_cached=True)

        self.__update_referee_info()
        self.__update_ball_placement()

    def __update_ball_placement_status(self, ball_state: BallState) -> None:
        """Update ball placement circle color corresponding to ball position.
        If the ball lies inside the tolerance circle, the circle will be green, otherwise red.
        :param ball_state: state of the ball
        """
        if tbots_cpp.contains(
            self.ball_placement_tolerance_circle,
            tbots_cpp.createPoint(ball_state.global_position),
        ):
            self.placement_tolerance_graphic.set_outline_color(
                self.BALL_PLACEMENT_TOLERANCE_VISUALIZATION_COLOR_ON
            )
        else:
            self.placement_tolerance_graphic.set_outline_color(
                self.BALL_PLACEMENT_TOLERANCE_VISUALIZATION_COLOR_OFF
            )

    def __update_target_visual(self) -> None:
        """Update the ball placement target graphic to shrink or expand"""
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

        # update the count-down graphics
        if self.cached_referee_info:
            time_left = max(
                int(self.cached_referee_info["currentActionTimeRemaining"]) // 1000000,
                0,
            )
            self.ball_placement_countdown_graphic.setData(text=f"{time_left}s")

    def __display_ball_placement_visuals(self, new_placement_point: Point) -> None:
        """Display ball placement visuals
        :param new_placement_point: ball placement point
        """
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
        self.placement_target_graphic.set_radius(BALL_PLACEMENT_TOLERANCE_RADIUS_METERS)
        self.placement_target_graphic.show()

        self.robot_avoid_circle_graphic.set_position(
            self.ball_placement_point.x_meters,
            self.ball_placement_point.y_meters,
        )
        self.robot_avoid_circle_graphic.show()

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
        self.ball_placement_in_progress = False

    def __hide_ball_placement_visuals(self) -> None:
        """Hide all the visuals for ball placement"""
        self.placement_tolerance_graphic.hide()
        self.placement_target_graphic.hide()
        self.robot_avoid_circle_graphic.hide()
        self.ball_placement_countdown_graphic.hide()
        self.shrink_target = True
