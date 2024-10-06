import math
import time
from typing import Optional

from PyQt6 import QtGui
from google.protobuf.json_format import MessageToDict
from pyqtgraph.opengl.items.GLTextItem import GLTextItem

from proto.import_all_protos import *
from software.py_constants import *
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

    @staticmethod
    def is_point_in_circle(
        point: tuple[float | int, float | int],
        center: tuple[float | int, float | int],
        radius: float | int,
    ) -> bool:
        """Returns true if the point is in the circle.

        :param point: coordinates of a point in xy plane.
        :param center: coordinates of the circle center in xy plane.
        :param radius: radius of the circle
        :return: true if the point is in the circle.
        """
        return math.dist(point, center) < radius

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
        self.ball_placement_point_hidden = False
        self.shrink_target = True
        self.placement_start_time = 0

    def __update_ball_placement(self) -> None:
        """Update ball placement visuals"""
        ball_placement_vis_proto = self.ball_placement_vis_buffer.get(
            block=False, return_cached=False
        )

        if not self.ball_placement_countdown_graphic:
            self.ball_placement_countdown_graphic = GLTextItem(
                parentItem=self,
                font=QtGui.QFont("Roboto", 7, weight=700),
                color=self.COUNT_DOWN_TEXT_COLOR,
            )

        if not ball_placement_vis_proto and not self.ball_placement_point_hidden:
            self.placement_tolerance_graphic.hide()
            self.placement_target_graphic.hide()
            self.robot_avoid_circle_graphic.hide()
            self.ball_placement_countdown_graphic.hide()

            self.ball_placement_point_hidden = True
            self.shrink_target = True
            return

        if ball_placement_vis_proto:
            # move the ball placement graphics to the new point
            new_placement_point = ball_placement_vis_proto.ball_placement_point

            # update the color of the target circle according to the position of the ball.
            # if the ball lies inside the tolerance circle, the circle will be green, otherwise red.
            ball_state = self.cached_world.ball.current_state
            if not self.ball_placement_point_hidden:
                if GLRefereeInfoLayer.is_point_in_circle(
                    (
                        ball_state.global_position.x_meters,
                        ball_state.global_position.y_meters,
                    ),
                    (new_placement_point.x_meters, new_placement_point.y_meters),
                    BALL_PLACEMENT_TOLERANCE_RADIUS_METERS,
                ):
                    self.placement_tolerance_graphic.set_outline_color(
                        self.BALL_PLACEMENT_TOLERANCE_VISUALIZATION_COLOR_ON
                    )
                else:
                    self.placement_tolerance_graphic.set_outline_color(
                        self.BALL_PLACEMENT_TOLERANCE_VISUALIZATION_COLOR_OFF
                    )

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

            # shrinking or expanding placement target graphic
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
            time_left = max(
                int(self.cached_referee_info["currentActionTimeRemaining"]) // 1000000,
                0,
            )
            self.ball_placement_countdown_graphic.setData(text=f"{time_left}s")

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
        """Refresh all displays in the layer"""
        self.cached_world = self.world_buffer.get(block=False, return_cached=True)

        self.__update_referee_info()
        self.__update_ball_placement()
