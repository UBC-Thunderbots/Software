import time

from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *

from proto.visualization_pb2 import BallPlacementVisualization

from software.py_constants import *
from software.thunderscope.constants import Colors, DepthValues
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle


class GLBallPlacementLayer(GLLayer):
    """GLLayer that visualizes ball placement"""

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLBallPlacementLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size. Default is arbitrary

        """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.ball_placement_vis_buffer = ThreadSafeBuffer(buffer_size, BallPlacementVisualization)
        
        self.placement_tolerance_graphic = GLCircle(
            parent_item=self, radius=BALL_PLACEMENT_TOLERANCE_RADIUS_METERS, 
            outline_color=Colors.BALL_PLACEMENT_TOLERANCE_VISUALIZATION_COLOR
        )

        self.placement_target_graphic = GLCircle(
            parent_item=self, radius=BALL_PLACEMENT_TOLERANCE_RADIUS_METERS, 
            outline_color=Colors.BALL_PLACEMENT_TARGET_VISUALIZATION_COLOR
        )

        self.robot_avoid_circle_graphic = GLCircle(
            parent_item=self, radius=BALL_PLACEMENT_ROBOT_AVOID_RADIUS_METERS, 
            outline_color=Colors.BALL_PLACEMENT_ROBOT_AVOID_AREA_VISUALIZATION_COLOR
        )
        self.ball_placement_countdown_graphic = GLTextItem(
            font=QtGui.QFont("Roboto", 10, weight=700),
            color=Colors.RED_TEXT_COLOR,
        )

        self.ball_placement_point = None
        self.ball_placement_point_hidden = False
        self.shrink_target = True
        self.placement_start_time = 0

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""
        ball_placement_vis_proto = self.ball_placement_vis_buffer.get(
            block=False, return_cached=False
        )

        if ball_placement_vis_proto is None and not self.ball_placement_point_hidden:
            self.placement_tolerance_graphic.hide()
            self.placement_target_graphic.hide()
            self.robot_avoid_circle_graphic.hide()
            self.ball_placement_countdown_graphic.hide()
            self.ball_placement_point_hidden = True
            self.shrink_target = True
            return
        elif ball_placement_vis_proto is not None:
            new_placement_point = ball_placement_vis_proto.ball_placement_point
            if (self.ball_placement_point_hidden and (
                    self.ball_placement_point == None or new_placement_point != self.ball_placement_point
                )
            ):
                self.ball_placement_point = new_placement_point
                              
                self.placement_tolerance_graphic.set_position(
                    self.ball_placement_point.x_meters, self.ball_placement_point.y_meters,
                )
                self.placement_tolerance_graphic.show()  

                self.placement_target_graphic.set_position(
                    self.ball_placement_point.x_meters, self.ball_placement_point.y_meters,
                )
                self.placement_target_graphic.set_radius(BALL_PLACEMENT_TOLERANCE_RADIUS_METERS)
                self.placement_target_graphic.show()

                self.robot_avoid_circle_graphic.set_position(
                    self.ball_placement_point.x_meters, self.ball_placement_point.y_meters
                )
                self.robot_avoid_circle_graphic.show()
                self.ball_placement_countdown_graphic.setData(
                    text=f"{BALL_PLACEMENT_TIME_LIMIT_S}s"
                    pos=[
                        self.ball_placement_point.x_meters,
                        self.ball_placement_point.y_meters + BALL_PLACEMENT_ROBOT_AVOID_RADIUS_METERS + 0.1,
                        0
                    ]
                )

                self.placement_start_time = time.time()
                self.ball_placement_point_hidden = False

            if (self.shrink_target):
                self.placement_target_graphic.set_radius(self.placement_target_graphic.radius - 0.01)
                self.shrink_target = self.placement_target_graphic.radius >= 0
            else:
                self.placement_target_graphic.set_radius(self.placement_target_graphic.radius + 0.01)
                self.shrink_target = self.placement_target_graphic.radius >= BALL_PLACEMENT_TOLERANCE_RADIUS_METERS

            time_since_start = time.time() - self.placement_start_time
            if (time_since_start <= BALL_PLACEMENT_TIME_LIMIT_S):
                self.ball_placement_countdown_graphic.setData(
                    text=f"{BALL_PLACEMENT_TIME_LIMIT_S - (curr_time - self.placement_start_time)}s"
                )
            else:
                self.ball_placement_countdown_graphic.setData(
                    text=f"{0}s"
                )


