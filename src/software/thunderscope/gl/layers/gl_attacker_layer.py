import math
import time

from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *

from proto.visualization_pb2 import AttackerVisualization

from software.thunderscope.constants import Colors, DepthValues
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLAttackerLayer(GLLayer):
    """GLLayer that visualizes attacker tactic's strategy"""

    TIMEOUT_DURATION_SEC = 0.5
    CHIP_TARGET_RADIUS_METERS = 0.05

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLAttackerLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.attacker_vis_buffer = ThreadSafeBuffer(buffer_size, AttackerVisualization)

        self.pass_graphics = ObservableList(self._graphics_changed)
        self.shot_graphics = ObservableList(self._graphics_changed)
        self.shot_open_angle_graphics = ObservableList(self._graphics_changed)
        self.chip_target_graphics = ObservableList(self._graphics_changed)

        self.cache_vis_proto = None
        self.timeout = time.time() + self.TIMEOUT_DURATION_SEC

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""
        attacker_vis_proto = self.attacker_vis_buffer.get(
            block=False, return_cached=False
        )  # TODO (NIMA) , return_cached=False)

        now = time.time()
        if attacker_vis_proto is None:
            if now > self.timeout:
                self.pass_graphics.clear()
                self.shot_graphics.clear()
                self.shot_open_angle_graphics.clear()
                self.chip_target_graphics.clear()

            return

        self.pass_graphics.clear()
        self.shot_graphics.clear()
        self.shot_open_angle_graphics.clear()
        self.chip_target_graphics.clear()
        self.timeout = now + self.TIMEOUT_DURATION_SEC
        self.cache_vis_proto = attacker_vis_proto

        if attacker_vis_proto.HasField("pass_"):
            pass_proto = attacker_vis_proto.pass_
            self.pass_graphics.resize(
                1,
                lambda: GLPolygon(
                    outline_color=Colors.COMMITTED_PASS_VISUALIZATION_COLOR
                    if attacker_vis_proto.pass_committed
                    else Colors.UNCOMMITTED_PASS_VISUALIZATION_COLOR
                ),
            )
            self.pass_graphics[0].set_points(
                [
                    [
                        pass_proto.passer_point.x_meters,
                        pass_proto.passer_point.y_meters,
                    ],
                    [
                        pass_proto.receiver_point.x_meters,
                        pass_proto.receiver_point.y_meters,
                    ],
                ]
            )

        if attacker_vis_proto.HasField("shot"):
            shot_proto = attacker_vis_proto.shot
            self.shot_graphics.resize(
                1, lambda: GLPolygon(outline_color=Colors.SHOT_VISUALIZATION_COLOR),
            )
            self.shot_open_angle_graphics.resize(
                1,
                lambda: GLTextItem(
                    font=QtGui.QFont("Roboto", 8), color=Colors.SHOT_VISUALIZATION_COLOR
                ),
            )

            self.shot_graphics[0].set_points(
                [
                    [shot_proto.shot_origin.x_meters, shot_proto.shot_origin.y_meters,],
                    [shot_proto.shot_target.x_meters, shot_proto.shot_target.y_meters,],
                ]
            )
            self.shot_open_angle_graphics[0].setData(
                text=f"OA:{math.degrees(shot_proto.open_angle.radians):.1f}°",
                pos=[
                    shot_proto.shot_origin.x_meters + 0.1,
                    shot_proto.shot_origin.y_meters - 0.1,
                    0,
                ],
            )

        if attacker_vis_proto.HasField("chip_target"):
            chip_target_proto = attacker_vis_proto.chip_target
            self.chip_target_graphics.resize(
                1,
                lambda: GLCircle(outline_color=Colors.CHIP_TARGET_VISUALIZATION_COLOR),
            )
            self.chip_target_graphics[0].set_radius(self.CHIP_TARGET_RADIUS_METERS,)
            self.chip_target_graphics[0].set_position(
                chip_target_proto.x_meters, chip_target_proto.y_meters,
            )
