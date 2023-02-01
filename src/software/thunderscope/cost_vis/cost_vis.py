import time
import queue
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, mkQApp, QtCore
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class CostVisualizationWidget(QtWidgets.QMainWindow):

    COST_VISUALIZATION_TIMEOUT_S = 0.5

    def __init__(self, buffer_size=5):
        super(CostVisualizationWidget, self).__init__()
        self.cost_visualization_buffer = ThreadSafeBuffer(
            buffer_size, CostVisualization
        )
        self.cached_cost_vis = CostVisualization()
        self.timeout = (
            time.time() + CostVisualizationWidget.COST_VISUALIZATION_TIMEOUT_S
        )

        # initialize arrays with zeros
        self.data = np.zeros(shape=(6, 3))
        # self.static_position_quality = np.zeros(shape=(6, 3))
        # self.pass_friendly_capability = np.zeros(shape=(6, 3))
        # self.pass_enemy_risk = np.zeros(shape=(6, 3))
        # self.pass_shoot_score = np.zeros(shape=(6, 3))

        layout = pg.LayoutWidget(parent=self)

        win = pg.GraphicsLayoutWidget(show=True)
        self.vb = win.addViewBox()
        self.img = pg.ImageItem(self.data)
        self.vb.addItem(self.img)
        self.vb.setAspectLocked()  # remove this to make it stretch to fit the window
        layout.addWidget(win, row=0, col=0, colspan=5, rowspan=1)

        # push buttons - span from column 0 to 4 on row 1
        self.static_pos_box = QtWidgets.QCheckBox("Static Position Quality")
        layout.addWidget(self.static_pos_box, row=1, col=0, colspan=1, rowspan=1)
        self.pass_friend_box = QtWidgets.QCheckBox("Pass Friendly Capability")
        layout.addWidget(self.pass_friend_box, row=1, col=1, colspan=1, rowspan=1)
        self.pass_enemy_box = QtWidgets.QCheckBox("Pass Enemy Risk")
        layout.addWidget(self.pass_enemy_box, row=1, col=2, colspan=1, rowspan=1)
        self.pass_shoot_box = QtWidgets.QCheckBox("Pass Shoot Score")
        layout.addWidget(self.pass_shoot_box, row=1, col=3, colspan=1, rowspan=1)

        self.setCentralWidget(layout)

    def refresh(self):
        try:
            cost_vis = self.cost_visualization_buffer.queue.get_nowait()
        except queue.Empty as empty:
            cost_vis = None

        if not cost_vis:
            cost_vis = self.cached_cost_vis
            # If we haven't received pass visualizations for a bit, clear the layer
            if time.time() > self.timeout:
                return
        else:
            # We received new pass data, so lets update our timeout
            self.timeout = (
                time.time() + CostVisualizationWidget.COST_VISUALIZATION_TIMEOUT_S
            )
            self.cached_cost_vis = cost_vis

        self.static_position_quality = np.array(
            cost_vis.static_position_quality.cost
        ).reshape(cost_vis.num_cols, cost_vis.num_rows)
        self.pass_friendly_capability = np.array(
            cost_vis.pass_friendly_capability.cost
        ).reshape(cost_vis.num_cols, cost_vis.num_rows)
        self.pass_enemy_risk = np.array(cost_vis.pass_enemy_risk.cost).reshape(
            cost_vis.num_cols, cost_vis.num_rows
        )
        self.pass_shoot_score = np.array(cost_vis.pass_shoot_score.cost).reshape(
            cost_vis.num_cols, cost_vis.num_rows
        )

        # multiply all the arrays together
        self.data = np.ones(shape=(6, 3))
        if self.static_pos_box.isChecked():
            self.data = self.static_position_quality
        if self.pass_friend_box.isChecked():
            self.data *= self.pass_friendly_capability
        if self.pass_enemy_box.isChecked():
            self.data *= self.pass_enemy_risk
        if self.pass_shoot_box.isChecked():
            self.data *= self.pass_shoot_score

        self.img.setImage(self.data)
