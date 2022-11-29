import random
import time
import queue
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, mkQApp, QtCore
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

class CostVisualizationWidget(QtWidgets.QMainWindow):

    COST_VISUALIZATION_TIMEOUT_S = 0.5

    MIN_VAL = 0
    MAX_VAL = 1
    X_SIZE = 40
    Y_SIZE = 40

    def __init__(self, buffer_size=5):
        super(CostVisualizationWidget, self).__init__()
        self.cost_visualization_buffer = ThreadSafeBuffer(
            buffer_size, CostVisualization
        )
        self.cached_cost_vis = CostVisualization()
        self.timeout = time.time() + CostVisualizationWidget.COST_VISUALIZATION_TIMEOUT_S

        # initialize arrays with zeros
        self.data = np.zeros(shape=(6,3))
        self.static_position_quality = np.zeros(shape=(6,3))
        self.pass_friendly_capability = np.zeros(shape=(6,3))
        self.pass_enemy_risk = np.zeros(shape=(6,3))
        self.pass_shoot_score = np.zeros(shape=(6,3))
        self.zone_rating = np.zeros(shape=(6,3))

        win = pg.GraphicsLayoutWidget(show=True)
        self.vb = win.addViewBox()
        self.img = pg.ImageItem(self.data)
        self.vb.addItem(self.img)
        self.vb.setAspectLocked()
        self.setCentralWidget(win)

        ## generate empty curves
        # self.curves = []
        # self.levels = np.linspace(self.data.min(), self.data.max(), 10)
        # for i in range(len(self.levels)):
        #     v = self.levels[i]
        #     ## generate isocurve with automatic color selection
        #     c = pg.IsocurveItem(level=v, pen=(i, len(self.levels)*1.5))
        #     c.setParentItem(self.img)  ## make sure isocurve is always correctly displayed over image
        #     c.setZValue(10)
        #     self.curves.append(c)

        ## animate!
        # self.ptr = 0
        # self.imgLevels = (self.data.min(), self.data.max() * 2)

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
            self.timeout = time.time() + CostVisualizationWidget.COST_VISUALIZATION_TIMEOUT_S
            self.cached_cost_vis = cost_vis

        self.static_position_quality = np.array(cost_vis.static_position_quality.cost).reshape(6,3)
        self.pass_friendly_capability = np.array(cost_vis.pass_friendly_capability.cost).reshape(6,3)
        self.pass_enemy_risk = np.array(cost_vis.pass_enemy_risk.cost).reshape(6,3)
        self.pass_shoot_score = np.array(cost_vis.pass_shoot_score.cost).reshape(6,3)
        self.zone_rating = np.array(cost_vis.zone_rating.cost).reshape(6,3)

        self.data = self.static_position_quality + self.pass_friendly_capability + self.pass_enemy_risk + self.pass_shoot_score + self.zone_rating
        
        print(self.data)

        self.img.setImage(self.data)
        # for c in self.curves:
        #     c.setParentItem(self.img)
        #     c.setData(self.data)
