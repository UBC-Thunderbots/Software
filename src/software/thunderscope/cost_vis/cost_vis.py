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

        ## make pretty looping data
        frames = 200
        self.data = np.random.normal(size=(frames,30,30), loc=0, scale=100)
        self.data = np.concatenate([self.data, self.data], axis=0)
        self.data = pg.gaussianFilter(self.data, (10, 10, 10))[frames//2:frames + frames//2]
        self.data[:, 15:16, 15:17] += 1

        win = pg.GraphicsLayoutWidget(show=True)
        self.vb = win.addViewBox()
        self.img = pg.ImageItem(self.data[0])
        self.vb.addItem(self.img)
        self.vb.setAspectLocked()
        self.setCentralWidget(win)

        ## generate empty curves
        self.curves = []
        self.levels = np.linspace(self.data.min(), self.data.max(), 10)
        for i in range(len(self.levels)):
            v = self.levels[i]
            ## generate isocurve with automatic color selection
            c = pg.IsocurveItem(level=v, pen=(i, len(self.levels)*1.5))
            c.setParentItem(self.img)  ## make sure isocurve is always correctly displayed over image
            c.setZValue(10)
            self.curves.append(c)

        ## animate!
        self.ptr = 0
        self.imgLevels = (self.data.min(), self.data.max() * 2)

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
        
        print(cost_vis)


        self.ptr = (self.ptr + 1) % self.data.shape[0]
        self.img.setImage(self.data[self.ptr])
        for c in self.curves:
            c.setParentItem(self.img)
            c.setData(self.data[self.ptr])
