import time
import queue
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
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

        layout = pg.LayoutWidget(parent=self)
        win = pg.GraphicsLayoutWidget(show=True)

        # image settings
        self.data = np.zeros(shape=(6, 3))
        self.img = pg.ImageItem(self.data)
        self.img.setBorder(pg.mkPen(color=(255, 255, 255)))

        # layout settings
        self.vb = win.addViewBox()
        self.vb.addItem(self.img)
        self.vb.setAspectLocked()  # remove this to make it stretch to fit the window
        layout.addWidget(win)
        # ^ pass in row, col, rowspan, and colspan if you want to add more items to the layout
        self.setCentralWidget(layout)

        # isocurve settings
        self.curves = []
        self.levels = np.linspace(0, 1, 10)  # 10 isocurves
        for i in range(len(self.levels)):
            v = self.levels[i]
            c = pg.IsocurveItem(level=v, pen=(i, len(self.levels)*1.5))
            c.setParentItem(self.img)
            self.curves.append(c)

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

        # Update the data and image
        self.data = np.array(cost_vis.cost).reshape(
            cost_vis.num_cols, cost_vis.num_rows
        )

        self.img.setImage(self.data)

        for c in self.curves:
            c.setParentItem(self.img)
            c.setData(self.data)
