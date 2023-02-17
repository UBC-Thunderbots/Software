import time
import queue
import numpy as np

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class CostVisualizationWidget(QtWidgets.QMainWindow):

    """Plot pass cost visualization data a 2d plot as a heatmap image

    This widget enables us to sample different pass cost functions
    in the field and visualize them.

    The data is sampled at different points in the field in our ai and sent
    over protobuf to thunderscope.
    This widget plots the CostVisualization protobuf data as a heatmap image.

    To turn on this widget, pass the --cost_visualization flag to
    thunderscope_main.py, then toggle generate_sample_passes in parameters
    widget to start sampling data.
    (Note that without the flag, no data will be sent over protobuf)
    Now to visualize different cost functions, check the appropriate boxes
    in the parameters widget.

    WARNING: This widget is very slow and will slow down ai significantly if
    the number of points sampled is too high.

    WARNING: The checkbox for generate_sample_passes should only be checked
    when this widget is on. Otherwise the values will be calculated and
    protobuf data will be sent but not plotted.
    """

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
        self.view_box = win.addViewBox()
        self.view_box.addItem(self.img)
        self.view_box.setAspectLocked()
        layout.addWidget(win)
        self.setCentralWidget(layout)

        # max number
        self.max_label = pg.TextItem("PLOTTING OFF")
        self.view_box.addItem(self.max_label)

        # isocurve settings
        self.curves = []
        self.levels = np.linspace(0, 1, 10)  # 10 isocurves
        for i in range(len(self.levels)):
            level = self.levels[i]
            isocurve = pg.IsocurveItem(level=level, pen=(i, len(self.levels) * 1.5))
            isocurve.setParentItem(self.img)
            self.curves.append(isocurve)

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

        # Update the max number
        self.max_label.setText("max: " + str(np.max(self.data)))

        for isocurve in self.curves:
            isocurve.setParentItem(self.img)
            isocurve.setData(self.data)
