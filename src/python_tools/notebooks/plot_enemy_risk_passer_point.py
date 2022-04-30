# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:light
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.11.2
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# +
from proto.ssl_vision_wrapper_pb2 import SSL_WrapperPacket
from python_tools.proto_log import ProtoLog
import ipywidgets
from IPython.display import display
from software.python_bindings import world, passing, keep_away
import numpy as np

wrapper_proto_log = ProtoLog(
    "test_data/SensorFusion_SSL_WrapperPacket", SSL_WrapperPacket,
)
# -


world.getDefaultSensorFusionConfig()

# +
from bokeh.models import ColumnDataSource


class PointsPlotter:
    def __init__(self, fig, legend_label, marker_type, color, size=10):
        self.fig = fig
        self.points_source = ColumnDataSource(dict(xs=[], ys=[]))
        self.fig.scatter(
            source=self.points_source,
            x="xs",
            y="ys",
            fill_color=color,
            size=size,
            marker=marker_type,
        )

    def plot_points(self, points):
        data_dict = dict(xs=[pt.x() for pt in points], ys=[pt.y() for pt in points])
        self.points_source.data.update(data_dict)


# +
from bokeh.plotting import figure
from bokeh.io import output_notebook, show, push_notebook
from python_tools.plotting.plot_ssl_wrapper import SSLWrapperPlotter, MM_PER_M
from python_tools.plotting.plot_heatmap import HeatmapPlotter

output_notebook()

fig = figure(plot_width=1000, plot_height=900, match_aspect=True)
fig.background_fill_color = "lightgrey"

field_length = wrapper_proto_log[0].geometry.field.field_length / MM_PER_M
field_width = wrapper_proto_log[0].geometry.field.field_width / MM_PER_M
heatmap_x_bounds = (-field_length / 2, field_length / 2)
heatmap_y_bounds = (-field_width / 2, field_width / 2)
heatmap_grid_size = 0.05

ssl_wrapper_plotter = SSLWrapperPlotter(fig)

enemy_risk_heatmap = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "ratePassEnemyRisk"
)

keep_away_heatmap = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "keep away cost"
)

fig.legend.click_policy = "hide"

heatmap_grid_size = 0.05

config = passing.getPassingConfig()
display(config)
config["enemy_reaction_time"] = 0
config["enemy_proximity_importance"] = 0

plotter = PointsPlotter(fig, "keep away point", "square", "red")


def plot_ssl_wrapper_at_idx(idx):
    ssl_wrapper_plotter.plot_ssl_wrapper(wrapper_proto_log[idx])

    field_length = wrapper_proto_log[idx].geometry.field.field_length / MM_PER_M
    field_width = wrapper_proto_log[idx].geometry.field.field_width / MM_PER_M

    the_world = world.World(
        wrapper_proto_log[idx].SerializeToString(), dict(friendly_color_yellow=True)
    )

    pass_dict = {
        "receiver_point": world.Point(-3, 2.5),
        "pass_speed": 3,
    }

    def ratePassEnemyRiskCost(x, y):
        pass_dict["passer_point"] = world.Point(x, y)
        return passing.ratePassEnemyRisk(the_world, pass_dict, config)

    def keepAwayCost(x, y):
        pass_dict["passer_point"] = world.Point(x, y)
        return keep_away.ratePasserPointForKeepAway(the_world, pass_dict)

    enemy_risk_heatmap.plot_heatmap(ratePassEnemyRiskCost)
    keep_away_heatmap.plot_heatmap(keepAwayCost)

    plotter.plot_points([keep_away.findKeepAwayTargetPosition(the_world, pass_dict)])

    push_notebook()


show(fig, notebook_handle=True)

slider = ipywidgets.IntSlider(min=0, max=len(wrapper_proto_log) - 1)
ipywidgets.interact(plot_ssl_wrapper_at_idx, idx=slider)
