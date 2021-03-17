# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:light
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.10.1
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# +
from software.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from python_tools.proto_log import ProtoLog
import ipywidgets
from IPython.display import display
from software.python_bindings import world, passing
import numpy as np

wrapper_proto_log = ProtoLog(
    "test_data/SensorFusion_SSL_WrapperPacket", SSL_WrapperPacket,
)
# -


world.getDefaultSensorFusionConfig()

# +
from bokeh.plotting import figure
from bokeh.io import output_notebook, show, push_notebook
from python_tools.plotting.plot_ssl_wrapper import SSLWrapperPlotter, MM_PER_M
from python_tools.plotting.plot_heatmap import HeatmapPlotter

output_notebook()

fig = figure(plot_width=900, plot_height=900, match_aspect=True)

field_length = wrapper_proto_log[0].geometry.field.field_length / MM_PER_M
field_width = wrapper_proto_log[0].geometry.field.field_width / MM_PER_M
heatmap_x_bounds = (-field_length / 2, field_length / 2)
heatmap_y_bounds = (-field_width / 2, field_width / 2)
heatmap_grid_size = 0.05

ssl_wrapper_plotter = SSLWrapperPlotter(fig)

rate_pass_heatmap_plotter = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "ratePass"
)
rate_pass_enemy_heatmap_plotter = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "ratePassEnemyRisk"
)
rate_pass_friendly_heatmap_plotter = HeatmapPlotter(
    fig,
    heatmap_x_bounds,
    heatmap_y_bounds,
    heatmap_grid_size,
    "ratePassFriendlyCapability",
)

fig.legend.click_policy = "hide"

heatmap_grid_size = 0.05


def plot_ssl_wrapper_at_idx(idx):
    ssl_wrapper_plotter.plot_ssl_wrapper(wrapper_proto_log[idx])

    field_length = wrapper_proto_log[idx].geometry.field.field_length / 1000
    field_width = wrapper_proto_log[idx].geometry.field.field_width / 1000

    the_world = world.World(wrapper_proto_log[idx].SerializeToString(), dict())

    def ratePassCost(x, y):
        receiver_point = world.Point(x, y)
        pass_dict = {
            "passer_point": world.Point(4, 2),
            "receiver_point": receiver_point,
            "pass_speed": 5.0,
            "receive_and_dribble": False,
        }
        return passing.ratePass(the_world, pass_dict)

    def ratePassEnemyRiskCost(x, y):
        receiver_point = world.Point(x, y)
        pass_dict = {
            "passer_point": world.Point(4, 2),
            "receiver_point": receiver_point,
            "pass_speed": 5.0,
            "receive_and_dribble": False,
        }
        return passing.ratePassEnemyRisk(the_world, pass_dict)

    def ratePassFriendlyCapabilityCost(x, y):
        receiver_point = world.Point(x, y)
        pass_dict = {
            "passer_point": world.Point(4, 2),
            "receiver_point": receiver_point,
            "pass_speed": 5.0,
            "receive_and_dribble": False,
        }
        return passing.ratePassFriendlyCapability(the_world, pass_dict)

    rate_pass_heatmap_plotter.plot_heatmap(ratePassCost)
    rate_pass_enemy_heatmap_plotter.plot_heatmap(ratePassEnemyRiskCost)
    rate_pass_friendly_heatmap_plotter.plot_heatmap(ratePassFriendlyCapabilityCost)
    push_notebook()


show(fig, notebook_handle=True)

slider = ipywidgets.IntSlider(min=0, max=len(wrapper_proto_log) - 1)
ipywidgets.interact(plot_ssl_wrapper_at_idx, idx=slider)
# -
