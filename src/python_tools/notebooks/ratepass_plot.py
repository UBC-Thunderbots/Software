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
from software.python_bindings import world, passing, pass_generator
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
from python_tools.plotting.plot_pass_generator import PassGeneratorPlotter

output_notebook()

fig = figure(plot_width=1000, plot_height=900, match_aspect=True)
fig.background_fill_color = "lightgrey"

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
rate_pass_shoot_score_plotter = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "ratePassShootScore",
)

pass_generator_plotter = PassGeneratorPlotter(fig)

fig.legend.click_policy = "hide"

heatmap_grid_size = 0.05

config = passing.getPassingConfig()


def plot_ssl_wrapper_at_idx(idx):
    ssl_wrapper_plotter.plot_ssl_wrapper(wrapper_proto_log[idx])

    field_length = wrapper_proto_log[idx].geometry.field.field_length / 1000
    field_width = wrapper_proto_log[idx].geometry.field.field_width / 1000

    the_world = world.World(wrapper_proto_log[idx].SerializeToString(), dict())
    generator = pass_generator.EighteenZonePassGenerator(the_world, config)

    pass_dict = {
        "passer_point": the_world.ball().position(),
        "pass_speed": 3,
    }

    def ratePassCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.ratePass(the_world, pass_dict, config)

    def ratePassEnemyRiskCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.ratePassEnemyRisk(the_world, pass_dict, config)

    def ratePassFriendlyCapabilityCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.ratePassFriendlyCapability(the_world, pass_dict, config)

    def ratePassShootScoreCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.ratePassShootScore(the_world, pass_dict, config)

    rate_pass_heatmap_plotter.plot_heatmap(ratePassCost)
    rate_pass_enemy_heatmap_plotter.plot_heatmap(ratePassEnemyRiskCost)
    rate_pass_friendly_heatmap_plotter.plot_heatmap(ratePassFriendlyCapabilityCost)
    rate_pass_shoot_score_plotter.plot_heatmap(ratePassShootScoreCost)

    zones = pass_generator.getAllZones(the_world)
    pass_generator_plotter.plot_zones(zones)

    passes = generator.getBestPassesForAllZones(the_world)
    pass_generator_plotter.plot_passes(passes)

    push_notebook()


show(fig, notebook_handle=True)

slider = ipywidgets.IntSlider(min=0, max=len(wrapper_proto_log) - 1)
ipywidgets.interact(plot_ssl_wrapper_at_idx, idx=slider)
