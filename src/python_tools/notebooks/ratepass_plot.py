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
from software.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from python_tools.proto_log import ProtoLog
import ipywidgets
from IPython.display import display
from software.python_bindings import world, passing, pass_generator
import numpy as np

wrapper_proto_log = ProtoLog(
    "/home/akhil/Downloads/23062021_033324/SensorFusion_SSL_WrapperPacket", SSL_WrapperPacket,
)
# -


sensor_fusion_config = world.getDefaultSensorFusionConfig()
print(sensor_fusion_config)
sensor_fusion_config['friendly_color_yellow'] = False


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
heatmap_grid_size = 0.2

ssl_wrapper_plotter = SSLWrapperPlotter(fig)

rate_pass_heatmap_plotter = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "ratePass"
)
rate_kick_pass_enemy_heatmap_plotter = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "rateKickPassEnemyRisk"
)
rate_chip_pass_enemy_heatmap_plotter = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "rateChipPassEnemyRisk"
)
rate_kick_pass_friendly_heatmap_plotter = HeatmapPlotter(
    fig,
    heatmap_x_bounds,
    heatmap_y_bounds,
    heatmap_grid_size,
    "rateKickPassFriendlyCapability",
)
rate_chip_pass_friendly_heatmap_plotter = HeatmapPlotter(
    fig,
    heatmap_x_bounds,
    heatmap_y_bounds,
    heatmap_grid_size,
    "rateChipPassFriendlyCapability",
)
rate_pass_shoot_score_plotter = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "ratePassShootScore",
)
static_pass_quality = HeatmapPlotter(
    fig, heatmap_x_bounds, heatmap_y_bounds, heatmap_grid_size, "staticPassQuality",
)

pass_generator_plotter = PassGeneratorPlotter(fig)

fig.legend.click_policy = "hide"

heatmap_grid_size = 0.2

config = passing.getPassingConfig()


def plot_ssl_wrapper_at_idx(idx):
    ssl_wrapper_plotter.plot_ssl_wrapper(wrapper_proto_log[idx])

    field_length = wrapper_proto_log[idx].geometry.field.field_length / 1000
    field_width = wrapper_proto_log[idx].geometry.field.field_width / 1000

    the_world = world.World(
        wrapper_proto_log[idx].SerializeToString(), sensor_fusion_config
    )
    generator = pass_generator.EighteenZonePassGenerator(the_world, config)

    pass_dict = {
        "passer_point": the_world.ball().position(),
        "pass_speed": 5.0,
    }

    def ratePassCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.ratePass(the_world, pass_dict, config)

    def rateKickPassEnemyRiskCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.rateKickPassEnemyRisk(the_world, pass_dict, config)

    def rateKickPassFriendlyCapabilityCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.rateKickPassFriendlyCapability(the_world, pass_dict, config)

    def rateChipPassEnemyRiskCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.rateChipPassEnemyRisk(the_world, pass_dict, config)

    def rateChipPassFriendlyCapabilityCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.rateChipPassFriendlyCapability(the_world, pass_dict, config)

    def ratePassShootScoreCost(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.ratePassShootScore(the_world, pass_dict, config)

    def rateStaticPassQuality(x, y):
        pass_dict["receiver_point"] = world.Point(x, y)
        return passing.getStaticPositionQuality(the_world, pass_dict, config)

    #rate_pass_heatmap_plotter.plot_heatmap(ratePassCost)
    #rate_kick_pass_enemy_heatmap_plotter.plot_heatmap(rateKickPassEnemyRiskCost)
    #rate_kick_pass_friendly_heatmap_plotter.plot_heatmap(
    #    rateKickPassFriendlyCapabilityCost
    #)
    #rate_chip_pass_enemy_heatmap_plotter.plot_heatmap(rateChipPassEnemyRiskCost)
    rate_chip_pass_friendly_heatmap_plotter.plot_heatmap(
        rateChipPassFriendlyCapabilityCost
    )
    #rate_pass_shoot_score_plotter.plot_heatmap(ratePassShootScoreCost)
    #static_pass_quality.plot_heatmap(rateStaticPassQuality)

    #zones = pass_generator.getAllZones(the_world)
    #pass_generator_plotter.plot_zones(zones)

    #passes = generator.getBestPassesForAllZones(the_world)
    #pass_generator_plotter.plot_passes(passes)

    push_notebook()


show(fig, notebook_handle=True)

slider = ipywidgets.IntSlider(min=0, max=len(wrapper_proto_log) - 1, step=10)
ipywidgets.interact(plot_ssl_wrapper_at_idx, idx=slider)
# -


