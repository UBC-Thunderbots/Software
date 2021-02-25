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
    "/home/jordan/log_ssl_wrapper_filtered_world_test/SensorFusion_SSL_WrapperPacket",
    SSL_WrapperPacket,
)
# -


world.getDefaultSensorFusionConfig()

# +
from bokeh.plotting import figure
from bokeh.io import output_notebook, show, push_notebook
from python_tools.plotting.plot_ssl_wrapper import SSLWrapperPlotter
from bokeh import palettes
from bokeh.models import ColorBar, tickers
from bokeh.layouts import row

output_notebook()

fig = figure(plot_width=900, plot_height=900, match_aspect=True)

ssl_wrapper_plotter = SSLWrapperPlotter(fig)

color_bar = ColorBar(label_standoff=12)


def get_ratepass_grid(ssl_wrapper):
    the_world = world.World(ssl_wrapper.SerializeToString(), dict())

    field_length = ssl_wrapper.geometry.field.field_length / 1000
    field_width = ssl_wrapper.geometry.field.field_width / 1000

    grid_size = 0.05
    grid_dims = (int(field_length // grid_size), int(field_width // grid_size))
    ratepass_grid = np.ndarray(grid_dims)

    xcoords = np.arange(-field_length / 2, field_length / 2, grid_size)
    ycoords = np.arange(-field_width / 2, field_width / 2, grid_size)

    for x_idx, x in enumerate(xcoords):
        for y_idx, y in enumerate(ycoords):
            pass_dict = {
                "passer_point": world.Point(4, 2),
                "receiver_point": world.Point(x, y),
                "pass_speed": 5.0,
                "receive_and_dribble": False,
            }
            if x_idx < grid_dims[0] and y_idx < grid_dims[1]:
                ratepass_grid[x_idx, y_idx] = passing.ratePassShootScore(the_world, pass_dict)
    return np.flip(np.rot90(ratepass_grid), axis=0)


def plot_ssl_wrapper_at_idx(idx):
    ssl_wrapper_plotter.plot_ssl_wrapper(wrapper_proto_log[idx])

    field_length = wrapper_proto_log[idx].geometry.field.field_length / 1000
    field_width = wrapper_proto_log[idx].geometry.field.field_width / 1000

    ratepass_grid = get_ratepass_grid(wrapper_proto_log[idx])
    image = fig.image(
        image=[ratepass_grid],
        x=-field_length / 2,
        y=-field_width / 2,
        dh=field_width,
        dw=field_length,
        palette=palettes.viridis(100),
        level="image",
    )
    color_bar = ColorBar(color_mapper=image.glyph.color_mapper, label_standoff=12)
    fig.add_layout(color_bar, 'right')
    push_notebook()


show(fig, notebook_handle=True)

slider = ipywidgets.IntSlider(min=0, max=len(wrapper_proto_log) - 1)
ipywidgets.interact(plot_ssl_wrapper_at_idx, idx=slider)
# -


