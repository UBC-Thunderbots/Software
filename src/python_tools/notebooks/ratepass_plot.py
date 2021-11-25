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
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from python_tools.proto_log import ProtoLog
from proto.ai_config_pb2 import AiControlConfig
import ipywidgets
from IPython.display import display

wrapper_proto_log = ProtoLog(
    "test_data/SensorFusion_SSL_WrapperPacket", SSL_WrapperPacket,
)


# +
from bokeh.plotting import figure
from bokeh.io import output_notebook, show, push_notebook, curdoc
from bokeh.models import Button, CustomJS, Select
from python_tools.plotting.plot_ssl_wrapper import SSLWrapperPlotter, MM_PER_M

output_notebook()

fig = figure(plot_width=1000, plot_height=900, match_aspect=True)
fig.background_fill_color = "white"
field_length = wrapper_proto_log[0].geometry.field.field_length / MM_PER_M
field_width = wrapper_proto_log[0].geometry.field.field_width / MM_PER_M

ssl_wrapper_plotter = SSLWrapperPlotter(fig)

# TODO: UNIX SOCKET CODE
def plot_ssl_wrapper_at_idx(idx):
    ssl_wrapper_plotter.plot_ssl_wrapper(wrapper_proto_log[idx])
    push_notebook()


slider = ipywidgets.IntSlider(min=0, max=len(wrapper_proto_log) - 1)
ipywidgets.interact(plot_ssl_wrapper_at_idx, idx=slider)


# START AI BUTTON 
buttonStart = ipywidgets.Button(description="Start AI")
output = ipywidgets.Output()

display(buttonStart, output)

def start_clicked(b):
    with output:
        AiControlConfig.run_ai = 'true'
        print(AiControlConfig.run_ai)

buttonStart.on_click(start_clicked)


# STOP AI BUTTON
buttonStop = ipywidgets.Button(description="Stop AI")
output = ipywidgets.Output()

display(buttonStop, output)

def stop_clicked(b):
    with output:
        AiControlConfig.run_ai = 'false'
        print(AiControlConfig.run_ai)

buttonStop.on_click(stop_clicked)


# GAMESTATE OVERRIDE
def gamestate(b):
    with output:
        AiControlConfig.override_ai_play = b.new 
        print(AiControlConfig.override_ai_play)
        
gamestateOver = ipywidgets.Dropdown(
    options=[(''), ('Use GameController'), ('HALT'), ('STOP'), ('NORMAL_START'), ('FORCE_START'), ('PREPARE_KICKOFF_US'), ('PREPARE_KICKOFF_THEM'), ('PREPARE_PENALTY_US'), ('PREPARE_PENALTY_THEM'), ('DIRECT_FREE_US'), ('DIRECT_FREE_THEM'), ('INDIRECT_FREE_US'), ('INDIRECT_FREE_THEM'), ('TIMEOUT_US'), ('TIMEOUT_THEM'), ('GOAL_US'), ('GOAL_THEM'), ('BALL_PLACEMENT_US'), ('BALL_PLACEMENT_THEM')],
    value= '',
    description='Gamestate Override:',
)
output = ipywidgets.Output()

gamestateOver.observe(gamestate, names = 'value')
display(gamestateOver, output)


# PLAY OVERRIDE 
def play(b):
    with output:
        AiControlConfig.current_ai_play = b.new
        print(AiControlConfig.current_ai_play)


playOver = ipywidgets.Dropdown(
    options=[(''), ('Use AI Selection'), ('HaltPlay')],
    value= '',
    description='Play Override:',
)

output = ipywidgets.Output()

playOver.observe(play, names = 'value')
display(playOver, output)

show(fig, notebook_handle=True)


# -


