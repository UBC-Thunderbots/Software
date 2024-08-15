from software.thunderscope.common.frametime_counter import FrameTimeCounter
from software.thunderscope.widget_setup_functions import *
from software.thunderscope.constants import ProtoUnixIOTypes
from software.thunderscope.proto_unix_io import ProtoUnixIO
from typing import Sequence
from dataclasses import dataclass
from software.thunderscope.thunderscope_types import (
    TScopeTab,
    TScopeWidget,
    WidgetPosition,
    WidgetStretchData,
)
import pyqtgraph
import signal
import os


@dataclass
class TScopeConfig:
    """Data that describes a whole Thunderscope view"""

    proto_unix_io_map: dict[ProtoUnixIOTypes, ProtoUnixIO]
    """Mapping of protos needed for this view"""

    tabs: Sequence[TScopeTab]
    """List of tabs for this view"""


def initialize_application() -> None:
    """Initializes a QApplication

    MUST be done before any QWidgets are initialized, so this is called
    at the start of every config initialize method
    """
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Setup MainApp and initialize DockArea
    app = pyqtgraph.mkQApp("Thunderscope")


def configure_robot_view_fullsystem(
    fullsystem_proto_unix_io: ProtoUnixIO,
) -> TScopeWidget:
    """Returns Widget Data for the Robot View Widget for FullSystem

    :param fullsystem_proto_unix_io: the proto unix io to configure the widget with
    :return: the widget data
    """
    return TScopeWidget(
        name="Robot View",
        widget=setup_robot_view(
            **{
                "proto_unix_io": fullsystem_proto_unix_io,
                "available_control_modes": [
                    IndividualRobotMode.NONE,
                    IndividualRobotMode.MANUAL,
                    IndividualRobotMode.AI,
                ],
            }
        ),
        anchor="Logs",
        position=WidgetPosition.ABOVE,
    )


def configure_robot_view_diagnostics(
    diagnostics_proto_unix_io: ProtoUnixIO,
) -> TScopeWidget:
    """Returns Widget Data for the Robot View Widget for Diagnostics

    :param diagnostics_proto_unix_io: the diagnostics proto unix io to
                                      configure the widget with
    :return: the widget data
    """
    return TScopeWidget(
        name="Robot View",
        widget=setup_robot_view(
            **{
                "proto_unix_io": diagnostics_proto_unix_io,
                "available_control_modes": [
                    IndividualRobotMode.NONE,
                    IndividualRobotMode.MANUAL,
                ],
            }
        ),
        anchor="Logs",
        stretch=WidgetStretchData(y=5),
        position=WidgetPosition.ABOVE,
    )


def configure_estop(proto_unix_io):
    """Returns Widget Data for the Estop widget

    :param proto_unix_io: the proto unix io to configure the widget with
    :return: the widget data
    """
    return TScopeWidget(
        name="Estop",
        widget=setup_estop_view(**{"proto_unix_io": proto_unix_io}),
        anchor="Logs",
        stretch=WidgetStretchData(y=1),
        position=WidgetPosition.BOTTOM,
    )


def configure_base_fullsystem(
    full_system_proto_unix_io: ProtoUnixIO,
    sim_proto_unix_io: ProtoUnixIO,
    friendly_colour_yellow: bool,
    sandbox_mode: bool = False,
    replay: bool = False,
    replay_log: os.PathLike = None,
    visualization_buffer_size: int = 5,
    extra_widgets: list[TScopeWidget] = [],
    frame_swap_counter: FrameTimeCounter = None,
    refresh_counter: FrameTimeCounter = None,
) -> list:
    """Returns a list of widget data for a FullSystem tab
    along with any extra widgets passed in

    :param full_system_proto_unix_io: the proto unix io to configure widgets with
    :param sim_proto_unix_io: the proto unix io for the simulator
    :param friendly_colour_yellow: if this is Yellow FullSystem (True) or Blue (False)
    :param sandbox_mode: if sandbox mode should be enabled
    :param replay: True if in replay mode, False if not
    :param replay_log: the file path of the replay protos
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :param extra_widgets: a list of additional widget data to append
    :param frame_swap_counter: a FrameTimeCounter for the GLWidget to track
                               the time between frame swaps
    :param refresh_counter: a FrameTimeCounter for the refresh function
    :return: list of widget data for FullSystem
    """
    return [
        TScopeWidget(
            name="Field",
            widget=setup_gl_widget(
                **{
                    "sandbox_mode": sandbox_mode,
                    "replay": replay,
                    "replay_log": replay_log,
                    "full_system_proto_unix_io": full_system_proto_unix_io,
                    "sim_proto_unix_io": sim_proto_unix_io,
                    "friendly_colour_yellow": friendly_colour_yellow,
                    "visualization_buffer_size": visualization_buffer_size,
                    "frame_swap_counter": frame_swap_counter,
                }
            ),
        ),
        TScopeWidget(
            name="Parameters",
            widget=setup_parameter_widget(
                **{
                    "proto_unix_io": full_system_proto_unix_io,
                    "friendly_colour_yellow": friendly_colour_yellow,
                }
            ),
            anchor="Field",
            position=WidgetPosition.LEFT,
            has_refresh_func=False,
            stretch=WidgetStretchData(x=5),
        ),
        TScopeWidget(
            name="Error Log",
            widget=setup_robot_error_log_view_widget(
                **{"proto_unix_io": full_system_proto_unix_io}
            ),
            anchor="Parameters",
            position=WidgetPosition.ABOVE,
            stretch=WidgetStretchData(x=5),
        ),
        TScopeWidget(
            name="Logs",
            widget=setup_log_widget(**{"proto_unix_io": full_system_proto_unix_io}),
            anchor="Parameters",
            position=WidgetPosition.ABOVE,
            stretch=WidgetStretchData(x=5),
        ),
        TScopeWidget(
            name="Referee Info",
            widget=setup_referee_info(**{"proto_unix_io": full_system_proto_unix_io}),
            anchor="Field",
            position=WidgetPosition.BOTTOM,
            stretch=WidgetStretchData(y=4),
        ),
        TScopeWidget(
            name="Performance",
            widget=setup_performance_plot(
                **{"proto_unix_io": full_system_proto_unix_io}
            ),
            # this is because this widget specifically has to be added like so:
            # dock.addWidget(widget.win) instead of dock.addWidget(widget)
            # otherwise, it opens in a new window
            # the setup functions returns the widget.win and the refresh function separately
            in_window=True,
            anchor="Referee Info",
            position=WidgetPosition.BELOW,
            stretch=WidgetStretchData(y=4),
        ),
        TScopeWidget(
            name="FPS Widget",
            widget=setup_fps_widget(
                **{
                    "frame_swap_counter": frame_swap_counter,
                    "refresh_counter": refresh_counter,
                }
            ),
            anchor="Performance",
            position=WidgetPosition.BELOW,
            stretch=WidgetStretchData(y=4),
        ),
        TScopeWidget(
            name="Play Info",
            widget=setup_play_info(**{"proto_unix_io": full_system_proto_unix_io}),
            anchor="Referee Info",
            position=WidgetPosition.ABOVE,
            stretch=WidgetStretchData(y=4),
        ),
    ] + extra_widgets


def configure_base_diagnostics(
    diagnostics_proto_unix_io: ProtoUnixIO,
    current_proto_unix_io: ProtoUnixIO,
    extra_widgets: list[TScopeWidget] = [],
) -> list:
    """Returns a list of widget data for a Diagnostics tab
    along with any extra widgets passed in

    :param diagnostics_proto_unix_io: the proto unix io for diagnostics
    :param current_proto_unix_io: the current fullsystem proto unix io if it is running
    :param extra_widgets: a list of additional widget data to append
    :return: list of widget data for Diagnostics
    """
    return [
        TScopeWidget(
            name="Logs",
            widget=setup_log_widget(**{"proto_unix_io": diagnostics_proto_unix_io}),
        ),
        TScopeWidget(
            name="Error Log",
            widget=setup_robot_error_log_view_widget(
                **{"proto_unix_io": current_proto_unix_io}
            ),
            position=WidgetPosition.BELOW,
            anchor="Logs",
        ),
        TScopeWidget(
            name="Drive and Dribbler",
            widget=setup_drive_and_dribbler_widget(
                **{"proto_unix_io": diagnostics_proto_unix_io}
            ),
            anchor="Logs",
            position=WidgetPosition.RIGHT,
        ),
        TScopeWidget(
            name="Chicker",
            widget=setup_chicker_widget(**{"proto_unix_io": diagnostics_proto_unix_io}),
            anchor="Drive and Dribbler",
            position=WidgetPosition.BELOW,
        ),
        TScopeWidget(
            name="Manual Control Input",
            widget=setup_diagnostics_input_widget(),
            anchor="Chicker",
            position=WidgetPosition.TOP,
        ),
    ] + extra_widgets


def configure_two_ai_gamecontroller_view(
    visualization_buffer_size: int = 5,
) -> TScopeConfig:
    """Constructs the Thunderscope Config for a view with 2 FullSystem tabs (Blue and Yellow)
    And 1 Gamecontroller tab

    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :return: the Thunderscope Config for this view
    """
    proto_unix_io_map = {
        ProtoUnixIOTypes.BLUE: ProtoUnixIO(),
        ProtoUnixIOTypes.YELLOW: ProtoUnixIO(),
        ProtoUnixIOTypes.SIM: ProtoUnixIO(),
    }

    # Must be called before widgets are initialized below
    initialize_application()

    # setup frametime counter
    blue_refresh_func_frametime_counter = FrameTimeCounter()
    blue_frame_swap_frametime_counter = FrameTimeCounter()

    yellow_refresh_func_frametime_counter = FrameTimeCounter()
    yellow_frame_swap_frametime_counter = FrameTimeCounter()

    return TScopeConfig(
        proto_unix_io_map=proto_unix_io_map,
        tabs=[
            TScopeTab(
                name="Blue FullSystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=False,
                    visualization_buffer_size=visualization_buffer_size,
                    sandbox_mode=True,
                    extra_widgets=[],
                    frame_swap_counter=blue_frame_swap_frametime_counter,
                    refresh_counter=blue_refresh_func_frametime_counter,
                ),
                refresh_counter=blue_refresh_func_frametime_counter,
            ),
            TScopeTab(
                name="Yellow FullSystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.YELLOW
                    ],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=True,
                    visualization_buffer_size=visualization_buffer_size,
                    sandbox_mode=True,
                    extra_widgets=[],
                    frame_swap_counter=yellow_frame_swap_frametime_counter,
                    refresh_counter=yellow_refresh_func_frametime_counter,
                ),
                refresh_counter=yellow_refresh_func_frametime_counter,
            ),
        ],
    )


def configure_simulated_test_view(
    simulator_proto_unix_io: ProtoUnixIO,
    blue_full_system_proto_unix_io: ProtoUnixIO,
    yellow_full_system_proto_unix_io: ProtoUnixIO,
    visualization_buffer_size: int = 5,
) -> TScopeConfig:
    """Constructs the Thunderscope Config for simulated tests
    A view with 2 FullSystem tabs (Blue and Yellow)
    And 1 Gamecontroller tab

    :param simulator_proto_unix_io: the proto unix io for the simulator
    :param blue_full_system_proto_unix_io: the proto unix io for the blue fullsystem
    :param yellow_full_system_proto_unix_io: the proto unix io for the yellow fullsystem
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :return: the Thunderscope Config for this view
    """
    proto_unix_io_map = {
        ProtoUnixIOTypes.BLUE: blue_full_system_proto_unix_io,
        ProtoUnixIOTypes.YELLOW: yellow_full_system_proto_unix_io,
        ProtoUnixIOTypes.SIM: simulator_proto_unix_io,
    }

    # Must be called before widgets are initialized below
    initialize_application()

    return TScopeConfig(
        proto_unix_io_map=proto_unix_io_map,
        tabs=[
            TScopeTab(
                name="Blue FullSystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=False,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[],
                ),
            ),
            TScopeTab(
                name="Yellow FullSystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.YELLOW
                    ],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=True,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[],
                ),
            ),
        ],
    )


def configure_field_test_view(
    simulator_proto_unix_io: ProtoUnixIO,
    blue_full_system_proto_unix_io: ProtoUnixIO,
    yellow_full_system_proto_unix_io: ProtoUnixIO,
    visualization_buffer_size: int = 5,
    yellow_is_friendly: bool = False,
) -> TScopeConfig:
    """Constructs the Thunderscope Config for field tests
    A view with 2 FullSystem tabs (Blue and Yellow)
    And 1 Gamecontroller tab

    :param simulator_proto_unix_io: the proto unix io for the simulator
    :param blue_full_system_proto_unix_io: the proto unix io for the blue fullsystem
    :param yellow_full_system_proto_unix_io: the proto unix io for the yellow fullsystem
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :return: the Thunderscope Config for this view
    """
    proto_unix_io_map = {
        ProtoUnixIOTypes.BLUE: blue_full_system_proto_unix_io,
        ProtoUnixIOTypes.YELLOW: yellow_full_system_proto_unix_io,
        ProtoUnixIOTypes.SIM: simulator_proto_unix_io,
    }

    # Must be called before widgets are initialized below
    initialize_application()

    tabs = []
    # Choose the right tab based on yellow/blue
    if yellow_is_friendly:
        tabs = [
            TScopeTab(
                name="Yellow FullSystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.YELLOW
                    ],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=True,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[],
                ),
            )
        ]
    else:
        tabs = [
            TScopeTab(
                name="Blue FullSystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=False,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[],
                ),
            )
        ]

    return TScopeConfig(proto_unix_io_map=proto_unix_io_map, tabs=tabs)


def configure_replay_view(
    blue_replay_log: os.PathLike,
    yellow_replay_log: os.PathLike,
    visualization_buffer_size: int = 5,
) -> TScopeConfig:
    """Constructs the Thunderscope Config for a replay view
    Can have 1 or 2 FullSystem tabs but no GameController tab
    GLWidget will now have Player controls

    :param blue_replay_log: the file path for the blue replay log
    :param yellow_replay_log: the file path for the yellow replay log
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :return: the Thunderscope Config for this view
    """
    proto_unix_io_map = {ProtoUnixIOTypes.SIM: ProtoUnixIO()}
    tabs = []

    # Must be called before widgets are initialized below
    initialize_application()

    if blue_replay_log:
        proto_unix_io_map[ProtoUnixIOTypes.BLUE] = ProtoUnixIO()
        tabs.append(
            TScopeTab(
                name="Blue FullSystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=False,
                    replay=True,
                    replay_log=blue_replay_log,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[],
                ),
            )
        )

    if yellow_replay_log:
        proto_unix_io_map[ProtoUnixIOTypes.YELLOW] = ProtoUnixIO()
        tabs.append(
            TScopeTab(
                name="Yellow FullSystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.YELLOW
                    ],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=True,
                    replay=True,
                    replay_log=yellow_replay_log,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[],
                ),
            )
        )

    return TScopeConfig(proto_unix_io_map=proto_unix_io_map, tabs=tabs)


def configure_ai_or_diagnostics(
    load_blue: bool,
    load_yellow: bool,
    load_diagnostics: bool,
    visualization_buffer_size: int = 5,
) -> TScopeConfig:
    """Constructs a view with one of:
        - 1 Fullsystem (Blue or Yellow)
        - 1 Fullsystem (Blue or Yellow) and Diagnostics
        - Diagnostics

    :param load_blue: if blue fullsystem should be loaded
    :param load_yellow: if yellow fullsystem should be loaded
    :param load_diagnostics: if diagnostics should be loaded
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :return: the Thunderscope Config for this view
    """

    def get_extra_widgets(proto_unix_io):
        """Gets the extra widgets for the fullsystem tab
        :param proto_unix_io: the proto unix io to configure widgets with
        :return: list of widget data for the extra widgets
        """
        extra_widgets = [configure_robot_view_fullsystem(proto_unix_io)]
        extra_widgets.append(configure_estop(proto_unix_io))
        return extra_widgets

    proto_unix_io_map = {ProtoUnixIOTypes.SIM: ProtoUnixIO()}
    tabs = []

    # Must be called before widgets are initialized below
    initialize_application()

    if load_blue:
        proto_unix_io_map[ProtoUnixIOTypes.BLUE] = ProtoUnixIO()
        proto_unix_io_map[ProtoUnixIOTypes.CURRENT] = proto_unix_io_map[
            ProtoUnixIOTypes.BLUE
        ]
        tabs.append(
            TScopeTab(
                name="Blue Fullsystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=False,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=get_extra_widgets(
                        proto_unix_io_map[ProtoUnixIOTypes.BLUE]
                    ),
                ),
            )
        )
    elif load_yellow:
        proto_unix_io_map[ProtoUnixIOTypes.YELLOW] = ProtoUnixIO()
        proto_unix_io_map[ProtoUnixIOTypes.CURRENT] = proto_unix_io_map[
            ProtoUnixIOTypes.YELLOW
        ]
        tabs.append(
            TScopeTab(
                name="Yellow Fullsystem",
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.YELLOW
                    ],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=True,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=get_extra_widgets(
                        proto_unix_io_map[ProtoUnixIOTypes.YELLOW]
                    ),
                ),
            )
        )

    if load_diagnostics:
        proto_unix_io_map[ProtoUnixIOTypes.DIAGNOSTICS] = (
            proto_unix_io_map[ProtoUnixIOTypes.BLUE]
            if load_blue
            else (
                proto_unix_io_map[ProtoUnixIOTypes.YELLOW]
                if load_yellow
                else ProtoUnixIO()
            )
        )
        if not load_blue and not load_yellow:
            proto_unix_io_map[ProtoUnixIOTypes.CURRENT] = proto_unix_io_map[
                ProtoUnixIOTypes.DIAGNOSTICS
            ]
        diagnostics_extra_widgets = (
            []
            if load_blue or load_yellow
            else [
                configure_robot_view_diagnostics(
                    proto_unix_io_map[ProtoUnixIOTypes.DIAGNOSTICS]
                ),
            ]
        )
        tabs.append(
            TScopeTab(
                name="Robot Diagnostics",
                widgets=configure_base_diagnostics(
                    diagnostics_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.DIAGNOSTICS
                    ],
                    current_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.CURRENT],
                    extra_widgets=[
                        configure_estop(proto_unix_io_map[ProtoUnixIOTypes.DIAGNOSTICS])
                    ]
                    + diagnostics_extra_widgets,
                ),
            )
        )

    return TScopeConfig(proto_unix_io_map=proto_unix_io_map, tabs=tabs)
