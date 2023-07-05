from software.thunderscope.widget_setup_functions import *
from software.thunderscope.constants import (
    TabNames,
    ProtoUnixIOTypes,
    GAME_CONTROLLER_URL,
)
from software.thunderscope.proto_unix_io import ProtoUnixIO
from typing import Sequence, Dict
from software.thunderscope.thunderscope_types import (
    TScopeTab,
    TScopeWidget,
    TScopeQTTab,
    TScopeWebTab,
    WidgetStretchData,
)
import pyqtgraph
import signal
import qdarktheme
from qt_material import apply_stylesheet, list_themes


class TScopeConfig:
    """
    Data that described a whole Thunderscope view
    """

    proto_unix_io_map: Dict[
        ProtoUnixIOTypes, ProtoUnixIO
    ]  # mapping of protos needed for this view
    tabs: Sequence[TScopeTab]  # list of tabs for this view

    def __init__(
        self,
        proto_unix_io_map: Dict[ProtoUnixIOTypes, ProtoUnixIO],
        tabs: Sequence[TScopeTab],
    ):
        self.proto_unix_io_map = proto_unix_io_map
        self.tabs = tabs


def initialize_application():
    """
    Initializes a QApplication

    MUST be done before any QWidgets are initialized, so this is called
    at the start of every config initialize method
    """
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Setup MainApp and initialize DockArea
    app = pyqtgraph.mkQApp("Thunderscope")

    # Setup stylesheet
    apply_stylesheet(app, theme="dark_blue.xml")


def configure_cost_vis(proto_unix_io):
    """
    Returns Widget Data for the Cost Visualization Widget
    :param proto_unix_io: the proto unix io to configure the widget with
    :return: the widget data
    """
    return TScopeWidget(
        name="Cost Visualization",
        widget=setup_cost_visualization_widget(**{"proto_unix_io": proto_unix_io}),
        anchor="Field",
        position="right",
    )


def configure_robot_view_fullsystem(fullsystem_proto_unix_io):
    """
    Returns Widget Data for the Robot View Widget for FullSystem
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
        position="above",
    )


def configure_robot_view_diagnostics(diagnostics_proto_unix_io):
    """
    Returns Widget Data for the Robot View Widget for Diagnostics
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
        position="above",
    )


def configure_estop(proto_unix_io):
    """
    Returns Widget Data for the Estop widget
    :param proto_unix_io: the proto unix io to configure the widget with
    :return:
    """
    return TScopeWidget(
        name="Estop",
        widget=setup_estop_view(**{"proto_unix_io": proto_unix_io}),
        anchor="Logs",
        stretch=WidgetStretchData(y=1),
        position="bottom",
    )


def configure_base_fullsystem(
    full_system_proto_unix_io,
    sim_proto_unix_io,
    friendly_colour_yellow,
    replay=False,
    replay_log=None,
    visualization_buffer_size=5,
    extra_widgets=[],
):
    """
    Returns a list of widget data for a FullSystem tab
    along with any extra widgets passed in

    :param full_system_proto_unix_io: the proto unix io to configure widgets with
    :param sim_proto_unix_io: the proto unix io for the simulator
    :param friendly_colour_yellow: if this is Yellow FullSystem (True) or Blue (False)
    :param replay: True if in replay mode, False if not
    :param replay_log: the file path of the replay protos
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :param extra_widgets: a list of additional widget data to append
    :return: list of widget data for FullSystem
    """
    return [
        TScopeWidget(
            name="Field",
            widget=setup_field_widget(
                **{
                    "replay": replay,
                    "replay_log": replay_log,
                    "full_system_proto_unix_io": full_system_proto_unix_io,
                    "sim_proto_unix_io": sim_proto_unix_io,
                    "friendly_colour_yellow": friendly_colour_yellow,
                    "visualization_buffer_size": visualization_buffer_size,
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
            position="left",
            has_refresh_func=False,
        ),
        TScopeWidget(
            name="Logs",
            widget=setup_log_widget(**{"proto_unix_io": full_system_proto_unix_io}),
            anchor="Parameters",
            position="above",
        ),
        TScopeWidget(
            name="Referee Info",
            widget=setup_referee_info(**{"proto_unix_io": full_system_proto_unix_io}),
            anchor="Field",
            position="bottom",
        ),
        TScopeWidget(
            name="Play Info",
            widget=setup_play_info(**{"proto_unix_io": full_system_proto_unix_io}),
            anchor="Referee Info",
            position="above",
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
            anchor="Play Info",
            position="right",
        ),
    ] + extra_widgets


def configure_base_diagnostics(diagnostics_proto_unix_io, extra_widgets=[]):
    """
    Returns a list of widget data for a Diagnostics tab
    along with any extra widgets passed in

    :param diagnostics_proto_unix_io: the proto unix io for diagnostics
    :param extra_widgets: a list of additional widget data to append
    :return: list of widget data for Diagnostics
    """
    return [
        TScopeWidget(
            name="Logs",
            widget=setup_log_widget(**{"proto_unix_io": diagnostics_proto_unix_io}),
        ),
        TScopeWidget(
            name="Drive and Dribbler",
            widget=setup_drive_and_dribbler_widget(
                **{"proto_unix_io": diagnostics_proto_unix_io}
            ),
            anchor="Logs",
            position="right",
        ),
        TScopeWidget(
            name="Chicker",
            widget=setup_chicker_widget(**{"proto_unix_io": diagnostics_proto_unix_io}),
            anchor="Drive and Dribbler",
            position="below",
        ),
        TScopeWidget(
            name="Manual Control Input",
            widget=setup_diagnostics_input_widget(),
            anchor="Chicker",
            position="top",
        ),
    ] + extra_widgets


def configure_two_ai_gamecontroller_view(
    visualization_buffer_size=5, cost_visualization=False
):
    """
    Constructs the Thunderscope Config for a view with 2 FullSystem tabs (Blue and Yellow)
    And 1 Gamecontroller tab

    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :param cost_visualization: True if cost visualization widget should be enabled
                                False if not
    :return: the Thunderscope Config for this view
    """
    proto_unix_io_map = {
        ProtoUnixIOTypes.BLUE: ProtoUnixIO(),
        ProtoUnixIOTypes.YELLOW: ProtoUnixIO(),
        ProtoUnixIOTypes.SIM: ProtoUnixIO(),
    }

    # Must be called before widgets are initialized below
    initialize_application()

    return TScopeConfig(
        proto_unix_io_map=proto_unix_io_map,
        tabs=[
            TScopeQTTab(
                name="Blue FullSystem",
                key=TabNames.BLUE,
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=False,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[
                        configure_cost_vis(proto_unix_io_map[ProtoUnixIOTypes.BLUE])
                    ]
                    if cost_visualization
                    else [],
                ),
            ),
            TScopeQTTab(
                name="Yellow FullSystem",
                key=TabNames.YELLOW,
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.YELLOW
                    ],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=True,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[
                        configure_cost_vis(proto_unix_io_map[ProtoUnixIOTypes.YELLOW])
                    ]
                    if cost_visualization
                    else [],
                ),
            ),
            TScopeWebTab(
                name="Gamecontroller",
                key=TabNames.GAMECONTROLLER,
                url=GAME_CONTROLLER_URL,
            ),
        ],
    )


def configure_simulated_test_view(
    simulator_proto_unix_io,
    blue_full_system_proto_unix_io,
    yellow_full_system_proto_unix_io,
    visualization_buffer_size=5,
    cost_visualization=False,
):
    """
    Constructs the Thunderscope Config for simulated tests
    A view with 2 FullSystem tabs (Blue and Yellow)
    And 1 Gamecontroller tab

    :param simulator_proto_unix_io: the proto unix io for the simulator
    :param blue_full_system_proto_unix_io: the proto unix io for the blue fullsystem
    :param yellow_full_system_proto_unix_io: the proto unix io for the yellow fullsystem
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :param cost_visualization: True if cost visualization widget should be enabled
                                False if not
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
            TScopeQTTab(
                name="Blue FullSystem",
                key=TabNames.BLUE,
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=False,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[
                        configure_cost_vis(proto_unix_io_map[ProtoUnixIOTypes.BLUE])
                    ]
                    if cost_visualization
                    else [],
                ),
            ),
            TScopeQTTab(
                name="Yellow FullSystem",
                key=TabNames.YELLOW,
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.YELLOW
                    ],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=True,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[
                        configure_cost_vis(proto_unix_io_map[ProtoUnixIOTypes.YELLOW])
                    ]
                    if cost_visualization
                    else [],
                ),
            ),
        ],
    )


def configure_field_test_view(
    simulator_proto_unix_io,
    blue_full_system_proto_unix_io,
    yellow_full_system_proto_unix_io,
    visualization_buffer_size=5,
    cost_visualization=False,
    yellow_is_friendly=False,
):
    """
    Constructs the Thunderscope Config for field tests
    A view with 2 FullSystem tabs (Blue and Yellow)
    And 1 Gamecontroller tab

    :param simulator_proto_unix_io: the proto unix io for the simulator
    :param blue_full_system_proto_unix_io: the proto unix io for the blue fullsystem
    :param yellow_full_system_proto_unix_io: the proto unix io for the yellow fullsystem
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :param cost_visualization: True if cost visualization widget should be enabled
                                False if not
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
            TScopeQTTab(
                name="Yellow FullSystem",
                key=TabNames.YELLOW,
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.YELLOW
                    ],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=True,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[
                        configure_cost_vis(proto_unix_io_map[ProtoUnixIOTypes.YELLOW])
                    ]
                    if cost_visualization
                    else [],
                ),
            )
        ]
    else:
        tabs = [
            TScopeQTTab(
                name="Blue FullSystem",
                key=TabNames.BLUE,
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=False,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[
                        configure_cost_vis(proto_unix_io_map[ProtoUnixIOTypes.BLUE])
                    ]
                    if cost_visualization
                    else [],
                ),
            )
        ]

    return TScopeConfig(proto_unix_io_map=proto_unix_io_map, tabs=tabs)


def configure_replay_view(
    blue_replay_log,
    yellow_replay_log,
    visualization_buffer_size=5,
    cost_visualization=False,
):
    """
    Constructs the Thunderscope Config for a replay view
    Can have 1 or 2 FullSystem tabs but no GameController tab
    Field widget will now have Player controls

    :param blue_replay_log: the file path for the blue replay log
    :param yellow_replay_log: the file path for the yellow replay log
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :param cost_visualization: True if cost visualization widget should be enabled
                                False if not
    :return: the Thunderscope Config for this view
    """
    proto_unix_io_map = {ProtoUnixIOTypes.SIM: ProtoUnixIO()}
    tabs = []

    # Must be called before widgets are initialized below
    initialize_application()

    if blue_replay_log:
        proto_unix_io_map[ProtoUnixIOTypes.BLUE] = ProtoUnixIO()
        tabs.append(
            TScopeQTTab(
                name="Blue FullSystem",
                key=TabNames.BLUE,
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=False,
                    replay=True,
                    replay_log=blue_replay_log,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[
                        configure_cost_vis(proto_unix_io_map[ProtoUnixIOTypes.BLUE])
                    ]
                    if cost_visualization
                    else [],
                ),
            )
        )

    if yellow_replay_log:
        proto_unix_io_map[ProtoUnixIOTypes.YELLOW] = ProtoUnixIO()
        tabs.append(
            TScopeQTTab(
                name="Yellow FullSystem",
                key=TabNames.YELLOW,
                widgets=configure_base_fullsystem(
                    full_system_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.YELLOW
                    ],
                    sim_proto_unix_io=proto_unix_io_map[ProtoUnixIOTypes.SIM],
                    friendly_colour_yellow=True,
                    replay=True,
                    replay_log=yellow_replay_log,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[
                        configure_cost_vis(proto_unix_io_map[ProtoUnixIOTypes.YELLOW])
                    ]
                    if cost_visualization
                    else [],
                ),
            )
        )

    return TScopeConfig(proto_unix_io_map=proto_unix_io_map, tabs=tabs)


def configure_ai_or_diagnostics(
    load_blue,
    load_yellow,
    load_diagnostics,
    visualization_buffer_size=5,
    cost_visualization=False,
):
    """
    Constructs a view with one of:
        - 1 Fullsystem (Blue or Yellow)
        - 1 Fullsystem (Blue or Yellow) and Diagnostics
        - Diagnostics

    :param load_blue: if blue fullsystem should be loaded
    :param load_yellow: if yellow fullsystem should be loaded
    :param load_diagnostics: if diagnostics should be loaded
    :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime.
    :param cost_visualization: True if cost visualization widget should be enabled
                                False if not
    :return: the Thunderscope Config for this view
    """

    def get_extra_widgets(proto_unix_io):
        """
        Gets the extra widgets for the fullsystem tab
        :param proto_unix_io: the proto unix io to configure widgets with
        :return: list of widget data for the extra widgets
        """
        extra_widgets = (
            [configure_cost_vis(proto_unix_io)] if cost_visualization else []
        )
        extra_widgets.append(configure_robot_view_fullsystem(proto_unix_io))
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
            TScopeQTTab(
                name="Blue Fullsystem",
                key=TabNames.BLUE,
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
            TScopeQTTab(
                name="Yellow Fullsystem",
                key=TabNames.YELLOW,
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
            else proto_unix_io_map[ProtoUnixIOTypes.YELLOW]
            if load_yellow
            else ProtoUnixIO()
        )
        if not load_blue and not load_yellow:
            proto_unix_io_map[ProtoUnixIOTypes.CURRENT] = proto_unix_io_map[
                ProtoUnixIOTypes.DIAGNOSTICS
            ]
        tabs.append(
            TScopeQTTab(
                name="Robot Diagnostics",
                key=TabNames.DIAGNOSTICS,
                widgets=configure_base_diagnostics(
                    diagnostics_proto_unix_io=proto_unix_io_map[
                        ProtoUnixIOTypes.DIAGNOSTICS
                    ],
                    extra_widgets=[
                        configure_estop(
                            proto_unix_io_map[ProtoUnixIOTypes.DIAGNOSTICS]
                        ),
                    ] + ([
                        configure_robot_view_diagnostics(
                            proto_unix_io_map[ProtoUnixIOTypes.DIAGNOSTICS]
                        ),
                    ] if (not load_blue and not load_yellow) else []),
                ),
            )
        )

    return TScopeConfig(proto_unix_io_map=proto_unix_io_map, tabs=tabs)
