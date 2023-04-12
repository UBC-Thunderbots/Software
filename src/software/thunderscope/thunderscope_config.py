from software.thunderscope.widget_setup_functions import *
from software.thunderscope.thunderscope_config_types import *
from software.thunderscope.constants import TabKeys

GAME_CONTROLLER_URL = "http://localhost:8081"


def configure_cost_vis(proto_unix_io):
    """
    Returns Widget Data for the Cost Visualization Widget
    :param proto_unix_io: the proto unix io key to configure the widget with
    :return: the widget data
    """
    return TScopeWidget(
        name="Cost Visualization",
        setup=setup_cost_visualization_widget,
        anchor="Field",
        position="right",
        deps=[
            TScopeWidgetDep(
                name="proto_unix_io",
                dep_type=ParamTypes.PROTO_UNIX_IO,
                value=proto_unix_io,
            )
        ],
    )


def configure_robot_view_fullsystem(proto_unix_io):
    """
    Returns Widget Data for the Robot View Widget for FullSystem
    :param proto_unix_io: the proto unix io key to configure the widget with
    :return: the widget data
    """
    return TScopeWidget(
        name="Robot View",
        setup=setup_robot_view,
        anchor="Logs",
        position="above",
        deps=[
            TScopeWidgetDep(
                name="proto_unix_io",
                dep_type=ParamTypes.PROTO_UNIX_IO,
                value=proto_unix_io,
            ),
            TScopeWidgetDep(
                name="available_control_modes",
                dep_type=ParamTypes.LIST,
                value=[
                    IndividualRobotMode.NONE,
                    IndividualRobotMode.MANUAL,
                    IndividualRobotMode.AI,
                ],
            ),
        ],
    )


def configure_robot_view_diagnostics():
    """
    Returns Widget Data for the Robot View Widget for Diagnostics
    :return: the widget data
    """
    return TScopeWidget(
        name="Robot View",
        setup=setup_robot_view,
        anchor="Logs",
        stretch=WidgetStretchData(y=5),
        position="above",
        deps=[
            TScopeWidgetDep(
                name="proto_unix_io",
                dep_type=ParamTypes.PROTO_UNIX_IO,
                value=ProtoUnixIOTypes.DIAGNOSTICS,
            ),
            TScopeWidgetDep(
                name="available_control_modes",
                dep_type=ParamTypes.LIST,
                value=[IndividualRobotMode.NONE, IndividualRobotMode.MANUAL,],
            ),
        ],
    )


def configure_base_fullsystem(
    full_system_proto_unix_io,
    friendly_colour_yellow,
    replay=False,
    replay_log=None,
    visualization_buffer_size=5,
    extra_widgets=[],
):
    """
    Returns a list of widget data for a FullSystem tab
    along with any extra widgets passed in

    :param full_system_proto_unix_io: the proto unix io key to configure widgets with
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
            setup=setup_field_widget,
            deps=[
                TScopeWidgetDep(name="replay", dep_type=ParamTypes.BOOL, value=replay),
                TScopeWidgetDep(
                    name="replay_log", dep_type=ParamTypes.STRING, value=replay_log
                ),
                TScopeWidgetDep(
                    name="sim_proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=ProtoUnixIOTypes.SIM,
                ),
                TScopeWidgetDep(
                    name="full_system_proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=full_system_proto_unix_io,
                ),
                TScopeWidgetDep(
                    name="friendly_colour_yellow",
                    dep_type=ParamTypes.BOOL,
                    value=friendly_colour_yellow,
                ),
                TScopeWidgetDep(
                    name="visualization_buffer_size",
                    dep_type=ParamTypes.INT,
                    value=visualization_buffer_size,
                ),
            ],
        ),
        TScopeWidget(
            name="Parameters",
            setup=setup_parameter_widget,
            anchor="Field",
            position="left",
            no_refresh=True,
            deps=[
                TScopeWidgetDep(
                    name="proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=full_system_proto_unix_io,
                ),
                TScopeWidgetDep(
                    name="friendly_colour_yellow",
                    dep_type=ParamTypes.BOOL,
                    value=friendly_colour_yellow,
                ),
            ],
        ),
        TScopeWidget(
            name="Logs",
            setup=setup_log_widget,
            anchor="Parameters",
            position="above",
            deps=[
                TScopeWidgetDep(
                    name="proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=full_system_proto_unix_io,
                )
            ],
        ),
        TScopeWidget(
            name="Referee Info",
            setup=setup_referee_info,
            anchor="Field",
            position="bottom",
            deps=[
                TScopeWidgetDep(
                    name="proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=full_system_proto_unix_io,
                )
            ],
        ),
        TScopeWidget(
            name="Play Info",
            setup=setup_play_info,
            anchor="Referee Info",
            position="above",
            deps=[
                TScopeWidgetDep(
                    name="proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=full_system_proto_unix_io,
                )
            ],
        ),
        TScopeWidget(
            name="Performance",
            setup=setup_performance_plot,
            in_window=True,
            anchor="Play Info",
            position="right",
            deps=[
                TScopeWidgetDep(
                    name="proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=full_system_proto_unix_io,
                )
            ],
        ),
    ] + extra_widgets


def configure_base_diagnostics(extra_widgets=[]):
    """
    Returns a list of widget data for a Diagnostics tab
    along with any extra widgets passed in


    :param extra_widgets: a list of additional widget data to append
    :return: list of widget data for Diagnostics
    """
    return [
        TScopeWidget(
            name="Logs",
            setup=setup_log_widget,
            deps=[
                TScopeWidgetDep(
                    name="proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=ProtoUnixIOTypes.DIAGNOSTICS,
                )
            ],
        ),
        TScopeWidget(
            name="Drive and Dribbler",
            setup=setup_drive_and_dribbler_widget,
            anchor="Logs",
            position="right",
            deps=[
                TScopeWidgetDep(
                    name="proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=ProtoUnixIOTypes.DIAGNOSTICS,
                )
            ],
        ),
        TScopeWidget(
            name="Chicker",
            setup=setup_chicker_widget,
            anchor="Drive and Dribbler",
            position="below",
            deps=[
                TScopeWidgetDep(
                    name="proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=ProtoUnixIOTypes.DIAGNOSTICS,
                )
            ],
        ),
        TScopeWidget(
            name="Manual Control Input",
            setup=setup_diagnostics_input_widget,
            anchor="Chicker",
            position="top",
            deps=[],
        ),
        TScopeWidget(
            name="Estop",
            setup=setup_estop_view,
            anchor="Logs",
            stretch=WidgetStretchData(y=1),
            position="bottom",
            deps=[
                TScopeWidgetDep(
                    name="proto_unix_io",
                    dep_type=ParamTypes.PROTO_UNIX_IO,
                    value=ProtoUnixIOTypes.DIAGNOSTICS,
                )
            ],
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
    return TScopeConfig(
        protos={
            ProtoUnixIOTypes.BLUE: None,
            ProtoUnixIOTypes.YELLOW: None,
            ProtoUnixIOTypes.SIM: None,
        },
        tabs=[
            TScopeQTTab(
                name="Blue FullSystem",
                key=TabKeys.BLUE,
                widgets=configure_base_fullsystem(
                    ProtoUnixIOTypes.BLUE,
                    False,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[configure_cost_vis(ProtoUnixIOTypes.BLUE)]
                    if cost_visualization
                    else [],
                ),
            ),
            TScopeQTTab(
                name="Yellow FullSystem",
                key=TabKeys.YELLOW,
                widgets=configure_base_fullsystem(
                    ProtoUnixIOTypes.YELLOW,
                    True,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[configure_cost_vis(ProtoUnixIOTypes.YELLOW)]
                    if cost_visualization
                    else [],
                ),
            ),
            TScopeWebTab(
                name="Gamecontroller",
                key=TabKeys.GAMECONTROLLER,
                url=GAME_CONTROLLER_URL,
            ),
        ],
    )


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
    protos = {ProtoUnixIOTypes.SIM: None}
    tabs = []

    if blue_replay_log:
        protos[ProtoUnixIOTypes.BLUE] = None
        tabs.append(
            TScopeQTTab(
                name="Blue FullSystem",
                key=TabKeys.BLUE,
                widgets=configure_base_fullsystem(
                    ProtoUnixIOTypes.BLUE,
                    False,
                    replay=True,
                    replay_log=blue_replay_log,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[configure_cost_vis(ProtoUnixIOTypes.BLUE)]
                    if cost_visualization
                    else [],
                ),
            )
        )

    if yellow_replay_log:
        protos[ProtoUnixIOTypes.YELLOW] = None
        tabs.append(
            TScopeQTTab(
                name="Yellow FullSystem",
                key=TabKeys.YELLOW,
                widgets=configure_base_fullsystem(
                    ProtoUnixIOTypes.YELLOW,
                    False,
                    replay=True,
                    replay_log=yellow_replay_log,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=[configure_cost_vis(ProtoUnixIOTypes.YELLOW)]
                    if cost_visualization
                    else [],
                ),
            )
        )

    return TScopeConfig(protos=protos, tabs=tabs)


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
        :param proto_unix_io: the proto unix io key to configure widgets with
        :return: list of widget data for the extra widgets
        """
        extra_widgets = (
            [configure_cost_vis(proto_unix_io)] if cost_visualization else []
        )
        extra_widgets.append(configure_robot_view_fullsystem(proto_unix_io))
        return extra_widgets

    protos = {ProtoUnixIOTypes.SIM: None}
    tabs = []

    if load_blue:
        protos[ProtoUnixIOTypes.BLUE] = None
        protos[ProtoUnixIOTypes.CURRENT] = ProtoUnixIOTypes.BLUE
        tabs.append(
            TScopeQTTab(
                name="Blue Fullsystem",
                key=TabKeys.BLUE,
                widgets=configure_base_fullsystem(
                    ProtoUnixIOTypes.BLUE,
                    False,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=get_extra_widgets(ProtoUnixIOTypes.BLUE),
                ),
            )
        )
    elif load_yellow:
        protos[ProtoUnixIOTypes.YELLOW] = None
        protos[ProtoUnixIOTypes.CURRENT] = ProtoUnixIOTypes.YELLOW
        tabs.append(
            TScopeQTTab(
                name="Yellow Fullsystem",
                key=TabKeys.YELLOW,
                widgets=configure_base_fullsystem(
                    ProtoUnixIOTypes.YELLOW,
                    False,
                    visualization_buffer_size=visualization_buffer_size,
                    extra_widgets=get_extra_widgets(ProtoUnixIOTypes.YELLOW),
                ),
            )
        )

    if load_diagnostics:
        protos[ProtoUnixIOTypes.DIAGNOSTICS] = (
            ProtoUnixIOTypes.BLUE
            if load_blue
            else ProtoUnixIOTypes.YELLOW
            if load_yellow
            else None
        )
        if protos[ProtoUnixIOTypes.DIAGNOSTICS] is None:
            protos[ProtoUnixIOTypes.CURRENT] = ProtoUnixIOTypes.DIAGNOSTICS
        tabs.append(
            TScopeQTTab(
                name="Robot Diagnostics",
                key=TabKeys.DIAGNOSTICS,
                widgets=configure_base_diagnostics(
                    extra_widgets=[]
                    if (load_blue or load_yellow)
                    else [configure_robot_view_diagnostics()]
                ),
            )
        )

    return TScopeConfig(protos=protos, tabs=tabs)
