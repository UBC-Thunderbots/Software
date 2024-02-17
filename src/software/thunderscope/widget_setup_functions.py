import os

from typing import List, Any

from software.py_constants import *
from proto.import_all_protos import *
from software.thunderscope.common.proto_plotter import ProtoPlotter
from software.thunderscope.proto_unix_io import ProtoUnixIO
from proto.robot_log_msg_pb2 import RobotLog
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from software.thunderscope.dock_style import *
from software.thunderscope.proto_unix_io import ProtoUnixIO
from google.protobuf.message import Message

# Import Widgets
from software.thunderscope.gl.gl_widget import GLWidget
from software.thunderscope.gl.layers import (
    gl_obstacle_layer,
    gl_path_layer,
    gl_validation_layer,
    gl_passing_layer,
    gl_sandbox_world_layer,
    gl_world_layer,
    gl_debug_shapes_layer,
    gl_simulator_layer,
    gl_tactic_layer,
    gl_cost_vis_layer,
)

from software.thunderscope.common.proto_configuration_widget import (
    ProtoConfigurationWidget,
)
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.constants import IndividualRobotMode
from software.thunderscope.play.playinfo_widget import PlayInfoWidget
from software.thunderscope.play.refereeinfo_widget import RefereeInfoWidget
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import (
    FullSystemConnectWidget,
)
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget,
)
from software.thunderscope.robot_diagnostics.robot_view import RobotView
from software.thunderscope.robot_diagnostics.robot_error_log import RobotErrorLog
from software.thunderscope.robot_diagnostics.estop_view import EstopView
from software.thunderscope.replay.proto_player import ProtoPlayer

################################
#  FULLSYSTEM RELATED WIDGETS  #
################################


def setup_gl_widget(
    sim_proto_unix_io: ProtoUnixIO,
    full_system_proto_unix_io: ProtoUnixIO,
    friendly_colour_yellow: bool,
    visualization_buffer_size: int,
    sandbox_mode: bool = False,
    replay: bool = False,
    replay_log: os.PathLike = None,
) -> Field:
    """Setup the GLWidget with its constituent layers

    :param sim_proto_unix_io: The proto unix io object for the simulator
    :param full_system_proto_unix_io: The proto unix io object for the full system
    :param friendly_colour_yellow: Whether the friendly colour is yellow
    :param visualization_buffer_size: How many packets to buffer while rendering
    :param sandbox_mode: if sandbox mode should be enabled
    :param replay: Whether replay mode is currently enabled
    :param replay_log: The file path of the replay log
    :returns: The GLWidget

    """
    # Create ProtoPlayer if replay is enabled
    player = ProtoPlayer(replay_log, full_system_proto_unix_io) if replay else None

    # Create widget
    gl_widget = GLWidget(player=player, sandbox_mode=sandbox_mode)

    # Create layers
    validation_layer = gl_validation_layer.GLValidationLayer(
        "Validation", visualization_buffer_size
    )
    path_layer = gl_path_layer.GLPathLayer("Paths", visualization_buffer_size)
    obstacle_layer = gl_obstacle_layer.GLObstacleLayer(
        "Obstacles", visualization_buffer_size
    )
    debug_shapes_layer = gl_debug_shapes_layer.GLDebugShapesLayer(
        "Debug Shapes", visualization_buffer_size
    )
    passing_layer = gl_passing_layer.GLPassingLayer(
        "Passing", visualization_buffer_size
    )
    cost_vis_layer = gl_cost_vis_layer.GLCostVisLayer(
        "Passing Cost", visualization_buffer_size
    )
    world_layer = (
        gl_sandbox_world_layer.GLSandboxWorldLayer(
            "Vision",
            sim_proto_unix_io,
            friendly_colour_yellow,
            visualization_buffer_size,
        )
        if sandbox_mode
        else gl_world_layer.GLWorldLayer(
            "Vision",
            sim_proto_unix_io,
            friendly_colour_yellow,
            visualization_buffer_size,
        )
    )
    simulator_layer = gl_simulator_layer.GLSimulatorLayer(
        "Simulator", friendly_colour_yellow, visualization_buffer_size
    )
    tactic_layer = gl_tactic_layer.GLTacticLayer("Tactics", visualization_buffer_size)

    gl_widget.add_layer(world_layer)
    gl_widget.add_layer(simulator_layer, False)
    gl_widget.add_layer(path_layer)
    gl_widget.add_layer(obstacle_layer)
    gl_widget.add_layer(passing_layer)
    gl_widget.add_layer(cost_vis_layer, False)
    gl_widget.add_layer(tactic_layer, False)
    gl_widget.add_layer(validation_layer)
    gl_widget.add_layer(debug_shapes_layer, False)

    gl_widget.toolbar.pause_button.clicked.connect(world_layer.toggle_play_state)

    # connect all sandbox controls if using sandbox mode
    if sandbox_mode:
        gl_widget.toolbar.undo_button.clicked.connect(world_layer.undo)
        gl_widget.toolbar.redo_button.clicked.connect(world_layer.redo)
        gl_widget.toolbar.reset_button.clicked.connect(world_layer.reset_to_pre_sim)
        world_layer.undo_toggle_enabled_signal.connect(
            gl_widget.toolbar.toggle_undo_enabled
        )
        world_layer.redo_toggle_enabled_signal.connect(
            gl_widget.toolbar.toggle_redo_enabled
        )

    # Register observers
    sim_proto_unix_io.register_observer(
        SimulationState, gl_widget.toolbar.simulation_state_buffer
    )

    for arg in [
        (World, world_layer.world_buffer),
        (World, cost_vis_layer.world_buffer),
        (RobotStatus, world_layer.robot_status_buffer),
        (Referee, world_layer.referee_buffer),
        (ObstacleList, obstacle_layer.obstacles_list_buffer),
        (PrimitiveSet, path_layer.primitive_set_buffer),
        (PathVisualization, path_layer.path_visualization_buffer),
        (PassVisualization, passing_layer.pass_visualization_buffer),
        (World, tactic_layer.world_buffer),
        (PlayInfo, tactic_layer.play_info_buffer),
        (ValidationProtoSet, validation_layer.validation_set_buffer),
        (SimulationState, gl_widget.toolbar.simulation_state_buffer),
        (CostVisualization, cost_vis_layer.cost_visualization_buffer),
        (DebugShapesMap, debug_shapes_layer.debug_shape_map_buffer),
    ]:
        full_system_proto_unix_io.register_observer(*arg)

    return gl_widget


def setup_parameter_widget(
    proto_unix_io: ProtoUnixIO, friendly_colour_yellow: bool
) -> ProtoConfigurationWidget:
    """Setup the parameter widget

    :param proto_unix_io: The proto unix io object
    :param friendly_colour_yellow:
    :returns: The proto configuration widget

    """

    config = ThunderbotsConfig()
    config.sensor_fusion_config.friendly_color_yellow = friendly_colour_yellow

    def on_change_callback(
        attr: Any, value: Any, updated_proto: ThunderbotsConfig
    ) -> None:
        proto_unix_io.send_proto(ThunderbotsConfig, updated_proto)

    return ProtoConfigurationWidget(config, on_change_callback)


def setup_log_widget(proto_unix_io: ProtoUnixIO) -> g3logWidget:
    """Setup the wiget that receives logs from full system

    :param proto_unix_io: The proto unix io object
    :returns: The log widget

    """
    # Create widget
    logs = g3logWidget()

    # Register observer
    proto_unix_io.register_observer(RobotLog, logs.log_buffer)

    return logs


def setup_performance_plot(proto_unix_io: ProtoUnixIO) -> ProtoPlotter:
    """Setup the performance plot

    :param proto_unix_io: The proto unix io object
    :returns: The performance plot widget

    """

    def extract_namedvalue_data(named_value_data):
        return {named_value_data.name: named_value_data.value}

    # Performance Plots plot HZ so the values can't be negative
    proto_plotter = ProtoPlotter(
        min_y=0,
        max_y=100,
        window_secs=15,
        configuration={NamedValue: extract_namedvalue_data},
    )

    # Register observer
    proto_unix_io.register_observer(NamedValue, proto_plotter.buffers[NamedValue])
    return proto_plotter


def setup_play_info(proto_unix_io: ProtoUnixIO) -> PlayInfoWidget:
    """Setup the play info widget

    :param proto_unix_io: The proto unix io object
    :returns: The play info widget

    """

    play_info = PlayInfoWidget()
    proto_unix_io.register_observer(PlayInfo, play_info.playinfo_buffer)
    return play_info


def setup_referee_info(proto_unix_io: ProtoUnixIO) -> RefereeInfoWidget:
    """Setup the referee info widget

    :param proto_unix_io: The proto unix io object
    :returns: The referee info widget

    """

    referee_info = RefereeInfoWidget()
    proto_unix_io.register_observer(Referee, referee_info.referee_buffer)

    return referee_info


#################################
#  DIAGNOSTICS RELATED WIDGETS  #
#################################


def setup_robot_view(
    proto_unix_io: ProtoUnixIO, available_control_modes: List[IndividualRobotMode]
) -> RobotView:
    """Setup the robot view widget
    :param proto_unix_io: The proto unix io object for the full system
    :param available_control_modes: the currently available input modes for the robots
                                    according to what mode thunderscope is run in

    :returns: the robot view widget
    """
    robot_view = RobotView(available_control_modes)
    proto_unix_io.register_observer(RobotStatus, robot_view.robot_status_buffer)
    return robot_view


def setup_robot_error_log_view_widget(proto_unix_io: ProtoUnixIO) -> RobotErrorLog:
    """
    Setup the robot error log widget and connect its buffer to the proto unix io
    :param proto_unix_io: The proto unix io object for the full system
    :return: the robot error log widget
    """
    robot_error_log = RobotErrorLog()
    proto_unix_io.register_observer(RobotStatus, robot_error_log.robot_status_buffer)
    proto_unix_io.register_observer(RobotCrash, robot_error_log.robot_crash_buffer)
    proto_unix_io.register_observer(RobotLog, robot_error_log.robot_log_buffer)
    return robot_error_log


def setup_estop_view(proto_unix_io) -> EstopView:
    """Setup the estop view widget

    :param proto_unix_io: The proto unix io object for the full system
    :returns: the estop widget
    """
    estop_view = EstopView()

    proto_unix_io.register_observer(EstopState, estop_view.estop_state_buffer)
    return estop_view


def setup_chicker_widget(proto_unix_io: ProtoUnixIO) -> ChickerWidget:
    """Setup the chicker widget for robot diagnostics

    :param proto_unix_io: The proto unix io object
    :returns: The chicker widget

    """
    chicker_widget = ChickerWidget(proto_unix_io)
    return chicker_widget


def setup_diagnostics_input_widget() -> FullSystemConnectWidget:
    """
    Sets up the diagnostics input widget

    :returns: the diagnostics input widget
    """

    diagnostics_input_widget = FullSystemConnectWidget()

    return diagnostics_input_widget


def setup_drive_and_dribbler_widget(
    proto_unix_io: ProtoUnixIO,
) -> DriveAndDribblerWidget:
    """Setup the drive and dribbler widget

    :param proto_unix_io: The proto unix io object
    :returns: The drive and dribbler widget

    """
    drive_and_dribbler_widget = DriveAndDribblerWidget(proto_unix_io)

    return drive_and_dribbler_widget
