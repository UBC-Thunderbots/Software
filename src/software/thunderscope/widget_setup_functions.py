from typing import List

from software.py_constants import *
from proto.import_all_protos import *
from software.thunderscope.common.proto_plotter import ProtoPlotter
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from software.thunderscope.dock_label_style import *


# Import Widgets
from software.thunderscope.gl.gl_widget import GLWidget
from software.thunderscope.gl import (
    gl_obstacle_layer,
    gl_path_layer,
    gl_validation_layer,
    gl_passing_layer,
    gl_world_layer,
    gl_simulator_layer,
    gl_hrvo_layer,
    gl_tactic_layer,
)

from software.thunderscope.field import (
    obstacle_layer,
    path_layer,
    validation_layer,
    simulator_layer,
    world_layer,
    passing_layer,
    hrvo_layer,
)

from software.thunderscope.common.proto_configuration_widget import (
    ProtoConfigurationWidget,
)
from software.thunderscope.cost_vis.cost_vis import CostVisualizationWidget
from software.thunderscope.field.field import Field
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
from software.thunderscope.robot_diagnostics.estop_view import EstopView
from software.thunderscope.replay.proto_player import ProtoPlayer

################################
#  FULLSYSTEM RELATED WIDGETS  #
################################


def setup_gl_widget(
    sim_proto_unix_io,
    full_system_proto_unix_io,
    friendly_colour_yellow,
    visualization_buffer_size,
    replay=False,
    replay_log=None,
):
    """Setup the GLWidget with its constituent layers

    :param sim_proto_unix_io: The proto unix io object for the simulator
    :param full_system_proto_unix_io: The proto unix io object for the full system
    :param friendly_colour_yellow: Whether the friendly colour is yellow
    :param visualization_buffer_size: How many packets to buffer while rendering
    :param replay: Whether replay mode is currently enabled
    :param replay_log: The file path of the replay log
    :returns: The GLWidget

    """
    # Create ProtoPlayer if replay is enabled
    player = ProtoPlayer(replay_log, full_system_proto_unix_io) if replay else None

    # Create widget
    gl_widget = GLWidget(player=player)

    # Create layers
    validation_layer = gl_validation_layer.GLValidationLayer(visualization_buffer_size)
    path_layer = gl_path_layer.GLPathLayer(visualization_buffer_size)
    obstacle_layer = gl_obstacle_layer.GLObstacleLayer(visualization_buffer_size)
    passing_layer = gl_passing_layer.GLPassingLayer(visualization_buffer_size)
    world_layer = gl_world_layer.GLWorldLayer(
        sim_proto_unix_io, friendly_colour_yellow, visualization_buffer_size
    )
    simulator_layer = gl_simulator_layer.GLSimulatorLayer(
        friendly_colour_yellow, visualization_buffer_size
    )
    tactic_layer = gl_tactic_layer.GLTacticLayer(visualization_buffer_size)

    gl_widget.add_layer("Validation", validation_layer)
    gl_widget.add_layer("Paths", path_layer)
    gl_widget.add_layer("Obstacles", obstacle_layer)
    gl_widget.add_layer("Passing", passing_layer)
    gl_widget.add_layer("Vision", world_layer)
    gl_widget.add_layer("Simulator", simulator_layer, False)
    gl_widget.add_layer("Tactics", tactic_layer, False)

    # Add HRVO layers to field widget and have them hidden on startup
    # TODO (#2655): Add/Remove HRVO layers dynamically based on the HRVOVisualization proto messages
    hrvo_sim_states = []
    for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
        hrvo_sim_state = gl_hrvo_layer.GLHrvoLayer(robot_id, visualization_buffer_size)
        hrvo_sim_states.append(hrvo_sim_state)
        gl_widget.add_layer(f"HRVO {robot_id}", hrvo_sim_state, False)

    # Register observers
    for arg in [
        (World, world_layer.world_buffer),
        (RobotStatus, world_layer.robot_status_buffer),
        (Referee, world_layer.referee_buffer),
        (PrimitiveSet, obstacle_layer.primitive_set_buffer),
        (PrimitiveSet, path_layer.primitive_set_buffer),
        (PassVisualization, passing_layer.pass_visualization_buffer),
        (World, tactic_layer.world_buffer),
        (PlayInfo, tactic_layer.play_info_buffer),
        (ValidationProtoSet, validation_layer.validation_set_buffer),
        (SimulatorState, simulator_layer.simulator_state_buffer),
    ] + [
        (HRVOVisualization, hrvo_sim_state.hrvo_buffer)
        for hrvo_sim_state in hrvo_sim_states
    ]:
        full_system_proto_unix_io.register_observer(*arg)

    return gl_widget


def setup_field_widget(
    sim_proto_unix_io,
    full_system_proto_unix_io,
    friendly_colour_yellow,
    visualization_buffer_size,
    replay=False,
    replay_log=None,
):
    """setup the field widget with the constituent layers

    :param sim_proto_unix_io: The proto unix io object for the simulator
    :param full_system_proto_unix_io: The proto unix io object for the full system
    :param friendly_colour_yellow: Whether the friendly colour is yellow
    :param replay: whether replay mode is currently enabled
    :param replay_log: the file path of the replay log
    :param visualization_buffer_size: How many packets to buffer while rendering
    :returns: the field widget

    """
    player = None

    if replay:
        player = ProtoPlayer(replay_log, full_system_proto_unix_io,)
    field = Field(player=player)

    # Create layers
    paths = path_layer.PathLayer(visualization_buffer_size)
    obstacles = obstacle_layer.ObstacleLayer(visualization_buffer_size)
    validation = validation_layer.ValidationLayer(visualization_buffer_size)
    world = world_layer.WorldLayer(
        sim_proto_unix_io, friendly_colour_yellow, visualization_buffer_size
    )
    sim_state = simulator_layer.SimulatorLayer(
        friendly_colour_yellow, visualization_buffer_size
    )
    passing = passing_layer.PassingLayer(visualization_buffer_size)

    # Add field layers to field
    field.add_layer("Vision", world)
    field.add_layer("Obstacles", obstacles)
    field.add_layer("Paths", paths)
    field.add_layer("Validation", validation)
    field.add_layer("Passing", passing)
    field.add_layer("Simulator", sim_state)
    hrvo_sim_states = []
    # Add HRVO layers to field widget and have them hidden on startup
    # TODO (#2655): Add/Remove HRVO layers dynamically based on the HRVOVisualization proto messages
    for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
        hrvo_sim_state = hrvo_layer.HRVOLayer(robot_id, visualization_buffer_size)
        hrvo_sim_states.append(hrvo_sim_state)
        field.add_layer(f"HRVO {robot_id}", hrvo_sim_state, False)

    # Register observers
    sim_proto_unix_io.register_observer(
        SimulatorState, sim_state.simulator_state_buffer
    )

    for arg in [
        (World, world.world_buffer),
        (RobotStatus, world.robot_status_buffer),
        (Referee, world.referee_buffer),
        (PrimitiveSet, obstacles.primitive_set_buffer),
        (PrimitiveSet, paths.primitive_set_buffer),
        (PassVisualization, passing.pass_visualization_buffer),
        (ValidationProtoSet, validation.validation_set_buffer),
        (SimulatorState, sim_state.simulator_state_buffer),
    ] + [
        (HRVOVisualization, hrvo_sim_state.hrvo_buffer)
        for hrvo_sim_state in hrvo_sim_states
    ]:
        full_system_proto_unix_io.register_observer(*arg)

    return field


def setup_parameter_widget(proto_unix_io, friendly_colour_yellow):
    """Setup the parameter widget

    :param proto_unix_io: The proto unix io object
    :param friendly_colour_yellow:
    :returns: The proto configuration widget

    """

    config = ThunderbotsConfig()
    config.sensor_fusion_config.friendly_color_yellow = friendly_colour_yellow

    def on_change_callback(attr, value, updated_proto):
        proto_unix_io.send_proto(ThunderbotsConfig, updated_proto)

    return ProtoConfigurationWidget(config, on_change_callback)


def setup_log_widget(proto_unix_io):
    """Setup the wiget that receives logs from full system

    :param proto_unix_io: The proto unix io object
    :returns: The log widget

    """
    # Create widget
    logs = g3logWidget()

    # Register observer
    proto_unix_io.register_observer(RobotLog, logs.log_buffer)

    return logs


def setup_performance_plot(proto_unix_io):
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


def setup_play_info(proto_unix_io):
    """Setup the play info widget

    :param proto_unix_io: The proto unix io object
    :returns: The play info widget

    """

    play_info = PlayInfoWidget()
    proto_unix_io.register_observer(PlayInfo, play_info.playinfo_buffer)
    return play_info


def setup_referee_info(proto_unix_io):
    """Setup the referee info widget

    :param proto_unix_io: The proto unix io object
    :returns: The referee info widget

    """

    referee_info = RefereeInfoWidget()
    proto_unix_io.register_observer(Referee, referee_info.referee_buffer)

    return referee_info


def setup_cost_visualization_widget(proto_unix_io):
    """Setup the cost visualization widget

    :param proto_unix_io: The proto unix io object
    :returns: The cost visualization widget

    """
    cost_vis_widget = CostVisualizationWidget()
    proto_unix_io.register_observer(
        CostVisualization, cost_vis_widget.cost_visualization_buffer
    )
    return cost_vis_widget


#################################
#  DIAGNOSTICS RELATED WIDGETS  #
#################################


def setup_robot_view(proto_unix_io, available_control_modes: List[IndividualRobotMode]):
    """Setup the robot view widget
    :param proto_unix_io: The proto unix io object for the full system
    :param available_control_modes: the currently available input modes for the robots
                                    according to what mode thunderscope is run in

    :returns: the robot view widget
    """
    robot_view = RobotView(available_control_modes)
    proto_unix_io.register_observer(RobotStatus, robot_view.robot_status_buffer)
    return robot_view


def setup_estop_view(proto_unix_io):
    """Setup the estop view widget

    :param proto_unix_io: The proto unix io object for the full system
    :returns: the estop widget
    """
    estop_view = EstopView()

    proto_unix_io.register_observer(EstopState, estop_view.estop_state_buffer)
    return estop_view


def setup_chicker_widget(proto_unix_io):
    """Setup the chicker widget for robot diagnostics

    :param proto_unix_io: The proto unix io object
    :returns: The chicker widget

    """
    chicker_widget = ChickerWidget(proto_unix_io)
    return chicker_widget


def setup_diagnostics_input_widget():
    """
    Sets up the diagnostics input widget

    :returns: the diagnostics input widget
    """

    diagnostics_input_widget = FullSystemConnectWidget()

    return diagnostics_input_widget


def setup_drive_and_dribbler_widget(proto_unix_io):
    """Setup the drive and dribbler widget

    :param proto_unix_io: The proto unix io object
    :returns: The drive and dribbler widget

    """
    drive_and_dribbler_widget = DriveAndDribblerWidget(proto_unix_io)

    return drive_and_dribbler_widget
