import time

import pytest
from software.py_constants import (
    MAX_ROBOT_IDS_PER_SIDE,
    SSL_REFEREE_PORT,
    getRobotMulticastChannel,
)

from proto.import_all_protos import World
from software.gameplay_tests.field_test_runner import FieldTestRunner
from software.gameplay_tests.simulated_test_runner import SimulatedTestRunner
from software.gameplay_tests.util import (
    load_command_line_arguments,
    get_pytest_name,
    get_pytest_path_name,
)
from software.logger.logger import create_logger
from software.thunderscope.binary_context_managers.full_system import FullSystem
from software.thunderscope.binary_context_managers.game_controller import Gamecontroller
from software.thunderscope.binary_context_managers.simulator import Simulator
from software.thunderscope.constants import EstopMode, IndividualRobotMode
from software.thunderscope.estop_helpers import get_estop_config
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_communication import RobotCommunication
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.thunderscope_config import (
    configure_field_test_view,
    configure_simulated_test_view,
)
from software.thunderscope.wifi_communication_manager import WifiCommunicationManager

logger = create_logger(__name__)

LAUNCH_DELAY_S = 0.1


@pytest.fixture
def gameplay_test_runner():
    """Dispatches to the field or simulated test runner based on --run_field_test.

    :yields: A SimulatedTestRunner or FieldTestRunner.
    """
    args = load_command_line_arguments()
    if args.run_field_test:
        yield from field_test_runner(args)
    else:
        yield from simulated_test_runner(args)


def simulated_test_runner(args):
    """Starts the simulator, blue/yellow full systems, and gamecontroller binaries.

    :param args: Parsed command-line arguments.
    :yields: A SimulatedTestRunner.
    """
    tscope = None

    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()

    # Grab the current test name to store the proto log for the test case
    test_name = get_pytest_name()
    test_path_name = get_pytest_path_name()

    # Launch all binaries
    with (
        Simulator(
            f"{args.simulator_runtime_dir}/test/{test_path_name}",
            args.debug_simulator,
            args.enable_realism,
        ) as simulator,
        FullSystem(
            "software/unix_full_system",
            f"{args.blue_full_system_runtime_dir}/test/{test_path_name}",
            args.debug_blue_full_system,
            False,
            should_restart_on_crash=False,
            running_in_realtime=args.enable_thunderscope and not args.ci_mode,
        ) as blue_fs,
        FullSystem(
            "software/unix_full_system",
            f"{args.yellow_full_system_runtime_dir}/test/{test_path_name}",
            args.debug_yellow_full_system,
            True,
            should_restart_on_crash=False,
            running_in_realtime=args.enable_thunderscope and not args.ci_mode,
        ) as yellow_fs,
    ):
        with Gamecontroller(
            suppress_logs=(not args.show_gamecontroller_logs)
        ) as gamecontroller:
            blue_fs.setup_proto_unix_io(blue_full_system_proto_unix_io)
            yellow_fs.setup_proto_unix_io(yellow_full_system_proto_unix_io)
            simulator.setup_proto_unix_io(
                simulator_proto_unix_io,
                blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io,
                ProtoUnixIO(),
            )
            gamecontroller.setup_proto_unix_io(
                blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
                simulator_proto_unix_io=simulator_proto_unix_io,
            )

            # If we want to run thunderscope, inject the proto unix ios
            # and start the test
            if args.enable_thunderscope:
                tscope = Thunderscope(
                    configure_simulated_test_view(
                        blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                        yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
                        simulator_proto_unix_io=simulator_proto_unix_io,
                    ),
                    layout_path=args.layout,
                )

            time.sleep(LAUNCH_DELAY_S)

            runner = SimulatedTestRunner(
                test_name,
                tscope,
                simulator_proto_unix_io,
                blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io,
                gamecontroller,
                args.ci_mode,
            )

            yield runner


def field_test_runner(args):
    """Starts the friendly full system, gamecontroller, WiFi, and robot communication.

    :param args: Parsed command-line arguments.
    :yields: A FieldTestRunner.
    """
    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()

    # Grab the current test name to store the proto log for the test case
    test_name = get_pytest_name()
    test_path_name = get_pytest_path_name()

    debug_full_sys = args.debug_blue_full_system
    runtime_dir = f"{args.blue_full_system_runtime_dir}/test/{test_path_name}"
    friendly_proto_unix_io = blue_full_system_proto_unix_io

    if args.run_yellow:
        debug_full_sys = args.debug_yellow_full_system
        runtime_dir = f"{args.yellow_full_system_runtime_dir}/test/{test_path_name}"
        friendly_proto_unix_io = yellow_full_system_proto_unix_io

    estop_mode, estop_path = get_estop_config(
        args.keyboard_estop, args.disable_communication
    )

    # Launch all binaries
    with (
        FullSystem(
            "software/unix_full_system",
            full_system_runtime_dir=runtime_dir,
            debug_full_system=debug_full_sys,
            friendly_colour_yellow=args.run_yellow,
            should_restart_on_crash=False,
        ) as friendly_fs,
        Gamecontroller(
            # we would be using conventional port if and only if we are playing in robocup.
            suppress_logs=(not args.show_gamecontroller_logs),
            use_conventional_port=False,
        ) as gamecontroller,
        WifiCommunicationManager(
            current_proto_unix_io=friendly_proto_unix_io,
            multicast_channel=getRobotMulticastChannel(args.channel),
            should_setup_full_system=True,
            interface=args.interface,
            referee_port=gamecontroller.get_referee_port()
            if gamecontroller
            else SSL_REFEREE_PORT,
        ) as wifi_communication_manager,
        RobotCommunication(
            current_proto_unix_io=friendly_proto_unix_io,
            communication_manager=wifi_communication_manager,
            estop_mode=estop_mode,
            estop_path=estop_path,
        ) as rc_friendly,
    ):
        friendly_fs.setup_proto_unix_io(friendly_proto_unix_io)

        gamecontroller.setup_proto_unix_io(
            blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
            simulator_proto_unix_io=simulator_proto_unix_io,
        )
        # Inject the proto unix ios into thunderscope and start the test
        tscope = Thunderscope(
            configure_field_test_view(
                simulator_proto_unix_io=simulator_proto_unix_io,
                blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
                yellow_is_friendly=args.run_yellow,
            ),
            layout_path=None,
        )

        # Set control mode for all robots to AI so that packets are sent to the robots
        for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
            rc_friendly.toggle_individual_robot_control_mode(
                robot_id,
                IndividualRobotMode.AI,
            )

        # connect the keyboard estop toggle to the key event if needed
        if estop_mode == EstopMode.KEYBOARD_ESTOP:
            tscope.keyboard_estop_shortcut.activated.connect(
                rc_friendly.toggle_keyboard_estop
            )
            # we call this method to enable estop automatically when a field test starts
            rc_friendly.toggle_keyboard_estop()
            logger.warning(
                "\x1b[31;20m"
                + "Keyboard Estop Enabled, robots will start moving automatically when test starts!"
                + "\x1b[0m"
            )

        time.sleep(LAUNCH_DELAY_S)

        runner = FieldTestRunner(
            test_name=test_name,
            blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
            gamecontroller=gamecontroller,
            thunderscope=tscope,
            robot_communication=rc_friendly,
            is_yellow_friendly=args.run_yellow,
        )

        yield runner
