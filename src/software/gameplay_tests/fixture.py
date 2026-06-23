import contextlib
import time
from dataclasses import dataclass
from typing import Any, Optional

import pytest
from software.py_constants import (
    MAX_ROBOT_IDS_PER_SIDE,
    SSL_REFEREE_PORT,
    getRobotMulticastChannel,
)

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

# The test-mode session bound to the currently running test
ACTIVE_SESSION = None


@dataclass
class SessionContext:
    """The live binaries and ProtoUnixIOs of a running gameplay-test session.

    Yielded by the session context managers and reused both by the per-test
    fixtures and by the long-lived test-mode launcher.
    """

    blue_full_system_proto_unix_io: ProtoUnixIO
    yellow_full_system_proto_unix_io: ProtoUnixIO
    simulator_proto_unix_io: ProtoUnixIO
    gamecontroller: Gamecontroller
    robot_communication: Optional[RobotCommunication] = None
    estop_mode: Optional[EstopMode] = None
    simulator: Optional[Simulator] = None
    blue_full_system: Optional[FullSystem] = None
    yellow_full_system: Optional[FullSystem] = None


@pytest.fixture
def gameplay_test_runner():
    """Dispatches to the field or simulated test runner based on --run_field_test.

    When Thunderscope is launched with --test_mode, an active session exists and
    the selected test is bound to the already-running binaries instead of
    launching new ones.

    :yields: A SimulatedTestRunner or FieldTestRunner.
    """
    if ACTIVE_SESSION is not None:
        yield from bound_runner(ACTIVE_SESSION)
        return

    args = load_command_line_arguments()
    if args.run_field_test:
        yield from field_test_runner(args)
    else:
        yield from simulated_test_runner(args)


def bound_runner(session):
    """Yields a test runner bound to a test-mode session's live binaries.

    The runner does not own the Thunderscope (it must outlive the test), and its
    observers are deregistered after the test so repeated runs stay hermetic.

    :param session: the active test-mode session (see test_mode.TestModeSession).
    :yields: a SimulatedTestRunner or FieldTestRunner with owns_thunderscope=False.
    """
    context = session.context
    test_name = get_pytest_name()

    if session.run_field_test:
        runner = FieldTestRunner(
            test_name=test_name,
            blue_full_system_proto_unix_io=context.blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=context.yellow_full_system_proto_unix_io,
            gamecontroller=context.gamecontroller,
            thunderscope=session.thunderscope,
            robot_communication=context.robot_communication,
            is_yellow_friendly=session.is_yellow_friendly,
            owns_thunderscope=False,
        )
    else:
        # Relaunch the simulator and full systems so each test starts from clean state.
        # The ProtoUnixIOs reconnect automatically.
        context.simulator.restart()
        context.blue_full_system.restart()
        context.yellow_full_system.restart()
        time.sleep(LAUNCH_DELAY_S)

        runner = SimulatedTestRunner(
            test_name,
            session.thunderscope,
            context.simulator_proto_unix_io,
            context.blue_full_system_proto_unix_io,
            context.yellow_full_system_proto_unix_io,
            context.gamecontroller,
            ci_mode=session.ci_mode,
            owns_thunderscope=False,
        )

    runner.cancel_event = session.cancel_event

    try:
        yield runner
    finally:
        runner.cleanup()


@contextlib.contextmanager
def simulated_session(args, runtime_subpath):
    """Launches the simulator, blue/yellow full systems, and gamecontroller.

    :param args: Parsed command-line arguments.
    :param runtime_subpath: Subdirectory (under each runtime dir) for this session.
    :yields: A SessionContext with the live ProtoUnixIOs and gamecontroller.
    """
    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()

    with (
        Simulator(
            f"{args.simulator_runtime_dir}/{runtime_subpath}",
            args.debug_simulator,
            args.enable_realism,
        ) as simulator,
        FullSystem(
            "software/unix_full_system",
            f"{args.blue_full_system_runtime_dir}/{runtime_subpath}",
            args.debug_blue_full_system,
            False,
            should_restart_on_crash=False,
            running_in_realtime=args.enable_thunderscope and not args.ci_mode,
        ) as blue_fs,
        FullSystem(
            "software/unix_full_system",
            f"{args.yellow_full_system_runtime_dir}/{runtime_subpath}",
            args.debug_yellow_full_system,
            True,
            should_restart_on_crash=False,
            running_in_realtime=args.enable_thunderscope and not args.ci_mode,
        ) as yellow_fs,
        Gamecontroller(suppress_logs=(not args.verbose)) as gamecontroller,
    ):
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
            simulator_proto_unix_io=None,  # None so that gamecontroller doesn't set robot limit
        )

        time.sleep(LAUNCH_DELAY_S)

        yield SessionContext(
            blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
            simulator_proto_unix_io=simulator_proto_unix_io,
            gamecontroller=gamecontroller,
            simulator=simulator,
            blue_full_system=blue_fs,
            yellow_full_system=yellow_fs,
        )


@contextlib.contextmanager
def field_session(args, runtime_subpath):
    """Launches the friendly full system, gamecontroller, WiFi, and robot comms.

    :param args: Parsed command-line arguments.
    :param runtime_subpath: Subdirectory (under each runtime dir) for this session.
    :yields: A SessionContext with the live ProtoUnixIOs, gamecontroller,
        robot communication, and estop mode.
    """
    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()

    debug_full_sys = args.debug_blue_full_system
    runtime_dir = f"{args.blue_full_system_runtime_dir}/{runtime_subpath}"
    friendly_proto_unix_io = blue_full_system_proto_unix_io

    if args.run_yellow:
        debug_full_sys = args.debug_yellow_full_system
        runtime_dir = f"{args.yellow_full_system_runtime_dir}/{runtime_subpath}"
        friendly_proto_unix_io = yellow_full_system_proto_unix_io

    estop_mode, estop_path = get_estop_config(
        args.keyboard_estop, args.disable_communication
    )

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
            suppress_logs=(not args.verbose),
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

        # Set control mode for all robots to AI so that packets are sent to the robots
        for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
            rc_friendly.toggle_individual_robot_control_mode(
                robot_id,
                IndividualRobotMode.AI,
            )

        time.sleep(LAUNCH_DELAY_S)

        yield SessionContext(
            blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
            simulator_proto_unix_io=simulator_proto_unix_io,
            gamecontroller=gamecontroller,
            robot_communication=rc_friendly,
            estop_mode=estop_mode,
        )


def wire_field_keyboard_estop(thunderscope, robot_communication, estop_mode):
    """Connects Thunderscope's keyboard estop shortcut to robot communication.

    :param thunderscope: The Thunderscope instance.
    :param robot_communication: The RobotCommunication instance.
    :param estop_mode: The EstopMode in use.
    """
    if estop_mode == EstopMode.KEYBOARD_ESTOP:
        thunderscope.keyboard_estop_shortcut.activated.connect(
            robot_communication.toggle_keyboard_estop
        )
        # we call this method to enable estop automatically when a field test starts
        robot_communication.toggle_keyboard_estop()
        logger.warning(
            "\x1b[31;20m"
            + "Keyboard Estop Enabled, robots will start moving automatically when test starts!"
            + "\x1b[0m"
        )


def build_simulated_test_thunderscope(session, layout_path, extra_widgets=[]):
    """Builds a Thunderscope with the simulated-test view bound to a session.

    :param session: the SessionContext whose ProtoUnixIOs to visualize.
    :param layout_path: the Thunderscope layout to load, or None.
    :param extra_widgets: additional widget data to add to the view.
    :return: the Thunderscope instance.
    """
    return Thunderscope(
        configure_simulated_test_view(
            blue_full_system_proto_unix_io=session.blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=session.yellow_full_system_proto_unix_io,
            simulator_proto_unix_io=session.simulator_proto_unix_io,
            extra_widgets=extra_widgets,
        ),
        layout_path=layout_path,
    )


def build_field_test_thunderscope(
    session, yellow_is_friendly, layout_path, extra_widgets=[]
):
    """Builds a Thunderscope with the field-test view bound to a session.

    :param session: the SessionContext whose ProtoUnixIOs to visualize.
    :param yellow_is_friendly: whether the friendly team is yellow.
    :param layout_path: the Thunderscope layout to load, or None.
    :param extra_widgets: additional widget data to add to the view.
    :return: the Thunderscope instance.
    """
    thunderscope = Thunderscope(
        configure_field_test_view(
            simulator_proto_unix_io=session.simulator_proto_unix_io,
            blue_full_system_proto_unix_io=session.blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=session.yellow_full_system_proto_unix_io,
            yellow_is_friendly=yellow_is_friendly,
            extra_widgets=extra_widgets,
        ),
        layout_path=layout_path,
    )
    wire_field_keyboard_estop(
        thunderscope, session.robot_communication, session.estop_mode
    )
    return thunderscope


def simulated_test_runner(args):
    """Starts the simulated-test binaries and yields a SimulatedTestRunner.

    :param args: Parsed command-line arguments.
    :yields: A SimulatedTestRunner.
    """
    test_name = get_pytest_name()
    test_path_name = get_pytest_path_name()

    with simulated_session(args, f"test/{test_path_name}") as session:
        tscope = None
        if args.enable_thunderscope:
            tscope = build_simulated_test_thunderscope(session, args.layout)

        runner = SimulatedTestRunner(
            test_name,
            tscope,
            session.simulator_proto_unix_io,
            session.blue_full_system_proto_unix_io,
            session.yellow_full_system_proto_unix_io,
            session.gamecontroller,
            args.ci_mode,
        )

        yield runner


def field_test_runner(args):
    """Starts the field-test binaries and yields a FieldTestRunner.

    :param args: Parsed command-line arguments.
    :yields: A FieldTestRunner.
    """
    test_name = get_pytest_name()
    test_path_name = get_pytest_path_name()

    with field_session(args, f"test/{test_path_name}") as session:
        tscope = build_field_test_thunderscope(
            session, args.run_yellow, layout_path=None
        )

        runner = FieldTestRunner(
            test_name=test_name,
            blue_full_system_proto_unix_io=session.blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=session.yellow_full_system_proto_unix_io,
            gamecontroller=session.gamecontroller,
            thunderscope=tscope,
            robot_communication=session.robot_communication,
            is_yellow_friendly=args.run_yellow,
        )

        yield runner
