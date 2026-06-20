import queue
import time
import os
import glob
import threading

import pytest
import argparse
from proto.import_all_protos import *

from software.gameplay_tests.validation import validation
from software.thunderscope.constants import EstopMode, IndividualRobotMode
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.binary_context_managers.full_system import FullSystem
from software.thunderscope.binary_context_managers.game_controller import Gamecontroller
from software.thunderscope.wifi_communication_manager import WifiCommunicationManager
from software.logger.logger import create_logger


from software.thunderscope.thunderscope_config import configure_field_test_view
from software.gameplay_tests.tbots_test_runner import TbotsTestRunner
from software.thunderscope.robot_communication import RobotCommunication
from software.thunderscope.estop_helpers import get_estop_config
from software.py_constants import *
from typing import override

logger = create_logger(__name__)

WORLD_BUFFER_TIMEOUT = 5.0
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3
LAUNCH_DELAY_S = 0.1


class FieldTestRunner(TbotsTestRunner):
    """Run a field test"""

    def __init__(
        self,
        test_name,
        thunderscope,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
        robot_communication,
        publish_validation_protos=True,
        is_yellow_friendly=False,
    ):
        """Initialize the FieldTestRunner

        :param test_name: The name of the test to run
        :param thunderscope: The Thunderscope to use, None if not used
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance
        :param robot_communication: The robot communication instance
        :param publish_validation_protos: whether to publish validation protos
        :param: is_yellow_friendly: if yellow is the friendly team
        """
        super(FieldTestRunner, self).__init__(
            test_name,
            thunderscope,
            blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io,
            gamecontroller,
            is_yellow_friendly,
        )
        self.publish_validation_protos = publish_validation_protos
        self.is_yellow_friendly = is_yellow_friendly
        self.robot_communication = robot_communication

        logger.info("determining robots on field")
        # survey field for available robot ids
        survey_start_time = time.time()
        self.friendly_robot_ids_field = []
        while time.time() - survey_start_time < WORLD_BUFFER_TIMEOUT:
            try:
                world = self.world_buffer.get(block=True, timeout=0.1)
                self.initial_world = world
                self.friendly_robot_ids_field = [
                    robot.id for robot in world.friendly_team.team_robots
                ]

                if len(self.friendly_robot_ids_field) > 0:
                    logger.info(f"friendly team ids {self.friendly_robot_ids_field}")
                    break
            except queue.Empty:
                continue

        if len(self.friendly_robot_ids_field) == 0:
            raise Exception("no friendly robots found on field within timeout")

    def wait_for_estop_play(self):
        """Blocks until the estop is in the PLAY state"""
        if self.robot_communication.estop_is_playing:
            return

        logger.info("\x1b[33m" + "Waiting for Estop to be in PLAY state..." + "\x1b[0m")
        while not self.robot_communication.estop_is_playing:
            # We must process events if Thunderscope is running to keep it responsive
            if self.thunderscope:
                from pyqtgraph.Qt import QtWidgets

                QtWidgets.QApplication.processEvents()
            time.sleep(0.1)
        logger.info(
            "\x1b[32m" + "Estop is in PLAY state. Proceeding with test." + "\x1b[0m"
        )

    @override
    def send_gamecontroller_command(
        self,
        gc_command: proto.ssl_gc_state_pb2.Command,
        team: proto.ssl_gc_common_pb2.Team,
        final_ball_placement_point=None,
    ):
        """Send a command to the gamecontroller

        :param gc_command: The command to send
        :param team: The team which the command as attributed to
        :param final_ball_placement_point: The ball placement point
        """
        self.gamecontroller.send_gc_command(
            gc_command=gc_command,
            team=team,
            final_ball_placement_point=final_ball_placement_point,
        )

    @override
    def run_test(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=3,
    ):
        """Run a test. In a field test this means beginning validation.

        :param always_validation_sequence_set: Validation functions that should
                                hold on every tick
        :param eventually_validation_sequence_set: Validation that should
                                eventually be true, before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
        """

        def stop_test(delay):
            time.sleep(delay)
            # We no longer close thunderscope here, because a test might call run_test multiple times.
            # Thunderscope will be closed when the fixture is torn down.

        def __runner():
            time.sleep(LAUNCH_DELAY_S)

            test_end_time = time.time() + test_timeout_s

            # Keep track if we started with any eventually validations
            has_eventually_validations = any(
                len(seq) > 0 for seq in eventually_validation_sequence_set
            )

            while time.time() < test_end_time:
                while True:
                    try:
                        world = self.world_buffer.get(
                            block=True, timeout=WORLD_BUFFER_TIMEOUT
                        )
                        break
                    except queue.Empty:
                        # If we timeout, that means full_system missed the last
                        # wrapper and robot status, lets resend it.
                        logger.warning(
                            f"No World was received for {WORLD_BUFFER_TIMEOUT} seconds. Ending test early."
                        )

                # Validate
                (
                    eventually_validation_proto_set,
                    always_validation_proto_set,
                ) = validation.run_validation_sequence_sets(
                    world,
                    eventually_validation_sequence_set,
                    always_validation_sequence_set,
                )

                if self.publish_validation_protos:
                    # Set the test name
                    eventually_validation_proto_set.test_name = self.test_name
                    always_validation_proto_set.test_name = self.test_name

                    # Send out the validation proto to thunderscope
                    self.blue_full_system_proto_unix_io.send_proto(
                        ValidationProtoSet, eventually_validation_proto_set
                    )
                    self.blue_full_system_proto_unix_io.send_proto(
                        ValidationProtoSet, always_validation_proto_set
                    )

                # Check that all always validations are always valid
                validation.check_validation(always_validation_proto_set)

                # Break if eventually validation passes
                if has_eventually_validations and all(
                    len(seq) == 0 for seq in eventually_validation_sequence_set
                ):
                    break

            validation.check_validation(eventually_validation_proto_set)

            stop_test(delay=PROCESS_BUFFER_DELAY_S)

        def excepthook(args):
            """This function is _critical_ for show_thunderscope to work.
            If the test Thread will raises an exception we won't be able to close
            the window from the main thread.

            :param args: The args passed in from the hook
            """
            stop_test(delay=PAUSE_AFTER_FAIL_DELAY_S)
            self.last_exception = args.exc_value
            raise self.last_exception

        threading.excepthook = excepthook

        # If visualization is enabled, we need to be careful.
        # Thunderscope.show() is blocking.
        if self.thunderscope:
            run_test_thread = threading.Thread(target=__runner, daemon=True)
            run_test_thread.start()

            # Only call show if the window is not already open.
            # If it IS open, it means we are ALREADY in the Qt event loop,
            # which can only happen if we are running this run_test in a background thread.
            if not self.thunderscope.is_open():
                self.thunderscope.show()

            run_test_thread.join()

            if self.last_exception:
                pytest.fail(str(self.last_exception))

        else:
            __runner()


def get_runtime_dir():
    """Gets the base runtime directory for the test execution.

    TODO: Refactor #3744

    Creates a new persistent directory for each test so that tests
    running in parallel do not interfere with each other.

    :return: The path to the runtime directory.
    """
    import uuid

    runtime_dir = os.path.join("/tmp", f"tbots_{uuid.uuid4().hex[:8]}")
    os.makedirs(runtime_dir, exist_ok=True)
    return runtime_dir


RUNTIME_DIR = get_runtime_dir()


def load_command_line_arguments():
    """Load in command-line arguments using argparse

    NOTE: Pytest has its own built in argument parser (conftest.py, pytest_addoption)
    but it doesn't seem to play nicely with bazel. We just use argparse instead.
    """
    parser = argparse.ArgumentParser(description="Run simulated or field pytests")
    parser.add_argument(
        "--simulator_runtime_dir",
        type=str,
        help="simulator runtime directory",
        default=RUNTIME_DIR,
    )
    parser.add_argument(
        "--blue_full_system_runtime_dir",
        type=str,
        help="blue full_system runtime directory",
        default=os.path.join(RUNTIME_DIR, "blue"),
    )
    parser.add_argument(
        "--yellow_full_system_runtime_dir",
        type=str,
        help="yellow full_system runtime directory",
        default=os.path.join(RUNTIME_DIR, "yellow"),
    )
    parser.add_argument(
        "--layout",
        action="store",
        help="Which layout to run, if not specified the last layout will run",
    )
    parser.add_argument(
        "--debug_blue_full_system",
        action="store_true",
        default=False,
        help="Debug blue full_system",
    )
    parser.add_argument(
        "--debug_yellow_full_system",
        action="store_true",
        default=False,
        help="Debug yellow full_system",
    )
    parser.add_argument(
        "--debug_simulator",
        action="store_true",
        default=False,
        help="Debug the simulator",
    )
    parser.add_argument(
        "--visualization_buffer_size",
        action="store",
        type=int,
        default=5,
        help="How many packets to buffer while rendering",
    )
    parser.add_argument(
        "--show_gamecontroller_logs",
        action="store_true",
        default=False,
        help="How many packets to buffer while rendering",
    )
    parser.add_argument(
        "--run_field_test",
        action="store_true",
        default=False,
        help="whether to run test as a field test instead of a simulated test",
    )
    parser.add_argument(
        "--test_filter",
        action="store",
        default="",
        help="The test filter, if not specified all tests will run. "
        + "See https://docs.pytest.org/en/latest/how-to/usage.html#specifying-tests-selecting-tests",
    )

    parser.add_argument(
        "--interface",
        action="store",
        type=str,
        default=None,
        help="Which interface to communicate over",
    )

    parser.add_argument(
        "--channel",
        action="store",
        type=int,
        default=0,
        help="Which channel to communicate over",
    )

    parser.add_argument(
        "--estop_baudrate",
        action="store",
        type=int,
        default=115200,
        help="Estop Baudrate",
    )

    parser.add_argument(
        "--run_yellow",
        action="store_true",
        default=False,
        help="Run the test with friendly robots in yellow mode",
    )

    estop_group = parser.add_mutually_exclusive_group()
    estop_group.add_argument(
        "--keyboard_estop",
        action="store_true",
        default=False,
        help="Allows the use of the spacebar as an estop instead of a physical one",
    )
    estop_group.add_argument(
        "--disable_communication",
        action="store_true",
        default=False,
        help="Disables checking for estop plugged in (ONLY USE FOR LOCAL TESTING)",
    )

    return parser.parse_args()


def print_proto_log_replay_command(runtime_dir, friendly_colour_yellow):
    """Print the path to the saved proto log and the command to replay it.

    The full_system binary prints this on a clean shutdown, but field tests
    don't reliably surface that message (especially when the test fails), so we
    print it here too. This matches the behaviour of AI vs AI and simulated
    tests, which always tell you where the proto log was saved.

    :param runtime_dir: The full_system runtime directory the proto log was saved under
    :param friendly_colour_yellow: True if the friendly team is yellow, else blue
    """
    # The full_system ProtoLogger saves logs to a "proto_<timestamp>" subdirectory
    # of the runtime directory (see software/logger/proto_logger.cpp). The folder is
    # created shortly after full_system launches, so wait briefly for it to appear in
    # case we're called early (the up-front call before the test starts).
    proto_log_folders = []
    wait_until = time.time() + 3.0
    while time.time() < wait_until:
        proto_log_folders = sorted(glob.glob(os.path.join(runtime_dir, "proto_*")))
        if proto_log_folders:
            break
        time.sleep(0.1)

    if not proto_log_folders:
        logger.warning(f"No proto log was found in {runtime_dir}")
        return

    # Folder names are timestamp-sorted, so the last one is the most recent run
    proto_log_folder = proto_log_folders[-1]
    team = "yellow" if friendly_colour_yellow else "blue"
    log_flag = "--yellow_log" if friendly_colour_yellow else "--blue_log"
    logger.info(
        "\x1b[34m"
        + f"\nTo watch the replay for the {team} team, go to the `src` folder and run "
        + f"\n./tbots.py run thunderscope {log_flag} {proto_log_folder}"
        + "\x1b[0m"
    )


@pytest.fixture
def field_test_runner():
    """Runs a field test

    :return: yields the runner to the test fixture
    """
    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()
    args = load_command_line_arguments()

    # Grab the current test name to store the proto log for the test case
    current_test = os.environ.get("PYTEST_CURRENT_TEST").split(":")[-1].split(" ")[0]
    current_test = current_test.replace("]", "")
    current_test = current_test.replace("[", "-")

    test_name = current_test.split("-")[0]
    debug_full_sys = args.debug_blue_full_system
    runtime_dir = f"{args.blue_full_system_runtime_dir}/test/{test_name}"
    friendly_proto_unix_io = blue_full_system_proto_unix_io

    if args.run_yellow:
        debug_full_sys = args.debug_yellow_full_system
        runtime_dir = f"{args.yellow_full_system_runtime_dir}/test/{test_name}"
        friendly_proto_unix_io = yellow_full_system_proto_unix_io

    estop_mode, estop_path = get_estop_config(
        args.keyboard_estop, args.disable_communication
    )

    # Launch all binaries
    with FullSystem(
        "software/unix_full_system",
        full_system_runtime_dir=runtime_dir,
        debug_full_system=debug_full_sys,
        friendly_colour_yellow=args.run_yellow,
        should_restart_on_crash=False,
    ) as friendly_fs, Gamecontroller(
        # we would be using conventional port if and only if we are playing in robocup.
        suppress_logs=(not args.show_gamecontroller_logs),
        use_conventional_port=False,
    ) as gamecontroller, WifiCommunicationManager(
        current_proto_unix_io=friendly_proto_unix_io,
        multicast_channel=getRobotMulticastChannel(args.channel),
        should_setup_full_system=True,
        interface=args.interface,
        referee_port=gamecontroller.get_referee_port()
        if gamecontroller
        else SSL_REFEREE_PORT,
    ) as wifi_communication_manager, RobotCommunication(
        current_proto_unix_io=friendly_proto_unix_io,
        communication_manager=wifi_communication_manager,
        estop_mode=estop_mode,
        estop_path=estop_path,
    ) as rc_friendly:
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
            test_name=current_test,
            blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
            gamecontroller=gamecontroller,
            thunderscope=tscope,
            robot_communication=rc_friendly,
            is_yellow_friendly=args.run_yellow,
        )

        friendly_proto_unix_io.register_observer(World, runner.world_buffer)

        # Print the proto log path up front, before the test's blocking Thunderscope
        # Qt event loop starts. Field tests are usually exited with Ctrl+C while that
        # loop is running, which bypasses pytest fixture teardown, so printing here
        # guarantees the path is shown no matter how the test is exited.
        print_proto_log_replay_command(runtime_dir, args.run_yellow)

        try:
            yield runner
        finally:
            # Also print on teardown so the path is the last thing shown on a clean
            # exit (closing Thunderscope) or a test failure (when pytest throws the
            # failure into the fixture at yield).
            print_proto_log_replay_command(runtime_dir, args.run_yellow)
