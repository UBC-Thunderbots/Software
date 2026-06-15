import time
import os

import pytest

from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.binary_context_managers.full_system import FullSystem
from software.thunderscope.binary_context_managers.simulator import Simulator
from software.thunderscope.binary_context_managers.game_controller import Gamecontroller
from software.thunderscope.thunderscope_config import configure_simulated_test_view
from software.gameplay_tests.util import load_command_line_arguments
from software.gameplay_tests.simulated_test_runner import SimulatedTestRunner

LAUNCH_DELAY_S = 0.1


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


@pytest.fixture
def simulated_test_runner():
    args = load_command_line_arguments()
    tscope = None

    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()

    # Grab the current test name to store the proto log for the test case
    current_test = os.environ.get("PYTEST_CURRENT_TEST").split(":")[-1].split(" ")[0]
    current_test = current_test.replace("]", "")
    current_test = current_test.replace("[", "-")

    # Truncate the test name to 25 characters for UNIX path length limits
    test_name = current_test.split("-")[0][:25]

    # Launch all binaries
    with (
        Simulator(
            f"{args.simulator_runtime_dir}/test/{test_name}",
            args.debug_simulator,
            args.enable_realism,
        ) as simulator,
        FullSystem(
            "software/unix_full_system",
            f"{args.blue_full_system_runtime_dir}/test/{test_name}",
            args.debug_blue_full_system,
            False,
            should_restart_on_crash=False,
            running_in_realtime=args.enable_thunderscope and not args.ci_mode,
        ) as blue_fs,
        FullSystem(
            "software/unix_full_system",
            f"{args.yellow_full_system_runtime_dir}/test/{test_name}",
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
                current_test,
                tscope,
                simulator_proto_unix_io,
                blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io,
                gamecontroller,
                args.ci_mode,
            )

            yield runner
