import os
import time

import pytest

from software.gameplay_tests.util import load_command_line_arguments
from software.gameplay_tests.simulated_test_runner import (
    InvariantTestRunner,
    AggregateTestRunner,
)
from software.thunderscope.binary_context_managers.full_system import FullSystem
from software.thunderscope.binary_context_managers.game_controller import Gamecontroller
from software.thunderscope.binary_context_managers.simulator import Simulator
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.thunderscope_config import configure_simulated_test_view

LAUNCH_DELAY_S = 0.1


@pytest.fixture
def gameplay_test_runner():
    args = load_command_line_arguments()
    tscope = None

    aggregate = args.aggregate

    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()

    # Grab the current test name to store the proto log for the test case
    current_test = os.environ.get("PYTEST_CURRENT_TEST").split(":")[-1].split(" ")[0]
    current_test = current_test.replace("]", "")
    current_test = current_test.replace("[", "-")

    test_name = current_test.split("-")[0]

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
            running_in_realtime=args.enable_thunderscope,
        ) as blue_fs,
        FullSystem(
            "software/unix_full_system",
            f"{args.yellow_full_system_runtime_dir}/test/{test_name}",
            args.debug_yellow_full_system,
            True,
            should_restart_on_crash=False,
            running_in_realtime=args.enable_thunderscope,
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

            runner = None

            # Initialise the right runner based on which testing mode is selected
            if aggregate:
                runner = AggregateTestRunner(
                    current_test,
                    tscope,
                    simulator_proto_unix_io,
                    blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io,
                    gamecontroller,
                )
            else:
                runner = InvariantTestRunner(
                    current_test,
                    tscope,
                    simulator_proto_unix_io,
                    blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io,
                    gamecontroller,
                )

            yield runner
