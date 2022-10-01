import time

import decorator
import inspect

import pytest
from software.simulated_tests.pytest_main import load_command_line_arguments
from software.simulated_tests.simulated_test_fixture import simulated_test_initializer
from software.field_tests.field_test_fixture import field_test_initializer
import signal
import os
import threading
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thunderscope import Thunderscope
from software.logger.logger import createLogger

logger = createLogger(__name__)

PAUSE_AFTER_FAIL_DELAY_S = 3
PROCESS_BUFFER_DELAY_S=0.01
simulator_proto_unix_io = ProtoUnixIO()
yellow_full_system_proto_unix_io = ProtoUnixIO()
blue_full_system_proto_unix_io = ProtoUnixIO()

def enable_thunderscope(test):
    def wrapper(*args, **kw):

        def __stopper(delay=PROCESS_BUFFER_DELAY_S):
            """Stop running the test
            :param delay: How long to wait before closing everything, defaults
                          to PROCESS_BUFFER_DELAY_S to minimize buffer warnings
            """
            time.sleep(delay)

            if thunderscope:
                thunderscope.close()

        def excepthook(args):
            """This function is _critical_ for show_thunderscope to work.
            If the test Thread raises an exception we won't be able to close
            the window from the main thread.

            :param args: The args passed in from the hook

            """
            __stopper(delay=PAUSE_AFTER_FAIL_DELAY_S)
            last_exception = args.exc_value
            raise last_exception

        def __runner():
            LAUNCH_DELAY_S = 0.5

            time.sleep(LAUNCH_DELAY)
            #first argument is test itself
            test(*args[1:], **kw)
            __stopper()
            return

        cli_args = load_command_line_arguments()

        if cli_args.enable_thunderscope:
            thunderscope = Thunderscope(
                simulator_proto_unix_io=simulator_proto_unix_io,
                blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
            )
            
        else:
            thunderscope = None

        threading.excepthook = excepthook
        run_sim_thread = threading.Thread(target=__runner)
        run_sim_thread.start()

        if cli_args.enable_thunderscope:
            thunderscope.show()

        run_sim_thread.join()

    return decorator.decorator(wrapper, test)


@pytest.fixture
def field_test_runner():
    initializer = field_test_initializer(blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                                        yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io)

    yield_val = next(initializer)

    if isinstance(yield_val, Exception):
        raise yield_val

    logger.info("test teardown")
    # test teardown
    try:
        next(initializer)
    except StopIteration:
        pass


@pytest.fixture
def simulated_test_runner():
    initializer = simulated_test_initializer(blue_full_system_proto_unix_io=blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io, simulator_proto_unix_io=simulator_proto_unix_io)

    yield next(initializer)

    # teardown
    try:
        next(initializer)
    except StopIteration:
        pass


@pytest.fixture
def tbots_test_runner():
    args = load_command_line_arguments()

    if args.run_field_test:
        runner_fixture = field_test_initializer(blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                                                yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io)
    else:
        runner_fixture = simulated_test_initializer(blue_full_system_proto_unix_io=blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io, simulator_proto_unix_io=simulator_proto_unix_io)

    yield next(runner_fixture)

    # teardown
    try:
        yield next(runner_fixture)
    except StopIteration:
        pass
