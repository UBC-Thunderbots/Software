import time

import decorator

import pytest
from software.simulated_tests.pytest_main import load_command_line_arguments
from software.simulated_tests.simulated_test_fixture import simulated_test_initializer
from software.field_tests.field_test_fixture import field_test_initializer
import threading
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thunderscope import Thunderscope
from software.logger.logger import createLogger

simulator_proto_unix_io = None
yellow_full_system_proto_unix_io = None
blue_full_system_proto_unix_io = None

logger = createLogger(__name__)

LAUNCH_DELAY_S = 0.1
PAUSE_AFTER_FAIL_DELAY_S = 2
TEST_END_DELAY = 0.3


def initialize_unix_io():
    """
    Initializes proto unix sockets in global scope so that they are accessible from within tests

    """
    global simulator_proto_unix_io
    global yellow_full_system_proto_unix_io
    global blue_full_system_proto_unix_io

    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()


def enable_thunderscope(test):
    """ A decorator that runs a test with thunderscope, based on command line arguments

        test: the test to be run
    """

    def wrapper(*args, **kw):
        def stop_test(delay):
            time.sleep(delay)
            if thunderscope:
                thunderscope.close()

        class Excepthook(object):
            def __init__(self):
                self.last_exception = None

            def excepthook(self, args):
                """This function is _critical_ for show_thunderscope to work.
            If the test Thread raises an exception we won't be able to close
            the window from the main thread.

            :param args: The args passed in from the hook

            """
                stop_test(PAUSE_AFTER_FAIL_DELAY_S)
                self.last_exception = args.exc_value
                raise self.last_exception

        def run_test():
            time.sleep(LAUNCH_DELAY_S)
            # first argument is test itself
            test(*args[1:], **kw)
            stop_test(TEST_END_DELAY)
            return

        cli_args = load_command_line_arguments()

        if cli_args.enable_thunderscope:
            thunderscope = Thunderscope(
                simulator_proto_unix_io=simulator_proto_unix_io,
                blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
            )
            ex = Excepthook()
            threading.excepthook = ex.excepthook
            run_sim_thread = threading.Thread(target=run_test, daemon=True)
            run_sim_thread.start()

            thunderscope.show()
            run_sim_thread.join()

            if ex.last_exception:
                pytest.fail(str(ex.last_exception))

        else:
            thunderscope = None
            run_test()

    return decorator.decorator(wrapper, test)


@pytest.fixture
def field_test_runner():

    initialize_unix_io()

    initializer = field_test_initializer(
        blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
    )

    yield_val = next(initializer)

    yield yield_val

    # test teardown
    try:
        next(initializer)
    except StopIteration as e:
        raise e


@pytest.fixture
def simulated_test_runner():

    initialize_unix_io()

    args = load_command_line_arguments()

    initializer = simulated_test_initializer(
        blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
        simulator_proto_unix_io=simulator_proto_unix_io,
        sleep_between_ticks=args.enable_thunderscope,
    )

    yield_val = next(initializer)

    yield yield_val

    # teardown
    try:
        next(initializer)
    except StopIteration:
        pass


@pytest.fixture
def tbots_test_runner():

    args = load_command_line_arguments()

    initialize_unix_io()

    if args.run_field_test:
        runner_fixture = field_test_initializer(
            blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
        )
    else:
        runner_fixture = simulated_test_initializer(
            blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
            simulator_proto_unix_io=simulator_proto_unix_io,
            sleep_between_ticks=args.enable_thunderscope,
        )

    yield_val = next(runner_fixture)

    yield yield_val

    # teardown
    try:
        yield next(runner_fixture)
    except StopIteration:
        pass
