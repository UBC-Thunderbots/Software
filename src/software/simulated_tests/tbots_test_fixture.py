import threading
import queue
import argparse
import time
import sys
import os

import pytest
import software.python_bindings as tbots
from proto.import_all_protos import *

from pyqtgraph.Qt import QtCore, QtGui

from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.simulated_tests.robot_enters_region import RobotEntersRegion

from software.simulated_tests import validation
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.py_constants import MILLISECONDS_PER_SECOND
from software.thunderscope.binary_context_managers import (
    FullSystem,
    Simulator,
    Gamecontroller,
)
from software.thunderscope.replay.proto_logger import ProtoLogger

from software.logger.logger import createLogger
from software.simulated_tests.pytest_main import load_command_line_arguments
from software.simulated_tests.simulated_test_fixture import simulated_test_initializer
from software.simulated_tests.field_tests.field_test_fixture import field_test_initializer

logger = createLogger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 0.5
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3


@pytest.fixture
def tbots_test_runner():
    args = load_command_line_arguments()

    if args.run_field_test:
        runner_fixture = field_test_initializer()
    else:
        runner_fixture = simulated_test_initializer()

    yield next(runner_fixture)

    #teardown
    try:
        yield next(runner_fixture)
    except StopIteration:
        pass

