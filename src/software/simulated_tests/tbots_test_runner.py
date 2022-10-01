import threading
import time
import platform

import pytest

from proto.import_all_protos import *
from software.logger.logger import createLogger
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.thunderscope import Thunderscope
from proto.ssl_gc_common_pb2 import Team


logger = createLogger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 5.0
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3


class TbotsTestRunner(object):

    """Run a test"""

    def __init__(
        self,
        test_name,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
    ):
        """Initialize the TestRunner
        
        :param test_name: The name of the test to run
        :param thunderscope: The thunderscope to use, None if not used
        :param simulator_proto_unix_io: The simulator proto unix io to use
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance 

        """

        self.test_name = test_name
        self.blue_full_system_proto_unix_io = blue_full_system_proto_unix_io
        self.yellow_full_system_proto_unix_io = yellow_full_system_proto_unix_io
        self.gamecontroller = gamecontroller
        self.world_buffer = ThreadSafeBuffer(buffer_size=20, protobuf_type=World)

        self.last_exception = None

        self.ssl_wrapper_buffer = ThreadSafeBuffer(
            buffer_size=1, protobuf_type=SSL_WrapperPacket
        )
        self.robot_status_buffer = ThreadSafeBuffer(
            buffer_size=1, protobuf_type=RobotStatus
        )

        self.blue_full_system_proto_unix_io.register_observer(
            SSL_WrapperPacket, self.ssl_wrapper_buffer
        )
        self.blue_full_system_proto_unix_io.register_observer(
            RobotStatus, self.robot_status_buffer
        )

        self.blue_full_system_proto_unix_io.register_observer(World, self.world_buffer)

        self.timestamp = 0
        self.timestamp_mutex = threading.Lock()

        logger.info("setup runner")

    def send_gamecontroller_command(
        self,
        gc_command: proto.ssl_gc_state_pb2.Command,
        isBlue: bool,
        final_ball_placement_point=None,
    ):
        if isBlue:
            self.gamecontroller.send_ci_input(
                gc_command=gc_command,
                team=Team.BLUE,
                final_ball_placement_point=final_ball_placement_point,
            )
        else:
            self.gamecontroller.send_ci_input(
                gc_command=gc_command,
                team=Team.YELLOW,
                final_ball_placement_point=final_ball_placement_point,
            )

    def set_tactics(
        self,
        tactics: AssignedTacticPlayControlParams,
        isBlue: bool,
    ):
        raise NotImplementedError("set_tactic unimplemented")

    def set_play(self, play: Play, isBlue: bool):
        raise NotImplementedError("set_play unimplemented")

    def set_worldState(self, worldstate: WorldState):
        raise NotImplementedError("set_worldstate unimplemented")

    def time_provider(self):
        """Provide the current time in seconds since the epoch"""
        raise NotImplementedError("run_test unimplemented")

    def run_test(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=3,
        tick_duration_s=0.0166,  # Default to 60hz
    ):
        raise NotImplementedError("run_test unimplemented")
