import threading
import time

import pytest

from proto.import_all_protos import *
from software.logger.logger import createLogger
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

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
        thunderscope,
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
        self.thunderscope = thunderscope
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

    def send_gamecontroller_command(self, gc_command: proto.ssl_gc_state_pb2.Command,
                                    team: proto.ssl_gc_common_pb2.Team,
                                    final_ball_placement_point=None,
                                    ):

        self.gamecontroller.send_ci_input(
            gc_command=gc_command, team=team, final_ball_placement_point=final_ball_placement_point
        )

    def set_tactics(self, tactics:AssignedTacticPlayControlParams, team:proto.ssl_gc_common_pb2.Team):
        raise NotImplementedError("set_tactic unimplemented")

    def set_play(self, play:Play, team: proto.ssl_gc_common_pb2.Team):
        raise NotImplementedError("set_play unimplemented")

    def set_worldState(self, worldstate : WorldState):
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

    def _stop_tscope(self, delay=PROCESS_BUFFER_DELAY_S):
        """Stop running the test

        :param delay: How long to wait before closing everything, defaults
                      to PROCESS_BUFFER_DELAY_S to minimize buffer warnings

        """
        time.sleep(delay)

        if self.thunderscope:
            self.thunderscope.close()


    def _run_with_tscope(self,runner):

        def excepthook(args):
            """This function is _critical_ for show_thunderscope to work.
            If the test Thread will raises an exception we won't be able to close
            the window from the main thread.

            :param args: The args passed in from the hook

            """
            self._stop_tscope(delay=PAUSE_AFTER_FAIL_DELAY_S)
            self.last_exception = args.exc_value
            raise self.last_exception

        threading.excepthook = excepthook

        # If thunderscope is enabled, run the test in a thread and show
        # thunderscope on this thread. The excepthook is setup to catch
        # any test failures and propagate them to the main thread
        if self.thunderscope:

            run_sim_thread = threading.Thread(target=runner, daemon=True)
            run_sim_thread.start()
            self.thunderscope.show()
            run_sim_thread.join()

            if self.last_exception:
                pytest.fail(str(self.last_exception))

        # If thunderscope is disabled, just run the test
        else:
            runner()
