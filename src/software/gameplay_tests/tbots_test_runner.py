import threading
import time

import pytest

from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from abc import abstractmethod
from typing import Any

PAUSE_AFTER_FAIL_DELAY_S = 3
PROCESS_BUFFER_DELAY_S = 0.01


class TbotsTestRunner:
    """An abstract class that represents a test runner"""

    def __init__(
        self,
        test_name,
        thunderscope,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
        is_yellow_friendly=False,
        owns_thunderscope=True,
    ):
        """Initialize the TestRunner.

        :param test_name: The name of the test to run
        :param thunderscope: The Thunderscope to use, None if not used
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance
        :param: is_yellow_friendly: if yellow is the friendly team
        :param owns_thunderscope: Whether this runner controls the Thunderscope
            lifecycle. True (default) when the runner launched its own
            Thunderscope; False when binding to an already-open Thunderscope
            (test mode), in which case the runner must not show or close it.
        """
        self.test_name = test_name
        self.thunderscope = thunderscope
        self.blue_full_system_proto_unix_io = blue_full_system_proto_unix_io
        self.yellow_full_system_proto_unix_io = yellow_full_system_proto_unix_io
        self.gamecontroller = gamecontroller
        self.is_yellow_friendly = is_yellow_friendly
        self.owns_thunderscope = owns_thunderscope
        self.world_buffer = ThreadSafeBuffer(buffer_size=20, protobuf_type=World)
        self.primitive_set_buffer = ThreadSafeBuffer(
            buffer_size=1, protobuf_type=PrimitiveSet
        )
        self.last_exception = None

        self.ssl_wrapper_buffer = ThreadSafeBuffer(
            buffer_size=1, protobuf_type=SSL_WrapperPacket
        )
        self.robot_status_buffer = ThreadSafeBuffer(
            buffer_size=1, protobuf_type=RobotStatus
        )

        # Track every (proto_unix_io, proto_class, buffer) we register so that
        # cleanup() can deregister them. This keeps long-lived ProtoUnixIOs
        # (test mode) from accumulating dead buffers across runs.
        self._registered_observers = []

        self._register_observer(
            self.blue_full_system_proto_unix_io,
            SSL_WrapperPacket,
            self.ssl_wrapper_buffer,
        )
        self._register_observer(
            self.blue_full_system_proto_unix_io, RobotStatus, self.robot_status_buffer
        )
        if self.is_yellow_friendly:
            self._register_observer(
                self.yellow_full_system_proto_unix_io, World, self.world_buffer
            )
            self._register_observer(
                self.yellow_full_system_proto_unix_io,
                PrimitiveSet,
                self.primitive_set_buffer,
            )
        # Only validate on the blue worlds
        else:
            self._register_observer(
                self.blue_full_system_proto_unix_io, World, self.world_buffer
            )
            self._register_observer(
                self.blue_full_system_proto_unix_io,
                PrimitiveSet,
                self.primitive_set_buffer,
            )

    def _register_observer(self, proto_unix_io, proto_class, buffer):
        """Register an observer buffer and remember it for cleanup().

        :param proto_unix_io: The ProtoUnixIO to register the buffer on
        :param proto_class: Class of protobuf to consume
        :param buffer: buffer to register
        """
        proto_unix_io.register_observer(proto_class, buffer)
        self._registered_observers.append((proto_unix_io, proto_class, buffer))

    def cleanup(self):
        """Deregister all observers this runner placed on the shared ProtoUnixIOs.

        Must be called when binding to long-lived ProtoUnixIOs (test mode) so
        that buffers from finished runs do not accumulate across runs.
        """
        for proto_unix_io, proto_class, buffer in self._registered_observers:
            proto_unix_io.deregister_observer(proto_class, buffer)
        self._registered_observers = []

    def send_gamecontroller_command(
        self,
        gc_command: proto.ssl_gc_state_pb2.Command,
        team: proto.ssl_gc_common_pb2.Team,
        final_ball_placement_point=None,
    ):
        """Sends a gamecontroller command that is to be broadcasted to the given team

        :param gc_command: the gamecontroller command
        :param is_friendly: whether the command should be sent to the friendly team
        :param final_ball_placement_point: where to place the ball in ball placement
        """
        self.gamecontroller.send_gc_command(
            gc_command=gc_command,
            team=team,
            final_ball_placement_point=final_ball_placement_point,
        )

    def set_tactics(
        self,
        blue_tactics: dict[int, Any] | None = {},
        yellow_tactics: dict[int, Any] | None = {},
    ):
        """Overrides AI tactics for all robots on each team.
        By default, assigns no tactic for all robots whose id is not specified.
        Pass in a None value for a team's tactics to not send tactics override.

        :param blue_tactics: None or dict of robot_id -> tactic for blue team
        :param yellow_tactics: None or dict of robot_id -> tactic for yellow team
        """
        if blue_tactics is not None:
            blue_params = self._create_assigned_tactic_params(blue_tactics)
            self.blue_full_system_proto_unix_io.send_proto(
                AssignedTacticPlayControlParams, blue_params
            )

        if yellow_tactics is not None:
            yellow_params = self._create_assigned_tactic_params(yellow_tactics)
            self.yellow_full_system_proto_unix_io.send_proto(
                AssignedTacticPlayControlParams, yellow_params
            )

    def set_plays(self, blue_play: PlayName, yellow_play: PlayName):
        """Overrides current AI play for both teams

        :param blue_play: the play name for the blue team to use
        :param yellow_play: the play name for the yellow team to use
        """
        self.blue_full_system_proto_unix_io.send_proto(Play, Play(name=blue_play))
        self.yellow_full_system_proto_unix_io.send_proto(Play, Play(name=yellow_play))

    @abstractmethod
    def set_world_state(self, world_state: WorldState):
        """Sets the initial world state of the test.

        :param world_state: The WorldState proto to use
        """
        raise NotImplementedError("abstract class method called set_world_state")

    def run_test(
        self,
        setup: (lambda: None),
        always_validation_sequence_set=[],
        eventually_validation_sequence_set=[],
        test_timeout_s=3,
        gc_cmd_with_delay=[],
    ):
        """Begins validating a test based on incoming world protos.
        Runs the test in a background thread if thunderscope is enabled.

        :param setup: Function that sets up the world state
        :param always_validation_sequence_set: validation set that must always be true
        :param eventually_validation_sequence_set: validation set that must eventually be true
        :param test_timeout_s: how long the test will run
        :param gc_cmd_with_delay: timed GC commands
        """
        self._pre_run_setup(setup)

        args = (
            always_validation_sequence_set,
            eventually_validation_sequence_set,
            test_timeout_s,
            gc_cmd_with_delay,
        )

        # When bound to an already-open Thunderscope (test mode), we are already
        # running on a background thread while the Qt event loop runs on the main
        # thread, so run the test loop inline and let exceptions propagate to
        # pytest. Otherwise, run the loop in a background thread and block the
        # main thread on the Thunderscope we own.
        if self.thunderscope and self.owns_thunderscope:
            threading.excepthook = self._excepthook

            run_test_thread = threading.Thread(
                target=self._runner, daemon=True, args=args
            )
            run_test_thread.start()
            self.thunderscope.show()
            run_test_thread.join()

            if self.last_exception:
                pytest.fail(str(self.last_exception))
        else:
            self._runner(*args)

    @abstractmethod
    def _runner(
        self,
        always_validation_sequence_set,
        eventually_validation_sequence_set,
        test_timeout_s,
        gc_cmd_with_delay,
    ):
        """Internal test loop; implemented by subclasses.

        See run_test() method for param docs.
        """
        raise NotImplementedError("abstract class method called _runner")

    @abstractmethod
    def _pre_run_setup(self, setup: (lambda: None)):
        """Hook called before the test loop starts

        :param setup: Function that sets up the world state
        """
        raise NotImplementedError("abstract class method called _pre_run_setup")

    def _stopper(self, delay=PROCESS_BUFFER_DELAY_S):
        """Stop running the test

        :param delay: How long to wait before closing everything, defaults
                      to PROCESS_BUFFER_DELAY_S to minimize buffer warnings
        """
        time.sleep(delay)

        # Only close a Thunderscope that this runner owns. In test mode the
        # Thunderscope outlives the test, so it must be left open.
        if self.thunderscope and self.owns_thunderscope:
            self.thunderscope.close()

    def _excepthook(self, args):
        """This function is _critical_ for show_thunderscope to work.
        If the test Thread will raises an exception we won't be able to close
        the window from the main thread.

        :param args: The args passed in from the hook
        """
        self._stopper(delay=PAUSE_AFTER_FAIL_DELAY_S)
        self.last_exception = args.exc_value
        raise self.last_exception

    def _create_assigned_tactic_params(self, tactics: dict[int, Any]):
        """Converts dict of tactics to AssignedTacticPlayControlParams message

        :param tactics: dict of robot_id -> tactic
        """
        params = AssignedTacticPlayControlParams()

        # Checks which oneof field in Tactic to assign the specified tactic to
        for robot_id, specific_tactic in tactics.items():
            tactic = params.assigned_tactics[robot_id]
            for field in tactic.DESCRIPTOR.oneofs_by_name["tactic"].fields:
                if field.message_type == specific_tactic.DESCRIPTOR:
                    getattr(tactic, field.name).CopyFrom(specific_tactic)
                    break

        return params
