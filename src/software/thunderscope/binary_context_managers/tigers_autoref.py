from __future__ import annotations

from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team as SslTeam
from software.networking.ssl_proto_communication import (
    SslSocketProtoParseException,
    SslSocket,
)
from software.py_constants import NANOSECONDS_PER_MILLISECOND, SECONDS_PER_NANOSECOND
import software.python_bindings as tbots_cpp
from software.thunderscope.binary_context_managers.game_controller import Gamecontroller
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.time_provider import TimeProvider
from subprocess import Popen

import queue
import logging
import os
import threading
import time


class TigersAutoref(TimeProvider):
    """
    A wrapper over the TigersAutoref binary. It coordinates communication between the Simulator, TigersAutoref and
    Gamecontroller. 

    In CI mode, the flow of data corresponds to:

              SSL_DetectionFrame          AutoRefCiOutput: TrackerWrapperPacket                CiOutput
    Simulator ------------------> AutoRef ------------------------------------> Gamecontroller --------------> AI
                AutoRefCiInput                         CiInput                                 RefereeMessage

    For more information: https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/doc/AutoRefCi.md
    """

    AUTOREF_COMM_PORT = 10013
    AUTOREF_NUM_RETRIES = 20
    BUFFER_TIMEOUT = 10
    NEXT_PACKET_DELAY = 1.0 / 30  # 30 Hz

    def __init__(
        self,
        gc: Gamecontroller,
        tick_rate_ms: int,
        ci_mode: bool = False,
        buffer_size: int = 5,
        supress_logs: bool = True,
        show_gui: bool = False,
    ) -> None:
        """
        Constructor

        :param gc:              gamecontroller instance
        :param tick_rate_ms:    the interval between subsequent SSL Vision packets
        :param ci_mode:         true to run the autoref binary in CI mode (use system time or use the time from the
                                CiInput messages)
        :param buffer_size:     buffer size for the SSL wrapper and referee packets
        :supress_logs:          true silences logs from the Autoref binary, otherwise shows them (its very verbose)
        :show_gui:              true shows the Tigers' autoref GUI, false runs it in headless mode
        """
        self.tigers_autoref_proc = None
        self.auto_ref_proc_thread = None
        self.auto_ref_wrapper_thread = None
        self.ci_mode = ci_mode
        self.end_autoref = False
        self.wrapper_buffer = ThreadSafeBuffer(buffer_size, SSL_WrapperPacket)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee)
        self.gamecontroller = gc
        self.supress_logs = supress_logs
        self.tick_rate_ms = tick_rate_ms
        self.show_gui = show_gui

        self.current_timestamp = int(time.time_ns())
        self.timestamp_mutex = threading.Lock()

    def __enter__(self) -> "self":
        if not os.path.exists("/opt/tbotspython/autoReferee/bin/autoReferee"):
            logging.warning(
                "Could not find autoref binary, did you run ./setup_software.sh"
            )
            return

        self.auto_ref_proc_thread = threading.Thread(
            target=self._start_autoref, daemon=True
        )
        self.auto_ref_proc_thread.start()

        self.auto_ref_wrapper_thread = threading.Thread(
            target=self._send_to_autoref_and_forward_to_gamecontroller, daemon=True
        )
        self.auto_ref_wrapper_thread.start()

        return self

    def time_provider(self):
        with self.timestamp_mutex:
            return self.current_timestamp * SECONDS_PER_NANOSECOND

    def _force_gamecontroller_to_accept_all_events(self) -> list[CiOutput]:
        """
        Force the Gamecontroller to accept all game events proposed by the Autoref

        :return: a list of CiOutput protos from the Gamecontroller
        """
        game_event_proto_map = Config()

        for game_event in GameEvent.Type.DESCRIPTOR.values_by_name:
            game_event_proto_map.game_event_behavior[
                game_event
            ] = Config.Behavior.BEHAVIOR_ACCEPT

        return self.gamecontroller.update_game_engine_config(game_event_proto_map)

    def _send_geometry(self) -> None:
        """
        Sends updated field geometry to the AutoRef so that the TigersAutoref knows about field sizes.
        """
        ssl_wrapper = self.wrapper_buffer.get(block=True)
        ci_input = AutoRefCiInput()
        ci_input.detection.append(ssl_wrapper.detection)

        field = tbots_cpp.Field.createSSLDivisionBField()
        ci_input.geometry.CopyFrom(tbots_cpp.createGeometryData(field, 0.3))

        self.ci_socket.send(ci_input)

        while True:
            try:
                response_data = self.ci_socket.receive(AutoRefCiOutput)
                break
            except SslSocketProtoParseException as parse_err:
                logging.info("error with sending geometry data: \n" + parse_err.args)

        for ci_output in response_data:
            self._forward_to_gamecontroller(ci_output.tracker_wrapper_packet)

    def _persistently_connect_to_autoref(self) -> bool:
        """
        Connect to the TigersAutoref binary. Retry connection a few times if the connection doesn't go through in case
        the binary hasn't started yet.

        :return: True if the action was successful, False otherwise
        """
        tries = 0
        while tries < TigersAutoref.AUTOREF_NUM_RETRIES:
            try:
                # We cannot start the Autoref binary immediately, so we must wait until the binary has started before
                # we try to connect to it
                time.sleep(0.5)
                self.ci_socket = SslSocket(TigersAutoref.AUTOREF_COMM_PORT)
                return True
            except ConnectionRefusedError:
                tries += 1

        return False

    def _send_to_autoref_and_forward_to_gamecontroller(self) -> None:
        """
        Main communication loop that sets up the TigersAutoref and coordinates communication between Simulator,
        TigersAutoref and Gamecontroller. Returns early if connection to the TigersAutoref binary was unsuccessful.
        """
        if not self._persistently_connect_to_autoref():
            logging.warning("Failed to connect to autoref binary. Is it running?")
            return

        self._force_gamecontroller_to_accept_all_events()
        self._send_geometry()
        self.gamecontroller.reset_team_info(Division.DIV_B)

        self.gamecontroller.send_gc_command(
            gc_command=Command.Type.STOP, team=SslTeam.UNKNOWN
        )

        while not self.end_autoref:
            try:
                ssl_wrapper = self.wrapper_buffer.get(
                    block=True, timeout=TigersAutoref.BUFFER_TIMEOUT
                )
                referee_packet = self.referee_buffer.get(block=False)

                ci_input = AutoRefCiInput()
                ci_input.detection.append(ssl_wrapper.detection)
                if referee_packet.IsInitialized():
                    ci_input.referee_message.CopyFrom(referee_packet)

                self.ci_socket.send(ci_input)
                response_data = self.ci_socket.receive(AutoRefCiOutput)

                for ci_output in response_data:
                    self._forward_to_gamecontroller(ci_output.tracker_wrapper_packet)
            except SslSocketProtoParseException:
                logging.info(
                    "error with receiving AutoRefCiOutput, ignoring this packet..."
                )
            except queue.Empty:
                pass

            with self.timestamp_mutex:
                self.current_timestamp += int(
                    self.tick_rate_ms * NANOSECONDS_PER_MILLISECOND
                )

    def _forward_to_gamecontroller(
        self, tracker_wrapper: proto.ssl_vision_wrapper_tracked_pb2.TrackerWrapperPacket
    ) -> list[CiOutput]:
        """
        Uses the given tracker_wrapper to create a CiInput for the Gamecontroller to track. Uses the timestamp from the
        given tracker_wrapper to support asynchronous ticking.

        :param tracker_wrapper TrackerWrapperPacket for the Gamecontroller to track

        :return: a list of CiOutput protos received from the Gamecontroller
        """
        with self.timestamp_mutex:
            ci_input = CiInput(timestamp=self.current_timestamp)

        ci_input.api_inputs.append(Input())
        ci_input.tracker_packet.CopyFrom(tracker_wrapper)

        return self.gamecontroller.send_ci_input(ci_input)

    def _start_autoref(self) -> None:
        """
        Starts the TigersAutoref binary.
        """
        autoref_cmd = "software/autoref/run_autoref"

        if not self.show_gui:
            autoref_cmd += " -hl"

        if self.ci_mode:
            autoref_cmd += " --ci"

        if self.supress_logs:
            with open(os.devnull, "w") as fp:
                self.tigers_autoref_proc = Popen(
                    autoref_cmd.split(" "), stdout=fp, stderr=fp
                )
        else:
            self.tigers_autoref_proc = Popen(autoref_cmd.split(" "))

    def setup_ssl_wrapper_packets(self, autoref_proto_unix_io: ProtoUnixIO) -> None:
        """
        Registers as an observer of TrackerWrapperPackets from the Simulator, so that they can be forwarded to the
        Gamecontroller in CI mode.

        :param autoref_proto_unix_io:               the proto unix io for the Autoref to receive SSLWrapperPackets
        """
        autoref_proto_unix_io.register_observer(SSL_WrapperPacket, self.wrapper_buffer)
        autoref_proto_unix_io.register_observer(Referee, self.referee_buffer)

    def __exit__(self, type, value, traceback) -> None:
        if self.tigers_autoref_proc:
            self.tigers_autoref_proc.kill()
            self.tigers_autoref_proc.wait()

            self.auto_ref_proc_thread.join()

        self.end_autoref = True
        self.auto_ref_wrapper_thread.join()

        logging.info("[TigersAutoref] Process exited")
