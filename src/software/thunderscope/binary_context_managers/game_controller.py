import os
import socket
import time
import google.protobuf.internal.encoder as encoder
import google.protobuf.internal.decoder as decoder
from typing import Any

from subprocess import Popen
import software.python_bindings as tbots_cpp
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.python_bindings import *
from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.binary_context_managers.util import *


class Gamecontroller(object):

    """ Gamecontroller Context Manager """

    CI_MODE_LAUNCH_DELAY_S = 0.3
    REFEREE_IP = "224.5.23.1"
    CI_MODE_OUTPUT_RECEIVE_BUFFER_SIZE = 9000

    def __init__(self, supress_logs: bool = False, ci_mode: bool = False) -> None:
        """Run Gamecontroller

        :param supress_logs: Whether to suppress the logs
        :param ci_mode: Whether to run the gamecontroller in CI mode

        """
        self.supress_logs = supress_logs
        self.ci_mode = ci_mode

        # We need to find 2 free ports to use for the gamecontroller
        # so that we can run multiple gamecontroller instances in parallel
        self.referee_port = self.next_free_port()
        self.ci_port = self.next_free_port()

    def __enter__(self) -> "self":
        """Enter the gamecontroller context manager. 

        :return: gamecontroller context managed instance

        """
        command = ["/opt/tbotspython/gamecontroller"]

        if self.ci_mode:
            command = ["/opt/tbotspython/gamecontroller", "--timeAcquisitionMode", "ci"]

        command += ["-publishAddress", f"{self.REFEREE_IP}:{self.referee_port}"]
        command += ["-ciAddress", f"localhost:{self.ci_port}"]

        if self.supress_logs:
            with open(os.devnull, "w") as fp:
                self.gamecontroller_proc = Popen(command, stdout=fp, stderr=fp)

        else:
            self.gamecontroller_proc = Popen(command)

        if self.ci_mode:
            # We can't connect to the ci port right away, it takes
            # CI_MODE_LAUNCH_DELAY_S to start up the gamecontroller
            time.sleep(Gamecontroller.CI_MODE_LAUNCH_DELAY_S)

            self.ci_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ci_socket.connect(("", self.ci_port))

        return self

    def __exit__(self, type, value, traceback) -> None:
        """Exit the gamecontroller context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        self.gamecontroller_proc.terminate()
        self.gamecontroller_proc.wait()

        if self.ci_mode:
            self.ci_socket.close()

    def next_free_port(self, port: int = 40000, max_port: int = 65535) -> None:
        """Find the next free port. We need to find 2 free ports to use for the gamecontroller
        so that we can run multiple gamecontroller instances in parallel.

        :param port: The port to start looking from
        :param max_port: The maximum port to look up to
        :return: The next free port

        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        while port <= max_port:
            try:
                sock.bind(("", port))
                sock.close()
                return port
            except OSError:
                port += 1

        raise IOError("no free ports")

    def setup_proto_unix_io(
        self,
        blue_full_system_proto_unix_io: ProtoUnixIO,
        yellow_full_system_proto_unix_io: ProtoUnixIO,
    ) -> None:
        """Setup gamecontroller io

        :param blue_full_system_proto_unix_io: The proto unix io of the blue full system.
        :param yellow_full_system_proto_unix_io: The proto unix io of the yellow full system.

        """

        def __send_referee_command(data: Referee) -> None:
            """Send a referee command from the gamecontroller to both full
            systems.

            :param data: The referee command to send

            """
            blue_full_system_proto_unix_io.send_proto(Referee, data)
            yellow_full_system_proto_unix_io.send_proto(Referee, data)

        self.receive_referee_command = tbots_cpp.SSLRefereeProtoListener(
            Gamecontroller.REFEREE_IP, self.referee_port, __send_referee_command, True,
        )

    def send_ci_input(
        self,
        gc_command: proto.ssl_gc_state_pb2.Command,
        team: proto.ssl_gc_common_pb2.Team,
        final_ball_placement_point: tbots_cpp.Point = None,
    ) -> Any:
        """Send a ci input to the gamecontroller.

        CiInput -> Input -> Change ->  NewCommand -> Command -> (Type, Team)

        More info here:
        https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/cmd/ssl-ci-test-client/README.md

        :param gc_command: The gc command to send
        :param team: The team to send the command to
        :return: The response CiOutput containing 1 or more refree msgs

        """
        ci_ci_input = CiInput(timestamp=int(time.time_ns()))
        ci_input = Input()
        ci_change = Change()
        ci_new_command = NewCommand()
        ci_command = Command(type=gc_command, for_team=team)

        ci_new_command.command.CopyFrom(ci_command)
        ci_change.new_command.CopyFrom(ci_new_command)
        ci_input.change.CopyFrom(ci_change)
        ci_ci_input.api_inputs.append(ci_input)

        # Do this only if ball placement pos is specified
        if final_ball_placement_point:
            # Set position
            ball_placement_pos = SetBallPlacementPos()
            ball_placement_pos.pos.CopyFrom(
                Vector2(
                    x=float(final_ball_placement_point.x()),
                    y=float(final_ball_placement_point.y()),
                )
            )
            ci_change = Change()
            ci_input = Input()
            ci_change.set_ball_placement_pos.CopyFrom(ball_placement_pos)
            ci_input.change.CopyFrom(ci_change)
            ci_ci_input.api_inputs.append(ci_input)

            # Start Placement
            ci_change = Change()
            ci_input = Input()
            start_placement = StartBallPlacement()
            ci_change.start_ball_placement.CopyFrom(start_placement)
            ci_input.change.CopyFrom(ci_change)
            ci_ci_input.api_inputs.append(ci_input)

        # https://cwiki.apache.org/confluence/display/GEODE/Delimiting+Protobuf+Messages
        size = ci_ci_input.ByteSize()

        # Send a request to the host with the size of the message
        self.ci_socket.send(
            encoder._VarintBytes(size) + ci_ci_input.SerializeToString()
        )

        response_data = self.ci_socket.recv(
            Gamecontroller.CI_MODE_OUTPUT_RECEIVE_BUFFER_SIZE
        )

        msg_len, new_pos = decoder._DecodeVarint32(response_data, 0)
        ci_output = CiOutput()
        ci_output.ParseFromString(response_data[new_pos : new_pos + msg_len])
        return ci_output
