import time
import threading
import queue
import base64
import os
import logging
import bz2
import proto
from proto.import_all_protos import *
from proto.repeated_any_msg_pb2 import RepeatedAnyMsg
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from google.protobuf.any_pb2 import Any

REPLAY_METADATA_DELIMETER = "!#!"
REPLAY_FILE_EXTENSION = ".replay"
REPLAY_FILE_TIME_FORMAT = "%Y-%m-%d-%H-%M-%S"
PROTOBUF_BUFFER_SIZE = 1024 * 1024  # 1MB
PLAY_PAUSE_POLL_INTERVAL_SECONDS = 0.1


class ProtoLogger(object):

    """Logs incoming protobufs to a file. """

    def __init__(self, log_path, log_prefix="log", time_provider=None):
        """Creates a proto logger that logs all protos
        registered on the queue.

        Stores the file to
            log_path/log_prefix-YYYY-MM-DD-HH-MM-SS.replay

        :param log_path: The path to the log file.
        :param log_prefix: The prefix to use for the log file.
        :param time_provider: A function that returns the current time
            in seconds since epoch. Defaults to time.time

        """
        try:
            os.makedirs(log_path)
        except OSError:
            pass

        self.log_path = (
            log_path
            + "/"
            + log_prefix
            + "-"
            + time.strftime(REPLAY_FILE_TIME_FORMAT)
            + REPLAY_FILE_EXTENSION
        )

        self.buffer = queue.Queue(PROTOBUF_BUFFER_SIZE)
        self.time_provider = time_provider if time_provider else time.time
        self.start_time = self.time_provider()
        self.stop_logging = False

    def __enter__(self):
        """Starts the logger.

        We use bz2 to save the data with compression enabled to
        save a _lot_ of space.

        """

        self.thread = threading.Thread(target=self.__log_protobufs, daemon=True)
        self.thread.start()

        return self

    def __exit__(self, type, value, traceback):
        """Closes the log file.

        :param type: The type of the exception.
        :param value: The value of the exception.
        :param traceback: The traceback of the exception.

        """
        self.stop_logging = True
        self.thread.join()

    def __log_protobufs(self):
        """Logs all protos in the queue. 

        Stores it in the format: where !#! is the delimiter.

            timestamp!#!full_name!#!size!#!protobuf

        """
        try:
            with bz2.open(self.log_path, "wb") as self.log_file:
                while self.stop_logging is False:
                    try:
                        proto = self.buffer.get(block=True, timeout=0.01)
                    except queue.Empty:
                        continue

                    serialized_proto = base64.b64encode(proto.SerializeToString())
                    current_time = self.time_provider() - self.start_time

                    log_entry = (
                        f"{current_time}{REPLAY_METADATA_DELIMETER}"
                        + f"{proto.DESCRIPTOR.full_name}{REPLAY_METADATA_DELIMETER}"
                        + f"{serialized_proto}\n"
                    )

                    self.log_file.write(bytes(log_entry, encoding="utf-8"))

        except Exception:
            logging.exception("Exception detected in ProtoLogger")


class ProtoPlayer(object):

    """Plays back a log file. """

    def __init__(self, log_file_path, proto_unix_io):
        """Creates a proto player that plays back all protos

        :param log_file_path: The path to the log file.
        :param proto_unix_io: The proto_unix_io to send the protos to.
            
        """
        self.log_path = log_file_path
        self.proto_unix_io = proto_unix_io
        self.stop_playing = False
        self.exit_requested = False
        self.playback_speed = 1.0

    def __enter__(self):
        """Starts the player.

        We use bz2 to save the data with compression enabled to
        save a _lot_ of space. So we need to user bz2 to read
        the file.

        """

        self.log_file = bz2.open(self.log_path, "rb")
        self.thread = threading.Thread(target=self.__play_protobufs, daemon=True)
        self.thread.start()

        return self

    def __exit__(self, type, value, traceback):
        """Closes the log file.

        :param type: The type of the exception.
        :param value: The value of the exception.
        :param traceback: The traceback of the exception.

        """
        self.stop_playing = True
        self.exit_requested = True
        self.thread.join()
        self.log_file.close()

    def __play_protobufs(self):
        """Plays all protos in the file in chronologoical order. 

        """
        try:
            replay_ended = False
            start_playback_time = time.time()

            while self.exit_requested is False:

                time.sleep(PLAY_PAUSE_POLL_INTERVAL_SECONDS)

                while self.stop_playing is False:

                    # Read replay log entry
                    try:
                        bytes_retrieved = self.log_file.readline()

                        if len(bytes_retrieved) == 0:
                            replay_ended = True

                    except EOFError:
                        replay_ended = True

                    if replay_ended:
                        logging.info("Replay file ended.")
                        return

                    # Unpack metadata
                    timestamp, protobuf_type, data = bytes_retrieved.split(
                        bytes(REPLAY_METADATA_DELIMETER, encoding="utf-8")
                    )

                    # Convert string to type. eval is an order of mangnitude
                    # faster than iterating over the protobuf library.
                    proto_class = eval(
                        str(protobuf_type.split(b".")[1], encoding="utf-8")
                    )

                    # Deserialize protobuf
                    proto = proto_class.FromString(
                        base64.b64decode(data[len("b") : -len("\n")])
                    )

                    # Manage playback speed, if this packet needs to be sent
                    # out later, sleep until its time to send it.
                    time_elapsed = time.time() - start_playback_time
                    replay_current_packet_time = float(timestamp[:-2])

                    if replay_current_packet_time > (
                        time_elapsed / self.playback_speed
                    ):
                        time.sleep(
                            replay_current_packet_time
                            - (time_elapsed / self.playback_speed)
                        )
                        time_elapsed = time.time() - start_playback_time

                    self.proto_unix_io.send_proto(proto_class, proto)

        except Exception:
            logging.exception("Exception detected in ProtoPlayer")
