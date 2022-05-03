import time
import threading
import queue
import base64
import os
import logging
import gzip
import proto
from proto.import_all_protos import *
from proto.repeated_any_msg_pb2 import RepeatedAnyMsg
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from google.protobuf.any_pb2 import Any
from software.thunderscope.replay.replay_constants import *


class ProtoLogger(object):

    """Logs incoming protobufs with metadata to a folder to be played back later.

    Each entry will contain
        - The timestamp
        - The protobuf type
        - The protobuf serialized as base64 (to remove newline characters)

    Stored in log_path/protolog_YYYY_MM_DD_HH_MM_SS/
    With each entry in the file formatted as timestamp,protobuf_type,protobuf_base64

    We need to store the data in a way that we can:
        1. Replay the data chronologoically
        2. Seek to a specific time (random access)

    To seek to a specific time, we need to load the entire file into memory. To
    do this, we store the data in chunks. Each chunk contains
    REPLAY_MAX_CHUNK_SIZE_BYTES of serialized protos.

    We can load each chunk into memory and binary search for the entry we want.
    Or we can just play the chunks in order.

    """

    def __init__(self, log_path, log_prefix="protolog_", time_provider=None):
        """Creates a proto logger that logs all protos registered on the queue.

        Stores the files to
            logprefix_YYYY_MM_DD_HH_MM_SS/#.replay

        Where # is the chunk number

        NOTE: The files in the folder will be compressed with gzip.
        I've looked into bz2, but reading chunks was slower.

        Also NOTE: You need the entire folder to replay the data.

        :param log_path: The path to the log file.
        :param time_provider: A function that returns the current time
            in seconds since epoch. Defaults to time.time.

        """
        self.log_prefix = log_prefix
        self.log_folder = (
            log_path + "/" + log_prefix + time.strftime(REPLAY_FILE_TIME_FORMAT) + "/"
        )

        try:
            os.makedirs(self.log_folder)
        except OSError:
            pass

        self.buffer = queue.Queue(PROTOBUF_BUFFER_SIZE)
        self.time_provider = time_provider if time_provider else time.time
        self.start_time = self.time_provider()
        self.stop_logging = False

    def __enter__(self):
        """Starts the logger.

        We use gzip to save the data with compression enabled to
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
        replay_index = -1

        try:
            while not self.stop_logging:

                replay_index += 1

                with gzip.open(
                    self.log_folder + f"{replay_index}.{REPLAY_FILE_EXTENSION}", "wb"
                ) as self.log_file:

                    while self.stop_logging is False:

                        # Consume the buffer and log the protobuf
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

                        # Stop writing to this chunk if we've reached the max size
                        size = os.fstat(self.log_file.fileno()).st_size
                        if size >= REPLAY_MAX_CHUNK_SIZE_BYTES:
                            break

        except Exception:
            logging.exception("Exception detected in ProtoLogger")
