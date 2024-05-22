from __future__ import annotations
import time
import threading
import queue
import base64
import os
import logging
import gzip
from proto.import_all_protos import *
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from software.thunderscope.replay.replay_constants import *
from typing import Callable
from google.protobuf.message import Message


class ProtoLogger:

    """Logs incoming protobufs with metadata to a folder to be played back later.

    Each entry will contain
        - The timestamp
        - The protobuf type
        - The protobuf serialized as base64 (to remove newline characters)

    Stored in log_path/protolog_YYYY_MM_DD_HH_MM_SS/
    With each entry in the file formatted as timestamp,protobuf_type,protobuf_base64

    We need to store the data in a way that we can:
        1. Replay the data chronologically
        2. Seek to a specific time (random access)

    To seek to a specific time, we need to load the entire log file into memory.
    To make this feasible, we store the data in chunks. Each chunk contains
    REPLAY_MAX_CHUNK_SIZE_BYTES of serialized protos.

    We can load each chunk into memory and perform search operations to find the
    appropriate entry. Or we can just play the chunks in order.

    """

    BLOCK_TIMEOUT = 0.1

    def __init__(
        self,
        log_path: str,
        log_prefix: str = "proto_",
        time_provider: Callable[[], float] = None,
    ) -> None:
        """Creates a proto logger that logs all protos registered on the queue.

        Stores the files to
            logprefix_YYYY_MM_DD_HH_MM_SS/#.replay

        Where # is the chunk number

        NOTE: The files in the folder will be compressed with gzip.
        I've looked into bz2, but gzip can read chunks from disk into memory faster.

        :param log_path: The path to the log file.
        :param time_provider: A function that returns the current time
            in seconds since epoch. Defaults to time.time.

        """
        log_timestamp = time.strftime(REPLAY_FILE_TIME_FORMAT)
        self.log_folder = f"{log_path}/{log_prefix}{log_timestamp}/"

        try:
            os.makedirs(self.log_folder)
        except OSError:
            pass

        self.buffer = queue.Queue(PROTOBUF_BUFFER_SIZE)
        self.time_provider = time_provider if time_provider else time.time
        self.start_time = self.time_provider()
        self.stop_logging = False

    def __enter__(self) -> ProtoLogger:
        """Starts the logger.

        We use gzip to save the data with compression enabled to
        save a _lot_ of space.

        """
        self.thread = threading.Thread(target=self.__log_protobufs, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, type, value, traceback) -> None:
        """Closes the log file.

        :param type: The type of the exception.
        :param value: The value of the exception.
        :param traceback: The traceback of the exception.

        """
        self.stop_logging = True
        self.thread.join()

    def __log_protobufs(self) -> None:
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

                    # Allocates 1MB of disk for impending replay
                    os.ftruncate(self.log_file.fileno(), REPLAY_MAX_CHUNK_SIZE_BYTES)

                    while self.stop_logging is False:

                        # Consume the buffer and log the protobuf
                        # We provide a timeout here to not block forever if we don't receive anything
                        try:
                            proto = self.buffer.get(
                                block=True, timeout=ProtoLogger.BLOCK_TIMEOUT
                            )
                        except queue.Empty:
                            continue
                        current_time = self.time_provider() - self.start_time
                        log_entry = ProtoLogger.create_log_entry(proto, current_time)

                        content = bytes(log_entry, encoding="utf-8")
                        ProtoLogger.write_to_logfile(self.log_file, content)

                        # Stop writing to this chunk if we've reached the max size
                        size = os.fstat(self.log_file.fileno()).st_size
                        if size > REPLAY_MAX_CHUNK_SIZE_BYTES:
                            break

        except Exception:
            logging.exception("Exception detected in ProtoLogger")

    @staticmethod
    def write_to_logfile(log_file: gzip.GzipFile, data:bytes):
        """
        Writing to log file
        
        :param log_file: the log file being written 
        :param data: the binary data that is going to be written
        """
        log_file.write(data)

    @staticmethod
    def create_log_entry(proto: Message, current_time: float) -> str:
        serialized_proto = base64.b64encode(proto.SerializeToString())
        log_entry = (
            f"{current_time}{REPLAY_METADATA_DELIMETER}"
            + f"{proto.DESCRIPTOR.full_name}{REPLAY_METADATA_DELIMETER}"
            + f"{serialized_proto}\n"
        )
        return log_entry
