import time
import threading
import  queue
import base64
import os
import logging
import bz2
import proto
import importlib
import inspect
import glob
from proto.repeated_any_msg_pb2 import RepeatedAnyMsg
from google.protobuf.any_pb2 import Any

REPLAY_METADATA_DELIMETER = b"!#!"
REPLAY_FILE_EXTENSION = ".replay"
REPLAY_FILE_TIME_FORMAT = "%Y-%m-%d-%H-%M-%S"
PROTOBUF_BUFFER_SIZE = 1024 * 1024 # 1MB

class ProtoLogger(object):

    """Logs incoming protobufs to a file. """

    def __init__(self, log_path, log_prefix="log"):
        """Creates a proto logger that logs all protos
        registered on the queue.

        Stores the file to
            log_path/log_prefix-YYYY-MM-DD-HH-MM-SS.replay

        :param log_path: The path to the log file.
        :param log_prefix: The prefix to use for the log file.

        """
        try:
            os.makedirs(log_path)
        except OSError:
            pass

        self.log_path = log_path + "/" + log_prefix + "-"\
                + time.strftime(REPLAY_FILE_TIME_FORMAT)\
                + REPLAY_FILE_EXTENSION

        self.buffer = queue.Queue(PROTOBUF_BUFFER_SIZE)
        self.start_time = time.time()
        self.stop_logging = False


    def __enter__(self):
        """Starts the logger.

        We use bz2 to save the data with compression enabled to
        save a _lot_ of space.

        """

        self.log_file = bz2.open(self.log_path, 'wb')
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
        self.log_file.close()

    
    def __log_protobufs(self):
        """Logs all protos in the queue. 

        Stores it in the format: where !#! is the delimiter.

            timestamp!#!full_name!#!size!#!protobuf

        """
        try:
            while self.stop_logging is False:

                try:
                    proto = self.buffer.get(block=True)
                except queue.Empty:
                    continue

                serialized_proto = base64.b64encode(proto.SerializeToString())
                current_time = time.time() - self.start_time

                log_entry =\
                        f"{current_time}{REPLAY_METADATA_DELIMETER}" +\
                        f"{proto.DESCRIPTOR.full_name}{REPLAY_METADATA_DELIMETER}" +\
                        f"{serialized_proto}\n"

                self.log_file.write(bytes(log_entry, encoding="utf-8"))

        except Exception as e:
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

    def __enter__(self):
        """Starts the player.

        We use bz2 to save the data with compression enabled to
        save a _lot_ of space. So we need to user bz2 to read
        the file.

        """

        self.log_file = bz2.open(self.log_path, 'rb')
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
            start_playback_time = time.time()

            while self.exit_requested is False:
                while self.stop_playing is False:
                    bytes_retrieved = self.log_file.readline()
                    timestamp, protobuf_type, data = bytes_retrieved.split(REPLAY_METADATA_DELIMETER)
                    proto_class = self.find_proto_class(str(protobuf_type, encoding="utf-8"))
                    proto = proto_class.FromString(base64.b64decode(data[len('b'):-len('\n')]))
                    self.proto_unix_io.send_proto(proto_class, proto)

        except Exception as e:
            logging.exception("Exception detected in ProtoPlayer")
