import time
import threading
import  queue
import os
from proto.repeated_any_msg_pb2 import RepeatedAnyMsg

REPLAY_METADATA_DELIMETER = "!#!"
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
        """Opens the log file. """

        self.log_file = open(self.log_path, 'wb', buffering=PROTOBUF_BUFFER_SIZE)
        self.thread = threading.Thread(target=self.__log_protobufs, daemon=True)
        self.thread.start()
        print("HELLO WORLD")

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

            timestamp!#!size!#!protobuf

        """

        while self.stop_logging is False:

            proto = self.buffer.get(block=True)
            serialized_proto = proto.SerializeToString()
            size = len(serialized_proto)
            current_time = time.time() - self.start_time

            log_entry =\
                    f"{current_time}{REPLAY_METADATA_DELIMETER}" +\
                    f"{size}{REPLAY_METADATA_DELIMETER}{serialized_proto}"

            self.log_file.write(log_entry + "\n")

