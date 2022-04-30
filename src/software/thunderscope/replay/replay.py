import time
import threading
import queue
import base64
import bisect
import os
import logging
import gzip
import glob
import proto
from proto.import_all_protos import *
from proto.repeated_any_msg_pb2 import RepeatedAnyMsg
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from google.protobuf.any_pb2 import Any

REPLAY_METADATA_DELIMETER = ","
REPLAY_FILE_EXTENSION = "replay"
REPLAY_FILE_TIME_FORMAT = "%Y_%m_%d_%H_%M_%S"
REPLAY_MAX_CHUNK_SIZE_BYTES = 1024 * 1024 * 1  # 1 MiB

PROTOBUF_BUFFER_SIZE = 1024 * 1024  # 1MB
PLAY_PAUSE_POLL_INTERVAL_SECONDS = 0.1


class ProtoLogger(object):

    """Logs incoming protobufs with metadata to a folder for later replay.

    Each entry will contain
        - The timestamp
        - The protobuf type
        - The protobuf serialized as base64 (to remove newline characters)

    Stored in: log_path_YYYY_MM_DD_HH_MM_SS/
    Each entry formatted as: timestamp,protobuf_type,protobuf_base64

    We need to store the data in a way that we can:
        1. Replay the data chronologoically
        2. Seek to a specific time (random access)

    If we store all the data into a single file, chronologoical playback
    is trivial but it is challenging to seek to a specific time. We need
    to load the log file into memory to search for the entry we want. If the
    replay file is large, it won't fit into memory.

    So we store the data in chunks. Each chunk contains REPLAY_MAX_CHUNK_SIZE_BYTES
    of serialized protos.

    We can load each chunk into memory and search for the entry we want, or
    playback the data chronologically.

    """

    def __init__(self, log_path, time_provider=None):
        """Creates a proto logger that logs all protos registered on the queue.

        Stores the files to
            log_path_YYYY-MM-DD-HH-MM-SS/#.replay

        Where # is the chunk number

        NOTE: The files in the folder will be compressed with gzip.
        I've looked into bz2, but loading chunks into memory was slower.

        Also NOTE: You need the entire folder to replay the data.

        :param log_path: The path to the log file.
        :param time_provider: A function that returns the current time
            in seconds since epoch. Defaults to time.time

        """
        self.log_folder = (
            log_path + "/protolog_" + time.strftime(REPLAY_FILE_TIME_FORMAT) + "/"
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

                        size = os.fstat(self.log_file.fileno()).st_size
                        if size >= REPLAY_MAX_CHUNK_SIZE_BYTES:
                            break

        except Exception:
            logging.exception("Exception detected in ProtoLogger")


class ProtoPlayer(object):

    """Plays back a log file. """

    def __init__(self, log_folder_path, proto_unix_io):
        """Creates a proto player that plays back all protos

        :param log_folder_path: The path to the log file.
        :param proto_unix_io: The proto_unix_io to send the protos to.
            
        """
        self.log_folder_path = log_folder_path
        self.proto_unix_io = proto_unix_io
        self.exit_requested = False

        # Playback controls, see __play_protobufs
        self.stop_playing = False
        self.playback_speed = 1.0
        self.current_chunk_index = 0
        self.current_chunk = []
        self.current_entry_index_in_chunk = 0

    @staticmethod
    def load_replay_chunk(replay_chunk_path):
        """Reads a replay chunk.

        :param replay_chunk_path: The path to the replay chunk.
        :return: The replay chunk. List of log entries

        """
        cached_data = []

        # Load chunk into memory
        with gzip.open(replay_chunk_path, "rb") as log_file:
            while True:
                try:
                    line = log_file.readline()
                    if not line:
                        break
                    cached_data.append(line)
                except EOFError:
                    break

        return cached_data

    @staticmethod
    def unpack_log_entry(log_entry):
        """Unpacks a log entry into the timestamp and proto.

        :param log_entry: The log entry.
        :return: The timestamp, proto_class, deserialized protobuf

        """

        # Unpack metadata
        timestamp, protobuf_type, data = log_entry.split(bytes(REPLAY_METADATA_DELIMETER, encoding="utf-8"))

        # Convert string to type. eval is an order of mangnitude
        # faster than iterating over the protobuf library to find
        # the type from the string.
        proto_class = eval(
            str(protobuf_type.split(b".")[1], encoding="utf-8")
        )

        # Deserialize protobuf
        proto = proto_class.FromString(
            base64.b64decode(data[len("b") : -len("\n")])
        )

        return float(timestamp), proto_class, proto


    def __enter__(self):
        """Starts the player.
        """

        # Load up all replay files in the log folder
        replay_files = glob.glob(self.log_folder_path + f"/*.{REPLAY_FILE_EXTENSION}")

        # Sort the files by their chunk index
        def __sort_replay_chunks(file_path):
            head, tail = os.path.split(file_path)
            replay_index, _ = tail.split(".")
            return int(replay_index)

        self.sorted_chunks = sorted(replay_files, key=__sort_replay_chunks)
        last_chunk_data = ProtoPlayer.load_replay_chunk(self.sorted_chunks[-1])

        # Unpack metadata
        self.end_time, _, _ = ProtoPlayer.unpack_log_entry(last_chunk_data[-1])
        logging.info(
            "Loaded log file with total runtime of {:.2f} seconds".format(self.end_time)
        )

        # Start playing thread
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

    def play(self):
        self.stop_playing = False

    def pause(self):
        self.stop_playing = True

    def set_playback_speed(self, speed):
        self.playback_speed = speed

    def seek(self, time):
        """Seeks to a specific time.
        """
        self.current_chunk_index = 0

        # Lets binary search through the chunks to find the chunk that starts
        # with a timestamp less than but closest to the time we want to seek to
        def __bisect_chunks_by_timestamp(chunk):
            chunk = ProtoPlayer.load_replay_chunk(chunk)
            start_timestamp, _, _ = ProtoPlayer.unpack_log_entry(chunk[0])
            return start_timestamp

        seeked_chunk = bisect.bisect(self.sorted_chunks, time, key=__bisect_chunks_by_timestamp)

        # Lets binary search through the entries in the chunk to find the closest
        # timestamp to seek to
        def __bisect_log_entries_by_timestamp(entry):
            timestamp, _, _ = ProtoPlayer.unpack_log_entry(entry)
            return timestamp

        seeked_entry_in_chunk = bisect.bisect(seeked_chunk, time, key=__bisect_chunks_by_timestamp)

        print("FOUND: ", seeked_chunk, seeked_entry_in_chunk)


    def __play_protobufs(self):
        """Plays all protos in the file in chronologoical order. 

        Playback controls:
            - Play/Pause through self.stop_playing
            - Seek to a specific time through self.current_chunk and self.current_entry_index_in_chunk
            - Set playback speed through self.playback_speed

        """
        try:
            start_playback_time = time.time()

            while self.exit_requested is False:

                self.seek(0.5)
                self.seek(0.01)
                self.seek(0.68)
                self.seek(0.96)
                self.seek(15.0)
                self.seek(15.1)

                # Check if we've reached the end
                if self.current_chunk >= len(self.sorted_chunks):
                    logging.info("Replay ended.")
                    return

                logging.info("Loading chunk: {}".format(self.current_chunk_index))
                self.current_chunk = ProtoPlayer.load_replay_chunk(
                    self.sorted_chunks[self.current_chunk_index]
                )

                self.current_chunk_index += 1
                self.current_entry_index_in_chunk = 0

                while (
                    self.stop_playing is False
                    and self.current_entry_index_in_chunk < len(chunk_cache)
                ):
                    replay_current_packet_time, proto_class, proto =\
                            ProtoPlayer.unpack_log_entry(self.current_chunk[self.current_entry_index_in_chunk])

                    self.current_entry_index_in_chunk += 1

                    # Manage playback speed, if this packet needs to be sent
                    # out at a later time, sleep until its time to send it.
                    time_elapsed = (
                        time.time() - start_playback_time
                    ) / self.playback_speed

                    if replay_current_packet_time > time_elapsed:
                        time.sleep(replay_current_packet_time - time_elapsed)
                        time_elapsed = time.time() - start_playback_time

                    # Send protobuf
                    self.proto_unix_io.send_proto(proto_class, proto)

        except Exception:
            logging.exception("Exception detected in ProtoPlayer")
