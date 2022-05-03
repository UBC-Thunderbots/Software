import time
import threading
import queue
import base64
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
REPLAY_FILE_TIME_FORMAT = "%Y_%m_%d_%H%M%S"
REPLAY_MAX_CHUNK_SIZE_BYTES = 1024 * 1024 * 1  # 1 MiB

PROTOBUF_BUFFER_SIZE = 1024 * 1024  # 1MB
PLAY_PAUSE_POLL_INTERVAL_SECONDS = 0.1


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


class ProtoPlayer(object):

    """Plays back a log file. All the playback is handled by a worker thread
    running in the background.
        
        
                             current_chunk
                                  │
                                  │
                       ┌─────┐ ┌──▼──┐            ┌─────┐
                       │CHUNK│ │CHUNK│            │CHUNK│
                    ┌──┼─────┼─┼─►   │ ────────── │     │
                    │  │     │ │     │            │     │
                    │  │     │ │     │            │     │
                    │  └─────┘ └─────┘            └─────┘
                    │     0       1                  N
                    │             ▲
                    │             └──current_chunk_index
                    │
             current_entry_index
        
    The player will load chunks in order and play back at the given playback speed.
    If the seek function is called with a specific time, the player will update
    the 3 variables (shown above) to point to the chunk and entry that contains
    the data at that time and continue playing from there.

    """

    def __init__(self, log_folder_path, proto_unix_io):
        """Creates a proto player that plays back all protos

        :param log_folder_path: The path to the log file.
        :param proto_unix_io: The proto_unix_io to send the protos to.
            
        """
        self.log_folder_path = log_folder_path
        self.end_time = 0

        if self.log_folder_path is None:
            return

        self.proto_unix_io = proto_unix_io

        # Playback controls, see __play_protobufs
        self.stop_playing = False
        self.playback_speed = 1.0
        self.replay_controls_mutex = threading.RLock()
        self.seek_offset_time = 0.0
        self.current_packet_time = 0.0

        # Chunk cache and info
        self.current_chunk = []
        self.current_chunk_index = 0
        self.current_entry_index = 0

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
        self.seek(0.0)
        self.thread = threading.Thread(target=self.__play_protobufs, daemon=True)
        self.thread.start()

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
        timestamp, protobuf_type, data = log_entry.split(
            bytes(REPLAY_METADATA_DELIMETER, encoding="utf-8")
        )

        # Convert string to type. eval is an order of mangnitude
        # faster than iterating over the protobuf library to find
        # the type from the string.
        proto_class = eval(str(protobuf_type.split(b".")[1], encoding="utf-8"))

        # Deserialize protobuf
        proto = proto_class.FromString(base64.b64decode(data[len("b") : -len("\n")]))

        return float(timestamp), proto_class, proto

    def play(self):
        """Plays back the log file."""

        if self.stop_playing is False:
            return

        with self.replay_controls_mutex:
            self.playback_time = time.time()
            self.stop_playing = False

    def pause(self):
        """Pauses the player."""

        with self.replay_controls_mutex:
            self.stop_playing = True
            self.seek_offset_time = self.current_packet_time

    def set_playback_speed(self, speed):
        """Sets the playback speed.

        :param speed: The speed to set the playback to.

        """
        with self.replay_controls_mutex:
            self.pause()
            self.playback_speed = 1.0 / float(speed)
            self.play()

    def seek(self, seek_time):
        """Seeks to a specific time.

        :param seek_time: The time to seek to.

        """
        # Lets binary search through the chunks to find the chunk that starts
        # with a timestamp less than (but closest to) the seek_time we want
        # to seek to.
        def __bisect_chunks_by_timestamp(chunk):
            chunk = ProtoPlayer.load_replay_chunk(chunk)
            start_timestamp, _, _ = ProtoPlayer.unpack_log_entry(chunk[0])
            return start_timestamp

        with self.replay_controls_mutex:
            self.current_chunk_index = ProtoPlayer.binary_search(
                self.sorted_chunks, seek_time, key=__bisect_chunks_by_timestamp
            )

        # Lets binary search through the entries in the chunk to find the closest
        # timestamp to seek to
        def __bisect_log_entries_by_timestamp(entry):
            timestamp, _, _ = ProtoPlayer.unpack_log_entry(entry)
            return float(timestamp)

        with self.replay_controls_mutex:

            # Load the chunk that would have the entry
            self.current_chunk = ProtoPlayer.load_replay_chunk(
                self.sorted_chunks[self.current_chunk_index]
            )

            # Search through the chunk to find the entry that is closest to the seek_time
            self.current_entry_index = ProtoPlayer.binary_search(
                self.current_chunk, seek_time, key=__bisect_log_entries_by_timestamp
            )

            self.seek_offset_time = __bisect_log_entries_by_timestamp(
                self.current_chunk[self.current_entry_index]
            )

            self.current_packet_time = self.seek_offset_time

            logging.info(
                "Jumped to chunk {} at index {} with timestamp {:.2f}".format(
                    self.current_chunk_index,
                    self.current_entry_index,
                    self.seek_offset_time,
                )
            )

    @staticmethod
    def binary_search(arr, x, key=lambda x: x):
        """Binary search for an element in an array.

        Adapted from: https://www.geeksforgeeks.org/python-program-for-binary-search/

        :param arr: The array to search.
        :param x: The element to search for.
        :param key: The key to use for sorting.

        """
        low = 0
        high = len(arr) - 1
        mid = 0

        while low <= high:

            mid = (high + low) // 2

            # If x is greater, ignore left half
            if key(arr[mid]) < x:
                low = mid + 1

            # If x is smaller, ignore right half
            elif key(arr[mid]) > x:
                high = mid - 1

            # x is present at mid
            else:
                return mid

        return min(abs(low), abs(high))

    def __play_protobufs(self):
        """Plays all protos in the file in chronologoical order. 

        Playback controls:
            - Play/Pause through self.stop_playing
            - Seek to a specific time through self.current_chunk and self.current_entry_index
            - Set playback speed through self.playback_speed

        """
        self.playback_time = time.time()
        self.time_elapsed = 0.0

        while True:

            # Only play if we are not paused
            if self.stop_playing is True:
                time.sleep(PLAY_PAUSE_POLL_INTERVAL_SECONDS)
                continue

            # Check if replay has ended
            if self.current_chunk_index >= len(self.sorted_chunks):
                self.stop_playing = True
                continue

            # Playback loop
            while self.stop_playing is False and self.current_entry_index < len(
                self.current_chunk
            ):

                with self.replay_controls_mutex:
                    # Get the current entry
                    (
                        self.current_packet_time,
                        proto_class,
                        proto,
                    ) = ProtoPlayer.unpack_log_entry(
                        self.current_chunk[self.current_entry_index]
                    )
                    self.current_entry_index += 1

                    # Manage playback speed, if this packet needs to be sent
                    # out at a later time, sleep until its time to send it.
                    time_elapsed = self.seek_offset_time + (
                        (time.time() - self.playback_time) / self.playback_speed
                    )

                    if self.current_packet_time > time_elapsed:
                        time.sleep(self.current_packet_time - time_elapsed)

                # Send protobuf
                self.proto_unix_io.send_proto(proto_class, proto)

            # Load the next chunk and setup the current_chunk_index
            # and current_entry_index.
            with self.replay_controls_mutex:

                if not self.stop_playing:
                    self.current_chunk_index += 1

                    if self.current_chunk_index < len(self.sorted_chunks):
                        self.current_chunk = ProtoPlayer.load_replay_chunk(
                            self.sorted_chunks[self.current_chunk_index]
                        )
                        self.current_entry_index = 0
