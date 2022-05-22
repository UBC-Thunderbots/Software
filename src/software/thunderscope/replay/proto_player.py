import time
import threading
import base64
import os
import logging
import gzip
import glob
import proto
from proto.import_all_protos import *
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from software.thunderscope.replay.replay_constants import *
from software.thunderscope.replay.proto_logger import ProtoLogger


class ProtoPlayer(object):

    """Plays back a proto log folder. All the playback is handled by a worker
    thread running in the background.
        
        
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
        
    The player will load chunks in order and play them back at the given playback
    speed. If the seek function is called with a specific time, the player will
    update the 3 variables (shown above) to point to the chunk and entry (in the
    chunk) that contains the data at that time and continue playing from there.

    """

    def __init__(self, log_folder_path, proto_unix_io):
        """Creates a proto player that plays back all protos

        :param log_folder_path: The path to the log file.
        :param proto_unix_io: The proto_unix_io to send the protos to.
            
        """
        self.log_folder_path = log_folder_path
        self.proto_unix_io = proto_unix_io
        self.current_packet_time = 0.0
        self.end_time = 0

        # Don't continue setting things up if the log folder was not provided
        if self.log_folder_path is None:
            return

        # Playback controls, see __play_protobufs
        self.is_playing = True
        self.playback_speed = 1.0
        self.replay_controls_mutex = threading.RLock()
        self.seek_offset_time = 0.0

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

        # We can get the total runtime of the log from the last entry in the last chunk
        last_chunk_data = ProtoPlayer.load_replay_chunk(self.sorted_chunks[-1])
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

        # Convert string to type. eval is an order of magnitude
        # faster than iterating over the protobuf library to find
        # the type from the string.
        proto_class = eval(str(protobuf_type.split(b".")[1], encoding="utf-8"))

        # Deserialize protobuf
        proto = proto_class.FromString(base64.b64decode(data[len("b") : -len("\n")]))

        return float(timestamp), proto_class, proto

    def save_clip(self, filename, start_time, end_time):
        """Saves clip

        :param filename: The file to save to
        :param start_time: the start time for the clip
        :param end_time: the end time for the clip
    
        """
        if not filename:
            print("No filename selected")
            return
        if start_time >= end_time:
            print("Start time not less than end time")
            return

        logging.info(f"Saving clip from {start_time} to {end_time} to {filename}")

        directory = filename
        if "." in filename:
            directory = filename[: filename.rfind(".")]
        try:
            os.makedirs(directory)
        except OSError:
            pass

        replay_index = 0

        self.seek(start_time)

        while self.current_packet_time <= end_time:
            with gzip.open(
                f"{directory}/{replay_index}.{REPLAY_FILE_EXTENSION}", "wb"
            ) as log_file:
                logging.info(
                    f"Writing to {log_file.name} starting at {self.current_packet_time}"
                )
                while self.current_entry_index < len(self.current_chunk):
                    (
                        self.current_packet_time,
                        _,
                        proto,
                    ) = ProtoPlayer.unpack_log_entry(
                        self.current_chunk[self.current_entry_index]
                    )

                    current_time = self.current_packet_time - start_time
                    log_entry = ProtoLogger.create_log_entry(proto, current_time)
                    log_file.write(bytes(log_entry, encoding="utf-8"))
                    self.current_entry_index += 1

                # Load the next chunk
                self.current_chunk_index += 1
                replay_index += 1

                if self.current_chunk_index < len(self.sorted_chunks):
                    self.current_chunk = ProtoPlayer.load_replay_chunk(
                        self.sorted_chunks[self.current_chunk_index]
                    )
                    self.current_entry_index = 0

        logging.info("Clip saved!")

    def play(self):
        """Plays back the log file."""

        # Protection from spamming the play button
        if self.is_playing:
            return

        with self.replay_controls_mutex:
            self.start_playback_time = time.time()
            self.is_playing = True

    def pause(self):
        """Pauses the player."""

        with self.replay_controls_mutex:
            self.is_playing = False
            self.seek_offset_time = self.current_packet_time

    def toggle_play_pause(self):
        """Toggles the play/pause state."""

        with self.replay_controls_mutex:
            if not self.is_playing:
                self.play()
            else:
                self.pause()

    def set_playback_speed(self, speed):
        """Sets the playback speed.

        :param speed: The speed to set the playback to.

        """
        with self.replay_controls_mutex:
            self.pause()
            self.playback_speed = 1.0 / float(speed)
            self.play()

    def single_step_forward(self):
        """Steps the player forward by one log entry
        """
        self.pause()
        self.current_entry_index = self.current_entry_index + 1
        self.current_chunk_index = self.current_chunk_index
        if self.current_entry_index >= len(self.current_chunk):
            # handle case that player needs to step forward to the next chunk
            self.current_chunk_index += 1
            if self.current_chunk_index >= len(self.sorted_chunks):
                # do not step forwards beyond the end of the replay
                self.current_chunk_index = len(self.sorted_chunks) - 1
                self.current_entry_index = len(self.current_chunk) - 1
            else:
                # adjust log entry index and fetch the right chunk
                self.current_entry_index -= len(self.current_chunk)
                self.current_chunk = ProtoPlayer.load_replay_chunk(
                    self.sorted_chunks[self.current_chunk_index]
                )

        logging.info(
            "Stepped to chunk {} at index {} with timestamp {:.2f}".format(
                self.current_chunk_index,
                self.current_entry_index,
                self.seek_offset_time,
            )
        )

    def seek(self, seek_time):
        """Seeks to a specific time. We binary search through the chunks
        to find the chunk that would contain the data at the given time.

        We then binary search the entries in the chunk to find the entry
        cloest to the given time.

        We then set the current_chunk_index and current_entry_index to
        the correct values. We also update the seek_offset_time to help
        with realtime playback timing calculations in the worker thread.

        :param seek_time: The time to seek to.

        """
        # Let's binary search through the chunks to find the chunk that starts
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

        # Let's binary search through the entries in the chunk to find the closest
        # timestamp to seek to
        def __bisect_entries_by_timestamp(entry):
            timestamp, _, _ = ProtoPlayer.unpack_log_entry(entry)
            return timestamp

        with self.replay_controls_mutex:

            # Load the chunk that would have the entry
            self.current_chunk = ProtoPlayer.load_replay_chunk(
                self.sorted_chunks[self.current_chunk_index]
            )

            # Search through the chunk to find the entry that is closest to
            # the seek_time
            self.current_entry_index = ProtoPlayer.binary_search(
                self.current_chunk, seek_time, key=__bisect_entries_by_timestamp
            )

            # Update the seek_offset_time and current_packet_time
            # to the one we just found.
            self.seek_offset_time = __bisect_entries_by_timestamp(
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

        Stolen from: https://www.geeksforgeeks.org/python-program-for-binary-search/

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
            - Play/Pause through self.is_playing
            - Seek to a specific time through self.current_chunk and self.current_entry_index
            - Set playback speed through self.playback_speed

        """
        self.time_elapsed = 0.0
        self.start_playback_time = time.time()

        while True:

            # Only play if we are playing
            if not self.is_playing:
                time.sleep(PLAY_PAUSE_POLL_INTERVAL_SECONDS)
                continue

            # Check if replay has ended and stop playing if so
            if self.current_chunk_index >= len(self.sorted_chunks):
                self.is_playing = False
                continue

            # Playback loop to play current chunk
            while self.is_playing and self.current_entry_index < len(
                self.current_chunk
            ):

                with self.replay_controls_mutex:

                    # Unpack the current entry in the chunk
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
                        (time.time() - self.start_playback_time) / self.playback_speed
                    )

                    if self.current_packet_time > time_elapsed:
                        time.sleep(self.current_packet_time - time_elapsed)

                # Send protobuf
                self.proto_unix_io.send_proto(proto_class, proto)

            # Load the next chunk
            with self.replay_controls_mutex:

                if self.is_playing:
                    self.current_chunk_index += 1

                    if self.current_chunk_index < len(self.sorted_chunks):
                        self.current_chunk = ProtoPlayer.load_replay_chunk(
                            self.sorted_chunks[self.current_chunk_index]
                        )
                        self.current_entry_index = 0
