import logging
import time
import threading
import base64
import os
import gzip
import glob
from proto.import_all_protos import *
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from software.py_constants import *

from software.thunderscope.constants import ProtoPlayerFlags
from software.thunderscope.proto_unix_io import ProtoUnixIO
import software.python_bindings as tbots_cpp
from google.protobuf.message import Message
from typing import Callable, Type


class ProtoPlayer:
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

    PLAY_PAUSE_POLL_INTERVAL_SECONDS = 0.1
    CHUNK_INDEX_FILENAME = 'chunks.index'

    def __init__(self, log_folder_path: str, proto_unix_io: ProtoUnixIO) -> None:
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

        self.sorted_chunks = self.sort_and_get_replay_files(self.log_folder_path)

        self.version = ProtoPlayer.get_replay_chunk_format_version(
            self.sorted_chunks[0]
        )

        # build or load index for chunks
        self.chunks_indices = self.get_chunk_index()

        # We can get the total runtime of the log from the last entry in the last chunk
        self.end_time = self.find_actual_endtime()

        logging.info(
            "Loaded log file with total runtime of {:.2f} seconds".format(self.end_time)
        )

        # Start playing thread
        self.seek(0.0)
        self.thread = threading.Thread(
            target=self.__play_protobufs_wrapper, daemon=True
        )
        self.thread.start()

        self.error_bit_flag = ProtoPlayerFlags.NO_ERROR_FLAG

    @staticmethod
    def sort_and_get_replay_files(log_folder_path):
        """Sorting the replay files

        :param log_folder_path: the path to the folder that we are going to be sorting!
        :return: the sorted replay files
        """
        # Load up all replay files in the log folder
        replay_files = glob.glob(log_folder_path + f"/*.{REPLAY_FILE_EXTENSION}")

        if len(replay_files) == 0:
            raise ValueError(
                f'No replay files found in "{log_folder_path}", make sure that an absolute path '
                f"to the folder containing the replay files is provided."
            )

        # Sort the files by their chunk index
        def __sort_replay_chunks(file_path: str):
            head, tail = os.path.split(file_path)
            replay_index, _ = tail.split(".")
            return int(replay_index)

        return sorted(replay_files, key=__sort_replay_chunks)

    @staticmethod
    def is_log_entry_corrupt(log_entry: bytes, version: int) -> bool:
        """Check to see if we have can unpack the log entry

        :param log_entry: the log entry we are checking
        :param version: the version of the replay file
        :return: False if we could unpack the log entry, True otherwise
        """
        try:
            _ = ProtoPlayer.unpack_log_entry(log_entry, version)
            return False
        except Exception:
            return True

    def build_chunk_index(self) -> dict[str: float]:
        """
        Build the chunk index and store the index into the index file

        NOTE:
        A chunk index entry is a key value pair [filename: start timestamp]
         - Start timestamp: timestamp of the first log entry in the chunk
         - Filename: replay chunk file name (%d.reply)

        :return: a dictionary for mapping replay filename to the start
            timestamp of the first entry in the chunk.
        """
        chunk_indices: dict[str: float] = dict()
        for chunk_name in self.sorted_chunks:
            chunk_data = ProtoPlayer.load_replay_chunk(
                chunk_name, self.version
            )
            if chunk_data:
                start_timestamp, _, _ = ProtoPlayer.unpack_log_entry(chunk_data[0], self.version)
                chunk_indices[os.path.basename(chunk_name)] = start_timestamp
        if chunk_indices:
            with open(os.path.join(self.log_folder_path, ProtoPlayer.CHUNK_INDEX_FILENAME), 'w') as index_file:
                index_file.write(f'Generated on {time.time()}\n')
                for key, value in chunk_indices.items():
                    index_file.write(f'{value}, {key}\n')
            logging.info('Created chunk index file successfully.')
        else:
            logging.warning(f'Failed to build chunk index for {self.log_folder_path}')
        return chunk_indices


    def is_chunk_indexed(self) -> bool:
        """
        Returns true if the chunk index is already built.

        :return: if the chunk index file exists
        """
        return os.path.exists(os.path.join(self.log_folder_path, ProtoPlayer.CHUNK_INDEX_FILENAME))

    def load_chunk_index(self) -> dict[str: float]:
        """
        Loads the chunk index file.

        :return: the chunk indices.
        """
        chunk_indices: dict[str: float] = dict()
        try:
            with open(os.path.join(self.log_folder_path, ProtoPlayer.CHUNK_INDEX_FILENAME), 'r') as index_file:
                # skip the first timestamp line
                index_file.readline()

                for line in index_file:
                    start_timestamp, chunk_name = line.split(',')
                    start_timestamp = float(start_timestamp)
                    chunk_name = chunk_name.strip()
                    chunk_indices[chunk_name] = start_timestamp
            logging.info("Pre-existing chunk index file found and loaded.")
        except Exception as e:
            logging.warning(f"An Exception occurred when loading chunk index file {e}")
        return chunk_indices

    def get_chunk_index(self) -> dict[str: float]:
        """
        Returns the chunk indices.
        NOTE: if the chunk index was not built, this function will automatically build one.

        :return: chunk indices.
        """
        if not self.is_chunk_indexed():
            return self.build_chunk_index()
        else:
            try:
                return self.load_chunk_index()
            except Exception as e:
                logging.log(f"Exception occurred when loading chunk index, trying to rebuild. Message: {e}")
                return self.build_chunk_index()


    def is_proto_player_playing(self) -> bool:
        """Return whether or not the proto player is being played.

        :return: True if the proto player is playing, False otherwise.
        """
        return self.is_playing

    def find_actual_endtime(self) -> float:
        """Finding the last end time.
        Note that the end time may not necessarily be the last message in the last chunks since there may be
        file corruptions. We also assume a chronological order in the chunks data!

        :return: the last end time, if no end time are found, return 0.0s
        """
        # reverse iterating over the chunks (file)
        for i in reversed(range(len(self.sorted_chunks))):
            last_chunk_data = ProtoPlayer.load_replay_chunk(
                self.sorted_chunks[i], self.version
            )

            # reverse iterating the protobufs message in each and every file
            for j in reversed(range(len(last_chunk_data))):
                try:
                    end_time, _, _ = ProtoPlayer.unpack_log_entry(
                        last_chunk_data[j], self.version
                    )
                    return end_time
                except Exception:
                    pass

        return 0.0

    @staticmethod
    def load_replay_chunk(replay_chunk_path: str, version: int) -> list:
        """Reads a replay chunk.

        :param replay_chunk_path: The path to the replay chunk.
        :param version: The format version of the replay file
        :return: The replay chunk. List of log entries
        """
        cached_data = []

        # Load chunk into memory
        with gzip.open(replay_chunk_path, "rb") as log_file:
            # Starting version 2, the first line of the chunk contains
            # the replay file version
            if version >= 2:
                log_file.readline()

            while True:
                try:
                    line = log_file.readline()
                    if not line:
                        break

                    if not ProtoPlayer.is_log_entry_corrupt(line, version):
                        cached_data.append(line)
                    else:
                        logging.warning(
                            "There are log entries that are corrupted. Entries ignored!"
                        )
                except EOFError:
                    break

                except Exception as e:
                    logging.warning(
                        f"An unknown exception has occurred while reading {replay_chunk_path}: {e}"
                    )

        return cached_data

    @staticmethod
    def get_replay_chunk_format_version(replay_chunk_path: str) -> int:
        """Reads a replay chunk.

        :param replay_chunk_path: The path to the replay chunk.
        :return: The format version of the replay file
        """
        # Default to version 1
        file_version = 1

        # Starting version 2, the first line of the chunk should be
        # the replay file version
        with gzip.open(replay_chunk_path, "rb") as log_file:
            try:
                line = log_file.readline()
                file_version_prefix_bytes = bytes(
                    REPLAY_FILE_VERSION_PREFIX, encoding="utf-8"
                )
                if line is not None and line.startswith(file_version_prefix_bytes):
                    file_version = int(line.split(file_version_prefix_bytes)[1])
                else:
                    print(f"Could not find version in {replay_chunk_path}")
            except EOFError:
                pass

        return file_version

    @staticmethod
    def unpack_log_entry(
        log_entry: bytes, version: int
    ) -> (float, Type[Message], Message):
        """Unpacks a log entry into the timestamp and proto.

        :param log_entry: The log entry.
        :param version: The format version of the replay file
        :return: The timestamp, proto_class, deserialized protobuf
        """
        # Unpack metadata
        timestamp, protobuf_type, data = log_entry.split(
            bytes(REPLAY_METADATA_DELIMITER, encoding="utf-8")
        )

        # Convert string to type. eval is an order of magnitude
        # faster than iterating over the protobuf library to find
        # the type from the string.
        try:
            # The format of the protobuf type is:
            # package.proto_class (e.g. TbotsProto.Primitive)
            proto_class = eval(str(protobuf_type.split(b".")[-1], encoding="utf-8"))
        except NameError:
            raise TypeError(f"Unknown proto type in replay: '{protobuf_type}'")

        # Deserialize protobuf
        if version == 1:
            deserialized_proto = proto_class.FromString(
                base64.b64decode(data[len("b") : -len("\n")])
            )
        elif version == 2:
            deserialized_proto = proto_class.FromString(
                base64.b64decode(data[: -len("\n")])
            )
        else:
            raise ValueError(f"Unknown replay file version: {version}")

        return float(timestamp), proto_class, deserialized_proto

    def save_clip(self, filename: str, start_time: float, end_time: float) -> None:
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

        while True:
            with gzip.open(
                f"{directory}/{replay_index}.{REPLAY_FILE_EXTENSION}", "wb"
            ) as log_file:
                logging.info(
                    f"Writing to {log_file.name} starting at {self.current_packet_time}"
                )

                # Save all clips with the latest replay format version
                log_file.write(
                    bytes(
                        REPLAY_FILE_VERSION_PREFIX + str(REPLAY_FILE_VERSION) + "\n",
                        encoding="utf-8",
                    )
                )

                while self.current_entry_index < len(self.current_chunk):
                    (
                        self.current_packet_time,
                        _,
                        proto,
                    ) = ProtoPlayer.unpack_log_entry(
                        self.current_chunk[self.current_entry_index], self.version
                    )

                    log_entry = tbots_cpp.ProtoLogger.createLogEntry(
                        proto.DESCRIPTOR.full_name,
                        proto.SerializeToString(),
                        self.current_packet_time - start_time,
                    )
                    log_file.write(bytes(log_entry, encoding="utf-8"))
                    self.current_entry_index += 1
                    if self.current_packet_time >= end_time:
                        logging.info("Clip saved!")
                        return
                # Load the next chunk
                self.current_chunk_index += 1
                replay_index += 1

                if self.current_chunk_index < len(self.sorted_chunks):
                    self.current_chunk = ProtoPlayer.load_replay_chunk(
                        self.sorted_chunks[self.current_chunk_index], self.version
                    )
                    self.current_entry_index = 0

    def play(self) -> None:
        """Plays back the log file."""
        # Protection from spamming the play button
        if self.is_playing:
            return

        with self.replay_controls_mutex:
            self.start_playback_time = time.time()
            self.is_playing = True

    def pause(self) -> None:
        """Pauses the player."""
        with self.replay_controls_mutex:
            self.is_playing = False
            self.seek_offset_time = self.current_packet_time

    def toggle_play_pause(self) -> None:
        """Toggles the play/pause state."""
        with self.replay_controls_mutex:
            if not self.is_playing:
                self.play()
            else:
                self.pause()

    def set_playback_speed(self, speed: str) -> None:
        """Sets the playback speed.

        :param speed: The speed to set the playback to.
        """
        with self.replay_controls_mutex:
            self.pause()
            self.playback_speed = 1.0 / float(speed)
            self.play()

    def single_step_forward(self) -> None:
        """Steps the player forward by one log entry"""
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
                    self.sorted_chunks[self.current_chunk_index], self.version
                )

        logging.info(
            "Stepped to chunk {} at index {} with timestamp {:.2f}".format(
                self.current_chunk_index,
                self.current_entry_index,
                self.seek_offset_time,
            )
        )

    def seek(self, seek_time: float) -> None:
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
        def __bisect_chunks_by_timestamp(chunk: str) -> None:
            if self.chunks_indices and os.path.basename(chunk) in self.chunks_indices:
                return self.chunks_indices[os.path.basename(chunk)]
            else:
                logging.warning(
                   f"Use old algorithm to jump, which might result in lagging " +
                   f"Please try deleting {ProtoPlayer.CHUNK_INDEX_FILENAME} in the replay file folder" +
                   f" and re-run Thunderscope to enable the indexing for faster speed!"
                )
                chunk = ProtoPlayer.load_replay_chunk(chunk, self.version)
                start_timestamp, _, _ = ProtoPlayer.unpack_log_entry(chunk[0], self.version)
                return start_timestamp

        with self.replay_controls_mutex:
            self.current_chunk_index = ProtoPlayer.binary_search(
                self.sorted_chunks, seek_time, key=__bisect_chunks_by_timestamp
            )

        # Let's binary search through the entries in the chunk to find the closest
        # timestamp to seek to
        def __bisect_entries_by_timestamp(entry: str) -> float:
            timestamp, _, _ = ProtoPlayer.unpack_log_entry(entry, self.version)
            return timestamp

        with self.replay_controls_mutex:
            # Load the chunk that would have the entry
            self.current_chunk = ProtoPlayer.load_replay_chunk(
                self.sorted_chunks[self.current_chunk_index], self.version
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
    def binary_search(arr: list, x, key: Callable = lambda x: x) -> int:
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

    def __play_protobufs_wrapper(self) -> None:
        """This function essentially executes __play_protobufs. However, the intention of this function
        is for testing purposes. __play_protobufs is launched in a different thread, it would be useful to know
        if there are uncaught exceptions. This is then used to test the robustness of the __play_protobufs
        function when dealing with corrupted replay files.

        As such, most of time, this function acts the same as self.__play_protobufs

        :return: None
        """
        try:
            self.__play_protobufs()
        except Exception as e:
            logging.exception(
                "there is an uncaught exception when playing protobufs: {}".format(e)
            )
            # setting the error bit flags
            self.error_bit_flag |= ProtoPlayerFlags.UNCAUGHT_EXCEPTION_FLAG
            self.is_playing = False

    def get_error_bit_flag(self) -> ProtoPlayerFlags:
        """The error bit flags is defined as the following:
            1 if there is an uncaught exception in the code
            0 if success

        :return: the error bit flags.
        """
        return self.error_bit_flag

    def __play_protobufs(self) -> None:
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
                time.sleep(ProtoPlayer.PLAY_PAUSE_POLL_INTERVAL_SECONDS)
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
                    try:
                        # Unpack the current entry in the chunk
                        (
                            self.current_packet_time,
                            proto_class,
                            proto,
                        ) = ProtoPlayer.unpack_log_entry(
                            self.current_chunk[self.current_entry_index], self.version
                        )
                    except Exception:
                        self.current_entry_index += 1
                        logging.error("[ProtoPlayer] Error parsing log entry")
                        continue
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
                            self.sorted_chunks[self.current_chunk_index], self.version
                        )
                        self.current_entry_index = 0
