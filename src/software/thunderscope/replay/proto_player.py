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
from software.thunderscope.proto_unix_io import ProtoUnixIO
from datetime  import datetime
from google.protobuf.message import DecodeError, Message
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
        if self.is_from_field_test():
            print("Processing replay files")
            self.convert_field_test_replayfiles()

        # We can get the total runtime of the log from the last entry in the last chunk
        self.end_time = self.find_actual_endtime()

        logging.info(
            "Loaded log file with total runtime of {:.2f} seconds".format(self.end_time)
        )

        # Start playing thread
        self.seek(0.0)
        self.thread = threading.Thread(target=self.__play_protobufs, daemon=True)
        self.thread.start()

    def sort_and_get_replay_files(self, log_folder_path):
        """
        Sorting the replay files 

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
    def is_valid_protobuf(message):
        """
        True if the time send is greater than 2023 unix timestamp, and the protobuf have the timestamp field

        :return: True if the time send is greater than 2023 unix timestamp, and the protobuf have the timestamp field
        """
        unixtimestamp_2023 = datetime(year=2023, month=1, day=1).timestamp()

        try:
            if message.time_sent.epoch_timestamp_seconds > unixtimestamp_2023:
                # has the time send field and is greater than 2023 unix timestamp
                return True

            # return false where the timestamp is way to small!
            return False

        # does not contain the time_sent field, so we have a type error
        except AttributeError:
            return False

    def convert_field_test_replayfiles(self):
        """
        This is a four step operation!

        1. load all the replay files and their protobuf into memory while discarding protobufs.
        that does not meet requirements given by the self.is_valid_protobuf function.
        2. delete the protobufs that are in the self.log_folder.
        3. reindex the timestamp so it starts from 0 and write the new replay file to self.log_folder.
        4. resort all the chunks.
        """
        messages_that_has_timestamp = []

        # load all the protobufs into memory
        for file in os.listdir(self.log_folder_path):
            path_to_file = os.path.join(self.log_folder_path, file)
            entries = ProtoPlayer.load_replay_chunk(path_to_file)

            for entry in entries:
                _, _, message = ProtoPlayer.unpack_log_entry(entry)

                if ProtoPlayer.is_valid_protobuf(message):
                    messages_that_has_timestamp.append(message)

        # deleting all the replay files since we are writing new replay files
        for file in os.listdir(self.log_folder_path):
            path_to_file = os.path.join(self.log_folder_path, file)
            try:
                os.remove(path_to_file)
            except OSError:
                print("cannot delete file: {}".format(file))
                print("we may not be able to replay this file!")

        # sort the message as the protobuf may not be in chronological order!
        messages_that_has_timestamp = sorted(messages_that_has_timestamp, key=lambda x: x.time_sent.epoch_timestamp_seconds)

        smallest_timestamp = messages_that_has_timestamp[0].time_sent.epoch_timestamp_seconds
        # creating a log file
        with gzip.open(os.path.join(self.log_folder_path, "0.replay"), "wb") as log_file:
            # creating a logfile
            for message in messages_that_has_timestamp:
                # reset timestamp to be relative to when the game is started
                current_time = message.time_sent.epoch_timestamp_seconds - smallest_timestamp

                log_entry = ProtoLogger.create_log_entry(message, current_time)
                data = bytes(log_entry, encoding='utf-8')

                ProtoLogger.write_to_logfile(log_file, data)

        self.sorted_chunks = self.sort_and_get_replay_files(self.log_folder_path)


    def is_from_field_test(self):
        """
        Checking to see if they are 
        This is done so by checking if the last 10 time stamp is greater than the unix timestamp for 2023.
        We know that field testing replay files timestamp is the actual unix timestamp, not the timestamp relative 
        to when the user opens Thunderscope

        :return: True if the replay files came from field test, False if the replay files came 
        from simulated test
        """
        unixtimestamp_2023 = datetime(year=2023, month=1, day=1).timestamp()

        is_from_field_test = False
        loaded_chunk = ProtoPlayer.load_replay_chunk(self.sorted_chunks[-1])

        # iterating over the last 10 log entries that are valid
        for log_entry in loaded_chunk[-10:]:
            try:
                timestamp, _, _ = ProtoPlayer.unpack_log_entry(log_entry)
                if timestamp > unixtimestamp_2023:
                    is_from_field_test = True

            # we have an exception here because the log entries may not all be valid!
            # there may have been a file corruption somewhere causing error!
            except DecodeError: 
                pass

        return is_from_field_test

    def find_actual_endtime(self):
        """
        Finding the last end time.
        Note that the end time may not necessarily be the last message in the last chunks since there may be 
        file corruptions. We also assume a chronological order in the chunks data!
        
        :return: the last end time, if noe end time are found, return 0.0s
        """
        # reverse iterating over the chunks (file)
        for i in reversed(range(len(self.sorted_chunks))):
            last_chunk_data = ProtoPlayer.load_replay_chunk(self.sorted_chunks[i])

            # reverse iterating the protobufs message in each and every file
            for j in reversed(range(len(last_chunk_data))):
                try:
                    end_time, _, _ = ProtoPlayer.unpack_log_entry(last_chunk_data[j])
                    return end_time
                except Exception:
                    pass

        return 0.0

    @staticmethod
    def load_replay_chunk(replay_chunk_path: str) -> list:
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
    def unpack_log_entry(log_entry: str) -> (float, Type[Message], Message):
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
        try:
            # The format of the protobuf type is:
            # package.proto_class (e.g. TbotsProto.Primitive)
            proto_class = eval(str(protobuf_type.split(b".")[-1], encoding="utf-8"))
        except NameError:
            raise TypeError(f"Unknown proto type in replay: '{protobuf_type}'")

        # Deserialize protobuf
        proto = proto_class.FromString(base64.b64decode(data[len("b") : -len("\n")]))

        return float(timestamp), proto_class, proto

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
                while self.current_entry_index < len(self.current_chunk):
                    (
                        self.current_packet_time,
                        _,
                        proto,
                    ) = ProtoPlayer.unpack_log_entry(
                        self.current_chunk[self.current_entry_index]
                    )

                    log_entry = ProtoLogger.create_log_entry(
                        proto, self.current_packet_time - start_time
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
                        self.sorted_chunks[self.current_chunk_index]
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

    def set_playback_speed(self, speed: float) -> None:
        """Sets the playback speed.

        :param speed: The speed to set the playback to.

        """
        with self.replay_controls_mutex:
            self.pause()
            self.playback_speed = 1.0 / float(speed)
            self.play()

    def single_step_forward(self) -> None:
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
            chunk = ProtoPlayer.load_replay_chunk(chunk)
            start_timestamp, _, _ = ProtoPlayer.unpack_log_entry(chunk[0])
            return start_timestamp

        with self.replay_controls_mutex:
            self.current_chunk_index = ProtoPlayer.binary_search(
                self.sorted_chunks, seek_time, key=__bisect_chunks_by_timestamp
            )

        # Let's binary search through the entries in the chunk to find the closest
        # timestamp to seek to
        def __bisect_entries_by_timestamp(entry: str) -> float:
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

                    try:
                        # Unpack the current entry in the chunk
                        (
                            self.current_packet_time,
                            proto_class,
                            proto,
                        ) = ProtoPlayer.unpack_log_entry(
                            self.current_chunk[self.current_entry_index]
                        )
                    except ValueError:
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
                            self.sorted_chunks[self.current_chunk_index]
                        )
                        self.current_entry_index = 0
