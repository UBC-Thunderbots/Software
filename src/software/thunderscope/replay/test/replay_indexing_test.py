"""
This test is for testing the chunk indexing function of the proto player.
We are going to examine the index-building and index-loading functions.
Those functions are first introduced to optimize the response time of the progress bar in the replay mode.
"""
from typing import Callable
import math
import shutil
import gzip
import os
from google.protobuf.message import Message
from software.py_constants import *

from software.thunderscope.replay.proto_player import ProtoPlayer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.simulated_tests.simulated_test_fixture import pytest_main
from software.thunderscope.replay.test.replay_corruption_test import create_valid_log_entry, create_random_proto

# location to store the generated files
TMP_REPLAY_SAVE_PATH = "/tmp/test_indexing"

# number of chunk files being generated
CHUNK_FILES_NUM = 100

# number of entries per replay file
ENTRY_NUM_PER_FILE = 100

# length of time that every chunk represents
DURATION_PER_CHUNK = 5

def create_test_player() -> ProtoPlayer:
    """
    Create a dummy protoplayer for testing
    :return: a protoplayer for testing
    """
    return ProtoPlayer(TMP_REPLAY_SAVE_PATH, ProtoUnixIO())

def cleanup():
    """Clean up this test by deleting the replay files generated"""
    shutil.rmtree(TMP_REPLAY_SAVE_PATH)

def generate_chunk(
        list_of_protos: [Message],
        save_path: str,
        filename: str,
        duration: float,
        start_time: float,
        gen_log_entry_func: Callable[[Message, float], str]
):
    """
    Generate one replay chunk for testing.

    :param list_of_protos: the list of proto being referenced when generating log entries
    :param save_path: directory for saving the chunk file
    :param filename: the name of the chunk file
    :param duration: the length of the chunk (in seconds)
    :param start_time: the start time (in seconds)
    :param gen_log_entry_func: function used to generate log entries
    """
    os.makedirs(save_path, exist_ok=True)
    path_to_replay_file = os.path.join(save_path, filename)

    with gzip.open(path_to_replay_file, "wb") as log_file:
        for i, proto in enumerate(list_of_protos):
            proto_time = start_time + duration / len(list_of_protos) * i
            log_entry = gen_log_entry_func(proto, proto_time)
            log_file.write(bytes(log_entry, encoding="utf-8"))


def test_for_building_index_on_valid_chunks():
    """
    Test building index file on correctly formatted replay files

    Test steps:
    1. generate correctly formatted replay files (chunks)
    2. Build index files
    3. Load index into memory and validate
    3. Test ProtoPlayer.seek() to check if the player can jump correctly to the correct chunk
    """
    # generate correctly formatted replay files
    replay_proto = []
    for _ in range(ENTRY_NUM_PER_FILE):
        replay_proto.append(create_random_proto())
    for i in range(CHUNK_FILES_NUM):
        generate_chunk(
            replay_proto,
            TMP_REPLAY_SAVE_PATH,
            f"{i}.replay",
            DURATION_PER_CHUNK,
            DURATION_PER_CHUNK * i,
            create_valid_log_entry
        )

    player = create_test_player()

    # generate chunk index file
    player.build_chunk_index(player.log_folder_path)

    # validate index with load index function
    chunk_indices = player.load_chunk_index()
    assert len(chunk_indices) == CHUNK_FILES_NUM
    assert player.chunks_indices
    for filename, start_timestamp in chunk_indices.items():
        index_of_file = int(filename.replace(".replay", ""))
        assert math.isclose(start_timestamp, DURATION_PER_CHUNK * index_of_file)

    # test seeking a timestamp
    # jump to the middle chunk
    player.seek(DURATION_PER_CHUNK * CHUNK_FILES_NUM / 2)
    assert player.current_chunk_index == int(CHUNK_FILES_NUM / 2)

    # jump to the start chunk
    player.seek(0.0)
    assert player.current_chunk_index == 0

    # jump to the end chunk
    player.seek(DURATION_PER_CHUNK * CHUNK_FILES_NUM - 1)
    assert player.current_chunk_index == CHUNK_FILES_NUM - 1
    cleanup()


if __name__ == '__main__':
    pytest_main(__file__)