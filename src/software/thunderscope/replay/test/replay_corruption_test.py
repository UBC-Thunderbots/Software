"""
TO GIVE CONTEXT FOR THE THIS TEST: 

The idea of this test is to check how robust proto player is against corrupted file entries. 
We are going to test how robust proto player is by intentionally creating corrupted replay files 
and seeing whether or not proto player could play the replay files. 

Steps to test: 
    1. we create a directory to store the replay files 
    2. we create invalid entries of two type: the data is actually corrupted, and we are missing a delimeter. We 
        the mixed those replay entries with valid replay entries to check if proto player could actually player those
        entires
    4. we check to see if there are uncaught exception. If there are, we know fore sure proto player wouldn't work!
"""
import pytest
import sys 

import time
from typing import Callable
import random
import shutil
import gzip
import os
from proto.import_all_protos import *
from google.protobuf.message import Message
import base64
from software.thunderscope.replay.replay_constants import *
from software.thunderscope.replay.proto_player import ProtoPlayer
from software.thunderscope.replay.proto_logger import ProtoLogger
from software.thunderscope.proto_unix_io import ProtoUnixIO

random.seed(0)

# this would be the path where the file corruption would be tested
TMP_REPLAY_SAVE_PATH = "/tmp/test_file_corruption"

def create_random_proto() -> Message:
    """
    creating the proto that are going to be saved to the disk

    :return: the proto that we are referencing
    """
    some_random_proto = RobotId(id=1, team=2)
    return some_random_proto 

def create_valid_log_entry(proto: Message, current_time: float) -> str:
    """
    creating normal log entries

    :proto: the reference proto
    :current_time: the reference time
    :return: the log entries that the replay file format uses
    """
    serialized_proto = base64.b64encode(proto.SerializeToString())
    log_entry = (
        f"{current_time}{REPLAY_METADATA_DELIMETER}"
        + f"{proto.DESCRIPTOR.full_name}{REPLAY_METADATA_DELIMETER}"
        + f"{serialized_proto}\n"
    )
    return log_entry

def create_missing_delimeter_log_entry(proto: Message, current_time: float) -> str: 
    """
    creating corrupted log entries

    :proto: the reference proto
    :current_time: the reference time
    :return: the log entries that the replay file format uses
    """
    serialized_proto = base64.b64encode(proto.SerializeToString())
    return (
        f"{current_time}"
        + f"{proto.DESCRIPTOR.full_name}{REPLAY_METADATA_DELIMETER}"
        + f"{serialized_proto}\n"
    )

def create_corrupt_log_entry(proto: Message, current_time: float) -> str: 
    """
    creating corrupted log entries

    :proto: the reference proto
    :current_time: the reference time
    :return: the log entries that the replay file format uses
    """
    serialized_proto = base64.b64encode(proto.SerializeToString())
    return (
        f"{current_time}{REPLAY_METADATA_DELIMETER}"
        + f"{proto.DESCRIPTOR.full_name}thisissomethingthatwouldbebad{REPLAY_METADATA_DELIMETER}"
        + f"{serialized_proto}thisisalsosomethingbad\n"
    )


def make_part_replay_chunks(list_of_protos: [Message], save_path:str, duration:float, start_time: float, gen_log_entry_func: Callable[[Message, float], None], frequency=0.1):
    """
    making a part of the replay chunks and appending it to the 0.replay file. There would be a frequency% chance that a
    invalid log  entry are created


    :list_of_protos: the list of proto that we are referencing when creating the log entries
    :save_path: where we are saving the replay file
    :duration: how long do we want to create theses replay chunks
    :start_time: when is the replay chunk being started? 
    :gen_log_entry_func: the function that is used to generate invalid log entries 
    :frequency: what percent of the time should we call gen_log_entry_func
    """
    os.makedirs(save_path, exist_ok=True)
    path_to_replay_file = os.path.join(save_path, "0.replay")

    with gzip.open(path_to_replay_file, "ab") as log_file:
        for i in range(len(list_of_protos)): 
            # calculate time based on linear interpolation
            proto_time = start_time + duration/len(list_of_protos) * i   
            proto = list_of_protos[i]

            if random.random() > frequency: 
                # this would happen 90 % of the time
                log_entry = create_valid_log_entry(proto, proto_time)
            else: 
                # this would happen 10% of the time
                log_entry = gen_log_entry_func(proto, proto_time)

            ProtoLogger.write_to_logfile(log_file, bytes(log_entry, encoding="utf-8"))

def make_replay_chunk(size_of_replay_chunk=1000):
    """
    Creating a replay chunks that is like the following: 
    1. from 0 to 0.1 seconds, it all of the entries are valid
    2. from 0.1 to 0.2 seconds, 10% of the proto cannot be decoded 
    3. from 0.2 to 0.3 seconds, 10% of the proto are missing delimeter

    :size_of_replay_chunks: the size of the replay chunks we want to make
    """
    # making sure previous test results doesn't affect the new test
    shutil.rmtree(TMP_REPLAY_SAVE_PATH, ignore_errors=True)

    replay_proto =  []
    # we divide by three because we make three copies in the code below, so we have
    # close to size_of_replay_chunk in the end, assuming integer rounding that is. 
    for _ in range(int(size_of_replay_chunk/3)):
        some_proto = create_random_proto()
        replay_proto.append(some_proto)

    # making the replay chunks 
    # the first part of the replay chunks is from  0 to 0.1 seconds, all of the entries are valid
    # the seconds part of the replay chunk is from 0.1 to 0.2 seconds, 10% of the proto cannot be decoded 
    # the third part of the replay chunk is from 0.2 to 0.3 seconds, 10% of the proto are missing delimeter
    make_part_replay_chunks(replay_proto, TMP_REPLAY_SAVE_PATH, 0.1, 0.0, create_valid_log_entry)
    make_part_replay_chunks(replay_proto, TMP_REPLAY_SAVE_PATH, 0.1, 0.1, create_corrupt_log_entry)
    make_part_replay_chunks(replay_proto, TMP_REPLAY_SAVE_PATH, 0.1, 0.2, create_missing_delimeter_log_entry)

def cleanup():
    """
    cleaning up this test by deleting the replay files 
    """
    shutil.rmtree(TMP_REPLAY_SAVE_PATH)

def create_test_player() -> ProtoPlayer:
    """
    Creating a protoplayer and setting a seed so that the generated 
    replay file stays the same over time!
    """


    make_replay_chunk()

    io = ProtoUnixIO()
    player = ProtoPlayer(TMP_REPLAY_SAVE_PATH, io)
    return player 

def test_for_file_corruption():
    """
    testing whether or not the proto player could play corrupted zipfile! 
    """
    player = create_test_player()

    # block until the player has finished playing!
    # in total, this should play for around 0.4 seconds 
    while player.is_it_playing(): 
        time.sleep(0.1)

    # asserting that no exception has occurred when the protobufs are being played
    assert player.get_error_bit_flag() == NO_ERROR_FLAG

    cleanup()

if __name__ == "__main__":
    """
    Please read the header of this file for more context on what it is testing!
    """
    sys.exit(pytest.main([__file__, "-vv"]))
