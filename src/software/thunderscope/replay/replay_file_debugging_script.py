import traceback
import base64
import glob
import gzip
import os
import argparse

from proto.import_all_protos import *
from software.py_constants import *

from software.thunderscope.replay.proto_player import ProtoPlayer


def read_one_chunk(replay_file_name: str):
    """
    read one chunk of the replay file:

    :param replay_file_name: the name of the replay file that is going to be read
    :return: the number of lines that have been read
    """

    version = ProtoPlayer.get_replay_chunk_format_version(replay_file_name)

    line_num = 0
    with gzip.open(replay_file_name, "rb") as replay_file:
        # Skip the metadata line
        if version >= 2:
            replay_file.readline()

        for line in replay_file.readlines():
            # do not parse empty line
            if line == b"":
                continue

            try:
                timestamp, protobuf_type, proto = ProtoPlayer.unpack_log_entry(line, version)
            except Exception as e:
                print("Exception ignored. Please see below for more!")
                print(e)
                continue

            #######################################
            # Do something with the protobuf here #
            #######################################
            print(
                "{}: {}: {} - {}".format(
                    line_num, float(timestamp), protobuf_type, proto
                )
            )
            line_num += 1

    return line_num


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script for debugging replay files")
    parser.add_argument(
        "--replay_dir",
        action="store",
        help=f"Folder containing the .{REPLAY_FILE_EXTENSION} files",
        required=True,
        type=os.path.abspath,
    )
    args = parser.parse_args()

    # Load up all replay files in the log folder
    replay_files = glob.glob(args.replay_dir + f"/*.{REPLAY_FILE_EXTENSION}")

    # Sort the files by their chunk index
    def __sort_replay_chunks(file_path: str):
        head, tail = os.path.split(file_path)
        replay_index, _ = tail.split(".")
        return int(replay_index)

    sorted_chunks = sorted(replay_files, key=__sort_replay_chunks)

    print(f"Found {len(sorted_chunks)} replay files in {args.replay_dir}")

    total_line_num = 0
    for replay_file_name in sorted_chunks:
        try:
            total_line_num += read_one_chunk(replay_file_name)

        except gzip.BadGzipFile as e:
            print(
                "replay file is a invalid gzip file:{} with exception: {}".format(
                    replay_file_name, e
                )
            )
        except Exception:
            print(f"Cannot read {replay_file_name}")
            print(
                "Please note that the following traceback is ignored. The program is still running"
            )
            print(traceback.format_exc())

    print(f"Replayed {total_line_num} log lines")
