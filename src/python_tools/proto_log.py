from proto.repeated_any_msg_pb2 import RepeatedAnyMsg
from google.protobuf.internal.decoder import _DecodeVarint32
from typing import TypeVar, Generic, Type, Any, Iterator, List, Dict
import os

MsgClass = TypeVar("MsgClass")


class ProtoLog(Generic[MsgClass]):
    """
    ProtoLog allows users to work with directories containing serialized, delimited "RepeatedAnyMsg"
    chunks, representing a consecutive series of messages encapsulated in "Any" messages.

    Usage:
    `proto_log = ProtoLog('/tmp/test/PrimitiveSet/', PrimitiveSet)`
    loads the directory of chunks that represent a series of PrimitiveSet

    `proto_log[1000]` gets the 1000th PrimitiveSet message in the directory

    ```
    for primitive_set_msg in proto_log:
        print(primitive_set_msg.time_sent)
    ```
    will iterate over the messages in the directory and print the `time_sent` field.

    """

    def __init__(self, directory: str, msg_class: Type[MsgClass]):
        """
        Constructs a ProtoLog from the directory of delimited Protobuf 'RepeatedAnyMsg' messages
        at the given path.
        :param directory: The path of a directory containing delimited RepeatedAnyMsg messages in files
        :param msg_class: The type of the message contained in the RepeatedAnyMsg chunks
        """
        self.msg_class: Type[MsgClass] = msg_class
        self.repeated_any_msgs: List[RepeatedAnyMsg] = []
        self.chunk_start_idxs: List[int] = []
        self.cached_unpacked_msgs: Dict[int, MsgClass] = dict()
        cur_start_idx = 0
        for file in os.listdir(directory):
            filepath = os.path.join(directory, file)
            if file.isnumeric() and os.path.isfile(filepath):
                buf = open(filepath, "rb").read()
                msg_len, new_pos = _DecodeVarint32(buf, 0)
                repeated_any_msg = RepeatedAnyMsg()
                repeated_any_msg.ParseFromString(buf[new_pos : new_pos + msg_len])
                self.repeated_any_msgs.append(repeated_any_msg)
                self.chunk_start_idxs.append(cur_start_idx)
                cur_start_idx += len(repeated_any_msg.messages)

    def __len__(self) -> int:
        """
        Returns the total number of messages in the data directory.
        :return: the total number of messages in the data directory.
        """
        return self.chunk_start_idxs[-1] + len(self.repeated_any_msgs[-1].messages)

    def _get_item_at_idx(self, idx: int) -> MsgClass:
        """
        Returns the idx'th message out of all the messages in the data directory.
        :param idx: index of the message
        :return: the message at the given index
        """
        if idx in self.cached_unpacked_msgs:
            return self.cached_unpacked_msgs[idx]

        if idx >= self.chunk_start_idxs[-1] + len(self.repeated_any_msgs[-1].messages):
            raise IndexError(
                "Tried to access msg idx {} when we only have {} msgs!".format(
                    idx,
                    self.chunk_start_idxs[-1]
                    + len(self.repeated_any_msgs[-1].messages),
                )
            )

        item_chunk_idx = len(self.chunk_start_idxs) - 1
        for chunk_idx in range(len(self.chunk_start_idxs) - 1):
            if self.chunk_start_idxs[chunk_idx + 1] > idx:
                item_chunk_idx = chunk_idx
                break

        msg_idx = idx - self.chunk_start_idxs[item_chunk_idx]
        msg = self.msg_class()
        self.repeated_any_msgs[item_chunk_idx].messages[msg_idx].Unpack(msg)
        self.cached_unpacked_msgs[idx] = msg
        return msg

    def __getitem__(self, key: Any) -> MsgClass:
        """
        Returns the message at the given index if key is an integer, returns every message
        of messages from the slice.start and slice.stop at intervals of slice.step, if the key
        is a slice, else throws IndexError
        :param key: an integer or a slice
        :return: a message at the given index or a series of messages as specified by the slice
        """
        if isinstance(key, slice):
            start = key.start if key.start else 0
            stop = key.stop if key.stop else len(self)
            step = key.step if key.step else 1

            result = []

            for idx in range(start, stop, step):
                result.append(self._get_item_at_idx(idx))

            return result
        elif isinstance(key, int):
            return self._get_item_at_idx(key)
        else:
            raise IndexError("Invalid index type!")

    def __iter__(self) -> Iterator[MsgClass]:
        """
        Returns an iterator over the messages in the data directory.
        :return: An iterator
        """
        self.iter_idx = 0
        return self

    def __next__(self) -> MsgClass:
        """
        Increments the iterator and returns the message at the current iterator position.
        :return: the message at the current iterator position.
        """
        try:
            result = self[self.iter_idx]
            self.iter_idx += 1
            return result
        except IndexError:
            raise StopIteration

    def get_chunk_count(self) -> int:
        """
        Returns the number of chunk files in the data directory that this object was
        constructed with.
        :return: the number of chunk files in the data directory.
        """
        return len(self.repeated_any_msgs)
