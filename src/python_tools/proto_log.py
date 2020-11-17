from software.proto.repeated_any_msg_pb2 import RepeatedAnyMsg
from google.protobuf.internal.decoder import _DecodeVarint32
import os


class ProtoLog:
    def __init__(self, directory, msg_class):
        self.msg_class = msg_class
        self.repeated_any_msgs = []
        self.chunk_start_idxs = []
        self.cached_unpacked_msgs = dict()
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

        print("Loaded {} RepeatedAnyMsg chunks".format(len(self.repeated_any_msgs)))

    def __len__(self):
        return self.chunk_start_idxs[-1] + len(self.repeated_any_msgs[-1].messages)

    def _get_item_at_idx(self, idx):
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

        chunk_idx = len(self.chunk_start_idxs) - 1
        for cidx in range(len(self.chunk_start_idxs) - 1):
            if self.chunk_start_idxs[cidx + 1] > idx:
                chunk_idx = cidx
                break

        msg_idx = idx - self.chunk_start_idxs[chunk_idx]
        msg = self.msg_class()
        self.repeated_any_msgs[chunk_idx].messages[msg_idx].Unpack(msg)
        self.cached_unpacked_msgs[idx] = msg
        return msg

    def __getitem__(self, key):
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

    def __iter__(self):
        self.iter_idx = 0
        return self

    def __next__(self):
        try:
            result = self[self.iter_idx]
            self.iter_idx += 1
            return result
        except IndexError:
            raise StopIteration
