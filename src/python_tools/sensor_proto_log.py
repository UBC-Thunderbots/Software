from .proto_log import ProtoLog
from proto.sensor_msg_pb2 import SensorProto
from google.protobuf.message import Message
from typing import List, Dict


class SensorProtoLog:
    """
    SensorProtoLog allows users to work with different types of messages 'multiplexed' into 'SensorProto'.
    A list is created for each member message type of 'SensorProto', with the same name. All of the
    messages of that type are put into the list for the given 'SensorProto' type data directory.
    The timestamps for each message type are placed into the <field name>_timestamp attribute of this class.

    Usage:
    ```
    sensor_proto_log = SensorProtoLog('/tmp/test/SensorProto')
    sensor_proto_log.ssl_vision_msg[1000]
    ```
    This gets the 1000th SSL_WrapperPacket message in the series of SensorProtos, which corresponds to
    the member `ssl_vision_msg`.
    """

    def __init__(self, directory: str):
        """
        Constructs a SensorProtoLog for the given data directory.
        :param directory: A directory containing RepeatedAnyMsg delimited files containg SensorProto's
        """
        proto_log = ProtoLog(directory, SensorProto)
        self._messages: Dict[str, List[Message]] = dict()
        for sensor_msg in proto_log:
            # ListFields() gives a list of tuple of (FieldDescriptor, Message)
            field_names = [field[0].name for field in sensor_msg.ListFields()]

            # iterate over field names for each message and add them to the dict entry
            # for that particular field
            for field_name in field_names:
                if field_name != "backend_received_time":
                    if field_name not in self._messages:
                        self._messages[field_name] = []
                    self._messages[field_name].append(getattr(sensor_msg, field_name))

                    # add the associated timestamp to the list of timestamps for
                    # the current field name
                    timestamp_key_name = field_name + "_timestamp"
                    timestamp = sensor_msg.backend_received_time.epoch_timestamp_seconds
                    if timestamp_key_name not in self._messages:
                        self._messages[timestamp_key_name] = []
                    self._messages[timestamp_key_name].append(timestamp)

    def __getattr__(self, attr_name: str) -> List[Message]:
        """
        Returns a list of messages corresponding to the given member of SensorProto.
        :param attr_name: a member message of SensorProto.
        :return: a list of Messages of the given member.
        """
        if attr_name in self._messages:
            return self._messages[attr_name]
        else:
            raise AttributeError(
                "SensorProtoLog only has attributes {}".format(self._messages.keys())
            )
