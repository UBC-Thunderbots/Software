from .proto_log import ProtoLog
from software.proto.sensor_msg_pb2 import SensorProto


class SensorProtoLog:
    def __init__(self, directory):
        proto_log = ProtoLog(directory, SensorProto)
        self._messages = dict()
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
        print(
            [
                "Loaded {} messages of field {}".format(len(v), k)
                for k, v in self._messages.items()
            ]
        )

    def __getattr__(self, attr_name):
        if attr_name in self._messages:
            return self._messages[attr_name]
        else:
            raise AttributeError(
                "SensorProtoLog only has attributes {}".format(self._messages.keys())
            )
