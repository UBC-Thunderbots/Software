from proto import parameters_pb2
from google.protobuf.json_format import MessageToDict


def message_to_dict(message):
    """Converts a message to a dictionary

    :param message: The message to convert to a dictionary

    """

    message_dict = {}

    for descriptor in message.DESCRIPTOR.fields:
        key = descriptor.name
        value = getattr(message, descriptor.name)

        # Handle repeated
        if descriptor.label == descriptor.LABEL_REPEATED:

            message_list = []

            for sub_message in value:
                if descriptor.type == descriptor.TYPE_MESSAGE:
                    message_list.append(message_to_dict(sub_message))
                else:
                    message_list.append(sub_message)

            message_dict[key] = message_list

        # Handle the rest
        else:

            if descriptor.type == descriptor.TYPE_MESSAGE:
                message_dict[key] = message_to_dict(value)

            else:
                # Extract the options from the descriptor, and store it
                # in the dictionary.
                options = MessageToDict(
                    descriptor.GetOptions(), preserving_proto_field_name=True
                )

                message_dict[key] = {"value": value}

                for option in options.values():
                    message_dict[key].update(option)

    return message_dict
