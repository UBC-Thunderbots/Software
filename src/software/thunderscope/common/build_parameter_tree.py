from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph import parametertree
from google.protobuf.json_format import MessageToDict
from thefuzz import fuzz
from proto.import_all_protos import *

@staticmethod
def __create_int_parameter(key, value, descriptor):
    """Converts an int field of a proto to a SliderParameter with
    the min/max bounds set according to the provided ParameterRangeOptions

    min/vax options.

    :param key: The name of the parameter
    :param value: The default value
    :param descriptor: The proto descriptor

    """

    # Extract the options from the descriptor, and store it
    # in the dictionary.
    options = MessageToDict(
        descriptor.GetOptions(), preserving_proto_field_name=True
    )

    try:
        min_max = options["[TbotsProto.bounds]"]
    except KeyError:
        raise KeyError("{} missing ParameterRangeOptions".format(key))

    return {
        "name": key,
        "type": "slider",
        "value": value,
        "default": value,
        "limits": (int(min_max["min_int_value"]), int(min_max["max_int_value"])),
        "step": 1,
    }

@staticmethod
def __create_double_parameter(key, value, descriptor):
    """Converts a double field of a proto to a SliderParameter with
    the min/max bounds set according to the provided ParameterRangeOptions
    min/vax options.

    :param key: The name of the parameter
    :param value: The default value
    :param descriptor: The proto descriptor

    """

    # Extract the options from the descriptor, and store it
    # in the dictionary.
    options = MessageToDict(
        descriptor.GetOptions(), preserving_proto_field_name=True
    )

    try:
        min_max = options["[TbotsProto.bounds]"]
    except KeyError:
        raise KeyError("{} missing ParameterRangeOptions".format(key))

    return {
        "name": key,
        "type": "slider",
        "value": value,
        "default": value,
        "limits": (min_max["min_double_value"], min_max["max_double_value"],),
        "step": 0.01,
    }

@staticmethod
def __create_enum_parameter(key, value, descriptor):
    """Converts an enum field in a protobuf to a ListParameter. Uses
    the options to lookup all possible enum values and provides them
    as a dropdown option.

    :param key: The name of the parameter
    :param value: The default value
    :param descriptor: The proto descriptor

    """
    options = []

    for enum_desc in descriptor.enum_type.values:
        options.append(enum_desc.name)

    # The list index is indexed from 1
    current_enum_index = value - 1

    return parametertree.parameterTypes.ListParameter(
        name=key,
        default=None,
        value=descriptor.enum_type.values[current_enum_index].name,
        limits=options,
    )

@staticmethod
def __create_bool_parameter(key, value, _):
    """Convert a bool field in proto to a BoolParameter

    :param key: The name of the parameter
    :param value: The default value
    :param _: The proto descriptor, unused for bool

    """
    return {"name": key, "type": "bool", "value": value}

@staticmethod
def __create_string_parameter(key, value, descriptor):
    """Convert a string field in proto to a StrParameter

    :param key: The name of the parameter
    :param value: The default value
    :param descriptor: The proto descriptor

    """
    return {"name": key, "type": "text", "value": " "}

def config_proto_to_param_dict(
        self, message, search_term=None, current_attr=None,
):
    """Converts a protobuf to a pyqtgraph parameter tree dictionary
    that can loaded directly into a ParameterTree

    https://pyqtgraph.readthedocs.io/en/latest/parametertree/index.html

    :param message: The message to convert to a dictionary
    :param search_term: The search filter
    :param current_attr: Which attr we are currently on to set

    """
    message_dict = {}
    field_list = []

    if not current_attr:
        current_attr = "self.proto_to_configure"

    for descriptor in message.DESCRIPTOR.fields:

        key = descriptor.name
        value = getattr(message, descriptor.name)

        if search_term and descriptor.type != descriptor.TYPE_MESSAGE:
            if fuzz.partial_ratio(search_term, key) < self.search_filter_threshold:
                continue

        if descriptor.type == descriptor.TYPE_MESSAGE:
            field_list.append(
                {
                    "name": key,
                    "type": "group",
                    "children": self.config_proto_to_param_dict(
                        value, search_term, f"{current_attr}.{key}",
                    ),
                }
            )

        elif descriptor.type == descriptor.TYPE_BOOL:
            field_list.append(self.__create_bool_parameter(key, value, descriptor))

        elif descriptor.type == descriptor.TYPE_ENUM:
            field_list.append(self.__create_enum_parameter(key, value, descriptor))

        elif descriptor.type == descriptor.TYPE_STRING:
            field_list.append(
                self.__create_string_parameter(key, value, descriptor)
            )

        elif descriptor.type == descriptor.TYPE_DOUBLE:
            field_list.append(
                self.__create_double_parameter(key, value, descriptor)
            )

        elif descriptor.type in [descriptor.TYPE_INT32, descriptor.TYPE_INT64]:
            field_list.append(self.__create_int_parameter(key, value, descriptor))

        else:
            raise NotImplementedError(
                "Unsupported type {} in parameter config".format(descriptor.type)
            )

        # Protobuf doesn't set the default values by default, and won't let
        # us serialize the message if all the required fields are not set (even
        # if they have a default). So lets just set the default as the value
        if descriptor.type != descriptor.TYPE_MESSAGE:
            if descriptor.type == descriptor.TYPE_STRING:
                exec(f"{current_attr}.{key} = '{value}'")
            else:
                exec(f"{current_attr}.{key} = {value}")

    if field_list:
        return field_list

    return message_dict