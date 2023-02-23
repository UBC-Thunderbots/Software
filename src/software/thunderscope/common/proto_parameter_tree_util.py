from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph import parametertree
from google.protobuf.json_format import MessageToDict
from thefuzz import fuzz


def __create_int_parameter_writable(key, value, descriptor):
    """Converts an int field of a proto to a SliderParameter with
    the min/max bounds set according to the provided ParameterRangeOptions

    min/vax options.

    :param key: The name of the parameter
    :param value: The default value
    :param descriptor: The proto descriptor

    """

    # Extract the options from the descriptor, and store it
    # in the dictionary.
    options = MessageToDict(descriptor.GetOptions(), preserving_proto_field_name=True)

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


def __create_double_parameter_writable(key, value, descriptor):
    """Converts a double field of a proto to a SliderParameter with
    the min/max bounds set according to the provided ParameterRangeOptions
    min/vax options.

    :param key: The name of the parameter
    :param value: The default value
    :param descriptor: The proto descriptor

    """

    # Extract the options from the descriptor, and store it
    # in the dictionary.
    options = MessageToDict(descriptor.GetOptions(), preserving_proto_field_name=True)

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


def __create_bool_parameter(key, value, _):
    """Convert a bool field in proto to a BoolParameter

    :param key: The name of the parameter
    :param value: The default value
    :param _: The proto descriptor, unused for bool

    """
    return {"name": key, "type": "bool", "value": value}


def __create_string_parameter_writable(key, value, descriptor):
    """Convert a string field in proto to a StrParameter

    :param key: The name of the parameter
    :param value: The default value
    :param descriptor: The proto descriptor

    """
    return {"name": key, "type": "text", "value": " "}


def __create_parameter_read_only(key, value, descriptor):
    """Convert a string field in proto to a read only Str Parameter

    :param key: The name of the parameter
    :param value: The default value
    :param descriptor: The proto descriptor

    """
    return {"name": key, "type": "str", "value": value, "readonly": True}


def config_proto_to_field_list(
    message, read_only=False, search_term=None, search_filter_threshold=60
):
    """Converts a protobuf to a pyqtgraph parameter tree dictionary
    that can loaded directly into a ParameterTree

    https://pyqtgraph.readthedocs.io/en/latest/parametertree/index.html

    :param message: The message to convert to a dictionary
    :param read_only: Whether the parameters should be read only or writable
    :param search_term: The search filter
    :param search_filter_threshold: the search filter threshold

    """
    field_list = []

    for descriptor in message.DESCRIPTOR.fields:

        key = descriptor.name
        value = getattr(message, descriptor.name)

        if search_term and descriptor.type != descriptor.TYPE_MESSAGE:
            if fuzz.partial_ratio(search_term, key) < search_filter_threshold:
                continue

        if descriptor.type == descriptor.TYPE_MESSAGE:
            field_list.append(
                {
                    "name": key,
                    "type": "group",
                    "children": config_proto_to_field_list(
                        value,
                        read_only=read_only,
                        search_term=search_term,
                        search_filter_threshold=search_filter_threshold,
                    ),
                }
            )

        elif read_only:
            string_val = ""
            if descriptor.type in [descriptor.TYPE_DOUBLE, descriptor.TYPE_FLOAT]:
                string_val = "%.2f" % value
            elif descriptor.type == descriptor.TYPE_ENUM:
                if type(value) == int:
                    string_val = descriptor.enum_type.values[value - 1].name
            else:
                string_val = str(value)

            field_list.append(
                __create_parameter_read_only(key, string_val, descriptor)
            )

        elif descriptor.type == descriptor.TYPE_BOOL:
            field_list.append(
                __create_bool_parameter(key, value, descriptor)
            )

        elif descriptor.type == descriptor.TYPE_ENUM:
            field_list.append(
                __create_enum_parameter(key, value, descriptor)
            )

        elif descriptor.type == descriptor.TYPE_STRING:
            field_list.append(
                __create_string_parameter_writable(key, value, descriptor)
            )

        elif descriptor.type in [descriptor.TYPE_DOUBLE, descriptor.TYPE_FLOAT]:
            field_list.append(
                __create_double_parameter_writable(key, value, descriptor)
            )

        elif descriptor.type in [
            descriptor.TYPE_INT32,
            descriptor.TYPE_INT64,
            descriptor.TYPE_UINT32,
            descriptor.TYPE_UINT64,
        ]:
            field_list.append(
                __create_int_parameter_writable(key, value, descriptor)
            )

        else:
            raise NotImplementedError(
                "Unsupported type {} in parameter config".format(descriptor.type)
            )

    if field_list:
        return field_list

    return {}
