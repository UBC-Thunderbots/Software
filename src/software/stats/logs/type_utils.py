from google.protobuf.descriptor import Descriptor, FieldDescriptor


def count_primitive_fields(descriptor: Descriptor):
    """Recursively counts the number of primitive fields in a Protobuf message
    using its descriptor.

    :param message: the message descriptor to count all leaf-level primitive fields for
    :return: the count of primitive fields
    """
    count = 0

    for field in descriptor.fields:
        # Check if the field is a nested message
        if field.type == FieldDescriptor.TYPE_MESSAGE:
            # Get the nested message class to recurse into its descriptor
            nested_message = field.message_type
            # Recurse using the nested message's descriptor
            count += count_primitive_fields(nested_message)
        else:
            # It's a primitive type (double, float, int, bool, string, etc.)
            count += 1
    return count
