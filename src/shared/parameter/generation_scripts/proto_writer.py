from cpp_source_config import CppSourceConfig
from cpp_header_config import CppHeaderConfig
from cpp_parameter import CppParameter
from typing import List
from dynamic_parameter_schema import PARAMETER_KEY, INCLUDE_KEY
from case_conversion import to_pascal_case
import networkx as nx

#######################################################################
#                             Proto Writer                            #
#######################################################################

AUTOGEN_WARNING = """
/**
 *  !! WARNING !!
 *
 *  THIS FILE IS AUTOGENERATED, ANY CHANGES MADE WILL BE LOST
 *
 *  !! WARNING !!
 */
"""

CONFIG_PROTO = (
    "{autogen_warning}"
    'syntax = "proto3";\n'
    'import "google/protobuf/any.proto";\n'
    "package TbotsProto;\n"
    "{contents}"
)

PROTO_MESSAGE_DEFINITION = "message {name} {{\n" "{contents}" "}}\n\n"

PROTO_PARAM_ENTRY = "{type} {name} = {count};\n"

PROTO_CONFIG_ENTRY = "google.protobuf.Any {name} = {count};\n"


class ProtoWriter(object):
    @staticmethod
    def write_config_metadata_proto(
        output_proto: str, top_level_proto: str, config_metadata: dict,
    ):
        """Generates the .proto file contain all the protobuf representations
        of all dynamic parameter configs.

        :param output_proto: the name of the proto
        :param top_level_proto: the top level proto name
        :param config_metadata: the dictionary containing the config metadata

        """
        output_proto_contents = ""

        for config, config_definition in config_metadata.items():
            message_contents = ""
            entry_count = 1
            name = to_pascal_case(config.split(".")[0])

            # generate includes
            if "include" in config_definition:
                for included_config in config_definition["include"]:
                    message_contents += PROTO_CONFIG_ENTRY.format(
                        name=included_config.split(".")[0], count=entry_count
                    )
                    entry_count += 1

            # generate parameters
            if "parameters" in config_definition:
                for param_entry in config_definition["parameters"]:
                    for param_type, param_definition in param_entry.items():
                        message_contents += "".join(
                            PROTO_PARAM_ENTRY.format(
                                type=param_type,
                                name=param_definition["name"],
                                count=entry_count,
                            )
                        )
                        entry_count += 1

            # append to output
            output_proto_contents += PROTO_MESSAGE_DEFINITION.format(
                name=name, contents=message_contents,
            )

        with open(f"{output_proto}", "w") as proto_file:

            proto_file.write(
                CONFIG_PROTO.format(
                    autogen_warning=AUTOGEN_WARNING, contents=output_proto_contents,
                )
            )
