from cpp_config import CppConfig
from cpp_parameter import CppParameter
from typing import List
from dynamic_parameter_schema import PARAMETER_KEY, INCLUDE_KEY
from case_conversion import to_pascal_case
import networkx as nx

#######################################################################
#                             CPP Writer                              #
#######################################################################

# The escape characters at the front change the color to red when
# printed in a terminal, and the escape characters at the end clears and resets
# back to the original color. More info can be found here:
# https://stackoverflow.com/questions/287871/print-in-terminal-with-colors
AUTOGEN_FAILURE_MSG = """\033[91m====================================================\033[0m
\033[91m                CFG AUTOGEN FAILURE
\u001b[34;1m Reason: {}
\033[91m====================================================\033[0m"""

AUTOGEN_WARNING = """
/**
 *  !! WARNING !!
 *
 *  THIS FILE IS AUTOGENERATED, ANY CHANGES MADE WILL BE LOST
 *
 *  !! WARNING !!
 */
"""

CONFIG_H = (
    "{autogen_warning}"
    "#pragma once\n"
    "#include <boost/program_options.hpp>\n"
    "#include <iostream>\n"
    "#include <limits>\n"
    "\n"
    '#include "shared/parameter_v2/config.h"\n'
    '#include "shared/parameter_v2/enumerated_parameter.h"\n'
    '#include "shared/parameter_v2/numeric_parameter.h"\n'
    '#include "software/util/design_patterns/generic_factory.h"\n'
    "class Play;"
    "\n"
    "{include_headers}\n"
    "\n"
    "{forward_declarations}\n"
    "\n"
    "{contents}"
)

INCLUDE_HEADER = '#include "{header_file}"'


class CppWriter(object):
    @staticmethod
    def create_config_list_from_metadata(
        top_level_config_name: str, config_metadata: dict
    ) -> List[CppConfig]:
        """Takes the config metadata loaded by config_yaml_loader, and converts it to a list of CppConfig objects;
        this includes setting the dependency graphs needed for the configs.

        :param top_leve_config_name: the name of the top level config
        :param config_metadata: the dictionary containing the config metadata
        :return: list of CppConfig objects
        """
        cpp_configs_dict = {}
        dependency_graph = nx.DiGraph()
        top_level_config = CppConfig(top_level_config_name, True)

        # first pass to construct all CppConfig objects
        for config, metadata in config_metadata.items():
            config_name = to_pascal_case(config.split(".")[0])

            config = CppConfig(config_name)
            top_level_config.include_config(config)

            if PARAMETER_KEY in metadata:
                for parameter in metadata[PARAMETER_KEY]:
                    param_metadata = list(parameter.values())[0]
                    param_type = list(parameter.keys())[0]

                    cpp_param = CppParameter(param_type, param_metadata)
                    config.add_parameter(cpp_param)

            cpp_configs_dict[config_name] = config
            dependency_graph.add_node(config_name, config=config)

        # second pass to create dependency graph
        for config, metadata in config_metadata.items():
            config_name = to_pascal_case(config.split(".")[0])

            config = cpp_configs_dict[config_name]

            if INCLUDE_KEY in metadata:
                for included_yaml in metadata[INCLUDE_KEY]:
                    included_config_name = to_pascal_case(included_yaml.split(".")[0])
                    config.include_config(cpp_configs_dict[included_config_name])
                    # add an edge from config node to included config node
                    dependency_graph.add_edge(config_name, included_config_name)

        # for each node, create a subgraph of relevant dependencies
        # Note: This can be optimized by doing traversal from each source, and creating subgraphs for
        # all its descendants during the same traversal
        for node in dependency_graph.nodes:
            # find the subgraph of the dependency graph relevant to the current node
            dependency_graph.nodes[node][
                "config"
            ].dependency_graph = dependency_graph.subgraph(
                nx.algorithms.dag.descendants(dependency_graph, node)
            )

        top_level_config.dependency_graph = dependency_graph
        cpp_configs = [
            dependency_graph.nodes[node]["config"]
            for node in list(reversed(list(nx.topological_sort(dependency_graph))))
        ]
        cpp_configs.append(top_level_config)

        return cpp_configs

    @staticmethod
    def write_config_metadata(
        output_file: str,
        include_headers: List[str],
        top_level_config_name: str,
        config_metadata: dict,
    ):
        """Generates the .h config file.

        :param output_file: the name of the config file
        :param include_headers: the list of headers that need to be included in the config file
        :param top_level_config_name: the name of the top level config
        :param config_metadata: the dictionary containing the config metadata
        """
        cpp_configs = CppWriter.create_config_list_from_metadata(
            top_level_config_name, config_metadata
        )

        # generate header file
        with open(f"{output_file}", "w") as header_file:
            contents = "\n".join([conf.definition for conf in cpp_configs])
            include_headers_formatted = "\n".join(
                [
                    INCLUDE_HEADER.format(header_file=header_file)
                    for header_file in include_headers
                ]
            )
            forward_declarations = "\n".join(
                [conf.forward_declaration for conf in cpp_configs]
            )

            header_file.write(
                CONFIG_H.format(
                    autogen_warning=AUTOGEN_WARNING,
                    include_headers=include_headers_formatted,
                    forward_declarations=forward_declarations,
                    contents=contents,
                )
            )
