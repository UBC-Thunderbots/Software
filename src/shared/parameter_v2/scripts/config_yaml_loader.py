import os
import sys

import networkx
import yaml
from colorama import Fore, init
from jsonschema import validate

from dynamic_parameter_schema import (
    INCLUDE_DEF_SCHEMA,
    PARAM_DEF_SCHEMA,
    SUPPORTED_TYPES,
)

#######################################################################
#                         Config Yaml Loader                          #
#######################################################################


class ConfigYamlLoader(object):
    @staticmethod
    def get_config_metadata(yaml_paths):
        """Loads the yamls in the root config directory and verifies that the
        requested configuration is valid. Then returns config_metadata dict
        with a specified format captuing all the requested params and configs.

        Example config directory:
        .
        └── config
            ├── ai
            │   ├── ai.yaml
            │   ├── passing
            │   │   └── passing.yaml
            │   └── plays
            │       ├── plays.yaml
            │       ├── defense.yaml
            │       └── offense.yaml
            ├── sensor_fusion
            │   └── sensor_fusion.yaml
            └── tbots_config.yaml

        A valid config directory maintains the following properties:

        - All yamls are properly formed (i.e proper syntax, YAML Version 1.2)
        - All parameter definitions abide by the dynamic parameter schema
        - All include statements do NOT cause cycles in configs

        :param yaml_paths: the path to all the config yamls
        :type yaml_paths: list of str

        :returns config_metadata: the config name followed by a list of
            included configs and a list of defined parameters.
        {
            "ai.yaml": {
                "includes": ["passing.yaml", "plays.yaml"],
                "parameters" :
                [
                    "int": {
                        "name": "example_int", "value": 10, "min": 0, "max": 20
                    },
                    ...
                ]
            },

            "plays.yaml": {
                "includes": ["defense.yaml", "offense.yaml"],
                ...
            }
            ...
        }

        :rtype: dict

        """
        # load yaml into config_metadata
        config_metadata = ConfigYamlLoader.__load_yaml_into_dict(yaml_paths)

        # validate schema
        ConfigYamlLoader.__validate_config_metadata(config_metadata)

        # detect cycles
        ConfigYamlLoader.__detect_cycles_in_config_metadata(config_metadata)

        # valid config ready to be generated
        return config_metadata

    @staticmethod
    def __load_yaml_into_dict(yaml_paths):
        """Loads the yamls into an dictionary. Any errors while in the yaml
        syntax will raise to the main thread.

        :raises: ConfigYamlMalformed
        :param yaml_paths: the path to all the config yamls
        :type yaml_paths: list of str
        :returns: config_medata dict representing the data to generate
        :rtype: dict

        """
        raw_config_metadata = {}

        for filename in yaml_paths:
            with open(filename, "r") as param_yaml:

                try:
                    # extract config name from filename
                    _, tail = os.path.split(filename)

                    # safe load yaml into dictionary
                    raw_config_metadata[tail] = list(
                        yaml.safe_load_all(param_yaml))

                except yaml.YAMLError as ymle:
                    raise ConfigYamlMalformed(
                        "Check malformed {} \n {}".format(tail, ymle)
                    )

        return raw_config_metadata

    @staticmethod
    def __validate_config_metadata(config_metadata):
        """Validates the config_metadata that was loaded with against
        the INCLUDE_DEF_SCHEMA and PARAM_DEF_SCHEMA

        :raises: ConfigYamlMalformed
        :param config_metadata: Metadata describing params and config includes
        :type config_metadata: dict

        """

        def validate_include(config_metadata, config_file, include_dict):
            # check schema
            validate(include_dict, INCLUDE_DEF_SCHEMA)

            # check duplicates
            if len(include_dict["include"]) > len(set(include_dict["include"])):
                raise ConfigYamlMalformed(
                    "Duplicate include detected in {}".format(config_file)
                )

            # check that included yaml is defined elsewhere
            for included_yaml in include_dict["include"]:
                if included_yaml not in config_metadata.keys():
                    raise ConfigYamlMalformed(
                        "{} config definition could not be found".format(
                            included_yaml)
                    )

        def validate_params(config_metadata, config_file, parameter_list):
            # check schema
            validate(parameter_list, PARAM_DEF_SCHEMA)

            # get all parameter name list
            param_names = [
                list(param_entry.values())[0]["name"] for param_entry in parameter_list
            ]

            # check duplicates
            if len(param_names) > len(set(param_names)):
                raise ConfigYamlMalformed(
                    "Duplicate parameter detected in {}".format(config_file)
                )

            # This is an ugly artifact of how the yaml is loaded
            # we are extracting all the requested types to check that
            # they are all supported. This is the one thing the schema
            # can't catch that we would like to check
            requested_types = [
                key[0] for key in [list(entry.keys()) for entry in parameter_list]
            ]

            # check if type requested is supported
            for requested_type in requested_types:
                if requested_type not in SUPPORTED_TYPES:

                    raise ConfigYamlMalformed(
                        "{} type unsupported".format(requested_type)
                    )

        # validate schema
        for config_file, metadata in config_metadata.items():

            if len(metadata) == 1:

                # include only
                if isinstance(metadata[0], dict):
                    validate_include(config_metadata, config_file, metadata[0])

                # parameter definitions only
                if isinstance(metadata[0], list):
                    validate_params(config_metadata, config_file, metadata[0])

            elif len(metadata) == 2:

                # include and parameters
                validate_include(config_metadata, config_file, metadata[0])
                validate_params(config_metadata, config_file, metadata[1])

            else:
                raise ConfigYamlMalformed(
                    "More than 2 yaml documents found in a file, check format"
                )

    @staticmethod
    def __detect_cycles_in_config_metadata(config_metadata):
        """Creates a DiGraph from all the includes and checks if there are cycles.
        Raises to the main thread if a cycle is detected 

        :raises: ConfigYamlCycleDetected
        :param config_metadata: Metadata describing params and config includes
        :type config_metadata: dict

        """
        edges = []

        for config, metadata in config_metadata.items():
            if "include" in metadata[0]:
                for included_config in metadata[0]["include"]:
                    edges.append((config, included_config))

        G = networkx.DiGraph(edges)

        for cycle in networkx.simple_cycles(G):
            raise ConfigYamlCycleDetected(
                "Cycle detected in the include statements: "
                + " -> ".join(cycle + [cycle[0]])
            )


#######################################################################
#                             Exceptions                              #
#######################################################################


class ConfigYamlException(Exception):
    """We have custom exceptions to facilitate terminating the genrule
    using the ConfigYamlLoader class and to print the log in red, making
    it more visible in the build log clutter.

    :param message: The exception msg to print in red
    :type message: str

    """

    def __init__(self, message):
        init()  # init Fore to print w/ color
        super().__init__(Fore.RED + message + Fore.RESET)


class ConfigYamlCycleDetected(ConfigYamlException):
    """Indicates when there is a cycle in the included configs
    """

class ConfigYamlMalformed(ConfigYamlException):
    """Indicates when the yaml has a typo or doesn't follow proper syntax
    """
