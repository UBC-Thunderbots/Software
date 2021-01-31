import os
import sys

import networkx
import yaml
from colorama import Fore, init
import jsonschema

from typing import List

YamlPathList = List[str]

from dynamic_parameter_schema import (
    INCLUDE_DEF_SCHEMA,
    PARAM_DEF_SCHEMA,
    SUPPORTED_TYPES,
    INCLUDE_KEY,
    PARAMETER_KEY,
)

#######################################################################
#                         Config Yaml Loader                          #
#######################################################################


class ConfigYamlLoader(object):
    @staticmethod
    def get_config_metadata(yaml_paths: YamlPathList) -> dict:
        """Loads the yamls in the YamlPathList (a list of absolute paths to yamls)
        and verifies that the requested configuration is valid. Then returns
        config_metadata dict with a specified format capturing all the requested
        params and configs.

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

        - All yaml file names are unique
        - All yamls are properly formed (i.e proper syntax, YAML Version 1.2)
        - All parameter definitions abide by the dynamic parameter schema
        - All include statements do NOT cause cycles in configs

        :raises ConfigYamlMalformed: when the yaml is malformed
        :raises ConfigSchemaViolation: When the shema is violated
        :raises ConfigYamlCycleDetected: When a cycle is detected in the includes

        :param yaml_paths: the path to all the config yamls
        :type yaml_paths: list of str

        :returns config_metadata: the config name followed by a list of
            included configs and a list of defined parameters.
        {
            "ai.yaml": {
                "includes": ["passing.yaml", "plays.yaml"],
                "parameters" :
                [{
                    "int": {
                        "name": "example_int", "value": 10, "min": 0, "max": 20
                    },
                    ...
                }]
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

        # validate correct format with schema
        ConfigYamlLoader.__validate_config_metadata(config_metadata)

        # detect cycles
        ConfigYamlLoader.__detect_cycles_in_config_metadata(config_metadata)

        # valid config ready to be generated
        return config_metadata

    @staticmethod
    def __load_yaml_into_dict(yaml_paths: YamlPathList) -> dict:
        """Loads the yamls into an dictionary. Any errors while in the yaml
        syntax will raise to the main thread. We also adjust how the dictionary
        is stored for easier access later.

        :raises ConfigYamlMalformed: when the yaml is malformed
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
                    raw_config_metadata[tail] = list(yaml.safe_load_all(param_yaml))

                    if len(raw_config_metadata[tail]) == 1:

                        # include only in file
                        if isinstance(raw_config_metadata[tail][0], dict):
                            raw_config_metadata[tail] = {
                                INCLUDE_KEY: raw_config_metadata[tail][0][INCLUDE_KEY]
                            }

                        # parameter definitions only in file
                        elif isinstance(raw_config_metadata[tail][0], list):
                            raw_config_metadata[tail] = {
                                PARAMETER_KEY: raw_config_metadata[tail][0]
                            }

                    elif len(raw_config_metadata[tail]) == 2:

                        # include and param definition in file
                        raw_config_metadata[tail] = {
                            INCLUDE_KEY: raw_config_metadata[tail][0][INCLUDE_KEY],
                            PARAMETER_KEY: raw_config_metadata[tail][1],
                        }

                    else:
                        raise ConfigYamlMalformed(
                            "More than two yaml documents in {}".format(tail)
                        )

                except yaml.YAMLError as ymle:
                    raise ConfigYamlMalformed(
                        "Check malformed {} \n {}".format(tail, ymle)
                    ) from None
                except Exception as exc:
                    raise ConfigYamlMalformed(
                        "Check malformed {} \n {}".format(tail, exc)
                    ) from exc

        return raw_config_metadata

    @staticmethod
    def __validate_config_metadata(config_metadata: dict):
        """Validates the config_metadata that was loaded against the
        dynamic_parameter_schemas and then checks for duplicate includes
        and duplicate parameters in the same config.

        :raises ConfigYamlMalformed: When the yaml is malformed
        :raises ConfigSchemaViolation: When the shema is violated
        :param config_metadata: Metadata describing params and config includes
        :type config_metadata: dict

        """

        for config_file, metadata in config_metadata.items():

            if INCLUDE_KEY in metadata:

                # validate correct format with schema
                try:
                    jsonschema.validate(metadata[INCLUDE_KEY], INCLUDE_DEF_SCHEMA)
                except jsonschema.exceptions.ValidationError as jsval:
                    raise ConfigYamlSchemaViolation(
                        "Schema violation in {}: {}".format(config_file, jsval)
                    ) from None

                # check duplicates
                if len(metadata[INCLUDE_KEY]) > len(set(metadata[INCLUDE_KEY])):
                    raise ConfigYamlMalformed(
                        "Duplicate include detected in {}".format(config_file)
                    )

                # check that included yaml is defined elsewhere
                for included_yaml in metadata[INCLUDE_KEY]:
                    if included_yaml not in config_metadata.keys():
                        raise ConfigYamlMalformed(
                            "definition could not be found for {} in {}".format(
                                included_yaml, config_file
                            )
                        )

            if PARAMETER_KEY in metadata:
                # validate correct format with schema
                try:
                    jsonschema.validate(metadata[PARAMETER_KEY], PARAM_DEF_SCHEMA)
                except jsonschema.exceptions.ValidationError as jsval:
                    raise ConfigYamlSchemaViolation(
                        "Schema violation in {}: {}".format(config_file, jsval)
                    ) from None

                # Get all parameter names as a list, the parameter type comes
                # first and the name follows in the dictionary. If the schema
                # check above succeeded, its safe to assume the "name" key
                # exists in the parameter dictionary
                param_names = [
                    list(param_entry.values())[0]["name"]
                    for param_entry in metadata[PARAMETER_KEY]
                ]

                # check duplicates
                if len(param_names) > len(set(param_names)):
                    raise ConfigYamlMalformed(
                        "Duplicate parameter detected in {}".format(config_file)
                    )

                # This is an ugly artifact of how the yaml is defined and loaded
                # We are extracting all the requested types to check that
                # they are all supported. This is the one thing the schema
                # can't validate that we would like to check
                requested_types = [
                    key[0]
                    for key in [list(entry.keys()) for entry in metadata[PARAMETER_KEY]]
                ]

                # check if type requested is supported
                for requested_type in requested_types:
                    if requested_type not in SUPPORTED_TYPES:
                        raise ConfigYamlMalformed(
                            "{} type unsupported in {}".format(
                                requested_type, config_file
                            )
                        )

    @staticmethod
    def __detect_cycles_in_config_metadata(config_metadata: dict):
        """Creates a DiGraph from all the included configs and checks if there
        are cycles. Raises to the main thread if a cycle is detected

        :raises ConfigYamlCycleDetected: When a cycle is detected in the includes
        :param config_metadata: Metadata describing params and config includes
        :type config_metadata: dict

        """
        edges = []

        for config, metadata in config_metadata.items():
            if INCLUDE_KEY in metadata:
                for included_config in metadata[INCLUDE_KEY]:
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
    """We use custom exceptions to facilitate terminating the genrule
    using the ConfigYamlLoader class and to print the log in red and yellow,
    making it more visible in the build log clutter.

    :param message: The exception msg to print in red
    :type message: str

    """

    def __init__(self, message):
        init()  # init Fore
        super().__init__(
            Fore.LIGHTRED_EX
            + "Parameter generation error: "
            + Fore.YELLOW
            + message
            + Fore.RESET
        )


class ConfigYamlCycleDetected(ConfigYamlException):
    """Indicates when there is a cycle in the included configs
    """


class ConfigYamlMalformed(ConfigYamlException):
    """Indicates when the yaml has a typo or doesn't follow proper syntax
    """


class ConfigYamlSchemaViolation(ConfigYamlException):
    """Indicates when the yaml schema has been violated
    """
