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

    """A collection of static methods to load and validate yaml"""

    @staticmethod
    def get_config_metadata(yaml_paths):
        """Loads the yamls in the root config directory and verifies that the
        requested configuration is valid. Then returns config_metadata dict
        with a specified format capturing all the requested params and configs.

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

        # validate schema
        ConfigYamlLoader.__validate_config_metadata(config_metadata)

        # detect cycles
        ConfigYamlLoader.__detect_cycles_in_config_metadata(config_metadata)

        # valid config ready to be generated
        return config_metadata

    @staticmethod
    def __load_yaml_into_dict(yaml_paths):
        """Loads the yamls into an dictionary. Any errors while in the yaml
        syntax will raise to the main thread. We also adjust how the dictionary
        is stored for easier access later.

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
                    raw_config_metadata[tail] = list(yaml.safe_load_all(param_yaml))

                    if len(raw_config_metadata[tail]) == 1:

                        # include only in file
                        if isinstance(raw_config_metadata[tail][0], dict):
                            raw_config_metadata[tail] = {
                                "include": raw_config_metadata[tail][0]["include"]
                            }

                        # parameter definitions only in file
                        if isinstance(raw_config_metadata[tail][0], list):
                            raw_config_metadata[tail] = {
                                "parameters": raw_config_metadata[tail][0]
                            }

                    elif len(raw_config_metadata[tail]) == 2:

                        # include and param definition in file
                        raw_config_metadata[tail] = {
                            "include": raw_config_metadata[tail][0]["include"],
                            "parameters": raw_config_metadata[tail][1],
                        }

                except yaml.YAMLError as ymle:
                    raise ConfigYamlMalformed(
                        "Check malformed {} \n {}".format(tail, ymle)
                    )

        return raw_config_metadata

    @staticmethod
    def __validate_config_metadata(config_metadata):
        """Validates the config_metadata that was loaded against the
        dynamic_parameter_schemas and then checks for duplicate includes
        and duplicate parameters in the same config.

        :raises: ConfigYamlMalformed
        :param config_metadata: Metadata describing params and config includes
        :type config_metadata: dict

        """

        for config_file, metadata in config_metadata.items():

            if "include" in metadata:

                # check schema
                validate(metadata["include"], INCLUDE_DEF_SCHEMA)

                # check duplicates
                if len(metadata["include"]) > len(set(metadata["include"])):

                    raise ConfigYamlMalformed(
                        "Duplicate include detected in {}".format(config_file)
                    )

                # check that included yaml is defined elsewhere
                for included_yaml in metadata["include"]:
                    if included_yaml not in config_metadata.keys():

                        raise ConfigYamlMalformed(
                            "definition could not be found for {} in {}".format(
                                included_yaml, config_file
                            )
                        )

            if "parameters" in metadata:

                # check schema
                validate(metadata["parameters"], PARAM_DEF_SCHEMA)

                # get all parameter names as a list, the parameter type comes
                # first and the name follows in the dictionary. If the schema
                # check above succeeded, its safe to assume the "name" key
                # exists in the parameter dictionary
                param_names = [
                    list(param_entry.values())[0]["name"]
                    for param_entry in metadata["parameters"]
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
                    for key in [list(entry.keys()) for entry in metadata["parameters"]]
                ]

                # check if type requested is supported
                for requested_type in requested_types:
                    if requested_type not in SUPPORTED_TYPES:

                        raise ConfigYamlMalformed(
                            "{} type unsupported".format(requested_type)
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
            if "include" in metadata:
                for included_config in metadata["include"]:
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
