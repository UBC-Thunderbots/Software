import sys
import os

import yaml
from colorama import Fore, init
from jsonschema import validate

from dynamic_parameter_schema import (
    PARAM_DEF_SCHEMA,
    INCLUDE_DEF_SCHEMA,
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
        with a specific format captuing all the requested params and configs.

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
        - "include" statements do NOT cause cycles in configs

        :param yaml_paths: the path to all the config yamls
        :type yaml_paths: list of str
        :returns config_metadata: the config name followed by a list of
            included configs and a list of parameters.
        {
            "ai": {
                "includes": ["passing", "plays"],
                "parameters" :
                [
                    "int": {
                        "name": "example_int", "value": 10, "min": 0, "max": 20
                    },
                    ...
                ]
            },

            "plays": {
                "includes": ["defense", "offense"],
                ...
            }
            ...
        }

        :rtype config_metadata: dict

        """
        # load yaml into raw_configs
        raw_configs = ConfigYamlLoader.__load_yaml_into_dict(yaml_paths)

        # validate schema
        ConfigYamlLoader.__validate_raw_loaded_config(raw_configs)

        # strip yaml extensions from include statements
        processed_configs = {}

        for raw_config in raw_configs.values():
            __import__('pprint').pprint(raw_config)

    @staticmethod
    def __load_yaml_into_dict(yaml_paths):
        """Loads the yamls into an dictionary, after validating with a schema.
        Any errors while validating will raise to the main thread.

        Each file will be loaded into a new config entry as follows:

        :param yaml_paths: the path to all the config yamls
        :type yaml_paths: list of str
        :returns: Dictionary representing the data to generate
        :rtype: dict

        """
        raw_configs = {}

        for filename in yaml_paths:
            with open(filename, "r") as param_yaml:

                try:
                    # extract config name from filename
                    head, tail = os.path.split(filename)
                    config_name = tail.split(".")[0]

                    # safe load yaml into dictionary
                    raw_configs[config_name] =\
                        list(yaml.safe_load_all(param_yaml))

                except yaml.YAMLError as ymle:
                    raise ConfigYamlMalformed(
                        "Check malformed {} \n {}".format(filename, ymle)
                    )

        return raw_configs

    @staticmethod
    def __validate_raw_loaded_config(raw_configs):
        """Validates the "raw config" that was loaded with against
        the INCLUDE_DEF_SCHEMA and PARAM_DEF_SCHEMA

        :param raw_configs: The output of __load_yaml_into_dict 
        :type raw_configs: dict
        :returns: None

        """
        # validate schema
        for raw_config in raw_configs.values():

            if len(raw_config) == 1:

                # includes only
                if isinstance(raw_config[0], dict):
                    validate(raw_config[0], INCLUDE_DEF_SCHEMA)

                # parameter definitions only
                if isinstance(raw_config[0], list):
                    validate(raw_config[0], PARAM_DEF_SCHEMA)

            elif len(raw_config) == 2:

                # includes and parameter definitions only
                validate(raw_config[0], INCLUDE_DEF_SCHEMA)
                validate(raw_config[1], PARAM_DEF_SCHEMA)

                # This is an ugly artifact of how the yaml is loaded
                # we are extracting all the requested types to check that
                # they are all supported. This is the one thing the schema
                # can't catch that we would like to check
                requested_types = [
                    key[0] for key in [list(entry.keys()) for entry in raw_config[1]]
                ]

                for requested_type in requested_types:
                    if requested_type not in SUPPORTED_TYPES:

                        raise ConfigYamlMalformed(
                            "{} type unsupported".format(requested_type)
                        )
            else:
                raise ConfigYamlMalformed(
                    "More than 2 yaml documents found in one file, check format"
                )

    def __detect_cycles_in_config_medata(arg1):
        """TODO: Docstring for __detect_cycles_in_config_medata.

        :param arg1: TODO
        :returns: TODO

        """
        pass


#######################################################################
#                             Exceptions                              #
#######################################################################


class ConfigYamlException(Exception):
    """We have custom exceptions to facilitate terminating the program
    using the ConfigYamlLoader and print the log in red, making it more
    visible in the build log clutter.

    :param message: The exception msg to print in red
    :type message: str

    """

    def __init__(self, message):
        init()  # init Fore to print w/ color
        super().__init__(Fore.RED + message + Fore.RESET)


class ConfigYamlCycleDetected(ConfigYamlException):
    """Indicates when there is a cycle in the included configs.
    """


class ConfigYamlMalformed(ConfigYamlException):
    """Indicates when there is a typo in a specific config or if the config
    doesn't follow proper Yaml syntax.
    """
