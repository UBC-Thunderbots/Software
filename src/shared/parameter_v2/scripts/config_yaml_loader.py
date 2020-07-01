import sys

import yaml
from colorama import Fore, init
from jsonschema import validate

from dynamic_parameter_schema import DYNAMIC_PARAMETER_YAML_SCHEMA

#######################################################################
#                         Config Yaml Loader                          #
#######################################################################


class ConfigYamlLoader(object):

    """Loads the yamls in the root config directory and verifies that the
    requested configuration is valid.

    Example config directory:
    .
    └── config
        ├── ai
        │   ├── ai.yaml
        │   ├── passing
        │   │   └── passing.yaml
        │   └── plays
        │       ├── defense.yaml
        │       └── offsense.yaml
        ├── sensor_fusion
        │   └── sensor_fusion.yaml
        └── tbots_config.yaml

    A valid config directory maintains the following properties:

    - All yamls are properly formed (i.e proper syntax, YAML Version 1.2)
    - All parameter definitions abide by the dynamic parameter schema
    - "include" statements do NOT cause cycles in configs

    :param yaml_paths: the path to all the config yamls
    :type yaml_paths: list of str

    """

    def __init__(self, yaml_paths: list):
        self.__yaml_paths = yaml_paths
        self.__yaml_dict = {}
        self.__load_yaml_into_ordered_dict()

    def __load_yaml_into_ordered_dict(self):
        """Loads the yamls into an ordered dictionary, after validating with
        schema. Any errors while validating will raise to the main thread.

        Each file will be loaded into a new config entry as follows:

        {
            "config_name_1" :
                    {
                        "includes" : ["config_name_2", "config_name_3"],
                        "int" : {
                            "name" : "example_int", "value" : 10,
                            "min" : 0, "max" : 20
                        },
                        ...
                    },
            "config_name_2" :
                    {
                        "includes" : ["config_name_7", "config_name_3"],
                        ...
                    }
            ...
        }

        :returns: Dictionary representing the data to generate
        :rtype: dict

        """
        param_info = {}

        for filename in self.__yaml_paths:
            with open(filename, "r") as param_yaml:

                try:
                    config_descriptions = yaml.safe_load_all(param_yaml)
                    __import__("pprint").pprint(config_descriptions)

                except yaml.YAMLError as ymle:

                    raise ConfigYamlMalformed(
                        "Check malformed {} \n {}".format(filename, ymle)
                    )

        return param_info


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
        init()  # init fore
        super().__init__(Fore.RED + message + Fore.RESET)


class ConfigYamlSchemaViolation(ConfigYamlException):
    """Indicates when the schema defined in param_definition_schema.py
    is violated.
    """


class ConfigYamlCycleDetected(ConfigYamlException):
    """Indicates when there is a cycle in the included configs.
    """


class ConfigYamlMalformed(ConfigYamlException):
    """Indicates when there is a typo in a specific config or if the config
    doesn't follow proper Yaml syntax.
    """
