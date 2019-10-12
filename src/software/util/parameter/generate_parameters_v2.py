#!/usr/bin/env python
"""
This script runs as a gen_rule for bazel build //software/util/parameter and
generates the necessary c++ files before compiling to setup DynamicParametersV2

NOTE: This script is super similar to the one that exists for DynamicParametersV1
for ROS, and will eventual replace that, until then please bare with the horrible file
naming/versioning
"""
import yaml
import os
import time
import sys
import constants

#######################################################################
#                              Load Yaml                              #
#######################################################################


def load_configuration(paths_to_yaml: list):
    """Loads the yaml files in the current directory and
    makes a dictionary containg the parameter name and its
    attributes with the proper hierarchy

    NOTE: No exception handling in place so that when an error happens
    it raises to the main thread when bazel build runs

    :param paths_to_yaml: The paths to where the YAML files are stored
    :type paths_to_yaml: list of str
    :returns: Dictionary containing the parameters with proper hierarchy
    :rtype: dict

    """
    param_info = {}

    # parse yaml and create parameter dictionaries
    for filename in paths_to_yaml:
        with open(filename, 'r') as param_yaml:
            try:
                param_info[filename.split(".")[0].split('/')[-1]] = yaml.load(param_yaml)
            except yaml.YAMLError as ymle:
                error_msg = "{} could not be loaded correctly, please check format".format(
                    filename)
                print(constants.AUTOGEN_FAILURE_MSG.format(error_msg))
                sys.exit(error_msg)
    return param_info

#######################################################################
#                              Parameter                              #
#######################################################################


class Parameter(object):

    """Container to hold the required data to generate
    a DynamicParameter in any Config

    :param parameter_name: the name of the parameter
    :param parameter_description: the yaml configuration for the parameter 
    :type parameter_name: str
    :type parameter_description: dict

    """

    def __init__(self, parameter_name: str, parameter_description: dict):
        """INITIALIZER"""

        self.parameter_description = parameter_description
        self.param_name = convert_to_camel_case(parameter_name)
        self.param_variable_name = parameter_name + '_param'

        self.default_value = parameter_description['default']
        self.ptype = parameter_description['type']

        if self.ptype == 'bool':
            self.default_value = 'false' if self.default_value == False else 'true'

        # min max will only be in a parameter if it is of type int
        # or float, the rest of the parameters will NOT.
        self.min_value = None
        self.max_value = None

        if 'min' and 'max' in parameter_description:
            self.min_value = parameter_description['min']
            self.max_value = parameter_description['max']

        # any parameter can provide options, which is a list of valid
        # values the parameter can take
        self.options = []

        if 'options' in parameter_description:
            self.options = parameter_description['options']

    def get_constructor_entries(self):
        """Returns the c++ lines that go in the constructor of the parent
        config this parameter will be in.

        :returns: string that goes in the constructor of the parent config
        :rtype: str

        """
        return constants.PARAMETER_CONSTRUCTOR_ENTRY.format(
            type=self.ptype,
            param_variable_name=self.param_variable_name,
            quote="\"" if self.ptype == 'std::string' else "",
            param_name=self.param_name,
            options=','.join('"{0}"'.format(option) for option in self.options),
            value=self.default_value)

    def get_private_entries(self):
        """Returns the public members of the Config class that are required
        for this parameter

        :returns: entries into the private section of the Config class
        :rtype: str

        """
        return constants.PARAMETER_PRIVATE_ENTRY.format(
            type=self.ptype,
            param_variable_name=self.param_variable_name)

    def get_public_entries(self):
        """Returns the public members of the Config class that are required
        for this parameter.

        :returns: entries into the public section of the Config class
        :rtype: str

        """
        return constants.PARAMETER_PUBLIC_ENTRY.format(
            type=self.ptype,
            immutable_accessor_name=''+self.param_name.strip(),
            mutable_accessor_name='mutable'+self.param_name.strip(),
            param_variable_name=self.param_variable_name)

#######################################################################
#                            CPP Generator                            #
#######################################################################


class Config(object):

    """Recursive data structure representing the equivalent c++ class in config.hpp
    that defines a configuration. Created from a yaml configuration dictionary that the
    user specifies.

    :param config_name: the name of the config
    :param config_description: the yaml configuration for the config 
    :type config_name: str
    :type config_description: dict

    """

    # stores all the configurations in a "flat" structure
    # globally to print all the classes and their
    # forward declarations
    all_configs = []

    def __init__(self, config_name: str,  config_description: dict):
        """INITIALIZER"""

        self.config_description = config_description
        self.config_name = convert_to_camel_case(config_name)+'Config'
        self.config_variable_name = config_name + '_config'
        self.parameters = []
        self.configs = []

        for name, param_or_config in self.config_description.items():

            # if the dictionary has type/value in the values, then it must
            # be a paremeter, so create one
            if "type" in param_or_config:
                self.parameters.append(Parameter(name, param_or_config))

            # otherwise this config has a nested config so create that that
            else:
                # add to local and global lsit
                config = Config(name, param_or_config)
                self.configs.append(config)
                Config.all_configs.append(config)

    def __str__(self):
        """Overrides the toString method to print the c++ config class
        that will go in config.hpp 

        :returns: string containing everything needed to define a Config class
        :rtype: str

        """
        # constructor entries: The constructor of a config class initializes
        # all parameters and nested configs as well as the parameterlist which
        # contains a list of all parameters and configs
        constructor_config_entires = '\n'.join(config.get_constructor_entries()
                                               for config in self.configs)
        constructor_parameter_entries = '\n'.join(parameter.get_constructor_entries()
                                                  for parameter in self.parameters)

        # parameter list initialization
        parameter_list_entries = [parameter.param_variable_name for parameter in self.parameters]
        parameter_list_entries += [config.config_variable_name for config in self.configs]

        # public section of the config class is where all the accessors reside
        # for each param and nested config
        public_parameter_entries = '\n'.join(parameter.get_public_entries()
                                             for parameter in self.parameters)
        public_config_entries = '\n'.join(config.get_public_entries()
                                          for config in self.configs)

        # private section of the config class
        private_parameter_entries = '\n'.join(parameter.get_private_entries()
                                              for parameter in self.parameters)
        private_config_entries = '\n'.join(config.get_private_entries()
                                           for config in self.configs)

        return constants.CONFIG_CLASS.format(
            config_name=self.config_name,
            constructor_entries=constructor_config_entires + constructor_parameter_entries,
            parameter_list_entries=',\n'.join(parameter_list_entries),
            public_entries=public_parameter_entries + public_config_entries,
            private_entries=private_parameter_entries + private_config_entries)

    def get_constructor_entries(self):
        """Returns the c++ code needed in the constructor of the parent config
        for this child config to be nested in.

        :returns: entry to put in constructor of ANOTHER config
        :rtype: str

        """
        return constants.CONFIG_CONSTRUCTOR_ENTRY.format(
            config_name=self.config_name,
            config_variable_name=self.config_variable_name)

    def get_private_entries(self):
        """Returns the private members of the parent Config class that are required
        for this nested Config.

        :returns: entries into the private section of the Config class
        :rtype: str

        """
        return constants.CONFIG_PRIVATE_ENTRY.format(
            config_name=self.config_name,
            config_variable_name=self.config_variable_name)

    def get_public_entries(self):
        """Returns the public members of the parent Config class that are required
        for this nested Config.

        :returns: entries into the public section of the Config class
        :rtype: str

        """
        return constants.CONFIG_PUBLIC_ENTRY.format(
            config_name=self.config_name,
            immutable_accessor_name='get'+self.config_name.strip(),
            mutable_accessor_name='getMutable'+self.config_name.strip(),
            config_variable_name=self.config_variable_name)


#######################################################################
#                                Utils                                #
#######################################################################

def convert_to_camel_case(string_to_convert):
    """Takes a string, and regardless of the format,
    will convert it to CamelCase. This is mostly a ROS convention
    that was in place for our old parameter system, where all of
    our parameters will snake_case. 

    NOTE: if the string is already camel case it will return the same
    string

    :param string_to_convert: the snake_case_string_to_convert
    :type arg1: str
    :returns: the stringConvertedToCamelCase
    :rtype: TODO

    """
    parts = string_to_convert.split('_')

    # if the string is not snake case, just return
    if len(parts) == 1:
        return string_to_convert

    # we return the camel case version of the string, the first
    # part of the string is also capitalized
    return ''.join(part.title() for part in parts)


#######################################################################
#                                MAIN                                 #
#######################################################################
if __name__ == '__main__':

    # get config
    config = load_configuration(sys.argv[1:-1])

    ThunderbotsConfig = Config('Thunderbots', config)

    for config in ThunderbotsConfig.all_configs:
        print(config)

    with open(sys.argv[-1], 'w') as config_gen:

        config_gen.write(constants.H_HEADER)

        for config in ThunderbotsConfig.all_configs:
            config_gen.write(config.__str__())
        config_gen.write(ThunderbotsConfig.__str__())
