#!/usr/bin/env python
"""
This script runs as a gen_rule for bazel build //software/parameter and
generates the necessary c++ files before compiling to setup DynamicParameters

"""
import yaml
from yaml import SafeLoader
import os
import time
import sys
import constants

#######################################################################
#                              Load Yaml                              #
#######################################################################


def load_configuration(paths_to_yaml: list):
    """Loads the yaml files in the current directory and
    makes a dictionary containing the parameter name and its
    keys with the proper hierarchy

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
        with open(filename, "r") as param_yaml:
            try:
                config_descriptions = yaml.load(param_yaml, SafeLoader)
                param_info.update(config_descriptions)
            except yaml.YAMLError as ymle:
                error_msg = "{} could not be loaded correctly, please check format".format(
                    filename
                )
                print(constants.AUTOGEN_FAILURE_MSG.format(error_msg))
                sys.exit(error_msg)
    return param_info


#######################################################################
#                              Parameter                              #
#######################################################################


class Parameter(object):

    """Container to hold the required data to generate
    a Parameter that can be in any Config

    :param parameter_name: the name of the parameter
    :param parameter_description: the yaml configuration for the parameter 
    :type parameter_name: str
    :type parameter_description: dict

    """

    def __init__(self, parameter_name: str, parameter_description: dict):

        self.parameter_description = parameter_description
        self.param_name = parameter_name
        self.param_variable_name = parameter_name + "_param"

        self.default_value = parameter_description["default"]
        self.ptype = parameter_description["type"]

        # python stores booleans as True and False, but we need them to be
        # lowercase for c++
        if self.ptype == "bool":
            self.default_value = "false" if self.default_value == False else "true"

        # min max will only be in a parameter if it is of type int
        # or float, the rest of the parameters will be nullopt
        # NOTE: we either have both min/max or no bounds at all
        self.min_value = "std::nullopt"
        self.max_value = "std::nullopt"
        self.quote = '"' if self.ptype == "std::string" else ""

        # load both min and max
        if "min" and "max" in parameter_description:
            self.min_value = "{quote}{min_value}{quote}".format(
                min_value=parameter_description["min"], quote=self.quote
            )
            self.max_value = "{quote}{max_value}{quote}".format(
                max_value=parameter_description["max"], quote=self.quote
            )

        # any parameter can provide options, which is a list of valid
        # values the parameter can take
        self.options = []

        if "options" in parameter_description:
            self.options = parameter_description["options"]

    def get_constructor_entries(self):
        """Returns the c++ that resides in the constructor of the parent
        config this parameter will be in.

        :returns: string that goes in the constructor of the parent config
        :rtype: str

        """
        return constants.PARAMETER_CONSTRUCTOR_ENTRY.format(
            # the type of the parameter
            type=self.ptype,
            # the variable name
            param_variable_name=self.param_variable_name,
            # we need to add additional quotes around the value if it is a string
            # so the quotes are formatted here
            quote='"' if self.ptype == "std::string" else "",
            # the parameter name (different from variable name)
            param_name=self.param_name,
            # the default value of the parameter
            value=self.default_value,
        )

    def get_boost_command_line_entry(self):
        """Returns the line required to expose this parameter as a command line option

        :returns: string that goes into loadFromCommandLineArguments
        :rtype: str

        """
        return constants.COMMAND_LINE_ARG_ENTRY.format(
            # the type of the parameter
            param_type=self.ptype,
            # the variable name
            param_name=self.param_name,
            # we need to add additional quotes around the value if it is a string
            # so the quotes are formatted here
            quote='"' if self.ptype == "std::string" else "",
            # default value
            default_value=self.default_value,
        )

    def get_private_entries(self):
        """Returns the public members of the parent Config class
        that are required for this parameter

        :returns: entries into the private section of the Config class
        :rtype: str

        """
        return constants.PARAMETER_PRIVATE_ENTRY.format(
            type=self.ptype, param_variable_name=self.param_variable_name
        )

    def get_public_entries(self):
        """Returns the public members of the parent Config class
        that are required for this parameter.

        :returns: entries into the public section of the Config class
        :rtype: str

        """
        return constants.PARAMETER_PUBLIC_ENTRY.format(
            type=self.ptype,
            immutable_accessor_name="" + self.param_name.strip(),
            mutable_accessor_name="mutable" + self.param_name.strip(),
            param_variable_name=self.param_variable_name,
        )


#######################################################################
#                            CPP Generator                            #
#######################################################################


class Config(object):

    """Recursive data structure representing the equivalent c++ class
    in config.hpp that defines a configuration. Created from a yaml
    dictionary that the user specifies.

    :param config_name: the name of the config
    :param config_description: the yaml configuration for the config 
    :type config_name: str
    :type config_description: dict

    """

    # stores all the configurations in a "flat" structure
    # globally to print all the classes and their
    # forward declarations
    all_configs = []

    def __init__(self, config_name: str, config_description: dict):

        self.config_description = config_description
        self.config_name = config_name
        self.config_variable_name = config_name + "_config"
        self.parameters = []
        self.configs = []

        for name, param_or_config in self.config_description.items():

            # if the dictionary has type/value in the values, then it must
            # be a parameter, so create one
            if "type" in param_or_config:
                self.parameters.append(Parameter(name, param_or_config))

            # otherwise this config has a nested config so create that that
            else:
                # add to local and global list
                config = Config(name, param_or_config)
                self.configs.append(config)
                Config.all_configs.append(config)

    def get_class_str(self):
        """Returns the c++ config class string that will go in config.hpp 

        :returns: string containing everything needed to define a Config class
        :rtype: str

        """
        ################
        # Acquire Data #
        ################

        # constructor entries: The constructor of a config class initializes
        # all parameters and nested configs as well as the mutable/immutable parameterlists
        # which contain a list of all parameters and configs
        constructor_config_entires = "\n".join(
            config.get_constructor_entries() for config in self.configs
        )
        constructor_parameter_entries = "\n".join(
            parameter.get_constructor_entries() for parameter in self.parameters
        )

        # mutable parameter list initialization
        mutable_parameter_list_entries = [
            parameter.param_variable_name for parameter in self.parameters
        ]
        mutable_parameter_list_entries += [
            config.config_variable_name for config in self.configs
        ]

        # immutable parameter list initialization
        immutable_parameter_list_entries = [
            constants.IMMUTABLE_PARAMETER_LIST_PARAMETER_ENTRY.format(
                type=parameter.ptype, param_variable_name=parameter.param_variable_name
            )
            for parameter in self.parameters
        ]

        immutable_parameter_list_entries += [
            constants.IMMUTABLE_PARAMETER_LIST_CONFIG_ENTRY.format(
                config_name=config.config_name,
                config_variable_name=config.config_variable_name,
            )
            for config in self.configs
        ]

        # public section of the config class is where all the accessors reside
        # for each param and nested config
        public_parameter_entries = "\n".join(
            parameter.get_public_entries() for parameter in self.parameters
        )
        public_config_entries = "\n".join(
            config.get_public_entries() for config in self.configs
        )

        # private section of the config class
        private_parameter_entries = "\n".join(
            parameter.get_private_entries() for parameter in self.parameters
        )
        private_config_entries = "\n".join(
            config.get_private_entries() for config in self.configs
        )

        # command line parser method of the config class
        command_line_arg_struct_contents = ""
        parse_command_line_args_function_contents = ""
        load_command_line_args_into_config_contents = ""

        for parameter in self.parameters:

            command_line_arg_struct_contents += parameter.get_boost_command_line_entry()
            load_command_line_args_into_config_contents += constants.LOAD_COMMAND_LINE_ARG_INTO_CONFIG.format(
                param_name=parameter.param_name
            )

            if parameter.ptype == "bool":
                parse_command_line_args_function_contents += constants.PARAMETER_COMMAND_LINE_BOOL_SWITCH_ENTRY.format(
                    param_name=parameter.param_name,
                    param_desc=parameter.parameter_description["description"],
                )
            else:
                parse_command_line_args_function_contents += constants.PARAMETER_COMMAND_LINE_ENTRY.format(
                    param_name=parameter.param_name,
                    param_type=parameter.ptype,
                    param_desc=parameter.parameter_description["description"],
                )

        ##################
        # Generate Class #
        ##################

        return constants.CONFIG_CLASS.format(
            # the config name is the name of the configuration class
            config_name=self.config_name,
            # the entries that will go into the constructor of the config
            # class excluding the parameter_list initialization
            constructor_entries=constructor_config_entires
            + constructor_parameter_entries,
            # parameter list initialization with all the parameters and configs
            # that are a part of this class, without the const pointer cast
            mutable_parameter_list_entries=",\n".join(mutable_parameter_list_entries),
            # parameter list initialization with all the parameters and configs
            # that are a part of this class, with the const pointer cast
            immutable_parameter_list_entries=",\n".join(
                immutable_parameter_list_entries
            ),
            # public members of the config class
            public_entries=public_parameter_entries + public_config_entries,
            # private members of the config class
            private_entries=private_parameter_entries + private_config_entries,
            # command line parser
            parse_command_line_args_function_contents=parse_command_line_args_function_contents,
            command_line_arg_struct_contents=command_line_arg_struct_contents,
            load_command_line_args_into_config_contents=load_command_line_args_into_config_contents,
        )

    def get_constructor_entries(self):
        """Returns the c++ code needed in the constructor of the parent config
        for this child config to be nested in.

        :returns: entry to put in constructor of ANOTHER config
        :rtype: str

        """
        return constants.CONFIG_CONSTRUCTOR_ENTRY.format(
            config_name=self.config_name, config_variable_name=self.config_variable_name
        )

    def get_private_entries(self):
        """Returns the private members of the parent Config class that are required
        for this nested Config.

        :returns: entries into the private section of the Config class
        :rtype: str

        """
        return constants.CONFIG_PRIVATE_ENTRY.format(
            config_name=self.config_name, config_variable_name=self.config_variable_name
        )

    def get_public_entries(self):
        """Returns the public members of the parent Config class that are required
        for this nested Config.

        :returns: entries into the public section of the Config class
        :rtype: str

        """
        return constants.CONFIG_PUBLIC_ENTRY.format(
            config_name=self.config_name,
            immutable_accessor_name="get" + self.config_name.strip(),
            mutable_accessor_name="getMutable" + self.config_name.strip(),
            config_variable_name=self.config_variable_name,
        )


#######################################################################
#                                MAIN                                 #
#######################################################################
if __name__ == "__main__":

    # create config
    ThunderbotsConfig = Config("ThunderbotsConfig", load_configuration(sys.argv[1:-1]))

    with open(sys.argv[-1], "w") as config_gen:

        # header
        config_gen.write(constants.H_HEADER)

        # forward declarations
        for config in ThunderbotsConfig.all_configs:
            config_gen.write("class " + config.config_name + ";\n")

        # all the classes
        for config in ThunderbotsConfig.all_configs:
            config_gen.write(config.get_class_str())

        # main ThunderbotsConfig class
        config_gen.write(ThunderbotsConfig.get_class_str())

    print("INFO: Generated Dynamic Parameters")
