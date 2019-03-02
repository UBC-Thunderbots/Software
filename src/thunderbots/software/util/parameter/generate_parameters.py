#!/usr/bin/env python
"""
This script runs as a preprocessor for catkin_make and
generates the necessary cfg and c++ files before compiling
to setup the DynamicParameters
"""
import yaml
import os
import time
import pprint
import sys

AUTOGEN_FAILURE_MSG = '\033[91m ======== cfg autogen failure ======== \033[0m'

#######################################################################
#                              Load Yaml                              #
#######################################################################


def load_configuration(path_to_yaml):
    """Loads the yaml files in the current directory and
    makes a dictionary containg the parameter name and its
    attributes with the proper heirarchy

    NOTE: No exception handling in place so that when an error happens
    it raises to the main thread when catkin_make runs

    :param path_to_yaml: The path to where the YAML files are stored
    :type path_to_yaml: str
    :returns: Dictionary containg the parameters with proper heirarchy
    :rtype: dict

    """
    param_info = {}

    # parse yaml and create parameter dictionaries
    for filename in os.listdir(path_to_yaml):
        if filename.endswith(".yaml"):
            with open(path_to_yaml+filename, 'r') as param_yaml:
                try:
                    param_info[filename.split(".")[0]] = yaml.load(param_yaml)
                except yaml.YAMLError as ymle:
                    print AUTOGEN_FAILURE_MSG
                    msg = "{} could not be parsed correctly, please check format".format(
                        filename)
                    sys.exit(msg)

    return param_info

#######################################################################
#                          Python Generator                           #
#######################################################################


# shebang and imports
BOILER_PLATE = \
    """#!/usr/bin/env python
import os
import roslib
from dynamic_reconfigure.parameter_generator_catkin import *
gen = ParameterGenerator()
"""

# contants
PARAMS_CONFIG_NAMESPACE = "param_server"
NODE_NAME = "ps"
EXT = '.cfg'
RANGE_TYPES = ["int32_t", "double"]

# generator strs
NEW_NAMESPACE_FS = '{namespace} = gen.add_group({namespace})\n'
SUB_NAMESPACE_FS = '{sub_namespace} = {namespace}.add_group(\"{sub_namespace}\")\n'
PARAMETER_FS = '{namespace}.add(\"{name}\", {type}, 0, \"{description}\", {quote}{default}{quote}, {min}, {max})\n'


def generate_cfg(param_info, output_path):
    """Takes the input dictionary from the parsed yaml
    and generates the respective cfg files needed

    :param param_info: Return value of load_configuration, dict containing params
    :param output_path: Where the cfgs should be saved to
    :type param_info: dict
    :type output_path: str

    """
    # fist key resolves to filename
    for key, value in param_info.iteritems():
        with open(output_path+key+EXT, 'w') as fpe:

            # write boilder plate
            fpe.write(BOILER_PLATE)

            # generate cfg
            __cfg_gen(param_info, fpe)


def __cfg_gen(param_info, file_pointer, group_name=None):
    """Takes the information about parameters and namespaces,
    and resursively generates the configuration

    :param param_path: The path to parameter, heirarchy propagates down
    :param file_pointer: The file to store the values
    :param group_name: The name of the current group to add to
    :type param_info: dict
    :type file_pointer: file
    :type group_name: str

    """
    # iterate through keys
    for key in param_info.keys():

        # if type key is found, create a parameter
        if "type" in param_info[key]:

            parameter = param_info[key]
            param_name = key
            param_type = "None"
            param_max = "None"
            param_min = "None"
            param_default = "None"
            param_description = "None"

            # setup common keys
            try:
                param_default = parameter["default"]
                param_type = parameter["type"]
                param_description = parameter["description"]
            except KeyError as kerr:
                print AUTOGEN_FAILURE_MSG
                msg = 'Param {} missing one of default, type or description'.format(
                    param_name)
                sys.exit(msg)

            # setup min max
            try:
                param_min = parameter["min"] if param_type in RANGE_TYPES else None
                param_max = parameter["max"] if param_type in RANGE_TYPES else None
            except KeyError as kerr:
                print AUTOGEN_FAILURE_MSG
                msg = 'Parameter {} defined as {} but does not have a min/max'.format(
                    param_name, param_type)
                sys.exit(msg)

            # write parameter to file
            file_pointer.write(PARAMETER_FS.format(
                namespace=group_name,
                name=param_name,
                description=param_description,
                default=param_default,
                quote="\"" if param_type == "string" else "",
                type=param_type,
                min=param_min,
                max=param_min
            ))

        # if there is no current group, a new namespace must be created
        elif group_name is None:
            file_pointer.write(NEW_NAMESPACE_FS.format(
                namespace=key
            ))
            __cfg_gen(param_info[key], file_pointer, group_name=key)

        # if there already is a group, add to that namespace
        else:
            file_pointer.write(SUB_NAMESPACE_FS.format(
                namespace=group_name,
                sub_namespace=key
            ))
            __cfg_gen(param_info[key], file_pointer, group_name=key)


#######################################################################
#                              Contants                               #
#######################################################################


PATH_TO_YAML = './config/'
PATH_TO_AUTOGEN = './autogenerated/'

# format strings (FS)
PARAMETER_HEADER_FS = 'extern Parameter<{type}> {name}; \n'
PARAMETER_INSTNACE_FS = 'Parameter<{type}> {name}({path});'
NAMESPACE_FS = 'namespace {name} \{ {} \} \n;'

config = load_configuration(PATH_TO_YAML)
__import__('pprint').pprint(config)
generate_cfg(config, PATH_TO_AUTOGEN)
