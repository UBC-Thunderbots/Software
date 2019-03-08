#!/usr/bin/env python
"""
This script runs as a preprocessor for catkin_make and
generates the necessary cfg and c++ files before compiling
to setup the DynamicParameters
"""
import yaml
import os
import time
import sys

#######################################################################
#                              Constants                              #
#######################################################################

AUTOGEN_FAILURE_MSG = '\033[91m ======== cfg autogen failure ======== \033[0m'

# the working directory is set to the current folder in CMakeLists
PATH_TO_YAML = './config/'
PATH_TO_AUTOGEN = './autogenerated/'
PATH_TO_CPP = './'
DYNAMIC_PARMETERS_HEADER = 'dynamic_parameters.h'
DYNAMIC_PARMETERS_CPP = 'dynamic_parameters.cpp'

################
# CPP Contants #
################
    
H_PARAMETER_DECL = 'extern Parameter<{type}> {name}; \n'
CPP_PARAMETER_INSTNACE = 'Parameter<{type}> {name}(\"{path}\", {quote}{default}{quote});\n'

NAMESPACE_OPEN = 'namespace {name} {{ \n'
NAMESPACE_CLOSE = '}\n'

AUTOGEN_WARNING = "// THIS FILE IS AUTOGENERATED, ANY CHANGES WILLBE LOST"

H_HEADER = """# pragma once\n#include \"util/parameter/parameter.h\"
namespace Util\n{\nnamespace DynamicParameters\n{\n"""

CPP_HEADER = """# include \"util/parameter/dynamic_parameters.h\"\n
namespace Util\n{\nnamespace DynamicParameters\n{\n"""

FOOTER = "}\n}\n"

TOPIC_STR_VECTOR = "std::vector<std::string> topic_strs{{{}}};"
TOPIC_STR_VECTOR_DECL = "extern std::vector<std::string> topic_strs;"

RECONFIGURE_SERVER = """ros::NodeHandle {name}("/{name}");\n
    dynamic_reconfigure::Server<param_server::{name}Config> server({name});\n"""

CPP_TYPE_MAP = {
    "int": "int32_t",
    "double": "double",
    "string": "std::string",
    "bool": "bool",
}

#################
# CFG Contansts #
#################

CFG_NEW_NAMESPACE = '{namespace} = gen.add_group(\"{namespace}\")\n'
CFG_SUB_NAMESPACE = '{sub_namespace} = {namespace}.add_group(\"{sub_namespace}\")\n'
CFG_PARAMETER = '{namespace}.add(\"{name}\", {type}, 0, \"{description}\", {quote}{default}{quote}, {min_val}, {max_val})\n'
CFG_HEADER = """#!/usr/bin/env python
from dynamic_reconfigure.parameter_generator_catkin import *
gen = ParameterGenerator()\n"""
CFG_FOOTER = 'exit(gen.generate(\"param_server\", \"ps\", \"{}\"))'

RANGE_TYPES = ["int", "double"]
QUOTE_TYPES = ["string"]
EXT = '.cfg'

CFG_TYPE_MAP = {
    "int": "int_t",
    "double": "double_t",
    "string": "str_t",
    "bool": "bool_t",
}

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
                    sys.exit(
                        "{} could not be loaded correctly, please check format".format(filename))
    return param_info

#######################################################################
#                            CFG Generator                            #
#######################################################################


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
        with open(output_path+key+EXT, 'w+') as fpe:

            # write boilder plate
            fpe.write(CFG_HEADER)

            # generate cfg
            __cfg_gen(param_info[key], fpe)

            # write footer
            fpe.write(CFG_FOOTER.format(key))

            # make file read/write/executable
            os.chmod(output_path+key+EXT, 0o0754)
            print '=== generated {} ==='.format(key)


def __cfg_gen(param_info, file_pointer, group_name=None):
    """Takes the information about parameters and namespaces,
    and recursively generates the configuration

    :param param_info: Contains namespace and parameter information
        organized by namespaces until the parameter
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
            try:
                parameter = param_info[key]

                # write parameter to file
                file_pointer.write(
                    CFG_PARAMETER.format(
                        namespace=group_name,
                        name=key,
                        type=CFG_TYPE_MAP[parameter["type"]],
                        description=parameter["description"],
                        default=parameter["default"],
                        quote="\"" if parameter["type"] in QUOTE_TYPES else "",
                        min_val=parameter["min"] if parameter["type"] in RANGE_TYPES else None,
                        max_val=parameter["max"] if parameter["type"] in RANGE_TYPES else None
                    ))

            except KeyError as kerr:
                print AUTOGEN_FAILURE_MSG
                sys.exit('Required key missing from config: {}, please check parameter {}'.format(
                    kerr, key))

        # if there is no current group, a new namespace must be created
        elif group_name is None:
            file_pointer.write(CFG_NEW_NAMESPACE.format(
                namespace=key
            ))
            __cfg_gen(param_info[key], file_pointer, group_name=key)

        # if there already is a group, add to that namespace
        else:
            file_pointer.write(CFG_SUB_NAMESPACE.format(
                namespace=group_name,
                sub_namespace=key
            ))
            __cfg_gen(param_info[key], file_pointer, group_name=key)

#######################################################################
#                            CPP Generator                            #
#######################################################################

def generate_header_cpp(param_info, output_path):
    """Takes the input dictionary from the parsed yaml
    and generates the respective cfg files needed

    :param param_info: Return value of load_configuration, dict containing params
    :param output_path: Where the cfgs should be saved to
    :type param_info: dict
    :type output_path: str

    """
    # open files
    dynamic_parameters_h = open(output_path+DYNAMIC_PARMETERS_HEADER, 'w')
    dynamic_parameters_cpp = open(output_path+DYNAMIC_PARMETERS_CPP, 'w')

    # write the header to .h .cpp file
    dynamic_parameters_h.write(H_HEADER)
    dynamic_parameters_cpp.write(CPP_HEADER)

    # TODO Add topic_strs vector here where dynamic_parameters_util.cpp will look at
    # TODO Add initReconfigureServer function which contains all the servers which 
    # the parmeter server will call

    # iterate through param_info starting from namespaces
    # the first key is the file name, so that is skipped
    for value in param_info.itervalues():
        __header_cpp_gen(value, dynamic_parameters_h, dynamic_parameters_cpp)

    # append the footer
    dynamic_parameters_h.write(FOOTER)
    dynamic_parameters_cpp.write(FOOTER)

    # close files
    dynamic_parameters_h.close()
    dynamic_parameters_cpp.close()

    print '===== created header ====='


def __header_cpp_gen(param_info, header_file_pointer, cpp_file_pointer):
    """Takes the information about the parameters and namespaces
    and recursively generates the header file, returns a string to be written
    to the file

    NOTE: Both the header and the cpp have very similar structure, so they will
    both be generated by this function

    :param param_info: Contains namespace and parameter information
        organized by filename, followed by namespaces until the parameter
    :param header_file_pointer: The file pointer to the dynamic_parameters.h file
    :param cpp_file_pointer: The file pointer to the dynamic_parameters.cpp file
    :type param_info: dict
    :type header_file_pointer: file
    :type cpp_file_pointer: file

    :returns: The contents of the header file
    :rtype: str

    """
    # iterate through keys
    for key in param_info.keys():

        # if type key is found, create a parameter
        if "type" in param_info[key]:
            parameter = param_info[key]

            # write declaration to header file
            header_file_pointer.write(
                H_PARAMETER_DECL.format(
                    name=key,
                    type=CPP_TYPE_MAP[parameter["type"]]
                )
            )

            # write instantiation to cpp file
            cpp_file_pointer.write(
                CPP_PARAMETER_INSTNACE.format(
                    name=key,
                    path=key,
                    default=parameter["default"],
                    type=CPP_TYPE_MAP[parameter["type"]],
                    quote="\"" if parameter["type"] in QUOTE_TYPES else ""
                )
            )

        # else setup the namespaces
        else:
            # start the namespace
            header_file_pointer.write(NAMESPACE_OPEN.format(name=key))
            cpp_file_pointer.write(NAMESPACE_OPEN.format(name=key))

            __header_cpp_gen(
                param_info[key], header_file_pointer, cpp_file_pointer)

            # end the namespace
            header_file_pointer.write(NAMESPACE_CLOSE)
            cpp_file_pointer.write(NAMESPACE_CLOSE)


def generate_param_server_node():
    """This function must be called after the cfg, h and cpp files
    have been generated. Those generated cfg files will be setup
    as an indidual server containing the parameters to change

    """

    pass


#######################################################################
#                                MAIN                                 #
#######################################################################
if __name__ == '__main__':
    config = load_configuration(PATH_TO_YAML)
    __import__('pprint').pprint(config)
    generate_cfg(config, PATH_TO_AUTOGEN)
    generate_header_cpp(config, PATH_TO_CPP)
