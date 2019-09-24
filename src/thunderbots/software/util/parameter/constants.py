#######################################################################
#                              Constants                              #
#######################################################################

# the weird espace charecters at the front change the color to red when 
# printed in a terminal, the espace charecter at the end clears and resets
# back to the original color. More info can be found here:
# https://stackoverflow.com/questions/287871/print-in-terminal-with-colors
AUTOGEN_FAILURE_MSG = \
"""\033[91m====================================================\033[0m
\033[91m                CFG AUTOGEN FAILURE
\u001b[34;1m Reason: {}
\033[91m====================================================\033[0m"""

# the working directory is set to the current folder in CMakeLists
PATH_TO_YAML = './software/util/parameter/config/'
PATH_TO_AUTOGEN_CFG = './software/util/parameter/autogenerated/cfg/'
PATH_TO_AUTOGEN_CPP = './software/util/parameter/'
PATH_TO_AUTOGEN_NODE = './software/dynamic_reconfigure_manager/'
DYNAMIC_PARMETERS_HEADER = 'dynamic_parameters.h'
DYNAMIC_PARMETERS_CPP = 'dynamic_parameters.cpp'

# TODO: removed unused bits here

################
# CPP Contants #
################

H_PARAMETER_DECL = '// {comment}\nextern Parameter<{type}> {name}; \n\n'
CPP_PARAMETER_INSTNACE = 'Parameter<{type}> {name}(\"{name}\", \"{namespace}\", {quote}{default}{quote});\n\n'

NAMESPACE_OPEN = 'namespace {name} {{ \n'
NAMESPACE_CLOSE = '}\n'

AUTOGEN_WARNING = \
"""
/**
 *  !! WARNING !!
 *
 *  THIS FILE IS AUTOGENERATED, ANY CHANGES MADE WILL BE LOST
 *
 *  !! WARNING !!
 */
"""
H_HEADER = \
"""{}
#pragma once
#include \"software/util/parameter/parameter.h\"
namespace Util::DynamicParameters{{
""".format(AUTOGEN_WARNING)

CPP_HEADER = \
"""{}
# include \"software/util/parameter/dynamic_parameters.h\"
namespace Util::DynamicParameters{{
""".format(AUTOGEN_WARNING)

FOOTER = "}\n"

CFG_STR_VECTOR = "std::vector<std::string> cfg_strs{{{}}};\n"

RECONFIGURE_SERVER = \
"""ros::NodeHandle {name}("/{name}");
dynamic_reconfigure::Server<param_server::{name}Config> server({name});
"""

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
CFG_PARAMETER = '{namespace}.add(\"{name}\", {type}, 0, \"{description}\", {quote}{default}{quote}, {min_val}, {max_val}, edit_method={enum})\n'

CFG_CONST = "{qualified_name} = gen.const(\"{qualified_name}\", {type}, {quote}{value}{quote}, \"\")\n"
CFG_ENUM = "{qualified_name}_enum = gen.enum([{cfg_options}], \"Selector for {param_name}\")\n"

CFG_HEADER = \
"""#!/usr/bin/env python
from dynamic_reconfigure.parameter_generator_catkin import *
gen = ParameterGenerator()
"""
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

##################
# NODE Constants #
##################

NODE_HEADER = \
"""{}
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
""".format(AUTOGEN_WARNING)

INCLUDE_STATEMENT = "#include <thunderbots/{}Config.h>\n"
INIT_NODE = "ros::init(argc, argv, \"dynamic_parameters\");\n"
MAIN_FUNC = "int main(int argc, char** argv){{\n{}\n}};"
SPIN_NODE = "ros::spin();"

NEW_SERVER = """ros::NodeHandle nh_{name}(\"/{name}\");
dynamic_reconfigure::Server<param_server::{name}Config> {name}(nh_{name});\n"""
