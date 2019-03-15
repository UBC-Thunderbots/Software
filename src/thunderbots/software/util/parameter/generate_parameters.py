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
import constants


#######################################################################
#                              Load Yaml                              #
#######################################################################

def load_configuration(path_to_yaml: str):
    """Loads the yaml files in the current directory and
    makes a dictionary containg the parameter name and its
    attributes with the proper hierarchy

    NOTE: No exception handling in place so that when an error happens
    it raises to the main thread when catkin_make runs

    :param path_to_yaml: The path to where the YAML files are stored
    :type path_to_yaml: str
    :returns: Dictionary containing the parameters with proper hierarchy
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
                    print(constants.AUTOGEN_FAILURE_MSG)
                    sys.exit(
                        "{} could not be loaded correctly, please check format".format(filename))
    return param_info

#######################################################################
#                            CFG Generator                            #
#######################################################################


def generate_cfg(param_info: dict, output_path: str):
    """Takes the input dictionary from the parsed yaml
    and generates the respective cfg files needed

    :param param_info: Return value of load_configuration, dict containing params
    :param output_path: Where the cfgs should be saved to
    :type param_info: dict
    :type output_path: str

    """
    # first key resolves to filename
    for key, value in param_info.items():
        with open(output_path+key+constants.EXT, 'w+') as fpe:

            # write boiler plate
            fpe.write(constants.CFG_HEADER)

            # generate cfg
            __cfg_gen(param_info[key], fpe)

            # write footer
            fpe.write(constants.CFG_FOOTER.format(key))

            # make file read/write/executable
            os.chmod(output_path+key+constants.EXT, 0o0754)
            print('===== generated {} params cfg'.format(key))


def __cfg_gen(param_info: dict, file_pointer, group_name=None):
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
                    constants.CFG_PARAMETER.format(
                        namespace=group_name if group_name is not None else "gen",
                        name=key,
                        type=constants.CFG_TYPE_MAP[parameter["type"]],
                        description=parameter["description"],
                        default=parameter["default"],
                        quote="\"" if parameter["type"] in constants.QUOTE_TYPES else "",
                        min_val=parameter["min"] if parameter["type"] in constants.RANGE_TYPES else None,
                        max_val=parameter["max"] if parameter["type"] in constants.RANGE_TYPES else None
                    ))

            except KeyError as kerr:
                print(constants.AUTOGEN_FAILURE_MSG)
                sys.exit('Required key missing from config: {}, please check parameter {}'.format(
                    kerr, key))

        # if there is no current group, a new namespace must be created
        elif group_name is None:
            file_pointer.write(constants.CFG_NEW_NAMESPACE.format(
                namespace=key
            ))
            __cfg_gen(param_info[key], file_pointer, group_name=key)

        # if there already is a group, add to that namespace
        else:
            file_pointer.write(constants.CFG_SUB_NAMESPACE.format(
                namespace=group_name,
                sub_namespace=key
            ))
            __cfg_gen(param_info[key], file_pointer, group_name=key)

#######################################################################
#                            CPP Generator                            #
#######################################################################


def generate_header_and_cpp(param_info: dict, output_path: str):
    """Takes the input dictionary from the parsed yaml
    and generates the respective cfg files needed

    :param param_info: Return value of load_configuration, dict containing params
    :param output_path: Where the cfgs should be saved to
    :type param_info: dict
    :type output_path: str

    """
    # open files
    dynamic_parameters_h = open(
        output_path+constants.DYNAMIC_PARMETERS_HEADER, 'w')
    dynamic_parameters_cpp = open(
        output_path+constants.DYNAMIC_PARMETERS_CPP, 'w')

    # write the header to .h .cpp file
    dynamic_parameters_h.write(constants.H_HEADER)
    dynamic_parameters_cpp.write(constants.CPP_HEADER)

    # generate cfg str used for topic subsription
    cfg_strs = ",".join("\"{}\"".format(key) for key in param_info.keys())
    dynamic_parameters_cpp.write(constants.CFG_STR_VECTOR.format(cfg_strs))

    # iterate through param_info starting from namespaces
    # the first key is the file name, so that is skipped
    for key, value in param_info.items():
        __header_and_cpp_gen(
            value, key, dynamic_parameters_h, dynamic_parameters_cpp)

    # append the footer
    dynamic_parameters_h.write(constants.FOOTER)
    dynamic_parameters_cpp.write(constants.FOOTER)

    # close files
    dynamic_parameters_h.close()
    dynamic_parameters_cpp.close()

    print('===== created dynamic_parameters header and cpp')


def __header_and_cpp_gen(param_info: dict, cfg_name: str, header_file_pointer, cpp_file_pointer):
    """Takes the information about the parameters and namespaces
    and recursively generates the header file, returns a string to be written
    to the file

    NOTE: Both the header and the cpp have very similar structure, so they will
    both be generated by this function

    :param param_info: Contains namespace and parameter information
        organized by filename, followed by namespaces until the parameter
    :param cfg_name: The name of the configuration file this parameter is in
    :param header_file_pointer: The file pointer to the dynamic_parameters.h file
    :param cpp_file_pointer: The file pointer to the dynamic_parameters.cpp file
    :type param_info: dict
    :type cfg_name: str
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
                constants.H_PARAMETER_DECL.format(
                    name=key,
                    type=constants.CPP_TYPE_MAP[parameter["type"]],
                    comment=parameter["description"]
                )
            )

            # write instantiation to cpp file
            cpp_file_pointer.write(
                constants.CPP_PARAMETER_INSTNACE.format(
                    name=key,
                    path=key,
                    namespace=cfg_name,
                    default=str(parameter["default"]).lower(),
                    type=constants.CPP_TYPE_MAP[parameter["type"]],
                    quote="\"" if parameter["type"] in constants.QUOTE_TYPES else ""
                )
            )

        # else setup the namespaces
        else:
            # start the namespace
            header_file_pointer.write(
                constants.NAMESPACE_OPEN.format(name=key))
            cpp_file_pointer.write(constants.NAMESPACE_OPEN.format(name=key))

            __header_and_cpp_gen(
                param_info[key], cfg_name, header_file_pointer, cpp_file_pointer)

            # end the namespace
            header_file_pointer.write(constants.NAMESPACE_CLOSE)
            cpp_file_pointer.write(constants.NAMESPACE_CLOSE)


def generate_server_node(param_info: dict, output_path: str):
    """This function must be called after the cfg, h and cpp files
    have been generated. Those generated cfg files will be setup
    as an indidual server containing the parameters to change

    :param param_info: Return value of load_configuration, dict containing params
    :param output_path: Where the cfgs should be saved to
    :type param_info: dict
    :type output_path: str

    """
    # reconfigure server node file
    with open(output_path+"reconfigure_servers.cpp", 'w+') as reconfigure_server_node:
        reconfigure_server_node.write(constants.NODE_HEADER)

        # include statements
        for key in param_info.keys():
            reconfigure_server_node.write(
                constants.INCLUDE_STATEMENT.format(key))

        # init node
        main_contents = constants.INIT_NODE

        # main servers
        for key in param_info.keys():
            main_contents += constants.NEW_SERVER.format(name=key)

        # spin node
        main_contents += constants.SPIN_NODE

        # write the main function
        reconfigure_server_node.write(
            constants.MAIN_FUNC.format(main_contents))

    print('===== created server node header')


#######################################################################
#                                MAIN                                 #
#######################################################################
if __name__ == '__main__':
    # get config
    config = load_configuration(constants.PATH_TO_YAML)

    # create folders
    if not os.path.exists(constants.PATH_TO_AUTOGEN_CPP):
        os.mkdir(constants.PATH_TO_AUTOGEN_CPP)
    if not os.path.exists(constants.PATH_TO_AUTOGEN_CFG):
        os.mkdir(constants.PATH_TO_AUTOGEN_CFG)

    # generate files
    generate_cfg(config, constants.PATH_TO_AUTOGEN_CFG)
    generate_header_and_cpp(config, constants.PATH_TO_AUTOGEN_CPP)
    generate_server_node(config, constants.PATH_TO_AUTOGEN_NODE)
