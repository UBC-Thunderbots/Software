from c_config import CConfig
from c_parameter import CParameter
from typing import List, Set
from dynamic_parameter_schema import PARAMETER_KEY, INCLUDE_KEY, CONSTANT_KEY
from type_map import C_TYPE_MAP
from case_conversion import to_pascal_case

#######################################################################
#                              C Writer                               #
#######################################################################


class CWriter(object):

    CONFIG_H = (
        "#pragma once\n"
        "#include <memory.h>\n"
        "#include <stdlib.h>\n"
        "{contents}\n const "
        "{top_level_config_name}_t* app_dynamic_parameters_create(void);\n"
        "void app_dynamic_parameters_destroy(const {top_level_config_name}_t*"
        "{top_level_config_name}_ptr);\n"
        "const {top_level_config_name}_t* app_dynamic_parameters_create(void)\n"
        "{{{app_dynamic_parameters_create_impl} \n"
        "return {top_level_config_name}_config;}}\n"
        "void app_dynamic_parameters_destroy(const "
        "{top_level_config_name}_t* {top_level_config_name}_ptr)"
        "{{ {app_dynamic_parameters_destroy_impl} }}\n"
    )

    @staticmethod
    def create_config_list_from_metadata(
        top_level_config_name, config_metadata
    ) -> List[CConfig]:
        """Takes the config metadata loaded by config_yaml_loader and converts
        it to a list of CConfig objects. The CConfig objects contain useful
        string representations that can directly be written to the generated file.

        Note: The top level config includes all other configs by default.

        :param top_level_config_name: The name of the "top level" config that contains all configs
        :param config_metadata: The output of config_yaml_loader
        :returns: list of CConfig

        """
        c_configs = []

        top_level_config = CConfig(
            top_level_config_name, f"{top_level_config_name}_ptr"
        )

        for config, metadata in config_metadata.items():

            # we convert the yaml file name to pascal case
            # and store that as the config_name
            config_name = to_pascal_case(config.split(".")[0])

            config = CConfig(
                config_name, "{}_ptr->{}".format(top_level_config_name, config_name)
            )

            # a config can be empty if it has NO constant parameters
            # an empty config will get generated as an empty struct which is
            # NOT allowed in C, this flag will make sure we don't generate empty_configs
            # which is a config that has neither parameters nor includes
            config_empty = True

            # add all the valid CParameters to the CConfig
            if PARAMETER_KEY in metadata:
                for parameter in metadata[PARAMETER_KEY]:

                    param_metadata = list(parameter.values())[0]
                    param_type = list(parameter.keys())[0]

                    if CONSTANT_KEY in param_metadata:
                        if param_type not in C_TYPE_MAP:
                            # we need to ignore parameters that are
                            # meant for cpp parameters (ex. factory, enum)
                            continue

                        param_type = C_TYPE_MAP[param_type]

                        # this is the fully resolved pointer access
                        # to the location of this parameter
                        ptr_to_instance = "{}->{}".format(
                            config.ptr_to_instance, param_metadata["name"]
                        )

                        c_param = CParameter(
                            param_metadata["name"], param_type, param_metadata["value"],
                        )

                        config.add_parameter(c_param)
                        config_empty = False

            # add all the valid included configs to the CConfig
            if INCLUDE_KEY in metadata:
                for included_yaml in metadata["include"]:
                    config.include_config(to_pascal_case(included_yaml.split(".")[0]))
                    config_empty = False

            if not config_empty:
                top_level_config.include_config(config_name)
                c_configs.append(config)

        c_configs.append(top_level_config)

        return c_configs

    @staticmethod
    def write_config_metadata(
        output_file: str, top_level_config_name: str, config_metadata: dict
    ):
        """Generates the .c and .h files for the C Config and Parameter graph.

        :param output_file: The output file name for the .c and .h file
        :param top_level_config_name: The name of the top level config.
            This config contains all the other declared configs.
        :param config_metadata: The dictionary output from config_yaml_loader

        """
        c_configs = CWriter.create_config_list_from_metadata(
            top_level_config_name, config_metadata
        )

        # generate header file
        with open(f"{output_file}", "w") as header_file:

            # forward declerations and definitions
            contents = "".join([conf.forward_declaration for conf in c_configs])
            contents += "".join([conf.definition for conf in c_configs])

            app_dynamic_parameters_create_impl = ""
            app_dynamic_parameters_destroy_impl = ""

            # malloc all the configs
            for config in c_configs:
                app_dynamic_parameters_create_impl += config.malloc

            # create initialization structs
            for config in c_configs:
                app_dynamic_parameters_create_impl += config.initialization

            # memcpy initialization structs into malloced space
            for config in c_configs:
                app_dynamic_parameters_create_impl += config.memcpy

            # create destructor
            for config in c_configs:
                app_dynamic_parameters_destroy_impl += config.free

            header_file.write(
                CWriter.CONFIG_H.format(
                    contents=contents,
                    output_file=output_file,
                    top_level_config_name=top_level_config_name,
                    app_dynamic_parameters_create_impl=app_dynamic_parameters_create_impl,
                    app_dynamic_parameters_destroy_impl=app_dynamic_parameters_destroy_impl,
                )
            )
