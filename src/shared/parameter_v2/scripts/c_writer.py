from c_config import CConfig
from c_parameter import CParameter
from typing import List, Set
from dynamic_parameter_schema import PARAMETER_KEY, INCLUDE_KEY, CONSTANT_KEY
from type_map import C_TYPE_MAP

#######################################################################
#                              C Writer                               #
#######################################################################


class CWriter(object):

    CONFIG_H = (
        "#pragma once\n"
        "#include <memory.h>\n"
        "#include <stdlib.h>\n"
        '#include "shared/parameter_v2/c/parameter.h"\n'
        "{contents}\n const "
        "{top_level_config_name}_t* app_dynamic_parameters_create(void);\n"
        "void app_dynamic_parameters_destroy(const {top_level_config_name}_t*"
        "{top_level_config_name}_ptr);\n"
    )

    CONFIG_C = (
        '#include "shared/parameter_v2/c/parameter.h"\n'
        '#include "shared/parameter_v2/c/{output_file}.h"\n'
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

            # we convert the yaml file name to camel case
            # and store that as the config_name
            config_name = CWriter.to_camel_case(config.split(".")[0])

            config = CConfig(
                config_name, "{}_ptr->{}".format(top_level_config_name, config_name)
            )

            top_level_config.include_config(config_name)

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
                            param_metadata["name"],
                            param_type,
                            param_metadata["value"],
                            ptr_to_instance,
                        )

                        config.add_parameter(c_param)

            if INCLUDE_KEY in metadata:
                for included_yaml in metadata["include"]:

                    config.include_config(
                        CWriter.to_camel_case(included_yaml.split(".")[0])
                    )

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

        # generate c file
        with open(f"{output_file}.c", "w") as c_file:

            app_dynamic_parameters_create_impl = ""
            app_dynamic_parameters_destroy_impl = ""

            # malloc all the cofings
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
                for param in config.get_parameters():
                    app_dynamic_parameters_destroy_impl += param.destructor
                app_dynamic_parameters_destroy_impl += config.free

            c_file.write(
                CWriter.CONFIG_C.format(
                    output_file=output_file,
                    top_level_config_name=top_level_config_name,
                    app_dynamic_parameters_destroy_impl=app_dynamic_parameters_destroy_impl,
                    app_dynamic_parameters_create_impl=app_dynamic_parameters_create_impl,
                )
            )

        # generate header file
        with open(f"{output_file}.h", "w") as header_file:

            contents = "".join([conf.forward_decleration for conf in c_configs])
            contents += "".join([conf.definition for conf in c_configs])

            header_file.write(
                CWriter.CONFIG_H.format(
                    contents=contents, top_level_config_name=top_level_config_name
                )
            )

    @staticmethod
    def to_camel_case(snake_str):
        return "".join(x.title() for x in snake_str.split("_"))
