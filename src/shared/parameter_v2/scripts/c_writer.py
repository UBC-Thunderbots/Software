from c_config import CConfig
from c_parameter import CParameter
from typing import List, Set
from dynamic_parameter_schema import PARAMETER_KEY, INCLUDE_KEY, CONSTANT_KEY
from type_map import C_TYPE_MAP

#######################################################################
#                              C Writer                               #
#######################################################################

CONFIG_FILE_NAME = "tbots_c_config"
TOP_LEVEL_CONFIG_NAME = "ThunderbotsConfig"

CONFIG_H = (
    "#pragma once\n"
    "#include <memory.h>\n"
    "#include <stdlib.h>\n"
    '#include "shared/parameter_v2/c/parameter.h"\n'
    "const {top_level_config_name}* app_dynamic_parameters_create(void);\n"
    "void app_dynamic_parameters_destroy(const ThunderbotsConfig_t* tbots_config);\n"
    "{contents}"
)

CONFIG_C = (
    '#include "shared/parameter_v2/c/tbots_c_config.h" {contents} \nconst'
    " ThunderbotsConfig_t* app_dynamic_parameters_create(void)"
    " {{app_dynamic_parameters_create_impl}}\n"
    "void app_dynamic_parameters_destroy(const ThunderbotsConfig_t* tbots_config)"
    " {{app_dynamic_parameters_destroy_impl}}\n"
)


class CWriter(object):
    @staticmethod
    def write_config_metadata(
        top_level_config_name: str, config_metadata: dict, output_file_dir: str
    ):
        """
        :param top_level_config_name: The name of the top level config.
            This config contains all the other declared configs.
        :param config_metadata: The dictionary output from config_yaml_loader
        :param output_file_dir: The location to place the generated files

        """
        c_configs = []
        top_level_config = CConfig(top_level_config_name, top_level_config_name)

        for config, metadata in config_metadata.items():

            # we convert the yaml file name to camel case
            # and store that as the config_name
            config_name = CWriter.to_camel_case(config.split(".")[0])

            config = CConfig(
                config_name, "{}->{}".format(top_level_config_name, config_name)
            )

            top_level_config.include_config(config)

            # add all the valid CParameters to the CConfig
            if PARAMETER_KEY in metadata:

                for parameter in metadata[PARAMETER_KEY]:
                    param_metadata = list(parameter.values())[0]

                    if CONSTANT_KEY not in param_metadata:

                        param_type = list(parameter.keys())[0]

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

        # generate c file
        with open(f"{CONFIG_FILE_NAME}.c", "w") as c_file:
            contents = ""
            c_file.write(CONFIG_C)
            pass

        # generate header file
        with open(f"{CONFIG_FILE_NAME}.h", "w") as header_file:

            contents = "\n".join([conf.forward_decleration for conf in c_configs])
            contents += "\n".join([conf.definition for conf in c_configs])

            header_file.write(CONFIG_H.format(contents=contents))

    @staticmethod
    def to_camel_case(snake_str):
        return "".join(x.title() for x in snake_str.split("_"))
