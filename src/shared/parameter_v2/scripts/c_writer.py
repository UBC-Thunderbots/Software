from c_config import CConfig
from c_parameter import CParameter
from typing import List, Set
from dynamic_parameter_schema import PARAMETER_KEY, INCLUDE_KEY, CONSTANT_KEY
from type_map import C_TYPE_MAP

#######################################################################
#                              C Writer                               #
#######################################################################


class CWriter(object):
    @staticmethod
    def write_config_metadata(
        top_level_config_name: str, config_metadata: dict, output_file_path: str
    ):
        """TODO: Docstring for write_config.
        :returns: TODO

        """
        ptr_format_str = "{}->{}"
        c_configs = {}

        for config, metadata in config_metadata.items():

            # we convert the yaml file name to camel case
            # and store that as the config_name
            config_name = CWriter.to_camel_case(config.split(".")[0])

            config = CConfig(
                config_name, ptr_format_str.format(
                    top_level_config_name, config_name)
            )

            # add all the valid CParameters to the CConfig
            if PARAMETER_KEY in metadata:

                for parameter in metadata[PARAMETER_KEY]:
                    param_metadata = list(parameter.values())[0]

                    if CONSTANT_KEY not in param_metadata:

                        param_type = list(parameter.keys())[0]

                        if param_type not in C_TYPE_MAP:
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

                        print(c_param.constructor())
                        print(c_param.destructor())
                        print(c_param.initialization())
                        print(c_param.definition())

            if INCLUDE_KEY in metadata:

                for included_yaml in metadata["include"]:
                    config.include_config(CWriter.to_camel_case(
                        included_yaml.split(".")[0]))


    @staticmethod
    def to_camel_case(snake_str):
        components = snake_str.split("_")
        # We capitalize the first letter of each component except the first one
        # with the 'title' method and join them together.
        return components[0] + "".join(x.title() for x in components[1:])
