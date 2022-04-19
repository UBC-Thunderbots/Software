from proto.parameter_pb2 import ThunderbotsConfig

from dynamic_parameter_schema import (
    INCLUDE_KEY,
    PARAMETER_KEY,
)

class ConfigProtoLoader(object):

    @staticmethod
    def __create_bool_parameter(key, value, descriptor):
        """Convert a bool field in proto to a BoolParameter

        :param key: The name of the parameter
        :param value: The default value
        :param _: The proto descriptor, unused for bool

        """
        return {"name": key, "type": "bool", "value": value}


    @staticmethod
    def __create_int_parameter(key, value, descriptor):
        # Extract the options from the descriptor, and store it
        # in the dictionary.
        options = MessageToDict(
            descriptor.GetOptions(), preserving_proto_field_name=True
        )

        try:
            min_max = options["[TbotsProto.bounds]"]
        except KeyError:
            raise KeyError("{} missing ParameterRangeOptions".format(key))

        return {
            "name": key,
            "type": "slider",
            "value": value,
            "default": value,
            "limits": (int(min_max["min_int_value"]), int(min_max["max_int_value"])),
            "step": 1,
        }

    @staticmethod
    def get_config_metadata(proto) -> dict:
        """Loads the provided parameter proto file and returns the metadata.

        NOTE: This function is designed to be compatible with the legacy
        ConfigYamlLoader and the cpp dynamic parameter generation scripts.

        Eventually, we should be able to remove this code generation.

        :param proto: The proto type
        :returns config_metadata: the config name followed by a list of
            included configs and a list of defined parameters.
        {
            "robot_navigation_obstacle_config.yaml":
            {
                "include":[
                     "ai_control_config.yaml",
                     "attacker_tactic_config.yaml",
                     "dribble_tactic_config.yaml",
                     "corner_kick_play_config.yaml",
                     "defense_play_config.yaml",
                     "enemy_capability_config.yaml",
                     "goalie_tactic_config.yaml",
                     "navigator_config.yaml",
                ],
               "parameters":[
                  {
                     "double":
                     {
                        "name":"speed_scaling_factor",
                        "min":0.0,
                        "max":1.0,
                        "value":0.2,
                        "type":"double",
                     }
                  },
                  {
                     "double":
                     {
                        "name":"allowed_robot_collision_speed",
                        "min":0,
                        "max":1,
                        "value":0.2,
                        "type":"double",
                     }
                  }
               ]
            },
            ...
        }

        """

        config_metadata = {}
        field_list = []


        for descriptor in proto.DESCRIPTOR.fields:

            key = descriptor.name
            value = getattr(message, descriptor.name)

            if descriptor.type == descriptor.TYPE_MESSAGE:
                field_list.append(
                    {
                        "name": key,
                        "type": "group",
                        "children": self.config_proto_to_param_dict(
                            value, search_term, convert_all_fields_to_bools
                        ),
                    }
                )

            else:
                field_list.append({
                    descriptor.TYPE_BOOL: self.__create_bool_parameter,
                    descriptor.TYPE_ENUM: self.__create_enum_parameter,
                    descriptor.TYPE_INT32: self.__create_int_parameter,
                    descriptor.TYPE_INT64: self.__create_int_parameter,
                    descriptor.TYPE_DOUBLE: self.__create_double_parameter,
                    descriptor.TYPE_STRING: self.__create_string_parameter,
                }[descriptor.type](key, value, descriptor))

        if field_list:
            return field_list

        return message_dict
