from type_map import C_TYPE_MAP

#######################################################################
#                             C Parameter                             #
#######################################################################


class CParameter(object):

    DEFINITION = "const {type} {name};\n"
    INITIALIZATION = ".{name} = {value},\n"

    def __init__(self, param_name: str, param_type: str, param_value: str):
        """Initializes a CParameter with the given type and value. The
        corresponding generation strings (definition, initialization)
        are available through read-only properties.

        :param param_name: The name of the parameter, as defined in the yaml
        :param param_type: The type of the parameter, as defined in the yaml
        :param param_value: The constant value the parameter should hold

        """

        adjusted_value = ""

        # python capitalizes the first letter of the boolean, so we
        # convert that to a string that is all lowercase
        if param_type == "bool":
            adjusted_value = "true" if param_value else "false"

        elif param_type == "char*":
            adjusted_value = '"' + param_value + '"'

        elif param_type == "float":
            adjusted_value = str(param_value) + "f"

        else:
            adjusted_value = param_value

        self.__definition = CParameter.DEFINITION.format(
            type=param_type, name=param_name
        )

        self.__initialization = CParameter.INITIALIZATION.format(
            name=param_name, value=adjusted_value
        )

    @property
    def definition(self):
        return self.__definition

    @property
    def initialization(self):
        return self.__initialization
