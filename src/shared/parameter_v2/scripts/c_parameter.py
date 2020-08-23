from type_map import C_TYPE_MAP

#######################################################################
#                             C Parameter                             #
#######################################################################


class CParameter(object):

    DEFINITION = "const {type}_t* {name};\n"
    CONSTRUCTOR = "app_dynamic_parameters_create{type}({value})"
    INITIALIZATION = ".{name} = {constructor},\n"
    DESTRUCTOR = "app_dynamic_parameters_destroy{type}({ptr});\n"

    def __init__(
        self, param_name: str, param_type: str, param_value: str, ptr_to_instance: str
    ):
        """Initializes a CParameter with the given type and value. The
        corresponding generation strings (definition, constructor, destructor)
        are available through read-only properties.

        :param param_name: The name of the parameter, as defined in the yaml
        :param param_type: The psudo-class type of the parameter (without "_t")
        :param param_value: The constant value the parameter should hold
        :param ptr_to_instance: A string representation of where this parameter
            is located (ex: FooConfig->foo_int)

        """

        self.param_type = param_type
        self.param_value = param_value
        self.ptr_to_instance = ptr_to_instance

        self.__definition = CParameter.DEFINITION.format(
            type=param_type, name=param_name
        )

        self.__constructor = CParameter.CONSTRUCTOR.format(
            type=param_type, value=param_value
        )

        self.__destructor = CParameter.DESTRUCTOR.format(
            type=param_type, ptr=ptr_to_instance
        )

        self.__initialization = CParameter.INITIALIZATION.format(
            name=param_name, constructor=self.constructor
        )

    @property
    def definition(self):
        return self.__definition

    @property
    def constructor(self):
        return self.__constructor

    @property
    def destructor(self):
        return self.__destructor

    @property
    def initialization(self):
        return self.__initialization
