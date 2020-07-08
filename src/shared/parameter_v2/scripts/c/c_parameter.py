#######################################################################
#                             C Parameter                             #
#######################################################################


class CParameter(object):

    DEFINITION = "{{const {type}_t* {name}}};"
    CONSTRUCTOR = "app_dynamic_parameters_create{type}({value});"
    INITIALIZATION = ".{name} = {constructor},"
    DESTRUCTOR = "app_dynamic_parameters_destroy{type}({ptr})"

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
        :type param_name: str
        :type param_type: str
        :type param_value: str
        :type ptr_to_instance: str

        """

        self.param_name = param_name
        self.param_type = param_type
        self.param_value = param_value
        self.ptr_to_instance = ptr_to_instance

        self.definition = CParameter.DEFINITION.format(param_type, param_name)
        self.constructor = CParameter.CONSTRUCTOR.format(param_type, param_value)
        self.destructor = CParameter.DESTRUCTOR.format(param_type, ptr_to_instance)
        self.initialization = CParameter.INITIALIZATION.format(
            param_name, self.constructor
        )

    @property
    def definition(self):
        return self.definition

    @property
    def constructor(self):
        return self.constructor

    @property
    def destructor(self):
        return self.destructor

    @property
    def initialization(self):
        return self.initialization
