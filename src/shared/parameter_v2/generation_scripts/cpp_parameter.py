from type_map import CPP_TYPE_MAP
from dynamic_parameter_schema import CONSTANT_KEY
import re

#######################################################################
#                            CPP Parameter                            #
#######################################################################

PARAMETER_PUBLIC_ENTRY = """const std::shared_ptr<const {param_class}<{type}>> {immutable_accessor_name}() const
    {{
        return std::const_pointer_cast<const {param_class}<{type}>>({param_variable_name});
    }}

    const std::shared_ptr<{param_class}<{type}>> {mutable_accessor_name}()
    {{
        return {param_variable_name};
    }}"""

PARAMETER_PUBLIC_ENTRY_CONST = """const std::shared_ptr<const {param_class}<{type}>> {immutable_accessor_name}() const
    {{
        return std::const_pointer_cast<const {param_class}<{type}>>({param_variable_name});
    }}"""

PARAMETER_PRIVATE_ENTRY = (
    "std::shared_ptr<{param_class}<{type}>> {param_variable_name};"
)

PARAMETER_CONSTRUCTOR_ENTRY = '{param_variable_name} = std::make_shared<Parameter<{type}>>("{param_name}", {quote}{value}{quote});'
NUMERIC_PARAMETER_CONSTRUCTOR_ENTRY = '{param_variable_name} = std::make_shared<NumericParameter<{type}>>("{param_name}", {value}, {min_value}, {max_value});'
ENUMERATED_PARAMETER_CONSTRUCTOR_ENTRY = '{param_variable_name} = std::make_shared<EnumeratedParameter<{type}>>("{param_name}", {quote}{value}{quote}, {allowed_values});'

IMMUTABLE_PARAMETER_LIST_PARAMETER_ENTRY = (
    "std::const_pointer_cast<const {param_class}<{type}>>({param_variable_name})"
)

PARAMETER_COMMAND_LINE_BOOL_SWITCH_ENTRY = 'desc.add_options()("{param_name}", boost::program_options::bool_switch(&args.{arg_prefix}{param_name}), "{param_desc}");'

PARAMETER_COMMAND_LINE_ENTRY = 'desc.add_options()("{arg_prefix}{param_name}", boost::program_options::value<{type}>(&args.{arg_prefix}{param_name}), "{param_desc}");'

COMMAND_LINE_ARG_ENTRY = "{param_type} {param_name} = {quote}{value}{quote};"

LOAD_COMMAND_LINE_ARG_INTO_CONFIG = "this->{dependencies}mutable{param_name}()->setValue(args.{arg_prefix}{param_name});"

# These are to be used of minimum and maximum value attributes are missing from numeric parameters
NUMERIC_PARAMETER_MIN = "std::numeric_limits<{type}>::min()"
NUMERIC_PARAMETER_MAX = "std::numeric_limits<{type}>::max()"

# TODO: support constant


class CppParameter(object):
    def __init__(self, param_type: str, param_metadata: dict):
        self.param_type = param_type
        self.param_name = param_metadata["name"]
        self.__param_variable_name = self.param_name + "_param"
        self.param_description = param_metadata["description"] #TODO: change strings to KEY
        self.is_constant = param_metadata[CONSTANT_KEY] if CONSTANT_KEY in param_metadata else False
        param_value = param_metadata["value"]

        quote = CppParameter.find_quote(param_type)

        # Python stores booleans as True and False, but we need them to be
        # lowercase for C++
        if self.param_type == "bool":
            param_value = "true" if param_value else "false"

        if CppParameter.is_numeric_type(self.param_type):
            min_value = (
                param_metadata["min"]
                if "min" in param_metadata
                else NUMERIC_PARAMETER_MIN.format(type=self.param_type)
            )
            max_value = (
                param_metadata["max"]
                if "max" in param_metadata
                else NUMERIC_PARAMETER_MAX.format(type=self.param_type)
            )
            self.__constructor_entry = NUMERIC_PARAMETER_CONSTRUCTOR_ENTRY.format(
                param_variable_name=self.param_variable_name,
                type=self.param_type,
                param_name=self.param_name,
                value=param_value,
                min_value=min_value,
                max_value=max_value,
            )
        elif self.param_type == "enum":
            param_enum = param_metadata["enum"]
            self.__constructor_entry = ENUMERATED_PARAMETER_CONSTRUCTOR_ENTRY.format(
                param_variable_name=self.param_variable_name,
                type="std::string",
                param_name=self.param_name,
                quote=quote,
                value=param_value,
                allowed_values="allValues{enum}()".format(enum=param_enum),
            )
        elif self.param_type == "factory":
            param_index_type = param_metadata["index_type"]
            param_type_to_create = param_metadata["type_to_create"]
            self.__constructor_entry = ENUMERATED_PARAMETER_CONSTRUCTOR_ENTRY.format(
                param_variable_name=self.param_variable_name,
                type=param_index_type,
                param_name=self.param_name,
                quote=quote,
                value=param_value,
                allowed_values="GenericFactory<{index_type}, {type_to_create}>::getRegisteredNames()".format(
                    index_type=param_index_type, type_to_create=param_type_to_create
                ),
            )
        else:
            self.__constructor_entry = PARAMETER_CONSTRUCTOR_ENTRY.format(
                param_variable_name=self.param_variable_name,
                type=self.param_type if self.param_type != "string" else "std::string",
                param_name=self.param_name,
                quote=quote,
                value=param_value,
            )

        param_class = CppParameter.param_type_to_param_class(self.param_type)
        self.type_parameter = CppParameter.param_type_to_type_parameter(
            self.param_type, param_metadata
        )

        # TODO: Should these self.__ props just be defined in function instead?
        self.__immutable_paramter_list_entry = IMMUTABLE_PARAMETER_LIST_PARAMETER_ENTRY.format(
            param_class=param_class,
            type=self.type_parameter,
            param_variable_name=self.param_variable_name,
        )

        self.__parameter_public_entry = PARAMETER_PUBLIC_ENTRY_CONST.format(
            param_class=param_class,
            type=self.type_parameter,
            immutable_accessor_name=self.param_name,
            param_variable_name=self.param_variable_name,
        ) if self.is_constant else PARAMETER_PUBLIC_ENTRY.format(
            param_class=param_class,
            type=self.type_parameter,
            immutable_accessor_name=self.param_name,
            mutable_accessor_name="mutable" + self.param_name,
            param_variable_name=self.param_variable_name,
        )

        self.__parameter_private_entry = PARAMETER_PRIVATE_ENTRY.format(
            param_class=param_class,
            type=self.type_parameter,
            param_variable_name=self.param_variable_name,
        )

        self.__command_line_arg_entry = COMMAND_LINE_ARG_ENTRY.format(
            param_type=self.type_parameter,
            param_name=self.param_name,
            quote=quote,
            value=param_value if self.param_type != "float" else str(param_value) + "f",
        )

        self.__parameter_command_line_entry = (
            PARAMETER_COMMAND_LINE_ENTRY.format(
                param_name=self.param_name,
                type=self.type_parameter,
                param_desc=self.param_description.replace("\n", "\\n").replace(
                    '"', '\\"'
                ),
                arg_prefix="",
            )
            if self.param_type != "bool"
            else PARAMETER_COMMAND_LINE_BOOL_SWITCH_ENTRY.format(
                param_name=self.param_name,
                param_desc=self.param_description.replace("\n", "\\n").replace(
                    '"', '\\"'
                ),
                arg_prefix="",
            )
        )

        self.__load_command_line_arg_into_config = LOAD_COMMAND_LINE_ARG_INTO_CONFIG.format(
            param_name=self.param_name, dependencies="", arg_prefix="",
        )

    @staticmethod
    def is_numeric_type(param_type: str) -> bool:
        return re.match("int|uint|float", param_type)

    @staticmethod
    def find_quote(param_type: str) -> str:
        return '"' if re.match("string|enum|factory", param_type) else ""

    @staticmethod
    def param_type_to_param_class(param_type: str) -> str:
        if CppParameter.is_numeric_type(param_type):
            return "NumericParameter"
        elif param_type == "enum" or param_type == "factory":
            return "EnumeratedParameter"
        else:
            return "Parameter"

    @staticmethod
    def param_type_to_type_parameter(
        param_type: str, param_metadata: dict
    ) -> str:  # TODO: clear up type_parameter (ie. param_type_to_cpp_type), and param vs parameter
        if param_type == "bool" or CppParameter.is_numeric_type(param_type):
            return param_type
        elif param_type == "factory":
            return param_metadata["index_type"]
        else:
            # string or enum
            return "std::string"

    def command_line_option_entry_with_prefix(self, arg_prefix: str):
        # TODO: add "_option_ to others"
        return (
            PARAMETER_COMMAND_LINE_ENTRY.format(
                param_name=self.param_name,
                arg_prefix=arg_prefix,
                type=self.type_parameter,
                param_desc=self.param_description.replace("\n", "\\n").replace(
                    '"', '\\"'
                ),
            )
            if self.param_type != "bool"
            else PARAMETER_COMMAND_LINE_BOOL_SWITCH_ENTRY.format(
                param_name=self.param_name,
                arg_prefix=arg_prefix,
                param_desc=self.param_description.replace("\n", "\\n").replace(
                    '"', '\\"'
                ),
            )
        )

    def load_command_line_arg_into_config_with_dependencies(
        self, dependencies: str, arg_prefix: str
    ):
        return LOAD_COMMAND_LINE_ARG_INTO_CONFIG.format(
            param_name=self.param_name,
            dependencies=dependencies,
            arg_prefix=arg_prefix,
        )

    @property
    def param_variable_name(self):
        return self.__param_variable_name

    @property
    def parameter_public_entry(self):
        return self.__parameter_public_entry

    @property
    def parameter_private_entry(self):
        return self.__parameter_private_entry

    @property
    def constructor_entry(self):
        return self.__constructor_entry

    @property
    def immutable_parameter_list_entry(self):
        return self.__immutable_paramter_list_entry

    @property
    def command_line_arg_entry(self):
        return self.__command_line_arg_entry

    @property
    def parameter_command_line_entry(self):
        return self.__parameter_command_line_entry

    @property
    def load_command_line_arg_into_config(self):
        return self.__load_command_line_arg_into_config
