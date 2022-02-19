from type_map import CPP_TYPE_MAP
from dynamic_parameter_schema import CONSTANT_KEY
from case_conversion import to_pascal_case
import re

#######################################################################
#                            CPP Parameter                            #
#######################################################################

PARAMETER_PUBLIC_ENTRY = """const std::shared_ptr<const {param_class}<{type}>> get{immutable_accessor_name}() const
    {{
        return std::const_pointer_cast<const {param_class}<{type}>>({param_variable_name});
    }}

    const std::shared_ptr<{param_class}<{type}>> get{mutable_accessor_name}()
    {{
        return {param_variable_name};
    }}"""

PARAMETER_PUBLIC_ENTRY_CONST = """const std::shared_ptr<const {param_class}<{type}>> get{immutable_accessor_name}() const
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

PARAMETER_COMMAND_LINE_OPTION_BOOL_SWITCH_ENTRY = 'desc.add_options()("{arg_prefix}{param_name}", boost::program_options::bool_switch(&args.{arg_prefix}{param_name}), "{param_desc}");'

PARAMETER_COMMAND_LINE_OPTION_ENTRY = 'desc.add_options()("{arg_prefix}{param_name}", boost::program_options::value<{type}>(&args.{arg_prefix}{param_name}), "{param_desc}");'

COMMAND_LINE_ARG_ENTRY = "{param_type} {param_name} = {quote}{value}{quote};"

LOAD_COMMAND_LINE_ARG_INTO_CONFIG = "this->{dependencies}getMutable{param_accessor_name}()->setValue(args.{arg_prefix}{param_name});"

TO_PROTO_ENTRY = "config_proto.set_{param_name}({param_variable_name}->value());"

LOAD_FROM_PROTO_ENTRY = "{param_variable_name}->setValue(config_proto.{param_name}());"


class CppParameter(object):
    def __init__(self, param_type: str, param_metadata: dict):
        """Initializes a CppParameter object, which can generate various strings specific to a parameter through properties.

        :param param_type: the type of parameter (ex: bool, int)
        :param parm_metadata: dictionary containing the metadata about the parameter (ex: name, value)
        """
        self.param_type = param_type
        self.param_metadata = param_metadata
        self.param_name = param_metadata["name"]
        self.param_variable_name = self.param_name + "_param"
        self.param_description = param_metadata["description"]
        self.param_value = param_metadata["value"]
        self.is_constant = (
            param_metadata[CONSTANT_KEY] if CONSTANT_KEY in param_metadata else False
        )

        self.quote = CppParameter.find_quote(param_type)
        self.cpp_type = CppParameter.find_cpp_type(self.param_type, param_metadata)
        self.param_class = CppParameter.find_param_class(self.param_type)

        # Python stores booleans as True and False, but we need them to be
        # lowercase for C++
        if self.param_type == "bool":
            self.param_value = "true" if self.param_value else "false"

    @staticmethod
    def is_numeric_type(param_type: str) -> bool:
        return re.match("int|double", param_type)

    @staticmethod
    def find_quote(param_type: str) -> str:
        return '"' if re.match("string|enum|factory", param_type) else ""

    @staticmethod
    def find_param_class(param_type: str) -> str:
        if CppParameter.is_numeric_type(param_type):
            return "NumericParameter"
        elif param_type == "enum" or param_type == "factory":
            return "EnumeratedParameter"
        else:
            return "Parameter"

    @staticmethod
    def find_cpp_type(param_type: str, param_metadata: dict) -> str:
        if param_type == "factory":
            return param_metadata["index_type"]
        else:
            return CPP_TYPE_MAP[param_type]

    def command_line_option_entry_with_prefix(self, arg_prefix: str):
        return (
            PARAMETER_COMMAND_LINE_OPTION_ENTRY.format(
                param_name=self.param_name,
                arg_prefix=arg_prefix,
                type=self.cpp_type,
                param_desc=self.param_description.replace("\n", "\\n").replace(
                    '"', '\\"'
                ),
            )
            if self.param_type != "bool"
            else PARAMETER_COMMAND_LINE_OPTION_BOOL_SWITCH_ENTRY.format(
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
            param_accessor_name=to_pascal_case(self.param_name),
            dependencies=dependencies,
            arg_prefix=arg_prefix,
        )

    @property
    def parameter_public_entry(self):
        return (
            PARAMETER_PUBLIC_ENTRY_CONST.format(
                param_class=self.param_class,
                type=self.cpp_type,
                immutable_accessor_name=to_pascal_case(self.param_name),
                param_variable_name=self.param_variable_name,
            )
            if self.is_constant
            else PARAMETER_PUBLIC_ENTRY.format(
                param_class=self.param_class,
                type=self.cpp_type,
                immutable_accessor_name=to_pascal_case(self.param_name),
                mutable_accessor_name="Mutable" + to_pascal_case(self.param_name),
                param_variable_name=self.param_variable_name,
            )
        )

    @property
    def parameter_private_entry(self):
        return PARAMETER_PRIVATE_ENTRY.format(
            param_class=self.param_class,
            type=self.cpp_type,
            param_variable_name=self.param_variable_name,
        )

    @property
    def constructor_entry(self):
        if CppParameter.is_numeric_type(self.param_type):
            return NUMERIC_PARAMETER_CONSTRUCTOR_ENTRY.format(
                param_variable_name=self.param_variable_name,
                type=self.cpp_type,
                param_name=self.param_name,
                value=self.param_value,
                min_value=self.param_metadata["min"],
                max_value=self.param_metadata["max"],
            )
        elif self.param_type == "enum":
            param_enum = self.param_metadata["enum"]
            return ENUMERATED_PARAMETER_CONSTRUCTOR_ENTRY.format(
                param_variable_name=self.param_variable_name,
                type=self.cpp_type,
                param_name=self.param_name,
                quote=self.quote,
                value=self.param_value,
                allowed_values="allStringValues{enum}()".format(enum=param_enum),
            )
        elif self.param_type == "factory":
            param_index_type = self.param_metadata["index_type"]
            param_type_to_create = self.param_metadata["type_to_create"]
            param_config_type = self.param_metadata["config_type"]
            return ENUMERATED_PARAMETER_CONSTRUCTOR_ENTRY.format(
                param_variable_name=self.param_variable_name,
                type=param_index_type,
                param_name=self.param_name,
                quote=self.quote,
                value=self.param_value,
                forward_declare=param_type_to_create,
                allowed_values="GenericFactory<{index_type}, {type_to_create}, {config_type}>::getRegisteredNames()".format(
                    index_type=param_index_type,
                    type_to_create=param_type_to_create,
                    config_type=param_config_type,
                ),
            )
        else:
            return PARAMETER_CONSTRUCTOR_ENTRY.format(
                param_variable_name=self.param_variable_name,
                type=self.cpp_type,
                param_name=self.param_name,
                quote=self.quote,
                value=self.param_value,
            )

    @property
    def immutable_parameter_list_entry(self):
        return IMMUTABLE_PARAMETER_LIST_PARAMETER_ENTRY.format(
            param_class=self.param_class,
            type=self.cpp_type,
            param_variable_name=self.param_variable_name,
        )

    @property
    def command_line_arg_entry(self):
        return COMMAND_LINE_ARG_ENTRY.format(
            param_type=self.cpp_type,
            param_name=self.param_name,
            quote=self.quote,
            value=self.param_value,
        )

    @property
    def parameter_command_line_option_entry(self):
        return (
            PARAMETER_COMMAND_LINE_OPTION_ENTRY.format(
                param_name=self.param_name,
                type=self.cpp_type,
                param_desc=self.param_description.replace("\n", "\\n").replace(
                    '"', '\\"'
                ),
                arg_prefix="",
            )
            if self.param_type != "bool"
            else PARAMETER_COMMAND_LINE_OPTION_BOOL_SWITCH_ENTRY.format(
                param_name=self.param_name,
                param_desc=self.param_description.replace("\n", "\\n").replace(
                    '"', '\\"'
                ),
                arg_prefix="",
            )
        )

    @property
    def load_command_line_arg_into_config(self):
        return LOAD_COMMAND_LINE_ARG_INTO_CONFIG.format(
            param_name=self.param_name,
            param_accessor_name=to_pascal_case(self.param_name),
            dependencies="",
            arg_prefix="",
        )

    @property
    def to_proto_entry(self):
        return TO_PROTO_ENTRY.format(
            param_name=self.param_name, param_variable_name=self.param_variable_name,
        )

    @property
    def load_from_proto_entry(self):
        return LOAD_FROM_PROTO_ENTRY.format(
            param_name=self.param_name, param_variable_name=self.param_variable_name,
        )
