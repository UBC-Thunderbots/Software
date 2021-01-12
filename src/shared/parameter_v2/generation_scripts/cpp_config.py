from __future__ import annotations
from cpp_parameter import CppParameter
from typing import List
from collections.abc import Iterable
from case_conversion import to_snake_case
import networkx as nx

#######################################################################
#                             CPP Config                              #
#######################################################################

TAB_SIZE = 4
HALF_TAB_SIZE = 2

FORWARD_DECLARATION = "class {config_name};"

CONFIG_CONSTRUCTOR_HEADER_NO_INCLUDES = "{config_name}()"

CONFIG_CONSTRUCTOR_HEADER_WITH_INCLUDES = """{config_name}(
        {config_constructor_definition_args}
    )
    : {config_constructor_initializer_list}"""

INCLUDED_CONFIG_CONSTRUCTOR_ARG_ENTRY = (
    "std::shared_ptr<{config_name}> {config_arg_name}"
)

INCLUDED_CONFIG_CONSTRUCTOR_INITIALIZER_LIST_ENTRY = (
    "{config_variable_name}({config_arg_name})"
)

COMMAND_LINE_ARG_STRUCT = """struct commandLineArgs {{
            bool help = false;
            {command_line_arg_struct_contents}
        }};"""

INCLUDED_CONFIG_COMMAND_LINE_ARG_STRUCT = """struct {config_name}CommandLineArgs
        {{
            {command_line_arg_struct_contents}
        }};"""

INCLUDED_CONFIG_COMMAND_LINE_ARG_ENTRY = (
    "{config_name}CommandLineArgs {config_arg_name};"
)

CONFIG_PUBLIC_ENTRY = """const std::shared_ptr<const {config_name}> {immutable_accessor_name}() const
    {{
        return std::const_pointer_cast<const {config_name}>({config_variable_name});
    }}

    const std::shared_ptr<{config_name}> {mutable_accessor_name}()
    {{
        return {config_variable_name};
    }}"""

CONFIG_PRIVATE_ENTRY = "std::shared_ptr<{config_name}> {config_variable_name};"

CONFIG_CONSTRUCTOR_ENTRY = (
    "{config_variable_name} = std::make_shared<{config_name}>({args});"
)

IMMUTABLE_PARAMETER_LIST_CONFIG_ENTRY = (
    "std::const_pointer_cast<const {config_name}>({config_variable_name})"
)

CONFIG_CLASS = """class {config_name} : public Config
{{
   public:
    {config_constructor_header}
    {{
        {constructor_entries}
        mutable_internal_param_list = {{
            {mutable_parameter_list_entries}
        }};
        immutable_internal_param_list = {{
            {immutable_parameter_list_entries}
        }};
    }}

    {public_entries}

    const std::string name() const
    {{
        return "{config_name}";
    }}

    bool loadFromCommandLineArguments(int argc, char **argv) {{
        {command_line_arg_structs}

        commandLineArgs args;
        boost::program_options::options_description desc{{"Options"}};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help), "Help screen");
        {parse_command_line_args_function_contents}

        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        {load_command_line_args_into_config_contents}

        if (args.help)
        {{
            std::cout << desc << std::endl;
        }}

        return args.help;
    }}

    const MutableParameterList& getMutableParameterList()
    {{
        return mutable_internal_param_list;
    }}

    const ParameterList& getParameterList() const
    {{
        return immutable_internal_param_list;
    }}

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    {private_entries}
}};
"""


class CppConfig(object):
    def __init__(self, config_name: str, is_top_level_config: bool = False):
        """Initializes a CppConfig object, which can generate various strings specific to a config through properties. Some of the properties depend on having the dependency_graph set.

        :param config_name: the name of the config
        :param is_top_level_config: true if this is the top level config, false otherwise
        """
        self.config_name = config_name
        self.config_variable_name = to_snake_case(config_name) + "_config"
        self.is_top_level_config = is_top_level_config
        self.parameters: List[CppParameter] = list()
        self.configs: List[CppConfig] = list()

    def add_parameter(self, parameter: CppParameter):
        self.parameters.append(parameter)

    def get_parameters(self) -> List[CppParameter]:
        return self.parameters.copy()

    def include_config(self, config: CppConfig):
        self.configs.append(config)

    @staticmethod
    def join_with_tabs(
        str: str, iterable: Iterable, num_tabs: int, half_tab: bool = False
    ) -> str:
        tab_size = HALF_TAB_SIZE if half_tab else TAB_SIZE
        return (str + num_tabs * tab_size * " ").join(iterable)

    @property
    def dependency_graph(self):
        return self.__dependency_graph

    @dependency_graph.setter
    def dependency_graph(self, dependency_graph: nx.DiGraph):
        self.__dependency_graph = dependency_graph

        self.dependency_graph_topological_order_configs = [
            self.dependency_graph.nodes[node]["config"]
            for node in list(reversed(list(nx.topological_sort(self.dependency_graph))))
        ]

        self.included_config_command_line_arg_entries = []
        self.included_config_load_command_line_args_into_config_contents = []
        for config in self.configs:
            self.dfs_helper(config, "", "")

    def dfs_helper(self, config: CppConfig, arg_prefix: str, load_dependency: str):
        """A depth first search helper for adding the necessary prefix to accessing and setting
        parameters of included configs in loadFromCommmandLineArguments function

        :param config: the current CppConfig object
        :param arg_prefix: the prefix for accessing the arg struct
        :param load_depndency: the prefix for accessing the actual parameter
        """
        arg_prefix = (
            to_snake_case(config.config_name)
            if not arg_prefix
            else arg_prefix + "." + to_snake_case(config.config_name)
        )
        load_dependency = load_dependency + "getMutable{config_name}()->".format(
            config_name=config.config_name
        )

        mutable_param_gen = (
            param for param in config.parameters if not param.is_constant
        )
        for param in mutable_param_gen:
            self.included_config_command_line_arg_entries.append(
                param.command_line_option_entry_with_prefix(arg_prefix + ".")
            )
            self.included_config_load_command_line_args_into_config_contents.append(
                param.load_command_line_arg_into_config_with_dependencies(
                    load_dependency, arg_prefix + "."
                )
            )

        # top level config has access to all configs, so no need to recursively add options
        # if not self.is_top_level_config:
        for included_config in config.configs:
            self.dfs_helper(included_config, arg_prefix, load_dependency)

    @property
    def forward_declaration(self):
        return FORWARD_DECLARATION.format(config_name=self.config_name)

    @property
    def definition(self):
        return CONFIG_CLASS.format(
            config_name=self.config_name,
            config_constructor_header=self.config_constructor_header,
            constructor_entries=self.constructor_entries,
            mutable_parameter_list_entries=self.mutable_parameter_list_entries,
            immutable_parameter_list_entries=self.immutable_parameter_list_entries,
            public_entries=self.public_entries,
            private_entries=self.private_entries,
            parse_command_line_args_function_contents=self.parse_command_line_args_function_contents,
            command_line_arg_structs=self.command_line_arg_structs,
            load_command_line_args_into_config_contents=self.load_command_line_args_into_config_contents,
        )

    @property
    def included_config_constructor_arg_entry(self):
        return INCLUDED_CONFIG_CONSTRUCTOR_ARG_ENTRY.format(
            config_name=self.config_name,
            config_arg_name=to_snake_case(self.config_name),
        )

    @property
    def included_config_constructor_initializer_list_entry(self):
        return INCLUDED_CONFIG_CONSTRUCTOR_INITIALIZER_LIST_ENTRY.format(
            config_variable_name=self.config_variable_name,
            config_arg_name=to_snake_case(self.config_name),
        )

    @property
    def config_constructor_args(self):
        return CppConfig.join_with_tabs(
            ", ", [conf.config_variable_name for conf in self.configs], 0
        )

    @property
    def config_constructor_definition_args(self):
        return CppConfig.join_with_tabs(
            ",\n",
            [conf.included_config_constructor_arg_entry for conf in self.configs],
            2,
        )

    @property
    def config_constructor_initializer_list(self):
        return CppConfig.join_with_tabs(
            ",\n",
            [
                conf.included_config_constructor_initializer_list_entry
                for conf in self.configs
            ],
            3,
            True,
        )

    @property
    def config_constructor_header(self):
        return (
            CONFIG_CONSTRUCTOR_HEADER_NO_INCLUDES.format(config_name=self.config_name)
            if self.is_top_level_config or not self.configs
            else CONFIG_CONSTRUCTOR_HEADER_WITH_INCLUDES.format(
                config_name=self.config_name,
                config_constructor_definition_args=self.config_constructor_definition_args,
                config_constructor_initializer_list=self.config_constructor_initializer_list,
            )
        )

    @property
    def config_constructor_entry(self):
        return CONFIG_CONSTRUCTOR_ENTRY.format(
            config_variable_name=self.config_variable_name,
            config_name=self.config_name,
            args=self.config_constructor_args,
        )

    @property
    def constructor_entries(self):
        return CppConfig.join_with_tabs(
            "\n",
            [
                conf.config_constructor_entry
                for conf in self.dependency_graph_topological_order_configs
            ]
            if self.is_top_level_config
            else [param.constructor_entry for param in self.parameters],
            2,
        )

    @property
    def mutable_parameter_list_entries(self):
        return CppConfig.join_with_tabs(
            ",\n",
            [
                param.param_variable_name
                for param in self.parameters
                if not param.is_constant
            ]
            + [conf.config_variable_name for conf in self.configs],
            3,
        )

    @property
    def immutable_parameter_list_config_entry(self):
        return IMMUTABLE_PARAMETER_LIST_CONFIG_ENTRY.format(
            config_name=self.config_name,
            config_variable_name=self.config_variable_name,
        )

    @property
    def immutable_parameter_list_entries(self):
        return CppConfig.join_with_tabs(
            ",\n",
            [param.immutable_parameter_list_entry for param in self.parameters]
            + [conf.immutable_parameter_list_config_entry for conf in self.configs],
            3,
        )

    @property
    def config_public_entry(self):
        return CONFIG_PUBLIC_ENTRY.format(
            config_name=self.config_name,
            immutable_accessor_name="get" + self.config_name,
            mutable_accessor_name="getMutable" + self.config_name,
            config_variable_name=self.config_variable_name,
        )

    @property
    def config_private_entry(self):
        return CONFIG_PRIVATE_ENTRY.format(
            config_name=self.config_name, config_variable_name=self.config_variable_name
        )

    @property
    def public_entries(self):
        return CppConfig.join_with_tabs(
            "\n\n",
            [param.parameter_public_entry for param in self.parameters]
            + [conf.config_public_entry for conf in self.configs],
            1,
        )

    @property
    def private_entries(self):
        return CppConfig.join_with_tabs(
            "\n",
            [param.parameter_private_entry for param in self.parameters]
            + [conf.config_private_entry for conf in self.configs],
            1,
        )

    @property
    def parse_command_line_args_function_contents(self):
        return CppConfig.join_with_tabs(
            "\n",
            [
                param.parameter_command_line_option_entry
                for param in self.parameters
                if not param.is_constant
            ]
            + self.included_config_command_line_arg_entries,
            2,
        )

    @property
    def included_config_command_line_arg_entry(self):
        return INCLUDED_CONFIG_COMMAND_LINE_ARG_ENTRY.format(
            config_name=self.config_name,
            config_arg_name=to_snake_case(self.config_name),
        )

    @property
    def included_config_command_line_arg_struct(self):
        return INCLUDED_CONFIG_COMMAND_LINE_ARG_STRUCT.format(
            config_name=self.config_name,
            command_line_arg_struct_contents=self.command_line_arg_struct_contents,
        )

    @property
    def command_line_arg_struct_contents(self):
        return CppConfig.join_with_tabs(
            "\n",
            [
                param.command_line_arg_entry
                for param in self.parameters
                if not param.is_constant
            ]
            + [conf.included_config_command_line_arg_entry for conf in self.configs],
            3,
        )

    @property
    def command_line_arg_struct(self):
        return COMMAND_LINE_ARG_STRUCT.format(
            command_line_arg_struct_contents=self.command_line_arg_struct_contents
        )

    @property
    def command_line_arg_structs(self):
        return CppConfig.join_with_tabs(
            "\n\n",
            [
                conf.included_config_command_line_arg_struct
                for conf in self.dependency_graph_topological_order_configs
            ]
            + [self.command_line_arg_struct],
            2,
        )

    @property
    def load_command_line_args_into_config_contents(self):
        return CppConfig.join_with_tabs(
            "\n",
            [
                param.load_command_line_arg_into_config
                for param in self.parameters
                if not param.is_constant
            ]
            + self.included_config_load_command_line_args_into_config_contents,
            2,
        )
