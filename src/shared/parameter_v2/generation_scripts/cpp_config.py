from __future__ import annotations
from cpp_parameter import CppParameter
from typing import List
from collections.abc import Iterable
import networkx as nx

#######################################################################
#                             CPP Config                              #
#######################################################################

TAB_SIZE = 4
HALF_TAB_SIZE = 2

FORWARD_DECLARATION = "class {config_name};"

CONFIG_CONSTRUCTOR_HEADER_NO_INCLUDES = "{config_name}()"

CONFIG_CONSTRUCTOR_HEADER_WITH_INCLUDES = """{config_name}(
        {config_constructor_args}
    )
    : {config_constructor_initializer_list}"""

INCLUDED_CONFIG_CONSTRUCTOR_ARG_ENTRY = (
    "std::shared_ptr<{config_name}> {config_variable_name}"
)

INCLUDED_CONFIG_CONSTRUCTOR_INITIALIZER_LIST_ENTRY = (
    "{config_variable_name}({config_variable_name})"
)

INCLUDED_CONFIG_COMMAND_LINE_ARG_STRUCT = """struct {config_name}CommandLineArgs
        {{
            {command_line_arg_struct_contents}
        }};"""

INCLUDED_CONFIG_COMMAND_LINE_ARG_ENTRY = "{config_name}CommandLineArgs {config_name};"

CONFIG_PUBLIC_ENTRY = """const std::shared_ptr<const {config_name}> {immutable_accessor_name}() const
    {{
        return std::const_pointer_cast<const {config_name}>({config_variable_name});
    }}

    const std::shared_ptr<{config_name}> {mutable_accessor_name}()
    {{
        return {config_variable_name};
    }}"""

CONFIG_PRIVATE_ENTRY = "std::shared_ptr<{config_name}> {config_variable_name};"

CONFIG_CONSTRUCTOR_ENTRY = "{config_variable_name} = std::make_shared<{config_name}>();"

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
        {included_config_command_line_arg_structs}

        struct commandLineArgs {{
            bool help = false;
            {command_line_arg_struct_contents}
        }};

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
    def __init__(self, config_name: str):
        self.config_name = config_name  # TODO: add "Config"?
        self.config_variable_name = config_name + "_config"
        self.parameters: List[CppParameter] = list()
        self.configs: List[CppConfig] = list()

        self.__forward_declaration = FORWARD_DECLARATION.format(
            config_name=self.config_name
        )

        self.__included_config_constructor_arg_entry = INCLUDED_CONFIG_CONSTRUCTOR_ARG_ENTRY.format(
            config_name=self.config_name, config_variable_name=self.config_variable_name
        )

        self.__included_config_constructor_initializer_list_entry = INCLUDED_CONFIG_CONSTRUCTOR_INITIALIZER_LIST_ENTRY.format(
            config_variable_name=self.config_variable_name
        )

        self.__config_public_entry = CONFIG_PUBLIC_ENTRY.format(
            config_name=self.config_name,
            immutable_accessor_name="get" + self.config_name,
            mutable_accessor_name="getMutable" + self.config_name,
            config_variable_name=self.config_variable_name,
        )

        self.__config_private_entry = CONFIG_PRIVATE_ENTRY.format(
            config_name=self.config_name, config_variable_name=self.config_variable_name
        )

        self.__config_constructor_entry = CONFIG_CONSTRUCTOR_ENTRY.format(
            config_variable_name=self.config_variable_name,
            config_name=self.config_name,
        )

        self.__included_config_command_line_arg_entry = INCLUDED_CONFIG_COMMAND_LINE_ARG_ENTRY.format(
            config_name=self.config_name
        )

    def add_parameter(self, parameter: CppParameter):
        self.parameters.append(parameter)

    def get_parameters(self) -> List[CppParameter]:
        return self.parameters.copy()

    def include_config(self, config: CppConfig):
        self.configs.append(config)

    @staticmethod
    def join_with_tabs(
        str: str, iterable: Iterable, num_tabs: int, half_tab: bool = False
    ):
        tab_size = HALF_TAB_SIZE if half_tab else TAB_SIZE
        return (str + num_tabs * tab_size * " ").join(iterable)

    @property
    def dependency_graph(self):
        return self.__dependency_graph

    @dependency_graph.setter
    def dependency_graph(self, dependency_graph: nx.DiGraph):
        print("Setting dependency graph for: " + self.config_name)
        print(dependency_graph.nodes)
        print(dependency_graph.edges)

        self.__dependency_graph = dependency_graph

    # to be run after dependency_graph has been set for all configs
    # TODO: can this be avoided?
    def post_dependency_graph_init(self):
        self.__dependency_graph_source_nodes = [
            node
            for node in self.dependency_graph.nodes
            if self.dependency_graph.in_degree(node) == 0
        ]
        print("All dag sources:", self.dependency_graph_source_nodes)
        self.__dependency_graph_topological_order_nodes = list(
            nx.topological_sort(self.dependency_graph)
        )

        self.__config_constructor_header = (
            CONFIG_CONSTRUCTOR_HEADER_NO_INCLUDES.format(config_name=self.config_name)
            if not self.dependency_graph_source_nodes
            else CONFIG_CONSTRUCTOR_HEADER_WITH_INCLUDES.format(
                config_name=self.config_name,
                config_constructor_args=self.config_constructor_args,
                config_constructor_initializer_list=self.config_constructor_initializer_list,
            )
        )

        self.__included_config_command_line_arg_struct = INCLUDED_CONFIG_COMMAND_LINE_ARG_STRUCT.format(
            config_name=self.config_name,
            command_line_arg_struct_contents=self.command_line_arg_struct_contents,
        )

        self.included_config_command_line_arg_entries = []
        self.included_config_load_command_line_args_into_config_contents = []
        for source in self.dependency_graph_source_nodes:
            self.dfs_helper(source, "", "")

    def dfs_helper(self, node, arg_prefix, load_dependency):
        config = self.dependency_graph.nodes[node]["config"]
        arg_prefix = node if not arg_prefix else arg_prefix + "." + node
        load_dependency = load_dependency + "getMutable{config_name}()->".format(
            config_name=node
        )
        for param in config.parameters:
            self.included_config_command_line_arg_entries.append(
                param.command_line_option_entry_with_prefix(arg_prefix + ".")
            )
            self.included_config_load_command_line_args_into_config_contents.append(
                param.load_command_line_arg_into_config_with_dependencies(
                    load_dependency, arg_prefix + "."
                )
            )

        for neighbor in self.dependency_graph.neighbors(node):
            self.dfs_helper(neighbor, arg_prefix, load_dependency)

    @property
    def dependency_graph_source_nodes(self):
        return self.__dependency_graph_source_nodes

    @property
    def dependency_graph_topological_order_nodes(self):
        return self.__dependency_graph_topological_order_nodes

    @property
    def forward_declaration(self):
        return self.__forward_declaration

    @property
    def definition(self):
        return CONFIG_CLASS.format(
            config_name=self.config_name,
            config_constructor_header=self.config_constructor_header,
            constructor_entries=self.constructor_entries,
            mutable_parameter_list_entries=self.mutable_parameter_list_entries,
            immutable_parameter_list_entries=self.immutable_parameter_list_entires,
            public_entries=self.public_entries,
            private_entries=self.private_entries,
            parse_command_line_args_function_contents=self.parse_command_line_args_function_contents,
            included_config_command_line_arg_structs=self.included_config_command_line_arg_structs,
            command_line_arg_struct_contents=self.command_line_arg_struct_contents,
            load_command_line_args_into_config_contents=self.load_command_line_args_into_config_contents,
        )

    @property
    def included_config_constructor_arg_entry(self):
        return self.__included_config_constructor_arg_entry

    @property
    def included_config_constructor_initializer_list_entry(self):
        return self.__included_config_constructor_initializer_list_entry

    @property
    def config_constructor_args(self):
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
        return self.__config_constructor_header

    @property
    def config_constructor_entry(self):
        # TODO: constructor with included configs
        return self.__config_constructor_entry

    @property
    def constructor_entries(self):
        return CppConfig.join_with_tabs(
            "\n",
            [param.constructor_entry for param in self.parameters]
            + [conf.config_constructor_entry for conf in self.configs],
            2,
        )

    @property
    def mutable_parameter_list_entries(self):
        # TODO: is @property needed? (for this function and others)
        return CppConfig.join_with_tabs(
            ",\n", [param.param_variable_name for param in self.parameters], 3
        )

    @property
    def immutable_parameter_list_entires(self):
        return CppConfig.join_with_tabs(
            ",\n",
            [param.immutable_parameter_list_entry for param in self.parameters],
            3,
        )

    @property
    def config_public_entry(self):
        return self.__config_public_entry

    @property
    def config_private_entry(self):
        return self.__config_private_entry

    @property
    def public_entries(self):
        # TODO: camel case or snake case
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
            2,
        )

    @property
    def parse_command_line_args_function_contents(self):
        # TODO: add the dependencies
        return CppConfig.join_with_tabs(
            "\n",
            [param.parameter_command_line_entry for param in self.parameters]
            + self.included_config_command_line_arg_entries,
            2,
        )

    @property
    def included_config_command_line_arg_entry(self):
        return self.__included_config_command_line_arg_entry

    @property
    def included_config_command_line_arg_struct(self):
        return self.__included_config_command_line_arg_struct

    @property
    def included_config_command_line_arg_structs(self):
        dependency_graph_reverse_topological_order_configs = [
            self.dependency_graph.nodes[node]["config"]
            for node in list(reversed(self.dependency_graph_topological_order_nodes))
        ]
        return CppConfig.join_with_tabs(
            "\n\n",
            [
                conf.included_config_command_line_arg_struct
                for conf in dependency_graph_reverse_topological_order_configs
            ],
            2,
        )

    @property
    def command_line_arg_struct_contents(self):
        dependency_graph_source_configs = [
            self.dependency_graph.nodes[node]["config"]
            for node in self.dependency_graph_source_nodes
        ]
        return CppConfig.join_with_tabs(
            "\n",
            [param.command_line_arg_entry for param in self.parameters]
            + [
                conf.included_config_command_line_arg_entry
                for conf in dependency_graph_source_configs
            ],
            3,
        )

    @property
    def load_command_line_args_into_config_contents(self):
        return CppConfig.join_with_tabs(
            "\n",
            [param.load_command_line_arg_into_config for param in self.parameters]
            + self.included_config_load_command_line_args_into_config_contents,
            2,
        )
