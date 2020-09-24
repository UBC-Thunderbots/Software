from c_parameter import CParameter
from typing import List, Set

#######################################################################
#                              C Config                               #
#######################################################################


class CConfig(object):

    DEFINITION = "typedef struct {name}_s {{{contents}}} {name}_t;\n"
    FORWARD_DECLERATION = "typedef struct {name}_s {name}_t;\n"
    MALLOC = "{name}_t* {name}_config = ({name}_t*)malloc(sizeof({name}_t));\n"
    INITIALIZATION = "{name}_t {name}_init = {{{contents}}};\n"
    INCLUDED_CONFIG_INITIALIZATION = ".{name} = {name}_config,\n"
    MEMCPY = "memcpy({name}_config, &{name}_init, sizeof({name}_t));\n"
    FREE = "free((void*){ptr_to_instance});\n"
    INCLUDE_CONFIG = "const {name}_t* {name};\n"

    def __init__(self, config_name: str, ptr_to_instance: str):
        """Initializes a CConfig with the given name. The corresponding
        generation strings (definition, forward_decleration, malloc, initialization,
        memcpy, free, include_config) are available from read-only properties.

        :param config_name: The name of the config (i.e the filename of the yaml)
        :param ptr_to_instance: A string representation of where this config
            is located (ex: ThunderbotsConfig->FooConfig)

        """
        self.config_name = config_name
        self.ptr_to_instance = ptr_to_instance

        self.parameters: Set[CParameter] = set()
        self.configs: Set[str] = set()

    def add_parameter(self, parameter: CParameter):
        """Add a parameter to this config. This parameters represent
        the parameters defined in the yaml file. When generating the Config
        the properties from these parameters will be properly formatted
        into the appropriate properties.

        :param parameter: The CParameter to add to this config

        """
        self.parameters.add(parameter)

    def get_parameters(self) -> List[CParameter]:
        """Returns the list of parameters included in this CConfig

        :returns: List of parameters

        """
        return self.parameters.copy()

    def include_config(self, config: str):
        """Add a config to this config to generate.

        :param config: The CConfig to add to this CConfig

        """
        self.configs.add(config)

    @property
    def definition(self):
        """Gets the 'contents' to format the DEFINITION string.
        Joins all the nested configs definitions and the parameter
        definitions.

        :returns: The formatted DEFINITION string that provides the definition
            of this config (struct)

        """
        definition_contents = "".join(
            [parameter.definition for parameter in self.parameters]
        ) + "".join([CConfig.INCLUDE_CONFIG.format(name=conf) for conf in self.configs])
        return CConfig.DEFINITION.format(
            name=self.config_name, contents=definition_contents
        )

    @property
    def initialization(self):
        """Gets the 'contents' to format the INITIALIZATION string.
        Joins all the nested configs initializations and the parameter
        initializations.

        :returns: The formatted INITIALIZATION string that provides the
            initialization struct that will be memcpy'ed to initialize
            the memory that was malloced

        """
        initialization_contents = "".join(
            [parameter.initialization for parameter in self.parameters]
        )

        initialization_contents += "".join(
            [
                CConfig.INCLUDED_CONFIG_INITIALIZATION.format(name=conf)
                for conf in self.configs
            ]
        )

        return CConfig.INITIALIZATION.format(
            name=self.config_name, contents=initialization_contents
        )

    @property
    def forward_declaration(self):
        return CConfig.FORWARD_DECLERATION.format(name=self.config_name)

    @property
    def malloc(self):
        return CConfig.MALLOC.format(name=self.config_name)

    @property
    def memcpy(self):
        return CConfig.MEMCPY.format(name=self.config_name)

    @property
    def free(self):
        return CConfig.FREE.format(ptr_to_instance=self.ptr_to_instance)
